#pragma once
#ifndef DNG_ENCODER_H
#define DNG_ENCODER_H

#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>
#include <array>
#include <memory> // for std::shared_ptr
#include <string>
#include <atomic>
#include <optional>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include "encoder/encoder.hpp"
#include "raw_options.hpp"
#include "cinepi_frameinfo.hpp"
#include "log_lut.hpp"
#include "ccmp_lut.hpp"
#include "clip_ceiling.hpp"


class DngEncoder : public Encoder
{
public:
	DngEncoder(RawOptions const *options);
	~DngEncoder();
	
	/* NEW – let the controller push µs-since-epoch for each frame */
    void setWallClockTimestamp(uint64_t us);   // µs since 1970-01-01

	/* Sensor-mode bit depth, snapshotted on the event-loop thread in the same
	 * statement as the validated raw StreamConfiguration (cinepi_raw.cpp,
	 * immediately after StartCamera()). setup_encoder keys its 16-bit
	 * keep-full-depth decision and the CCMP gate off this instead of reading
	 * options_->mode.bit_depth live, which the redis subscriber thread
	 * mutates (a stale value could leak into a mid-reconfigure take). */
	void setSensorModeBitDepth(unsigned int bits) { sensor_mode_bit_depth_ = bits; }

	/* Pixels summed per output sample — 1 at full res, 4 for 2x2 binning.
	 * Snapshotted alongside the bit depth above and for the same reason.
	 *
	 * This SELECTS THE CCMP DECOMPAND TABLE. The compander's input is the
	 * binned signal, so the two 12-bit ClearHDR modes put their knees 4x apart
	 * in the delivered-linear domain and one table cannot serve both. Select on
	 * binning, never on ClearHDR alone and never on resolution-as-a-string:
	 * getting it backwards is wrong by 2.6x at knee1 and does not look
	 * obviously wrong in a render. */
	void setSensorBinning(double binning) { sensor_binning_ = binning; }

	/* Whether the bit-depth/binning snapshots above actually describe the
	 * stream setup_encoder() is about to configure. False when
	 * cinepi_raw.cpp found the requested mode's dimensions did not match the
	 * validated raw StreamConfiguration — the snapshots are still whatever
	 * was requested, not what the camera configured, so a 12-bit request
	 * landing on a genuinely 16-bit sensor mode must not be believed. See
	 * ccmp_gate.hpp for what this gates and why. Defaults to true: absent a
	 * call to this setter (only cinepi_raw.cpp calls it, once per
	 * reconfigure), the snapshots are trusted exactly as before this flag
	 * existed. */
	void setSensorModeTrusted(bool trusted) { sensor_mode_trusted_ = trusted; }

	/* The active picture's size, OUTPUT-domain, for the mode about to be
	 * recorded -- WP-CPR-3 (finding C4). std::nullopt when the sensor
	 * exposes no driver crop metadata (every stock sensor today) or the
	 * probe failed; dng_save() then writes no DefaultCropOrigin/Size/
	 * ActiveArea tags at all, exactly as before this package. Snapshotted
	 * at the same call site and for the same reason as setSensorBinning()
	 * above: cinepi_raw.cpp reads WP-CPR-2's driver metadata once per
	 * reconfigure, right after the validated raw StreamConfiguration is
	 * known, and hands the OUTPUT-domain size straight through (its own
	 * native-sensor-coordinate crop_width/crop_height already divided by
	 * the driver's linear binning -- see core/driver_mode_metadata.hpp and
	 * cinepi/ifd_builder.hpp's computeDngCropRect()).
	 *
	 * `sensor_window_crop` is the caller's driver_meta.crop_left != 0 ||
	 * driver_meta.crop_top != 0 (native sensor coordinates, WP-585-1):
	 * whether this mode reads a WINDOW of the sensor rather than the full
	 * field. It is NOT an origin -- see ifd_builder.hpp's file comment for
	 * why crop_left/crop_top cannot be used as one -- only a gate that
	 * makes computeDngCropRect() refuse rather than centre-guess for a
	 * windowed readout, whose buffer-local padding geometry this campaign
	 * has not established. Defaults to false, which is correct whenever
	 * `width`/`height` are std::nullopt (the gate is never consulted) and
	 * matches every full-field mode this campaign ships today. */
	void setActivePictureSize(std::optional<unsigned int> width, std::optional<unsigned int> height,
							   bool sensor_window_crop = false)
	{
		active_picture_width_  = width;
		active_picture_height_ = height;
		active_picture_is_sensor_window_crop_ = sensor_window_crop;
	}

	// Encode the given buffer.
	void EncodeBuffer(int fd, size_t size, void *mem, StreamInfo const &info, int64_t timestamp_us) override;
	void EncodeBuffer2(int fd, size_t size, void *mem, StreamInfo const &info, size_t losize, void *lomem, StreamInfo const &loinfo, int64_t timestamp_us, CompletedRequest::ControlList const &metadata);
	void resetFrameCount(){
		timestamps.clear();
		originationTimeCode.fill(0);
		originationDate.fill(0);
		index_ = 0;
		tc_origin_set_ = false;   // force re-capture of wall-clock origin on next frame
		tc_frame_count_ = 0;      // prevent stale value from previous take appearing in stats
		// Re-arm the ClearHDR clamp measurement for the new take: the merge
		// ceiling is an operating point, so last take's answer is not this
		// take's. Only the generation moves — clip_ceiling_white_ is left
		// alone on purpose, so any frame of the previous take still draining
		// through encodeThread() keeps reading the answer measured for IT.
		//
		// This runs on the rec trigger with the PREVIOUS take's frames still
		// in encode_queue_ (cinepi_raw.cpp leaves them flushing), which is why
		// EncodeItem carries its own ceiling_gen stamped at enqueue. Bumping a
		// counter that frames only read at dequeue would relabel every one of
		// those stragglers into this new take.
		{
			std::lock_guard<std::mutex> lk(encode_mutex_);
			clip_ceiling_claimed_ = false;
			++clip_ceiling_gen_;
		}
		dropped_frames_ = 0;
		write_failures_.store(0, std::memory_order_relaxed);  // reset disk-write-failure count
		buffer_hwm_.store(0, std::memory_order_relaxed);  // reset disk-backlog high-water mark
		// NOTE: frames_in_flight_ is intentionally NOT reset here. Unlike the
		// per-take counters above, it is a cross-take gauge of frames captured
		// but not yet on disk, maintained solely by the balanced ++/-- in
		// EncodeBuffer2 / encodeThread-drop / diskThread. The rec-gate keeps it
		// at 0 by the time a new take starts, so a reset would be a no-op in the
		// happy path; forcing 0 while stragglers are still draining would drive
		// it negative as their decrements land — i.e. it would falsely read
		// "done" mid-flush. Let the balanced accounting own this value.
	}

    /* What one frame needs to know about its take's WhiteLevel, resolved in
     * encodeThread() and handed to dng_save() whole.
     *
     * It is a struct rather than three more scalars on an already 11-parameter
     * signature, and it exists so dng_save() does NOT reach into the latch
     * itself: the value is read once, in encodeThread, immediately after the
     * wait and under the same lock that settles it. Reading it later — mid-
     * frame, while a newer take may already have published — is exactly the
     * bug this shape removes. */
    struct ClipCeilingJob
    {
        /* The take this frame belongs to. Stamped at ENQUEUE (EncodeBuffer2),
         * never read live from clip_ceiling_gen_ at dequeue: resetFrameCount()
         * bumps that counter on the rec trigger while the previous take is
         * still draining, so a dequeue-time read relabels a straggler into the
         * new take and lets it claim a measurement that is not its to make. */
        uint64_t gen     = 0;
        /* This frame owns the measurement and owes every other frame of its
         * take a published answer. */
        bool     publish = false;
        /* The take's answer, for frames that did not measure. 0 means "keep
         * the nominal dng_info.white" — which is also what a frame gets when
         * the wait timed out or the generation on offer belongs to someone
         * else, so a missing answer degrades to today's behaviour rather than
         * to another take's number. */
        uint32_t white   = 0;
    };

	size_t dng_save(int thread_num,
		const uint8_t *mem_buf,
		const uint8_t *raw,
		const StreamInfo &info,
		const uint8_t *lomem,
		const StreamInfo &loinfo,
		size_t losize,
		const libcamera::ControlList &metadata,
		int64_t timestamp_us,
		int64_t tc_frame_count,
		const ClipCeilingJob &ceiling);

	int bufferSize(){
		return disk_buffer_.size();
	}
	// Peak disk-write backlog observed since the previous call, then reset to 0.
	// The encode thread records the depth as it queues frames (independent of
	// frame delivery), so the GUI can surface transient buffer pressure that
	// instantaneous sampling — only published when a frame is delivered — misses.
	int bufferSizeMaxAndReset(){
		return buffer_hwm_.exchange(0, std::memory_order_relaxed);
	}
	void noteBufferDepth(int depth){
		int prev = buffer_hwm_.load(std::memory_order_relaxed);
		while (depth > prev &&
		       !buffer_hwm_.compare_exchange_weak(prev, depth, std::memory_order_relaxed)) {}
	}
	uint64_t getFrameCount(){
		return frames_;
	}

	int64_t getTcFrameCount() const {
		return tc_frame_count_;
	}

	// Frames that produced a hole in the take (inter-frame gap rounded to ≥2
	// frame periods). Does NOT include the tc_frame_count_ +1 floor used for
	// display continuity — those are jitter, not missing writes.
	int64_t getDroppedFrames() const {
		return dropped_frames_;
	}

	// Frames that were delivered and encoded but could NOT be written to disk
	// (open/write/close failure or short write). Distinct from getDroppedFrames():
	// a storage device that cannot keep up — e.g. NTFS under sustained 4K — loses
	// frames here with NO inter-frame sensor gap, so this is the only live signal
	// for write-stage data loss. Counted across all disk worker threads.
	int64_t getWriteFailures() const {
		return write_failures_.load(std::memory_order_relaxed);
	}

	// Total frames captured but not yet on disk: encode_queue_ + mid-encode + disk_buffer_.
	// Incremented when a frame enters encode_queue_ (EncodeBuffer2), decremented when
	// diskThread finishes with it (write success or any error/drop path).
	// Use this — not bufferSize() — to gate "still flushing" signals; bufferSize() only
	// covers the disk half and goes to zero while encode_queue_ is still draining.
	int64_t getFramesInFlight() const {
		return frames_in_flight_.load(std::memory_order_relaxed);
	}

	uint16_t photometric;
	uint16_t samples_per_pixel;
	uint8_t timecode[8];

	void log_ts(int64_t ts){
		timestamps.push_back(ts);
	}

	void setup_encoder(libcamera::StreamConfiguration const &cfg, libcamera::StreamConfiguration const &lo_cfg, CompletedRequest::ControlList const &metadata);
	bool initialized(){
		return encoder_initialized_;
	}
	void reset_encoder(){
		encoder_initialized_ = false;
	}
    uint64_t getWallClockTimestampUs() const { return wallclock_ts_us_; }
    bool buffer_full()           // inline definition
    {
        std::lock_guard<std::mutex> lk(ram_mtx_);
        return ram_buffers_ + 2 >= max_ram_buffers_;
    }

	bool mono_ = false;

	std::vector<int64_t> timestamps;
	std::array<uint8_t, 8> originationTimeCode;
	std::array<uint16_t, 3> originationDate;

	/* ---- PUBLIC: number of frame buffers that fit in RAM ---- */
	size_t maxRamBuffers() const { return max_ram_buffers_; }

private:
    /* NEW – cached wall-clock timestamp (0 = not set yet) */
    uint64_t wallclock_ts_us_ { 0 };

    /* Timecode: wall-clock origin captured at the first frame of each clip */
    bool     tc_origin_set_  { false };
    uint64_t tc_last_ts_us_  { 0 };    // ts_us of the previous frame, for delta-based counting
    int64_t  tc_frame_count_ { 0 };    // monotonic TC counter (display); +1 floor keeps it forward
    int64_t  dropped_frames_ { 0 };    // frames with no DNG written (inter-frame gap ≥ 2 periods)
    std::atomic<int64_t> write_failures_ { 0 };  // frames lost at disk write (open/write/close fail or short write)
    int      tc_start_hh_    { 0 };
    int      tc_start_mm_    { 0 };
    int      tc_start_ss_    { 0 };
    int      tc_fps_         { 24 };

    /* ──  NEW: in-RAM buffer accounting  ─────────────────────── */
    std::atomic<int>      buffer_hwm_{0};    /* peak disk_buffer_ depth since last publish */
    std::atomic<size_t>   ram_buffers_{0};   /* # TIFF blocks living in RAM   */
    std::atomic<int64_t>  frames_in_flight_{0}; /* capture→written: encode_queue_ + mid-encode + disk_buffer_ */
    size_t                max_ram_buffers_;  /* hard cap calculated at setup  */
    std::mutex            ram_mtx_;
    std::condition_variable ram_cv_;

    bool raw_packed_in_ = false;   /* true if DMA already delivers packed rows */
    bool raw_compressed_in_ = false;

    /* Output-depth overrides for the PiSP 16-bit container, resolved together
     * in setup_encoder() and mutually exclusive by construction — at most one
     * is ever true. See the comment at their assignment for why each conjunct
     * is there. Both false covers three different things, so don't read it as
     * one: a genuine 16-bit ClearHDR mode (container written verbatim); a row
     * depth that already matches the sensor's, i.e. Pi 4 / VC4; and any log
     * take, where the log block clears both because log_lut_ owns the row
     * conversion outright. dng_save()'s branch chain tests log_lut_ first for
     * exactly that reason. */
    bool write12bit_{false};
    bool write10bit_{false};

    /* ──  CineMate Log  ───────────────────────────────────────
     * Resolved once per configure in setup_encoder(), where the SOURCE depth
     * (the Bayer format) and the TARGET depth (--log-encode) are known
     * together; dng_save() only reads it. Non-null is the single "this clip is
     * log-encoded" switch, and it is only set when a spec ships for the pair —
     * a missing curve degrades to a normal linear recording rather than failing
     * the take. The pointer comes from the process-wide cache in log_lut.cpp
     * and stays valid for the process lifetime, so encode workers read it
     * without a lock. params().target_bits is the authoritative code depth. */
    const LogLut *log_lut_ = nullptr;

    /* How far the DMA row has to be shifted down to reach the curve's source
     * domain: 4 for a <=12-bit sensor mode on PiSP, which arrives MSB-aligned in
     * a 16-bit container, and 0 when the row is already right-justified. Set
     * beside log_lut_ and only meaningful while it is non-null. */
    unsigned log_src_shift_ = 0;

    /* ──  ClearHDR clamp latch (WhiteLevel)  ─────────────────
     *
     * The ClearHDR merge stops well below the container, but WhiteLevel is
     * written from the curve's nominal full scale, so no converter sees a
     * clipped pixel and every blown highlight renders magenta. clip_ceiling.hpp
     * has the mechanism in full. These four carry the measured answer.
     *
     * RESOLVED EXACTLY ONCE PER TAKE, from the take's FIRST frame, and held.
     * That is a hard requirement, not an optimisation: WhiteLevel is the
     * normalisation denominator, so re-deciding it mid-take would step the
     * whole frame's exposure (~0.18 EV at the measured clamp) and put a visible
     * jump in a graded clip. resetFrameCount() clears the latch on the rec
     * trigger; encodeThread() claims the measuring role under encode_mutex_,
     * where the FIFO dequeue order makes "first claim" mean "frame 0"; every
     * other frame waits on the CV until the answer is published.
     *
     * clip_ceiling_white_ is 0 for "no clamp found, keep the nominal
     * dng_info.white" — which is also what a refusal publishes, so a waiter
     * can never block on a frame that decided nothing.
     *
     * SCOPED BY GENERATION, not by a plain flag, because resetFrameCount()
     * runs on the rec trigger while the PREVIOUS take may still have frames
     * in flight. A straggler carrying generation N finds N already resolved
     * and reads its own take's answer; only frames of the new generation wait.
     * A bare "resolved" flag would instead send those stragglers to sleep
     * until the next take's first frame published — up to the full timeout
     * each, stalling the flush of a take that was already finished. */
    std::mutex              clip_ceiling_mutex_;
    std::condition_variable clip_ceiling_cv_;
    uint64_t                clip_ceiling_gen_      {1};   // guarded by encode_mutex_
    bool                    clip_ceiling_claimed_  {false};   // guarded by encode_mutex_
    uint64_t                clip_ceiling_resolved_gen_ {0};   // guarded by clip_ceiling_mutex_
    uint32_t                clip_ceiling_white_    {0};       // guarded by clip_ceiling_mutex_


    /* ──  DNG thumbnail (IFD1)  ─────────────────────────────
     * Snapshotted once per configure in setup_encoder() from
     * options_->thumbnail/thumbnailSize, exactly like log_lut_ above --
     * NOT read live from options_ in dng_save(). Two reasons, both from
     * the same fact: setup_encoder() re-runs at the start of every take
     * (reset_encoder() is called on the rec trigger and on every
     * resolution reconfigure; DngEncoder::initialized() then false-gates
     * the next EncodeBuffer() into a fresh setup_encoder() call), while
     * CONTROL_KEY_THUMBNAIL's own pub/sub handler applies live with no
     * restart of any kind.
     *   1. Per-take semantics with no camera restart: a `set thumbnail`
     *      mid-take changes options_->thumbnail immediately, but the
     *      snapshot -- and so the file on disk -- only picks it up at
     *      the NEXT take, never mid-take. Without this, two encode
     *      workers racing the live value could produce one take with a
     *      non-monotonic mix of thumbnail/no-thumbnail frames.
     *   2. dng_info.buffer_size can reserve exactly what this take needs
     *      (0 when off) instead of worst-case colour bytes on every take
     *      regardless of mode, which is what reading options_ live would
     *      have required (the mode could otherwise change after the
     *      buffer was sized but before the take that uses it starts).
     * Both values are consumed only through cinepi/dng_thumbnail.hpp's
     * thumbnail_geometry() -- setup_encoder()'s reservation and
     * dng_save()'s IFD1 write call the same formula, so they cannot
     * disagree about a take's thumbnail dimensions. dng_save() also calls
     * that header's add_thumbnail_ifd1_entries() to write IFD1's tags, so
     * the geometry formula and the tag layout both have exactly one
     * source, for every mode including JPEG (mode 3). */
    int thumb_mode_  = 0;   /* 0 off / 1 mono / 2 colour / 3 colour JPEG, this take */
    int thumb_shift_ = 0;   /* clamp(thumbnailSize, 0, 12), this take   */
    bool thumb_lores_warned_ = false;  /* one warning per take, not per frame */
    /* Mode 3 only: set once a JPEG-encoded frame overflows its own
     * uncompressed-worst-case reservation and dng_save() skips the
     * thumbnail for that frame (see dng_save()'s IFD1 block) -- same
     * one-warning-per-take shape as thumb_lores_warned_, a separate flag
     * because the two conditions are unrelated and can each recur on their
     * own schedule within a take. */
    bool thumb_jpeg_oversize_warned_ = false;

    /* ──  Reusable encoded-buffer pool  ───────────────────── */
    std::vector<uint8_t *> buffer_pool_;
    std::mutex              buffer_pool_mutex_;
    size_t                  pooled_buffer_size_{0};

    uint8_t *acquirePooledBuffer();
    void     releasePooledBuffer(uint8_t *buffer);
    void     drainPooledBuffers();

    std::shared_ptr<spdlog::logger> console;

    std::string make_tag_;
    std::string model_tag_;
    std::string software_tag_;
    std::string ucm_tag_;

        void encodeThread(int num);
        void diskThread(int num);
        void stopThreads();
        void configureThreadContext(const std::string &name,
                                     size_t index,
                                     size_t total,
                                     const std::optional<std::vector<int>> &affinity,
                                     const std::optional<int> &nice_value);

        bool encoder_initialized_;
        unsigned int sensor_mode_bit_depth_ = 0;
        double sensor_binning_ = 0.0;
        bool sensor_mode_trusted_ = true;
        std::optional<unsigned int> active_picture_width_;
        std::optional<unsigned int> active_picture_height_;
        bool active_picture_is_sensor_window_crop_ = false;

        /* The CCMP decompand table for this configuration, or nullptr when the
         * mode is not 12-bit ClearHDR. Owned by the process-wide cache in
         * ccmp_lut.cpp, so this is a borrowed pointer and stays valid. Resolved
         * once in setup_encoder; everything downstream keys off it. */
        const CcmpLut *ccmp_lut_ = nullptr;
	struct DngInfo
{
	uint8_t bits;

	uint32_t white;
	float black;

	float NEUTRAL[3];
	float ANALOGBALANCE[3];

	short cfa_repeat_pattern_dim[2];
	uint16_t black_level_repeat_dim[2];
	char bayer_order[4];

    float CAM_XYZ[9];     // daylight matrix
    float CAM_XYZ2[9];    // tungsten / second matrix   ← NEW

	uint16_t offset_y_start;
	uint16_t offset_y_end;
	uint16_t offset_x_start;
	uint16_t offset_x_end;
	float bppf;
	uint16_t byte_offset_x;

	uint16_t t_height;
	uint16_t t_width;

	uint16_t photometric;
	uint16_t samples_per_pixel;
	uint8_t timecode[8];

	unsigned int compression;

	size_t buffer_size;

	uint8_t thumbType;
	uint16_t thumbWidth;
	uint16_t thumbHeight;
	uint16_t thumbPhotometric;
	uint16_t thumbBitsPerSample;
	uint16_t thumbSamplesPerPixel;

	std::string make;
	std::string model;
	std::string serial;
	std::string ucm;
	std::string software;
};

	DngInfo dng_info;
	unsigned int max_buffer_frames;

	bool encodeCheck_;
        bool resetCount_;
        uint64_t index_;
        uint64_t frames_;

    RawOptions const *options_;

        size_t encode_worker_count_ { 0 };
        size_t disk_worker_count_   { 0 };
        std::vector<std::thread> encode_threads_;
        std::vector<std::thread> disk_threads_;
        std::atomic<bool> stop_encode_ { false };
        std::atomic<bool> stop_disk_   { false };
        std::optional<std::vector<int>> encode_affinity_;
        std::optional<std::vector<int>> disk_affinity_;
        std::optional<int> encode_nice_;
        std::optional<int> disk_nice_;

        struct EncodeItem
        {

                void *mem;
        size_t size;
		StreamInfo info;
		void *lomem;
		size_t losize;
		StreamInfo loinfo;
		CompletedRequest::ControlList met;
		int64_t timestamp_us;
		uint64_t index;
		std::string folder;
		int64_t tc_frame_count { 0 }; // pre-computed TC frame number, set under encode_mutex_
		// The take this frame belongs to, stamped at enqueue under
		// encode_mutex_ — the same mutex resetFrameCount() bumps the counter
		// under, so a frame cannot be stamped with a generation that is
		// changing. Stamped here for the same reason `folder` is: by the time
		// a frame is encoded, the camera may already be in the next take.
		uint64_t ceiling_gen { 0 };
	};
	std::queue<EncodeItem> encode_queue_;
	std::mutex encode_mutex_;
        std::condition_variable encode_cond_var_;

        struct DiskItem
        {
                void *mem_buf;
                size_t size;
                StreamInfo info;
                CompletedRequest::ControlList met;
                int64_t timestamp_us;
                uint64_t index;
                std::string folder;
        };
        std::queue<DiskItem> disk_buffer_;
        std::mutex disk_mutex_;
        std::condition_variable disk_cond_var_;
};

#endif
