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


class DngEncoder : public Encoder
{
public:
	DngEncoder(RawOptions const *options);
	~DngEncoder();
	
	/* NEW – let the controller push µs-since-epoch for each frame */
    void setWallClockTimestamp(uint64_t us);   // µs since 1970-01-01

	/* Sensor-mode bit depth, snapshotted on the event-loop thread right after
	 * ConfigureVideo/selectMode synced options->mode to the real sensor mode.
	 * setup_encoder keys its 16-bit keep-full-depth decision off this instead
	 * of reading options_->mode.bit_depth live, which the redis subscriber
	 * thread mutates (a stale value could leak into a mid-reconfigure take). */
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

	size_t dng_save(int thread_num,
		const uint8_t *mem_buf,
		const uint8_t *raw,
		const StreamInfo &info,
		const uint8_t *lomem,
		const StreamInfo &loinfo,
		size_t losize,
		const libcamera::ControlList &metadata,
		int64_t timestamp_us,
		int64_t tc_frame_count);

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

    bool write12bit_{false};

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
