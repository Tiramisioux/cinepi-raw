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


class DngEncoder : public Encoder
{
public:
	DngEncoder(RawOptions const *options);
	~DngEncoder();
	
	/* NEW – let the controller push µs-since-epoch for each frame */
    void setWallClockTimestamp(uint64_t us);   // µs since 1970-01-01

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

	/// Drain any queued frames and free their buffers so that
	/// buffer_full() will return false again.
	void clearPool();

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
    size_t                max_ram_buffers_;  /* hard cap calculated at setup  */
    std::mutex            ram_mtx_;
    std::condition_variable ram_cv_;

    bool raw_packed_in_ = false;   /* true if DMA already delivers packed rows */
    bool raw_compressed_in_ = false;

    bool write12bit_{false};

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
	struct DngInfo
{
	uint8_t bits;

	uint32_t white;
	float black;
	float black_levels[4];

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
