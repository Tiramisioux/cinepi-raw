/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * libcamera_encoder.cpp - libcamera video encoding class.
 */

#ifndef CINEPI_RECORDER_HPP
#define CINEPI_RECORDER_HPP

#include "core/rpicam_app.hpp"
#include "core/stream_info.hpp"
//#include "raw_options.hpp"
#include "cinepi_options.hpp"
#include <arm_neon.h>

#include "dng_encoder.hpp"
#include "encoder/encoder.hpp"

#include "preview/hdmi_utils.hpp"
#include "preview/preview.hpp"

#include <cmath>


typedef std::function<void(void *, size_t, int64_t, bool)> EncodeOutputReadyCallback;
typedef std::function<void(libcamera::ControlList &)> MetadataReadyCallback;

class CinePIRecorder : public RPiCamApp
{
public:
	using Stream = libcamera::Stream;
	using FrameBuffer = libcamera::FrameBuffer;

	// CinePIRecorder() : RPiCamApp(std::make_unique<RawOptions>()) {}
	CinePIRecorder() : RPiCamApp(std::make_unique<CinePiOptions>()) {}

	void StartEncoder()
	{
		createEncoder();
		encoder_->SetInputDoneCallback(std::bind(&CinePIRecorder::encodeBufferDone, this, std::placeholders::_1));
		encoder_->SetOutputReadyCallback(encode_output_ready_callback_);
	}
        uint64_t last_timestamp_ns = 0;
	// This is callback when the encoder gives you the encoded output data.
	void SetEncodeOutputReadyCallback(EncodeOutputReadyCallback callback) { encode_output_ready_callback_ = callback; }
	void SetMetadataReadyCallback(MetadataReadyCallback callback) { metadata_ready_callback_ = callback; }
	void EncodeBuffer(CompletedRequestPtr &completed_request, Stream *stream, Stream *lostream)
	{
		assert(encoder_);

		if(!encoder_->initialized()){
			libcamera::StreamConfiguration const &cfg = stream->configuration();
			libcamera::StreamConfiguration const &lo_cfg = lostream->configuration();
			encoder_->setup_encoder(cfg, lo_cfg, completed_request->metadata);
		}

		StreamInfo info = GetStreamInfo(stream);
		StreamInfo loinfo = GetStreamInfo(lostream);

		FrameBuffer *buffer = completed_request->buffers[stream];
		BufferWriteSync w(this, completed_request->buffers[stream]);
		const std::vector<libcamera::Span<uint8_t>> mem = w.Get();

		BufferReadSync r2(this, completed_request->buffers[lostream]);
		const std::vector<libcamera::Span<uint8_t>> lomem = r2.Get();
		
		if (!mem[0].data())
			throw std::runtime_error("no buffer to encode");

		if (!lomem[0].data())
			throw std::runtime_error("no buffer to encode, thumbnail");
			
		auto ts = completed_request->metadata.get(controls::SensorTimestamp);
		int64_t timestamp_ns = ts ? *ts : buffer->metadata().timestamp;

		auto fd = completed_request->metadata.get(controls::FrameDuration);
		int64_t frameduration_us = fd ? *fd : 0;

                StartupGate::Phase phase = encoder_->pipelinePhase();
                if (phase == StartupGate::Phase::Recording)
                {
                        if (!cadence_counting_)
                        {
                                recorded_frames_ = 0;
                                cadence_counting_ = true;
                        }
                }
                else
                {
                        cadence_counting_ = false;
                        recorded_frames_ = 0;
                }

                bool counts_towards_drop = phase == StartupGate::Phase::Recording && cadence_counting_;
                if (counts_towards_drop)
                        ++recorded_frames_;

                bool ignore_window = false;
                if (counts_towards_drop)
                {
                        auto *opts = GetOptions();
                        ignore_window = recorded_frames_ <= opts->ignore_start_frames;
                }

                if (last_timestamp_ns != 0 && frameduration_us > 0)
                {
                        double measured_interval = static_cast<double>(timestamp_ns - last_timestamp_ns);
                        if (measured_interval > 0.0)
                        {
                                double fps_measured = 1e9 / measured_interval;
                                double fps_setting  = 1e6 / static_cast<double>(frameduration_us);
                                double delta        = std::abs(fps_measured - fps_setting);

                                if (delta > 1.0)
                                {
                                        auto metrics = encoder_->snapshotStageMetrics();
                                        bool cadence_ready = encoder_->cadenceActive();
                                        std::string reason;

                                        if (!cadence_ready)
                                                reason = "STARTUP";
                                        else if (metrics.encode_ms > 26.0)
                                                reason = "ENCODE_OVERRUN";
                                        else if (metrics.disk_ms > 30.0 || metrics.queue_depth > 1)
                                                reason = "DISK_STALL";
                                        else
                                                reason = "THERMAL";

                                        if (reason == "STARTUP" || !ignore_window)
                                        {
                                                LOG(1, "DROP[" << reason << "] frame=" << encoder_->getFrameCount()
                                                       << " fps_measured=" << fps_measured
                                                       << " fps_expected=" << fps_setting
                                                       << " queue=" << metrics.queue_depth
                                                       << " encode_ms=" << metrics.encode_ms
                                                       << " disk_ms=" << metrics.disk_ms);
                                        }
                                }
                        }
                }

                last_timestamp_ns = timestamp_ns;
		encoder_->log_ts(timestamp_ns);
		{
			std::lock_guard<std::mutex> lock(encode_buffer_queue_mutex_);
			encode_buffer_queue_.push(completed_request); // creates a new reference
		}
		encoder_->EncodeBuffer2(buffer->planes()[0].fd.get(), mem[0].size(), (void *)mem[0].data(), info, lomem[0].size(), (void *)lomem[0].data(), loinfo, timestamp_ns / 1000, completed_request->metadata);
	}
	// RawOptions *GetOptions() const { return static_cast<RawOptions *>(options_.get()); }
	
	CinePiOptions *GetOptions() const { return static_cast<CinePiOptions *>(options_.get()); }
	
	DngEncoder *GetEncoder() { return encoder_.get(); }
        void StopEncoder() { encoder_.reset(); }

        void ResetCadence()
        {
                last_timestamp_ns = 0;
                recorded_frames_ = 0;
                cadence_counting_ = false;
        }

protected:
	virtual void createEncoder()
	{
		encoder_ = std::unique_ptr<DngEncoder>(new DngEncoder(GetOptions()));
	}
	std::unique_ptr<DngEncoder> encoder_;

private:

	void encodeBufferDone(void *mem)
	{
		// If non-NULL, mem would indicate which buffer has been completed, but
		// currently we're just assuming everything is done in order. (We could
		// handle this by replacing the queue with a vector of <mem, completed_request>
		// pairs.)
		assert(mem == nullptr);
		{
			std::lock_guard<std::mutex> lock(encode_buffer_queue_mutex_);
			if (encode_buffer_queue_.empty())
				throw std::runtime_error("no buffer available to return");
			CompletedRequestPtr &completed_request = encode_buffer_queue_.front();
			if (metadata_ready_callback_ && !GetOptions()->metadata.empty())
				metadata_ready_callback_(completed_request->metadata);
			encode_buffer_queue_.pop(); // drop shared_ptr reference
		}
	}

	std::queue<CompletedRequestPtr> encode_buffer_queue_;
	std::mutex encode_buffer_queue_mutex_;
        EncodeOutputReadyCallback encode_output_ready_callback_;
        MetadataReadyCallback metadata_ready_callback_;

        uint64_t recorded_frames_ { 0 };
        bool cadence_counting_ { false };
};
#endif // CINEPI_RECORDER_HPP