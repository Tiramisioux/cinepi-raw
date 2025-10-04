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

#include <utility>
#include <optional>
#include <sstream>
#include <iomanip>
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
                last_disk_drop_frame_.reset();
                encoder_->SetInputDoneCallback(std::bind(&CinePIRecorder::encodeBufferDone, this, std::placeholders::_1));
                encoder_->SetOutputReadyCallback(encode_output_ready_callback_);
                encoder_->SetDiskErrorCallback(disk_error_callback_);
        }
	uint64_t last_timestamp_ns;
	// This is callback when the encoder gives you the encoded output data.
        void SetEncodeOutputReadyCallback(EncodeOutputReadyCallback callback) { encode_output_ready_callback_ = callback; }
        void SetMetadataReadyCallback(MetadataReadyCallback callback) { metadata_ready_callback_ = callback; }
        void SetDiskErrorCallback(DngEncoder::DiskErrorCallback callback)
        {
                disk_error_callback_ = std::move(callback);
                if (encoder_)
                        encoder_->SetDiskErrorCallback(disk_error_callback_);
        }
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

                double fps_measured = 0.0;
                double fps_setting  = 0.0;
                bool have_rate_info = last_timestamp_ns != 0 && timestamp_ns > last_timestamp_ns && frameduration_us > 0;

                if (have_rate_info) {
                        fps_measured = 1000000000.0 / static_cast<double>(timestamp_ns - last_timestamp_ns);
                        fps_setting  = 1000000.0 / static_cast<double>(frameduration_us);

                        double fps_delta = std::fabs(fps_measured - fps_setting);
                        if (fps_delta > 1.0) {
                                LOG(1,"Frame Drop!!!!!     FPS measured:"  << fps_measured<< " FPS settings:" << fps_setting);

                                if (encoder_) {
                                        uint64_t frame_index = encoder_->getFrameCount();

                                        if (!last_disk_drop_frame_ || *last_disk_drop_frame_ != frame_index) {
                                                last_disk_drop_frame_ = frame_index;

                                                std::string filename = encoder_->MakeOutputFilename(frame_index);
                                                std::ostringstream reason;
                                                reason << std::fixed << std::setprecision(2)
                                                       << "Frame rate drop detected: measured " << fps_measured
                                                       << " FPS vs setting " << fps_setting << " FPS";
                                                encoder_->RecordDiskFailure(frame_index, filename, reason.str());
                                        }
                                }
                        } else {
                                last_disk_drop_frame_.reset();
                        }
                } else {
                        last_disk_drop_frame_.reset();
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
        DngEncoder::DiskErrorCallback disk_error_callback_;
        std::optional<uint64_t> last_disk_drop_frame_;
};
#endif // CINEPI_RECORDER_HPP
