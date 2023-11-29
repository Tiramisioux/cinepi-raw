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
#include "raw_options.hpp"
#include <arm_neon.h>

#include "dng_encoder.hpp"
#include "encoder/encoder.hpp"

typedef std::function<void(void *, size_t, int64_t, bool)> EncodeOutputReadyCallback;
typedef std::function<void(libcamera::ControlList &)> MetadataReadyCallback;

class CinePIRecorder : public RPiCamApp
{
public:
	using Stream = libcamera::Stream;
	using FrameBuffer = libcamera::FrameBuffer;

	CinePIRecorder() : RPiCamApp(std::make_unique<RawOptions>()) {}

	void StartEncoder()
	{
		createEncoder();
		encoder_->SetInputDoneCallback(std::bind(&CinePIRecorder::encodeBufferDone, this, std::placeholders::_1));
		encoder_->SetOutputReadyCallback(encode_output_ready_callback_);
	}
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

		// BufferReadSync r1(this, completed_request->buffers[stream]);
		// const std::vector<libcamera::Span<uint8_t>> mem = r1.Get();

		auto main_image_start = std::chrono::high_resolution_clock::now();

		BufferWriteSync w(this, completed_request->buffers[stream]);
		const std::vector<libcamera::Span<uint8_t>> mem = w.Get();
		uint16_t *ptr = (uint16_t *)mem[0].data();
		for (unsigned int i = 0; i < mem[0].size(); i += 2)
			*(ptr++) >>= 4;

		auto main_image_end = std::chrono::high_resolution_clock::now();
        auto main_image_duration = std::chrono::duration_cast<std::chrono::milliseconds>(main_image_end - main_image_start).count();

        LOG(1,"main image written to dng: " << main_image_duration << "ms");

		BufferReadSync r2(this, completed_request->buffers[lostream]);
		const std::vector<libcamera::Span<uint8_t>> lomem = r2.Get();
		
		if (!mem[0].data())
			throw std::runtime_error("no buffer to encode");

		if (!lomem[0].data())
			throw std::runtime_error("no buffer to encode, thumbnail");
			
		auto ts = completed_request->metadata.get(controls::SensorTimestamp);
		int64_t timestamp_ns = ts ? *ts : buffer->metadata().timestamp;
		encoder_->log_ts(timestamp_ns);
		{
			std::lock_guard<std::mutex> lock(encode_buffer_queue_mutex_);
			encode_buffer_queue_.push(completed_request); // creates a new reference
		}
		encoder_->EncodeBuffer2(buffer->planes()[0].fd.get(), mem[0].size(), (void *)mem[0].data(), info, lomem[0].size(), (void *)lomem[0].data(), loinfo, timestamp_ns / 1000, completed_request->metadata);
	}
	RawOptions *GetOptions() const { return static_cast<RawOptions *>(options_.get()); }
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
};
#endif // CINEPI_RECORDER_HPP