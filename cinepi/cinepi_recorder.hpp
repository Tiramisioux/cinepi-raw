/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * libcamera_encoder.cpp - libcamera video encoding class.
 */

#ifndef CINEPI_RECORDER_HPP
#define CINEPI_RECORDER_HPP

#include <cmath>
#include <optional>

#include "core/rpicam_app.hpp"
#include "core/stream_info.hpp"
//#include "raw_options.hpp"
#include "cinepi_options.hpp"
#include <arm_neon.h>

#include "dng_encoder.hpp"
#include "encoder/encoder.hpp"

#include "preview/hdmi_utils.hpp"
#include "preview/preview.hpp"


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
	uint64_t last_timestamp_ns;
	// This is callback when the encoder gives you the encoded output data.
	void SetEncodeOutputReadyCallback(EncodeOutputReadyCallback callback) { encode_output_ready_callback_ = callback; }
	void SetMetadataReadyCallback(MetadataReadyCallback callback) { metadata_ready_callback_ = callback; }
	void EncodeBuffer(CompletedRequestPtr &completed_request, Stream *stream, Stream *lostream)
	{
		assert(encoder_);

		if(!encoder_->initialized()){
			libcamera::StreamConfiguration const &cfg = stream->configuration();
			/* Lores is optional: a standalone launch without --lores-width/height
			 * has no lores stream, and the DNG writer is raw-only (no embedded
			 * thumbnail). Never dereference a null stream here — this used to
			 * segfault at the first recorded frame. */
			static const libcamera::StreamConfiguration empty_lo_cfg;
			encoder_->setup_encoder(cfg, lostream ? lostream->configuration() : empty_lo_cfg,
						completed_request->metadata);
		}

		StreamInfo info = GetStreamInfo(stream);
		StreamInfo loinfo = lostream ? GetStreamInfo(lostream) : StreamInfo();

		FrameBuffer *buffer = completed_request->buffers[stream];
		BufferWriteSync w(this, completed_request->buffers[stream]);
		const std::vector<libcamera::Span<uint8_t>> mem = w.Get();

		std::optional<BufferReadSync> r2;
		size_t losize = 0;
		void *lodata = nullptr;
		if (lostream) {
			r2.emplace(this, completed_request->buffers[lostream]);
			const std::vector<libcamera::Span<uint8_t>> &lomem = r2->Get();
			if (!lomem[0].data())
				throw std::runtime_error("no buffer to encode, thumbnail");
			losize = lomem[0].size();
			lodata = (void *)lomem[0].data();
		}

		if (!mem[0].data())
			throw std::runtime_error("no buffer to encode");
			
		auto ts = completed_request->metadata.get(controls::SensorTimestamp);
		int64_t timestamp_ns = ts ? *ts : buffer->metadata().timestamp;

		auto fd = completed_request->metadata.get(controls::FrameDuration);
		int64_t frameduration_us = fd ? *fd : 0;

		float fps_measured = 1000000000.0/(timestamp_ns-last_timestamp_ns);
		float fps_setting  = 1000000.0/frameduration_us;

		if(abs(fps_measured - fps_setting) > 1){
			LOG(1,"Frame Drop!!!!!     FPS measured:"  << fps_measured<< " FPS settings:" << fps_setting);
		}

		last_timestamp_ns = timestamp_ns;
		encoder_->log_ts(timestamp_ns);
		{
			std::lock_guard<std::mutex> lock(encode_buffer_queue_mutex_);
			encode_buffer_queue_.push(completed_request); // creates a new reference
		}
		encoder_->EncodeBuffer2(buffer->planes()[0].fd.get(), mem[0].size(), (void *)mem[0].data(), info, losize, lodata, loinfo, timestamp_ns / 1000, completed_request->metadata);
	}
	// RawOptions *GetOptions() const { return static_cast<RawOptions *>(options_.get()); }
	
	CinePiOptions *GetOptions() const { return static_cast<CinePiOptions *>(options_.get()); }
	
	DngEncoder *GetEncoder() { return encoder_.get(); }
	void StopEncoder() { encoder_.reset(); }

	/* Pixels summed per output sample for the current sensor mode: 1 at full
	 * res, 4 for 2x2 binning. Derived from the sensor's own active area rather
	 * than a resolution literal, so it follows the sensor rather than a table
	 * of magic sizes — the CCMP decompand table is selected on this, and
	 * selecting on a resolution string is one of the ways to get it backwards.
	 *
	 * Rounded per axis, which absorbs a mode that crops slightly inside the
	 * array (3856/3840 -> 1, 3856/1920 -> 2). Returns 0 when the sensor does
	 * not report an active area or the mode is empty; callers treat 0 as
	 * "unknown" and fall through to the linear path. */
	double SensorBinning(const Mode &mode) const
	{
		if (!camera_ || !mode.width || !mode.height)
			return 0.0;
		auto area = camera_->properties().get(libcamera::properties::PixelArrayActiveAreas);
		if (!area || area->empty())
			return 0.0;
		const libcamera::Size active = (*area)[0].size();
		const double h = std::round(static_cast<double>(active.width) / mode.width);
		const double v = std::round(static_cast<double>(active.height) / mode.height);
		if (h < 1.0 || v < 1.0)
			return 0.0;
		return h * v;
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
};
#endif // CINEPI_RECORDER_HPP