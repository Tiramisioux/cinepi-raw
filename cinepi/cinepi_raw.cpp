/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022, Csaba Nagy.
 *
 * cinepi_raw.cpp - cinepi raw dng recording app.
 */

#include <chrono>
#include <vector>
#include "cinepi_sound.hpp"
#include "cinepi_controller.hpp"

#include "dng_encoder.hpp"
#include "output/output.hpp"
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include "cinepi_options.hpp"
#include "core/driver_mode_metadata.hpp"
#include "sensor_binning_source.hpp"
#include <libcamera/controls.h>



using namespace std::placeholders;

libcamera::ControlList emptyMetadata;

static bool wasRecording = false;

// The main even loop for the application.
static void event_loop(CinePIRecorder &app, CinePIController &controller, CinePISound &sound)
{
	controller.start();
	controller.sync();

	sound.start();

	static auto console = spdlog::stdout_color_mt("event_loop"); 
	
	CinePiOptions *options = app.GetOptions();

	std::unique_ptr<Output> output = std::unique_ptr<Output>(Output::Create(options));
	app.SetEncodeOutputReadyCallback(std::bind(&Output::OutputReady, output.get(), _1, _2, _3, _4));
	app.SetMetadataReadyCallback(std::bind(&Output::MetadataReady, output.get(), _1));

	app.OpenCamera();

	// 12-bit ClearHDR reaches the ISP still companded, so every preview and the
	// DNG thumbnail render magenta while the recorded DNG — which carries a
	// LinearizationTable — does not. ccmpPreview re-renders the lores frame from
	// the raw Bayer with the decompand applied, and has to run before the stages
	// that consume that frame. Inserted here rather than left to the
	// post-process JSON because that file is written by the Cinemate installer,
	// so an existing Pi would not have the entry. The stage no-ops on every
	// other sensor mode.
	app.EnsureFirstPostProcessingStage("ccmpPreview");
        //app.ConfigureViewfinder();
	app.StartEncoder();
	std::vector<std::shared_ptr<libcamera::Camera>> cameras = app.GetCameras();
	if (cameras.size() == 0)
    	throw std::runtime_error("no cameras available");
	app.GetOptions()->model = app.CameraModel();

	for (unsigned int count = 0; ; count++)
	{
		// if we change the sensor mode, restart the camera. 
		if(controller.configChanged()){
			bool resumeRecording = controller.isRecording();
			if (resumeRecording)
			{
				console->warn("Resolution reconfigure requested while recording; splitting current recording before camera restart.");
				controller.setRecording(false);
				controller.folderOpen = false;
				sound.record_stop();
				controller.advanceClipNumber();
			}

			if(controller.cameraRunning){
				app.StopCamera();
				app.Teardown();
			}
			app.ConfigureVideo(CinePIRecorder::FLAG_VIDEO_RAW, 0);
                        //app.ConfigureViewfinder();

			if (!options->ScalerCrops().empty())
			{
				/* 1. grab the set */
				const auto &streamSet = cameras[0]->streams();   // std::set<Stream*>

				/* 2. copy into a vector so we can use [i] */
				std::vector<libcamera::Stream *> streams(streamSet.begin(), streamSet.end());

				/* 3. build per-stream rectangles */
				auto fracs = options->ScalerCrops();             // already padded
				std::vector<libcamera::Rectangle> pixelRects;

				for (size_t i = 0; i < streams.size(); ++i)
				{
					libcamera::Stream *s = streams[i];           // now index-able
					const auto &cfg      = s->configuration();

					uint32_t W = cfg.size.width;
					uint32_t H = cfg.size.height;

					const auto &f = fracs[i];                    // {x,y,w,h} 0-1
					uint32_t x = static_cast<uint32_t>(f[0] * W) & ~1U;
					uint32_t y = static_cast<uint32_t>(f[1] * H) & ~1U;
					uint32_t w = static_cast<uint32_t>(f[2] * W) & ~1U;
					uint32_t h = static_cast<uint32_t>(f[3] * H) & ~1U;

					pixelRects.emplace_back(x, y, w, h);
				}

				/* 4. push the control list */
				libcamera::ControlList ctrls(cameras[0]->controls());
				ctrls.set(controls::rpi::ScalerCrops, pixelRects);
				ctrls.set(controls::rpi::StatsOutputEnable, true);   // enable AGC on new FoV
				app.SetControls(std::move(ctrls));
			}


			app.StartCamera();
			controller.cameraRunning = true;

			libcamera::StreamConfiguration const &cfg = app.RawStream()->configuration();
			console->info("Raw stream: {}x{} : {} : {}", cfg.size.width, cfg.size.height, cfg.stride, cfg.pixelFormat.toString());

			// Freeze the requested mode HERE, in the same statement as reading
			// cfg, before anything below touches Redis. options->mode is a
			// live pointer into the controller's redis-mutable state — reading
			// it any later (readyAnnounced/announceReady are Redis round trips
			// that can interleave with the subscriber thread) risks a
			// mode-switch racing in between and silently mislabelling this
			// stream's bit depth. cfg is the actual validated raw stream and
			// cannot race, since it stays whatever StartCamera() just
			// negotiated until the next reconfigure.
			const unsigned int requested_width = options->mode.width;
			const unsigned int requested_height = options->mode.height;
			const unsigned int requested_bit_depth = options->mode.bit_depth;

			// Whether the frozen values above can be believed to describe the
			// stream cfg actually is. False on a dims mismatch: setSensorModeBitDepth()
			// still gets requested_bit_depth below (it has nothing else to snapshot),
			// but the encoder must not trust that value for the CCMP gate or the
			// 16-bit keep-full-depth decision — see setSensorModeTrusted()'s comment
			// and cinepi/ccmp_gate.hpp.
			const bool mode_trusted = (requested_width == cfg.size.width && requested_height == cfg.size.height);
			if (!mode_trusted)
				console->warn("Requested mode {}x{} does not match the configured raw stream "
							   "{}x{}; using the configured stream for binning and bit depth.",
							   requested_width, requested_height, cfg.size.width, cfg.size.height);

			/* ------------------------------------------------------------------ *
			*  Announce that this cinepi-raw instance is fully initialised.      *
			*  Key:  cinepi_ready_<camPort>   (e.g. cinepi_ready_cam0)           *
			* ------------------------------------------------------------------ */

			if (!controller.readyAnnounced())          // still unannounced?
			{
				std::string key = "cinepi_ready_" + options->CamPort();
				controller.announceReady(key);          // store one-shot flag
			}

			// Snapshot the validated sensor-mode bit depth for the encoder's
			// 16-bit keep-full-depth decision, frozen above alongside cfg.
			app.GetEncoder()->setSensorModeBitDepth(requested_bit_depth);
			// Same snapshot, same reason: the CCMP decompand table is selected
			// on the BINNING of the stream the camera actually configured —
			// cfg.size, not the (possibly since-mutated) requested mode.
			//
			// WP-CPR-2 (finding C2): the active-area/delivered-size ratio
			// below is wrong for a window crop (a 1440x1080 2x2 window rounds
			// to 6, a 1x1 1920x1120 crop rounds to 4 when the truth is 1),
			// which is exactly the case the aspect-ratio family and the
			// windowed HDR modes introduce. Prefer the sensor driver's own
			// "Mode Binning"/"Mode Crop *" controls when it exposes them and
			// the value is sane; a stock sensor (or a driver not yet carrying
			// WP-585-1/WP-283-5) reports no such control, so this falls back
			// to the ratio exactly as before — see sensor_binning_source.hpp.
			//
			// app.CameraId() is THIS process's own libcamera::Camera::id(),
			// the same signal cinepi_options.cpp's portFromCameraId() reads.
			// On a dual-sensor rig running two cinepi_raw processes it is
			// what lets the probe bind to this process's own sensor
			// sub-device instead of whichever one happens to sort first —
			// see core/driver_mode_metadata.hpp and cinepi/subdev_binding.hpp.
			// On a single-sensor rig this changes nothing.
			DriverModeMetadata driver_meta;
			const bool have_driver_meta = read_driver_mode_metadata(driver_meta, app.CameraId());
			const SensorBinningDecision binning_decision = choose_sensor_binning(
				have_driver_meta ? std::optional<int>(driver_meta.binning) : std::nullopt,
				app.SensorBinning(cfg.size.width, cfg.size.height));
			console->info("Sensor binning {} (source: {})", binning_decision.binning,
						   binning_decision.source == SensorBinningSource::kDriver
							   ? "driver" : "ratio");
			app.GetEncoder()->setSensorBinning(binning_decision.binning);
			// WP-CPR-3 (finding C4): hand the encoder the active picture's
			// OUTPUT-domain size, from the SAME driver metadata probe above
			// (never re-probed), so dng_save() can write DefaultCropOrigin/
			// Size/ActiveArea when the transport frame carries padding —
			// see cinepi/ifd_builder.hpp's computeDngCropRect(). The
			// driver's crop_width/crop_height are reported in NATIVE SENSOR
			// coordinates (core/driver_mode_metadata.hpp); dividing by the
			// driver's own linear binning (validated to 1 or 2 whenever
			// have_driver_meta is true) converts to the OUTPUT-pixel domain
			// dng_save()'s transport size (info.width/height) is already
			// in. A stock sensor, or a probe that failed, reports nothing
			// here either — same std::nullopt behaviour as before this
			// package, so no crop tags are written.
			//
			// WP-CPR-3 review round 3 (finding C4-followup-2): a prior
			// rework also passed whether the driver reports a non-zero
			// crop_left/crop_top (a windowed sensor readout, WP-585-1)
			// so computeDngCropRect() could refuse the crop tags for
			// every windowed mode. That refusal overshot the finding it
			// was meant to fix -- crop_left/crop_top are native-SENSOR
			// coordinates for FOV/zoom bookkeeping and were never usable
			// as the DNG origin (that part stands, see ifd_builder.hpp's
			// file comment), but the RAW16 OB padding this crop
			// rectangle describes is a uniform, vertical-only,
			// buffer-internal convention (ASPECT-RATIOS.md) independent
			// of where the readout window sits on the sensor, so the
			// windowed case does not need refusing. crop_left/crop_top
			// are no longer passed here at all.
			if (have_driver_meta)
				app.GetEncoder()->setActivePictureSize(
					static_cast<unsigned int>(driver_meta.crop_width / driver_meta.binning),
					static_cast<unsigned int>(driver_meta.crop_height / driver_meta.binning));
			else
				app.GetEncoder()->setActivePictureSize(std::nullopt, std::nullopt);
			// Tell the encoder whether the two snapshots above can be
			// believed at all — see setSensorModeTrusted()'s comment.
			app.GetEncoder()->setSensorModeTrusted(mode_trusted);
			app.GetEncoder()->reset_encoder();
			controller.process_stream_info(cfg);

			if (resumeRecording)
			{
				controller.folderOpen = create_clip_folder(app.GetOptions(), controller.getClipNumber());
				if (controller.folderOpen)
				{
					sound.record_start();
					app.GetEncoder()->resetFrameCount();
					app.GetEncoder()->reset_encoder();
					controller.setRecording(true);
					console->warn("Recording resumed after resolution reconfigure in clip folder: {}", app.GetOptions()->folder);
				}
				else
				{
					console->error("Failed to create clip folder after recording-time resolution reconfigure; recording remains stopped.");
				}
			}
		}

		CinePIRecorder::Msg msg = app.WaitFor(std::chrono::milliseconds(3000));

		//controller.setShutterAngle(180.0);

		if (msg.type == RPiCamApp::MsgType::Quit)
			return;

		if (msg.type == RPiCamApp::MsgType::Timeout)
		{
			console->error("No camera frames received for 3s, attempting a camera restart!!!");
			app.StopCamera();
			app.StartCamera();
			continue;
		}
		if (msg.type != CinePIRecorder::MsgType::RequestComplete)
			throw std::runtime_error("unrecognised message!");

		CompletedRequestPtr &completed_request = std::get<CompletedRequestPtr>(msg.payload);

		// parse the frame info metadata for the current frame, publish to redis stats channel
		controller.process(completed_request);

		// check for record trigger signal, open a new folder if rec_start or reset frame count if _rec_stop
		int trigger = controller.triggerRec();

        if (trigger > 0) {                       // recording just started
            // Nothing is dropped here.  The previous take's buffered frames
            // (encode_queue_ awaiting compression and disk_buffer_ awaiting
            // write) are left to finish flushing to their own clip folder, so
            // no recorded frame is lost.  Cinemate blocks the rec trigger while
            // the green is_writing_buf flush is in progress, so by the time a
            // start edge reaches us the RAM buffer has already drained and the
            // new take begins with free buffers.
            // Use the sensor-derived wall-clock set by process() for this
            // frame so the folder FXX equals llround(sub_us × fps / 1e6),
            // which is exactly how the DNG TC origin sub_frames is computed.
            uint64_t wall_ts_us = app.GetEncoder()->getWallClockTimestampUs();
			controller.folderOpen = create_clip_folder(app.GetOptions(), controller.getClipNumber(), wall_ts_us);
            if (controller.folderOpen)
                sound.record_start();
            app.GetEncoder()->resetFrameCount(); // folder already open
			app.GetEncoder()->reset_encoder();
        }
        else if (trigger < 0) {                  // recording stopped
			controller.folderOpen = false;
            sound.record_stop();
        }

		// send frame to dng encoder and save to disk
		bool nowRecording = controller.isRecording();
		bool justStarted  = nowRecording && !wasRecording;

		if (nowRecording && controller.folderOpen)
		{
			if (app.GetEncoder()->buffer_full())
			{
				// Buffer still full at the first frame of a new take — the
				// previous take's frames had not finished draining yet.  Let
				// the first frame through (justStarted); only a steady-state
				// take that fills RAM should stop here.
				if (!justStarted)
				{
					controller.setRecording(false);
					console->warn("RAM pool exhausted — recording stopped");
				}
			}
			app.EncodeBuffer(completed_request, app.RawStream(), app.LoresStream());
		}

		// update for next iteration
		wasRecording = nowRecording;


		// show frame on display
		app.ShowPreview(completed_request, app.LoresStream());//app.GetMainStream());

		//console->info("Frame Number: {}", count);
	}
}

int main(int argc, char *argv[])
{
	try
	{
		CinePIRecorder app;

		CinePiOptions *options = app.GetOptions();

		if (options->Parse(argc, argv))
		{
			CinePISound sound(&app);
			CinePIController controller(&app);

			options->mediaDest = "/media/RAW";
			options->rawCrop[0] = 0;
			options->rawCrop[1] = 0;
			options->rawCrop[2] = 0;
			options->rawCrop[3] = 0;

			if (options->verbose >= 2)
				options->Print();

			event_loop(app, controller, sound);
		}
	}
	catch (std::exception const &e)
	{
		LOG_ERROR("ERROR: *** " << e.what() << " ***");
		return -1;
	}
	return 0;
}
