/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022, Csaba Nagy.
 *
 * cinepi_raw.cpp - cinepi raw dng recording app.
 */

#include <chrono>
#include <vector>
#include <filesystem>
#include <ctime>
#include <sys/uio.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <algorithm>
#include "cinepi_sound.hpp"
#include "cinepi_controller.hpp"

#include "dng_encoder.hpp"
#include "output/output.hpp"
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include "cinepi_options.hpp"
#include "sync_utils.hpp"
#include <libcamera/controls.h>



using namespace std::placeholders;

libcamera::ControlList emptyMetadata;

static bool wasRecording = false;

namespace fs = std::filesystem;

// The main even loop for the application.
static bool write_frame_file(const fs::path &path,
                             const std::vector<uint8_t> &buffer,
                             RawOptions const *options,
                             uint32_t frame_number,
                             RawSyncPolicy policy,
                             RawOptions::SyncPolicy policy,
                             uint32_t sync_interval)
{
        constexpr size_t chunk = 1 << 20; // 1 MiB
        constexpr int max_iov = 8;

        int fd = open(path.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
        if (fd < 0)
        {
                spdlog::error("selftest: failed to open {}: {}", path.string(), strerror(errno));
                return false;
        }

        posix_fadvise(fd, 0, 0, POSIX_FADV_SEQUENTIAL);
        posix_fallocate(fd, 0, static_cast<off_t>(buffer.size()));

        const uint8_t *data = buffer.data();
        size_t remaining = buffer.size();
        off_t offset = 0;

        while (remaining > 0)
        {
                struct iovec iov[max_iov];
                int iovcnt = 0;
                size_t bytes_left = remaining;
                const uint8_t *ptr = data;

                while (bytes_left > 0 && iovcnt < max_iov)
                {
                        size_t len = std::min(bytes_left, chunk);
                        iov[iovcnt].iov_base = const_cast<uint8_t *>(ptr);
                        iov[iovcnt].iov_len  = len;
                        ptr        += len;
                        bytes_left -= len;
                        ++iovcnt;
                }

                ssize_t written = pwritev(fd, iov, iovcnt, offset);
                if (written < 0)
                {
                        spdlog::error("selftest: pwritev failed for {}: {}", path.string(), strerror(errno));
                        close(fd);
                        return false;
                }

                offset    += written;
                data      += written;
                remaining -= static_cast<size_t>(written);
        }

        bool sync_now = should_sync_frame(policy, sync_interval, frame_number);

        if (sync_now)
                fdatasync(fd);

        if (options && options->drop_cache_after_close)
                posix_fadvise(fd, 0, 0, POSIX_FADV_DONTNEED);

        close(fd);

        return true;
}

static int run_selftest(CinePiOptions *options)
{
        const uint32_t seconds = std::max<uint32_t>(1, options->selftest_seconds);
        const uint32_t frames  = 24 * seconds;
        const size_t   frame_size = static_cast<size_t>(13.5 * 1024 * 1024); // ~13.5 MiB

        fs::path base = fs::temp_directory_path() / "cinepi_selftest";
        fs::create_directories(base);

        std::time_t now = std::time(nullptr);
        std::tm *tm_now = std::localtime(&now);
        char stamp[32];
        std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", tm_now);
        fs::path take_dir = base / (std::string("selftest_") + stamp);
        fs::create_directories(take_dir);

        spdlog::info("selftest: writing {} frames to {}", frames, take_dir.string());

        std::vector<uint8_t> buffer(frame_size);
        for (size_t i = 0; i < frame_size; ++i)
                buffer[i] = static_cast<uint8_t>(i & 0xFF);

        RawSyncPolicy policy = options->sync_policy;
        RawOptions::SyncPolicy policy = options->sync_policy;
        uint32_t sync_interval = options->sync_interval ? options->sync_interval : 0;

        for (uint32_t i = 0; i < frames; ++i)
        {
                auto start = std::chrono::high_resolution_clock::now();
                fs::path frame_path = take_dir / (std::string("frame_") + std::to_string(i) + ".dng");
                if (!write_frame_file(frame_path, buffer, options, i + 1, policy, sync_interval))
                        return 1;

                auto end = std::chrono::high_resolution_clock::now();
                auto ms  = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                spdlog::info("selftest: frame {} took {} ms", i + 1, ms);
        }

        if (policy == RawSyncPolicy::Take)
        if (policy == RawOptions::SyncPolicy::Take)
        {
                int dir_fd = open(take_dir.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
                if (dir_fd >= 0)
                {
                        fdatasync(dir_fd);
                        close(dir_fd);
                }
        }

        spdlog::info("selftest complete: {} frame(s)", frames);
        return 0;
}

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

			/* ------------------------------------------------------------------ *
			*  Announce that this cinepi-raw instance is fully initialised.      *
			*  Key:  cinepi_ready_<camPort>   (e.g. cinepi_ready_cam0)           *
			* ------------------------------------------------------------------ */

			if (!controller.readyAnnounced())          // still unannounced?
			{
				std::string key = "cinepi_ready_" + options->CamPort();
				controller.announceReady(key);          // store one-shot flag
			}

			app.GetEncoder()->reset_encoder();
			controller.process_stream_info(cfg);
		}

		CinePIRecorder::Msg msg = app.Wait();

		//controller.setShutterAngle(180.0);

		if (msg.type == RPiCamApp::MsgType::Quit)
			return;

		if (msg.type == RPiCamApp::MsgType::Timeout)
		{
			console->error("Device timeout detected, attempting a restart!!!");
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
                        controller.folderOpen = create_clip_folder(app.GetOptions(), controller.getClipNumber());
            app.ResetCadence();
            app.GetEncoder()->resetFrameCount(); // folder already open
                        app.GetEncoder()->reset_encoder();
            if (controller.folderOpen)
            {
                app.GetEncoder()->markTakeDirectoryReady(true);
                app.GetEncoder()->armRecording();
            }
            else
            {
                app.GetEncoder()->markTakeDirectoryReady(false);
                console->error("Failed to create take directory; recording aborted");
            }
            sound.record_start();
        }
        else if (trigger < 0) {                  // recording stopped
                        controller.folderOpen = false;
            app.GetEncoder()->disarmRecording();
            app.GetEncoder()->markTakeDirectoryReady(false);
            app.ResetCadence();
            sound.record_stop();
        }

		// send frame to dng encoder and save to disk
		bool nowRecording = controller.isRecording();
		bool justStarted  = nowRecording && !wasRecording;

		if (nowRecording && controller.folderOpen)
		{
			if (app.GetEncoder()->buffer_full())
			{
				if (justStarted)
				{
					// first frame after you hit Record: clear and go on
					app.GetEncoder()->clearPool();
					console->warn("RAM pool was full at start — cleared and continuing");
				}
				else
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
		CinePISound sound(&app);
		CinePIController controller(&app);
		
		CinePiOptions *options = app.GetOptions();

                if (options->Parse(argc, argv))
                {
                        options->mediaDest = "/media/RAW";
                        options->rawCrop[0] = 0;
                        options->rawCrop[1] = 0;
                        options->rawCrop[2] = 0;
                        options->rawCrop[3] = 0;

                        if (options->verbose >= 2)
                                options->Print();

                        if (options->selftest)
                                return run_selftest(options);

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
