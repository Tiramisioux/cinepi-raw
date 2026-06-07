#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <stdint.h>
#include <thread>
#include <chrono>
#include <atomic>

//external dependancies
#include <sw/redis++/redis++.h>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <json/json.h>

//cinepi
#include "cinepi_recorder.hpp"
#include "cinepi_frameinfo.hpp"
#include "cinepi_state.hpp"
// #include "raw_options.hpp"
#include "cinepi_options.hpp"

#include "dng_encoder.hpp"
#include "utils.hpp"

#define CHANNEL_CONTROLS "cp_controls"
#define CHANNEL_STATS "cp_stats"
#define CHANNEL_HISTOGRAM "cp_histogram"

#define REDIS_DEFAULT "redis://127.0.0.1:6379/0"

using namespace sw::redis;

class CinePIController : public CinePIState
{
    public:
        CinePIController(CinePIRecorder *app)
            : CinePIState(),
            folderOpen(false),
            cameraRunning(false),
            trigger_(0),
            cameraInit_(true),
            app_(app),
            options_(app->GetOptions()),
            abortThread_(false)
        {
            console = spdlog::stdout_color_mt("cinepi_controller");
        };
        ~CinePIController() {
            abortThread_ = true;
            if (main_thread_.joinable())
                main_thread_.join();
        };

        bool buffer_size_sent_ = false;

        void start(){
            redis_ = std::make_unique<sw::redis::Redis>(options_->redis.value_or(REDIS_DEFAULT));
            console->debug(redis_->ping());
            main_thread_ = std::thread(std::bind(&CinePIController::mainThread, this));
        }

        void sync();

               /* -------------------------------------------------------------
        *  announceReady()
        *  – write a one-shot “cinepi_ready_camX = 1” key to Redis
        *    the *first* time it is called; subsequent calls are NOPs.
        * ----------------------------------------------------------- */
       void announceReady(const std::string &key)
       {
           if (!ready_announced_ && redis_)          // only once
           {
               redis_->set(key, "1");               // value must be a string
               ready_announced_ = true;
           }
       }

       [[nodiscard]] bool readyAnnounced() const noexcept
       { return ready_announced_; }

        void setShutterAngle(float angle){
            shutter_angle_ = angle;
            shutter_speed_ = 1.0 / ((framerate_ * 360.0) / shutter_angle_);
            uint64_t shutterTime = shutter_speed_ * 1e+6;
            libcamera::ControlList cl;
            cl.set(libcamera::controls::ExposureTime, shutterTime);
            app_->SetControls(cl);
        }

        void process(CompletedRequestPtr &completed_request);
        void process_stream_info(libcamera::StreamConfiguration const &cfg){

            Json::Value data;
            data["streamConfig"] = cfg.toString();
            redis_->publish(CHANNEL_STATS,  data.toStyledString());

            redis_->set(CONTROL_KEY_WIDTH, std::to_string(cfg.size.width));
            redis_->set(CONTROL_KEY_HEIGHT, std::to_string(cfg.size.height));
        }

        bool folderOpen;
        bool cameraRunning;

        bool configChanged(){
            return cameraInit_.exchange(false);
        }

    int triggerRec()
    {
        /* ── 0.  Bail out early if no medium mounted. ──────────────────────── */
        if (!disk_mounted(options_))
            return 0;

        /* ── 1. EDGE-trigger coming from UI / GPIO ─────────────────────────── */
        if (trigger_ != 0)
        {
            int state = trigger_;
            trigger_  = 0;                          // consume edge

            if (state > 0)                          /* ↑ start */
            {
                // Folder creation is handled by cinepi_raw.cpp after this
                // returns, using the sensor-derived wall-clock timestamp so
                // the folder FXX matches the DNG TC origin exactly.
                setRecording(true);
                is_recording_  = true;
                baseline_flag_ = 1;                 // keep level in sync
                return +1;
            }
            if (state < 0)                          /* ↓ stop  */
            {
                setRecording(false);
                is_recording_  = false;
                baseline_flag_ = 0;
                clip_number_++;                     // next take → new folder
                folderOpen     = false;
                return -1;
            }
        }

        /* ── 2.  Safety-net: act on Redis level changes only. ─────────────── */
        int rec_flag = 0;
        if (auto v = redis_->get("is_recording"); v && !v->empty())
            rec_flag = std::stoi(*v);               // 0 or 1

        /* ── first invocation: establish baseline, possibly join late. ────── */
        static bool first_call = true;
        if (first_call)
        {
            baseline_flag_ = rec_flag;
            first_call     = false;

            if (rec_flag && !is_recording_)         // already rolling → join
            {
                setRecording(true);
                is_recording_ = true;
                return +1;
            }
            return 0;
        }

        /* ── subsequent calls: react only on transitions. ─────────────────── */
        if (rec_flag != baseline_flag_)
        {
            baseline_flag_ = rec_flag;

            if (rec_flag && !is_recording_)         /* rising edge → start */
            {
                console->info("Safety-net started recording (late-join).");
                setRecording(true);
                is_recording_ = true;
                return +1;
            }
            if (!rec_flag && is_recording_)         /* falling edge → stop */
            {
                console->info("Safety-net stopped recording (others stopped).");
                setRecording(false);
                is_recording_ = false;
                clip_number_++;                     // prepare for next take
                folderOpen     = false;
                return -1;
            }
        }

        return 0;                                   // steady state, nothing to do
    }


    protected:

    private:
        // void getAllKeysAndValuesFromRedis() {
        //     // Fetch all keys.
        //     std::vector<OptionalString> keys;
        //     redis_->command("keys","*", std::back_inserter(keys));

        //     for (const auto &key : keys) {
        //         console->critical("{}",*key);
        //         auto value = redis_->get(*key);
        //         if (value && key) {
        //             allData[*key] = *value;
        //         }
        //     }
        // }

        bool ready_announced_ = false;

        int baseline_flag_{0};          // remembers last seen is_recording level

        std::shared_ptr<spdlog::logger> console;

        void mainThread();

        int trigger_;

        std::atomic_bool cameraInit_;

        CinePIRecorder *app_;

        // RawOptions *options_;
        CinePiOptions *options_;


        std::unique_ptr<sw::redis::Redis> redis_;

        Json::Value allData;

        bool abortThread_;
        std::thread main_thread_;
};
