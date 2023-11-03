#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <stdint.h>
#include <thread>
#include <chrono>

//external dependancies
#include <sw/redis++/redis++.h>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <json/json.h>

//cinepi
#include "cinepi_recorder.hpp"
#include "cinepi_frameinfo.hpp"
#include "cinepi_state.hpp"
#include "raw_options.hpp"
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
            app_(app),
            folderOpen(false),
            cameraRunning(false),
            trigger_(0),
            options_(app->GetOptions()), 
            cameraInit_(true), 
            abortThread_(false) 
        {
            console = spdlog::stdout_color_mt("cinepi_controller");
        };
        ~CinePIController() {
            abortThread_ = true;
            main_thread_.join();
        };

        void start(){
            redis_ = std::make_unique<sw::redis::Redis>(options_->redis.value_or(REDIS_DEFAULT));
            console->debug(redis_->ping());
            main_thread_ = std::thread(std::bind(&CinePIController::mainThread, this));
        }

        void sync();

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
            bool c = cameraInit_;
            cameraInit_ = false;
            return c;
        }

        int triggerRec(){
            if(!disk_mounted(const_cast<RawOptions *>(options_))){
                return 0;
            }
            int state = trigger_;
            if(state < 0){
                clip_number_++;
            }
            trigger_ = 0;
            return state;
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

        std::shared_ptr<spdlog::logger> console;

        void mainThread();

        int trigger_;

        bool cameraInit_;

        CinePIRecorder *app_;
        RawOptions *options_;

        std::unique_ptr<sw::redis::Redis> redis_;

        Json::Value allData;

        bool abortThread_;
        std::thread main_thread_;
};