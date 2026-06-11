#include "cinepi_controller.hpp"

#include <algorithm>
#include <cctype>
#include <iomanip>
#include <sstream>

using namespace std;
using namespace std::chrono;

#define CP_DEF_WIDTH 1920
#define CP_DEF_HEIGHT 1080
#define CP_DEF_FRAMERATE 30
#define CP_DEF_ISO 400
#define CP_DEF_SHUTTER 50
#define CP_DEF_AWB 1
#define CP_DEF_COMPRESS 0
#define CP_DEF_THUMBNAIL 1
#define CP_DEF_THUMBNAIL_SIZE 3

void CinePIController::sync(){
    // getAllKeysAndValuesFromRedis();

    auto pipe = redis_->pipeline();
    auto pipe_replies = pipe.get(CONTROL_KEY_WIDTH)
                            .get(CONTROL_KEY_HEIGHT)
                            .get(CONTROL_KEY_FRAMERATE)
                            .get(CONTROL_KEY_ISO)
                            .get(CONTROL_KEY_SHUTTER_SPEED)
                            .get(CONTROL_KEY_WB)
                            .get(CONTROL_KEY_COLORGAINS)
                            .get(CONTROL_KEY_COMPRESSION)
                            .get(CONTROL_KEY_THUMBNAIL)
                            .get(CONTROL_KEY_THUMBNAIL_SIZE)
                            .get("log_level")
                            .get("ucm")
                            .get("mic_gain")
                            .get(CONTROL_KEY_ZOOM)
                            .exec();

    auto width = pipe_replies.get<OptionalString>(0);
    if(width){
        width_ = stoi(*width);
    }else{
        width_ = CP_DEF_WIDTH;
        redis_->set(CONTROL_KEY_WIDTH, to_string(width_));
    }

    console->critical(1);
        
    auto height = pipe_replies.get<OptionalString>(1);
    if(height){
        height_ = stoi(*height);
    }else{
        height_ = CP_DEF_HEIGHT;
        redis_->set(CONTROL_KEY_HEIGHT, to_string(height_)); 
    }

    console->critical(2);

    auto framerate = pipe_replies.get<OptionalString>(2);
    if(framerate){
        framerate_ = stoi(*framerate);
    }else{
        framerate_ = CP_DEF_FRAMERATE;
        redis_->set(CONTROL_KEY_FRAMERATE, to_string(framerate_)); 
    }

    console->critical(3);

    auto iso = pipe_replies.get<OptionalString>(3);
    if(iso){
        iso_ = stoi(*iso)/100;
    }else{
        iso_ = CP_DEF_ISO;
        redis_->set(CONTROL_KEY_ISO, to_string(iso_)); 
    }

    console->critical(4);

    auto shutter_speed = pipe_replies.get<OptionalString>(4);
    if(shutter_speed){
        shutter_speed_ = stoi(*shutter_speed);
    }else{
        shutter_speed_ = CP_DEF_SHUTTER;
        redis_->set(CONTROL_KEY_SHUTTER_SPEED, to_string(shutter_speed_)); 
    }

    console->critical(5);

    auto awb = pipe_replies.get<OptionalString>(5);
    if(awb){
        awb_ = stoi(*awb);
    }else{
        awb_ = CP_DEF_AWB;
        redis_->set(CONTROL_KEY_WB, to_string(awb_)); 
    }

    console->critical(6);
    
    auto compress = pipe_replies.get<OptionalString>(7);
    if(compress){
        compression_ = stoi(*compress);
    }else{
        compression_ = CP_DEF_COMPRESS;
        redis_->set(CONTROL_KEY_COMPRESSION, to_string(compression_));
    }

    console->critical(7);

    char *ptr = strtok(&(*pipe_replies.get<OptionalString>(6))[0], ",");
    uint8_t i = 0;
    while(ptr != NULL){
        cg_rb_[i] = (float)stof(ptr);
        i++;
        ptr = strtok(NULL, ",");  
    }

    console->critical(8);

    auto thumbnail = pipe_replies.get<OptionalString>(8);
    if(thumbnail){
        thumbnail_ = stoi(*thumbnail);
    }else{
        thumbnail_ = CP_DEF_THUMBNAIL;
        redis_->set(CONTROL_KEY_THUMBNAIL, to_string(thumbnail_));
    }

    console->critical(9);

    auto thumbnail_size = pipe_replies.get<OptionalString>(9);
    if(thumbnail_size){
        thumbnail_size_ = stoi(*thumbnail_size);
    }else{
        thumbnail_size_ = CP_DEF_THUMBNAIL_SIZE;
        redis_->set(CONTROL_KEY_THUMBNAIL, to_string(thumbnail_size_));
    }

    console->critical(10);

    auto log_level = pipe_replies.get<OptionalString>(10);
    if(log_level){
        spdlog::set_level(spdlog::level::from_str(*log_level));
    }

    console->critical(11);

    auto ucm = pipe_replies.get<OptionalString>(11);
    if(ucm){
        options_->ucm = *ucm;
    }

    console->critical(12);

    auto mic_gain = pipe_replies.get<OptionalString>(12);
    if(mic_gain){
        options_->mic_gain = stoi(*mic_gain);
        system(("amixer -c 1 sset 'Mic' " + *mic_gain + " > /dev/null 2>&1").c_str());
    }

    auto zoom_str = pipe_replies.get<OptionalString>(13);
    if (zoom_str)
            options_->SetZoom(std::stof(*zoom_str));
    else
            redis_->set(CONTROL_KEY_ZOOM, std::to_string(options_->Zoom()));

    // ── Frame-rate phase-lock config (write defaults if the keys are absent) ──
    if (auto v = redis_->get(CONTROL_KEY_PHASE_LOCK); v && !v->empty()) {
        try { phaseLockEnabled_.store(std::stoi(*v) != 0); } catch (...) {}
    } else {
        redis_->set(CONTROL_KEY_PHASE_LOCK, "0");
    }
    if (auto v = redis_->get(CONTROL_KEY_PLL_KP); v && !v->empty()) {
        try { pllKp_ = std::stod(*v); } catch (...) {}
    } else {
        redis_->set(CONTROL_KEY_PLL_KP, std::to_string(pllKp_));
    }
    if (auto v = redis_->get(CONTROL_KEY_PLL_KI); v && !v->empty()) {
        try { pllKi_ = std::stod(*v); } catch (...) {}
    } else {
        redis_->set(CONTROL_KEY_PLL_KI, std::to_string(pllKi_));
    }
    if (auto v = redis_->get(CONTROL_KEY_PLL_DEADBAND); v && !v->empty()) {
        try { pllDeadbandUs_ = std::stod(*v); } catch (...) {}
    } else {
        redis_->set(CONTROL_KEY_PLL_DEADBAND, std::to_string(pllDeadbandUs_));
    }

    console->critical(14);

    // std::unordered_map<std::string, std::string> m;
    // redis_->hgetall("rawCrop", std::inserter(m, m.begin()));

    // options_->rawCrop[0] = std::stoi(m["offset_y_start"]);
    // options_->rawCrop[1] = std::stoi(m["offset_y_end"]);
    // options_->rawCrop[2] = std::stoi(m["offset_x_start"]);
    // options_->rawCrop[3] = std::stoi(m["offset_x_end"]);

    console->critical(15);

    libcamera::ControlList cl;
    cl.set(libcamera::controls::rpi::StatsOutputEnable, true);
    app_->SetControls(cl);

    options_->thumbnail = thumbnail_;
    options_->thumbnailSize = thumbnail_size_;
    
    options_->compression = compression_;
    // options_->width = width_;
    // options_->height = height_;
    options_->framerate = framerate_;
    options_->gain = iso_;

    options_->awbEn = awb_;
    if(awb_)
        options_->awb_index = 5; // daylight
    else{
        options_->awb_gain_r = cg_rb_[0];
        options_->awb_gain_b = cg_rb_[1];
    }
    
    options_->denoise = "off";
    // options_->lores_width = options_->width >> 3;
    // options_->lores_height = options_->height >> 3;
    options_->mode_string = options_->mode.ToString();

}

/* ------------------------------------------------------------------ */
/*  CinePIController::process – v2 (real TOD timestamps)              */
/* ------------------------------------------------------------------ */
void CinePIController::process(CompletedRequestPtr &completed_request)
{
    CinePIFrameInfo info(completed_request->metadata);   // info.ts = ns since boot

    /* ────────────────────────────────────────────────────────────── */
    /*  0. Convert sensor ts → Unix-epoch ns                         */
    /*     Prefer libcamera::controls::FrameWallClock if available.  */
    /* ────────────────────────────────────────────────────────────── */
    uint64_t epoch_ns = 0;                               // ns since 1970-01-01
    if (auto wc = completed_request->metadata.get(controls::FrameWallClock); wc)
    {
        /* FrameWallClock is µs since epoch */
        epoch_ns = static_cast<uint64_t>(*wc) * 1'000ULL;
    }
    else
    {
        /* Derive once-per-run offset between MONOTONIC and REALTIME */
        using clk_sys  = std::chrono::system_clock;

        static bool     have_offset   = false;
        static uint64_t boot0_ns      = 0;               // first sensor ts (ns)
        static int64_t  epoch0_ns     = 0;               // wall clock at that moment

        if (!have_offset)
        {
            boot0_ns  = info.ts;
            epoch0_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                             clk_sys::now().time_since_epoch())
                             .count();
            have_offset = true;
        }
        epoch_ns = epoch0_ns + (info.ts - boot0_ns);
    }

    /* ────────────────────────────────────────────────────────────── */
    /*  1. One-off buffer-pool size announcement                     */
    /* ────────────────────────────────────────────────────────────── */
    if (!buffer_size_sent_ && app_->GetEncoder()->initialized())
    {
        redis_->set("buffer_size",
                    std::to_string(app_->GetEncoder()->maxRamBuffers()));
        buffer_size_sent_ = true;
    }

    /* ────────────────────────────────────────────────────────────── */
    /*  2. Publish live stats                                        */
    /* ────────────────────────────────────────────────────────────── */
    Json::Value data;
    data["framerate"]  = completed_request->framerate;
    data["colorTemp"]  = info.colorTemp;
    data["focus"]      = info.focus;
    data["frameCount"]    = app_->GetEncoder()->getFrameCount();
    data["tcFrameCount"]  = static_cast<Json::Int64>(app_->GetEncoder()->getTcFrameCount());
    data["droppedFrames"] = static_cast<Json::Int64>(app_->GetEncoder()->getDroppedFrames());
    data["writeFailures"]   = static_cast<Json::Int64>(app_->GetEncoder()->getWriteFailures());
    data["bufferSize"]      = app_->GetEncoder()->bufferSize();
    data["bufferSizeMax"]   = app_->GetEncoder()->bufferSizeMaxAndReset();
    data["framesInFlight"]  = static_cast<Json::Int64>(app_->GetEncoder()->getFramesInFlight());
    data["timestamp"]  = static_cast<Json::Int64>(epoch_ns);   // ← TOD ns
    redis_->publish(CHANNEL_STATS, data.toStyledString());

    /* cache per-camera timestamp key (TOD ns) */
    const char *ts_key = (options_->CamPort() == "cam1")
                           ? "timestamp_cam1"
                           : "timestamp_cam0";
    redis_->set(ts_key, std::to_string(epoch_ns));

    /* ────────────────────────────────────────────────────────────── */
    /*  3. Feed encoder with µs-since-epoch (for DNG time-code)      */
    /* ────────────────────────────────────────────────────────────── */
    app_->GetEncoder()->setWallClockTimestamp(epoch_ns / 1'000ULL); // µs

    /* ────────────────────────────────────────────────────────────── */
    /*  4. Keep last encoder BCD time-code in Redis                  */
    /* ────────────────────────────────────────────────────────────── */
    auto &tc_bcd = app_->GetEncoder()->originationTimeCode;

    int hh = ((tc_bcd[3] >> 4) & 0xF) * 10 + (tc_bcd[3] & 0xF);
    int mm = ((tc_bcd[2] >> 4) & 0xF) * 10 + (tc_bcd[2] & 0xF);
    int ss = ((tc_bcd[1] >> 4) & 0xF) * 10 + (tc_bcd[1] & 0xF);
    int ff = ((tc_bcd[0] >> 4) & 0xF) * 10 + (tc_bcd[0] & 0xF);

    std::ostringstream tc;
    tc << std::setw(2) << std::setfill('0') << hh << ':'
       << std::setw(2) << mm << ':'
       << std::setw(2) << ss << ':'
       << std::setw(2) << ff;

    const char *tc_key = (options_->CamPort() == "cam1") ? "tc_cam1" : "tc_cam0";
    redis_->set(tc_key, tc.str());

    /* ────────────────────────────────────────────────────────────── */
    /*  5. Closed-loop frame-rate phase lock (uses the monotonic       */
    /*     SensorTimestamp; no-op unless enabled + recording)          */
    /* ────────────────────────────────────────────────────────────── */
    updatePhaseLock(info.ts);
}


void CinePIController::mainThread(){
    // spdlog::set_level(spdlog::level::debug); 
    console->info("CinePIController Started!");
    auto sub = redis_->subscriber();

    using MessageHandler = std::function<void(const std::optional<std::string>&)>;

    std::unordered_map<std::string, MessageHandler> handlers = {
        { CONTROL_KEY_RAW_CROP, [this](const std::optional<std::string>& r) {
            std::unordered_map<std::string, std::string> m;
            redis_->hgetall("rawCrop", std::inserter(m, m.begin()));

            options_->rawCrop[0] = std::stoi(m["offset_y_start"]);
            options_->rawCrop[1] = std::stoi(m["offset_y_end"]);
            options_->rawCrop[2] = std::stoi(m["offset_x_start"]);
            options_->rawCrop[3] = std::stoi(m["offset_x_end"]);
        }},
        
        { CONTROL_KEY_RECORD, [this](const std::optional<std::string> &r) {
            if (!r) return;

            bool level = std::stoi(*r);

            if (level != is_recording_) {
                trigger_ = level ? +1 : -1;
            } else {
                trigger_ = 0;
            }

            is_recording_ = level;
        }},


        // { CONTROL_KEY_RECORD, [this](const std::optional<std::string> &r) {
        //     if (!r) return;                               // nothing to do

        //     bool level = std::stoi(*r);                   // 0 or 1

        //     /* ● rise --------------------------------------------------------- */
        //     if (level && !is_recording_)                  // 0 → 1
        //         trigger_ = +1;

        //     /* ● fall --------------------------------------------------------- */
        //     else if (!level && is_recording_)             // 1 → 0
        //         trigger_ = -1;

        //     /* ● duplicate write – ignore ------------------------------------ */
        //     else
        //         trigger_ = 0;

        //     /* finally remember the level we’re in */
        //     is_recording_ = level;
        // }},

            
        { CONTROL_KEY_ISO, [this](const std::optional<std::string>& r) {
            if(r) {
                iso_ = (unsigned int)(stoi(*r)/100.0);
                libcamera::ControlList cl;
                cl.set(libcamera::controls::AnalogueGain, iso_);
                app_->SetControls(cl);
            }
        }},
        { CONTROL_KEY_WB, [this](const std::optional<std::string>& r) {
            if(r) {
                awb_ = (unsigned int)(stoi(*r));
                libcamera::ControlList cl;
                cl.set(libcamera::controls::AwbEnable, awb_);
                app_->SetControls(cl);
            }
        }},
        { CONTROL_KEY_COLORGAINS, [this](const std::optional<std::string>& r) {
            if(r) {
                libcamera::ControlList cl;
                cl.set(libcamera::controls::AwbEnable, false);
                app_->SetControls(cl);
                std::string cg_rb_s = *r;
                char *ptr = strtok(&cg_rb_s[0], ",");
                uint8_t i = 0;
                while(ptr != NULL){
                    cg_rb_[i] = (float)stof(ptr);
                    i++;
                    ptr = strtok(NULL, ",");
                }
                cl.set(libcamera::controls::ColourGains, libcamera::Span<const float, 2>({ cg_rb_[0], cg_rb_[1] }));
                app_->SetControls(cl);
            }
        }},
        { CONTROL_KEY_SHUTTER_ANGLE, [this](const std::optional<std::string>& r) {
            if(r) {
                shutter_angle_ = stof(*r);
                shutter_speed_ = 1.0 / ((framerate_ * 360.0) / shutter_angle_);
                uint64_t shutterTime = shutter_speed_ * 1e+6;
                libcamera::ControlList cl;
                cl.set(libcamera::controls::ExposureTime, shutterTime);
                app_->SetControls(cl);
            }
        }},
        { CONTROL_KEY_SHUTTER_SPEED, [this](const std::optional<std::string>& r) {
            if(r) {
                shutter_speed_ = stof(*r);
                uint64_t shutterTime = shutter_speed_ * 1e+3;
                libcamera::ControlList cl;
                cl.set(libcamera::controls::ExposureTime, shutterTime);
                app_->SetControls(cl);
            }
        }},
        { CONTROL_KEY_WIDTH, [this](const std::optional<std::string>& r) {
            if(r) {
                width_ = (uint16_t)(stoi(*r));
                options_->width = width_;
                options_->mode.width = width_;
            }
        }},
        { CONTROL_KEY_HEIGHT, [this](const std::optional<std::string>& r) {
            if(r) {
                height_ = (uint16_t)(stoi(*r));
                options_->height = height_;
                options_->mode.height = height_;
            }
        }},
        { CONTROL_KEY_BIT_DEPTH, [this](const std::optional<std::string>& r) {
            if(r) {
                auto bitDepth = static_cast<unsigned int>(stoi(*r));
                if (bitDepth > 0)
                    options_->mode.bit_depth = bitDepth;
            }
        }},
        { CONTROL_KEY_PACKING, [this](const std::optional<std::string>& r) {
            if(r && !r->empty()) {
                char packing = static_cast<char>(std::toupper((*r)[0]));
                if (packing == 'P')
                    options_->mode.packed = true;
                else if (packing == 'U')
                    options_->mode.packed = false;
            }
        }},
        { CONTROL_KEY_MODE, [this](const std::optional<std::string>& r) {
            if(r && !r->empty()) {
                options_->mode_string = *r;
                options_->mode = Mode(*r);
                options_->width = options_->mode.width;
                options_->height = options_->mode.height;
                width_ = static_cast<uint16_t>(options_->mode.width);
                height_ = static_cast<uint16_t>(options_->mode.height);
            }
        }},
        { CONTROL_KEY_LORES_WIDTH, [this](const std::optional<std::string>& r) {
            if(r) {
                options_->lores_width = static_cast<unsigned int>(stoi(*r));
            }
        }},
        { CONTROL_KEY_LORES_HEIGHT, [this](const std::optional<std::string>& r) {
            if(r) {
                options_->lores_height = static_cast<unsigned int>(stoi(*r));
            }
        }},
        { CONTROL_KEY_COMPRESSION, [this](const std::optional<std::string>& r) {
            if(r) {
                compression_ = stoi(*r);
                options_->compression = compression_;
                cameraInit_ = true;
                buffer_size_sent_ = false;
            }
        }},
        { "lv_zoom", [this](const std::optional<std::string>& r) {
            if(r) {
                double zoomFactor = stod(*r);
                // Retrieve the maximum sensor area
                libcamera::Rectangle sensor_area = app_->GetCameras()[0]->controls().at(&controls::ScalerCrop).max().get<libcamera::Rectangle>();

                // Calculate the dimensions of the zoomed-in area based on the zoom factor
                int w = static_cast<int>(sensor_area.width / zoomFactor);
                int h = static_cast<int>(sensor_area.height / zoomFactor);

                // Calculate the top-left corner of the new crop area to keep it centered
                int x = (sensor_area.width - w) / 2;
                int y = (sensor_area.height - h) / 2;

                // Define the crop rectangle
                libcamera::Rectangle crop(x, y, w, h);

                // Translate the crop rectangle by the sensor area's top-left point to align with the global coordinate system, if necessary
                // This step might be redundant if the sensor_area's top-left is already considered (0,0) in your coordinate system.
                // crop.translateBy(sensor_area.topLeft());

                // Log and apply the crop
                LOG(2, "Using crop " << crop.toString());
                libcamera::ControlList cl;
                cl.set(controls::ScalerCrop, crop);
                app_->SetControls(cl);
            }
        }},
        { CONTROL_KEY_FRAMERATE, [this](const std::optional<std::string>& r) {
            if(r) {
                framerate_ = stof(*r);
                options_->framerate = framerate_;

                long int durationValues[2] = { static_cast<long int>(1000000.0 / framerate_),
                                            static_cast<long int>(1000000.0 / framerate_) };

                libcamera::Span<const long int, 2> durationRange(durationValues, 2);
                libcamera::ControlList cl;
                cl.set(libcamera::controls::FrameDurationLimits, durationRange);
                app_->SetControls(cl);
            }
        }},
        { CONTROL_KEY_PHASE_LOCK, [this](const std::optional<std::string>& r) {
            if(r && !r->empty()) {
                try { phaseLockEnabled_.store(std::stoi(*r) != 0); } catch (...) {}
                console->info("Frame-rate phase lock {}", phaseLockEnabled_.load() ? "ENABLED" : "disabled");
            }
        }},
        { CONTROL_KEY_PLL_KP, [this](const std::optional<std::string>& r) {
            if(r && !r->empty()) { try { pllKp_ = std::stod(*r); } catch (...) {} }
        }},
        { CONTROL_KEY_PLL_KI, [this](const std::optional<std::string>& r) {
            if(r && !r->empty()) { try { pllKi_ = std::stod(*r); } catch (...) {} }
        }},
        { CONTROL_KEY_PLL_DEADBAND, [this](const std::optional<std::string>& r) {
            if(r && !r->empty()) { try { pllDeadbandUs_ = std::stod(*r); } catch (...) {} }
        }},
        { CONTROL_KEY_CAMERAINIT, [this](const std::optional<std::string>& r) {
            cameraInit_ = true;
            buffer_size_sent_ = false;
        }},
        { CONTROL_KEY_THUMBNAIL, [this](const std::optional<std::string>& r) {
            if(r) {
                options_->thumbnail = stoi(*r);
            }
        }},
        { CONTROL_KEY_THUMBNAIL_SIZE, [this](const std::optional<std::string>& r) {
            if(r) {
                options_->thumbnailSize = stoi(*r);
                cameraInit_ = true;
            }
        }},
        { "log_level", [this](const std::optional<std::string>& r) {
            if(r) {
                spdlog::set_level(spdlog::level::from_str(*r));
            }
        }},
        { "mic_gain", [this](const std::optional<std::string>& r) {
            if(r) {
                options_->mic_gain = stoi(*r);
                system(("amixer -c 1 sset 'Mic' " + *r + " > /dev/null 2>&1").c_str());
            }
        }},
        { CONTROL_KEY_ZOOM, [this](const std::optional<std::string> &r)
        {
            if (!r)                // empty publish → ignore
                return;

            /* ─────────── 0. parse & deduplicate ─────────── */
            static double last_z = 1.0;                         // remember previous
            double z = std::clamp(std::stod(*r), 0.10, 25.0);   // keep sane range

            console->debug("ZOOM raw='{}'  parsed={:.3f}  prev={:.3f}",
                        *r, z, last_z);

            if (std::abs(z - last_z) < 1e-3) {                  // no real change
                console->debug("… duplicate – ignored");
                return;
            }
            last_z = z;
            options_->SetZoom(z);                               // store for CLI / save

            /* ─────────── 1. active sensor area ──────────── */
            libcamera::Rectangle sensor =
                app_->GetCameras()[0]->properties()
                    .get(libcamera::properties::ScalerCropMaximum)
                    .value_or(libcamera::Rectangle());          // fallback 0,0,0,0

            uint32_t Sw = sensor.width;                         // e.g. 3856
            uint32_t Sh = sensor.height;                        // e.g. 2180
            console->debug("Sensor active {}×{}  {}", Sw, Sh, sensor.toString());

            /* ─────────── 2. requested FoV (pixels) ──────── */
            float w_frac = 1.f / z;
            float h_frac = w_frac;
            float x_frac = (1.f - w_frac) / 2.f;
            float y_frac = x_frac;

            uint32_t x = static_cast<uint32_t>(x_frac * Sw) & ~1U;   // even align
            uint32_t y = static_cast<uint32_t>(y_frac * Sh) & ~1U;
            uint32_t w = static_cast<uint32_t>(w_frac * Sw) & ~1U;
            uint32_t h = static_cast<uint32_t>(h_frac * Sh) & ~1U;

            libcamera::Rectangle crop(x, y, w, h);
            console->debug("Crop rect {}", crop.toString());

            /* ─────────── 3. per-stream rectangles ───────── */
            const auto &streams = app_->GetCameras()[0]->streams();
            std::vector<libcamera::Rectangle> rects;
            rects.reserve(streams.size());

            for (size_t i = 0; i < streams.size(); ++i)
            {
                bool crop_this = (i == 0 || i == 2) ||               // preview & lo-res
                                (i == 1 && options_->ZoomRaw());    // RAW if flag
                rects.emplace_back(crop_this ? crop : sensor);
                console->debug("· stream {}  {}", i, rects.back().toString());
            }

            /* ─────────── 4. push to ISP ─────────────────── */
            libcamera::ControlList cl(app_->GetCameras()[0]->controls());
            cl.set(controls::ScalerCrop, crop);          //  ← no “rpi::”
            cl.set(controls::rpi::StatsOutputEnable, true);
            app_->SetControls(cl);


            console->info("⇢ live zoom now {:.2f}×", z);

        }},   // end CONTROL_KEY_ZOOM

    };

    sub.on_message([this, &handlers](std::string channel, std::string msg) {
        console->trace("{} from: {}", msg, channel);

        auto r = redis_->get(msg);

        auto it = handlers.find(msg);
        if (it != handlers.end()) {
            it->second(r);
        }
        
        redis_->bgsave();
    });

    sub.subscribe(CHANNEL_CONTROLS);

    while (true) {
        try {
            if(abortThread_){
                return;
            }
            sub.consume();
        } catch (const Error &err) {
            // Handle exceptions.
        }
    }
}

/* ------------------------------------------------------------------ */
/*  Closed-loop frame-rate phase lock                                  */
/*                                                                     */
/*  Integral controller on accumulated phase error, measured against   */
/*  the monotonic SensorTimestamp. Output is FrameDurationLimits; the   */
/*  integer-VBLANK quantisation downstream turns the smoothly-varying   */
/*  request into a first-order sigma-delta dither between two adjacent  */
/*  lines, so the *average* recorded cadence equals the operator's      */
/*  nominal fps exactly — beating the ~half-line (~125 ppm) floor of a  */
/*  single fixed correction factor. VBLANK-only: never touches HMAX, so */
/*  the 4K line-length failure mode cannot recur. Closed-loop, so it    */
/*  idles harmlessly if the sensor is already on target.                */
/* ------------------------------------------------------------------ */
void CinePIController::updatePhaseLock(int64_t sensorTsNs)
{
    /* Disabled or not recording: release the lock (the normal fps handler
     * owns FrameDurationLimits) and re-arm for the next take. */
    if (!phaseLockEnabled_.load() || !is_recording_) {
        pllActive_ = false;
        return;
    }

    if (!pllActive_) {
        /* Arm at the first recorded frame. Target the operator's NOMINAL fps
         * (fps_user) — the true intent — not the corrected hardware fps. */
        double target = 0.0;
        if (auto v = redis_->get("fps_user"); v && !v->empty()) {
            try { target = std::stod(*v); } catch (...) { target = 0.0; }
        }
        if (target <= 1.0)
            return;                          /* no valid target yet; retry next frame */
        pllTargetFps_  = target;
        pllBaseDurUs_  = 1.0e6 / target;     /* ideal period (us) */
        pllReqDurUs_   = pllBaseDurUs_;       /* start from nominal */
        pllIntegral_   = 0.0;
        pllT0Ns_       = sensorTsNs;
        pllFrameCount_ = 0;
        pllLastDurUs_  = -1;
        pllActive_     = true;
        return;                               /* this frame is the t0 datum */
    }

    pllFrameCount_++;
    const double targetPeriodNs = 1.0e9 / pllTargetFps_;
    const double idealNs   = static_cast<double>(pllFrameCount_) * targetPeriodNs;
    const double elapsedNs = static_cast<double>(sensorTsNs - pllT0Ns_);
    /* phaseErr > 0  → running slow / behind (need shorter frames)
     * phaseErr < 0  → running fast / ahead  (need longer frames)        */
    const double phaseErrUs = (elapsedNs - idealNs) * 1e-3;

    /* PI clock servo. The proportional term provides damping — pure integral
     * control here is an undamped oscillator (phase'' ∝ −phase). The small
     * integral removes the steady-state offset left by the VBLANK quantisation
     * bias. Held inside the deadband so a locked loop doesn't chatter
     * (anti-jitter guard #1). */
    const double kClampUs = 150.0;
    if (std::abs(phaseErrUs) > pllDeadbandUs_) {
        pllIntegral_ += phaseErrUs;
        const double iClamp = kClampUs / std::max(pllKi_, 1e-9);   /* anti-windup */
        pllIntegral_ = std::clamp(pllIntegral_, -iClamp, iClamp);
        pllReqDurUs_ = pllBaseDurUs_ - (pllKp_ * phaseErrUs + pllKi_ * pllIntegral_);
        /* Never wander far from nominal (anti-jitter guard #2 + safety net). */
        pllReqDurUs_ = std::clamp(pllReqDurUs_,
                                  pllBaseDurUs_ - kClampUs,
                                  pllBaseDurUs_ + kClampUs);
    }

    /* Push the duration only when the integer-us value changes: this is where
     * the integer-VBLANK quantisation produces the sigma-delta dither, and it
     * caps control traffic to actual line flips (anti-jitter guard #3). */
    const long durUs = std::lround(pllReqDurUs_);
    if (durUs != pllLastDurUs_) {
        pllLastDurUs_ = durUs;
        long int dv[2] = { durUs, durUs };
        libcamera::Span<const long int, 2> range(dv, 2);
        libcamera::ControlList cl;
        cl.set(libcamera::controls::FrameDurationLimits, range);
        app_->SetControls(cl);
    }

    /* Telemetry for the test harness. */
    redis_->set("pll_phase_err_us", std::to_string(std::lround(phaseErrUs)));
    redis_->set("pll_req_dur_us", std::to_string(durUs));
}
