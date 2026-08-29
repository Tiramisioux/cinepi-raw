#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <stdint.h>
#include <thread>
#include <chrono>
#include <atomic>
#include <cmath>
#include <algorithm>

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
#include "phase_lock_core.hpp"

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

        // Closed-loop frame-rate phase lock. Drives the recorded frame cadence
        // onto the operator's nominal fps (fps_user) by trimming
        // FrameDurationLimits frame-to-frame; the integer-VBLANK quantisation is
        // dithered out (first-order sigma-delta) so the *average* rate is exact.
        // VBLANK-only: never touches HMAX/line length. Runs continuously while
        // enabled (preview + recording) so the sensor is already locked when
        // recording starts — no head-of-take transient. Closed-loop, so it idles
        // harmlessly if the sensor is already on target (e.g. a future exact-rate
        // libcamera patch). No-op when disabled (default).
        //
        // Reference clock = the Pi wall clock (controls::FrameWallClock, passed in
        // as refTsNs), which is the clock the audio is captured against, so video
        // and audio share one timebase across all sensors. This instance is the
        // single ABSOLUTE disciplinarian: it runs on a single sensor (--sync off)
        // and on the dual-sensor master (--sync server), but suppresses itself on
        // the --sync client, where libcamera rpi.sync owns that sensor's VBLANK to
        // hold the relative A->B lock. Role is inferred from options_->sync, so the
        // same phase_lock setting works for single and dual with no per-camera key.
        // fpsUser is the operator's nominal fps ("fps_user"), fetched fresh each
        // frame as part of process()'s pipelined Redis batch so an fps change
        // still re-arms the lock without a dedicated per-frame GET.
        void updatePhaseLock(int64_t refTsNs, const OptionalString &fpsUser);

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

        // Call after every StartCamera(): a camera (re)start resets the ISP's
        // ScalerCrop to full frame, so the zoom handler's dedup baseline must
        // follow (1.0 = no zoom applied). Otherwise re-publishing the
        // operator's pre-switch zoom compares equal to the stale baseline and
        // is dropped, and the crop can never be reprogrammed — the same
        // defect class as cinemate's shutter_a set_value dedup after a mode
        // switch (cinemate fix/mode-switch-control-reapply).
        void resetZoomDedup() { last_zoom_.store(1.0); }

    // ── Dual-sensor record gate ─────────────────────────────────────────────
    // Which sensor(s) record the current take is published by cinemate in the
    // Redis key `record_cams` (tokens: "cam0", "cam1", "cam0+cam1", "both").
    // Each cinepi-raw instance records only when its own --cam-port is selected;
    // the others keep previewing (and stay genlocked) but open no clip folder.
    // Absent/empty key → both record, i.e. the pre-gate behaviour, so a
    // single-sensor rig and any older cinemate are unaffected.
    bool recordCamsIncludesSelf()
    {
        const std::string port = options_->CamPort();
        if (port.empty())
            return true;                         // unlabelled single camera
        OptionalString v;
        try { v = redis_->get("record_cams"); }
        catch (const Error &) {
            /* Redis unreachable in the window between a record edge and this
             * gate read: fall back to the legacy both-record default rather
             * than letting the exception kill the process (or a sensor sit
             * the take out). Recording too much beats recording nothing.   */
            return true;
        }
        if (!v || v->empty())
            return true;                         // legacy: no gate published
        const std::string &sel = *v;
        if (sel == "both")
            return true;
        // "cam0" is never a substring of "cam1", so a plain find is safe and
        // matches "cam0+cam1" for either port.
        return sel.find(port) != std::string::npos;
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
                if (!recordCamsIncludesSelf())      // this sensor sits the take out
                {
                    baseline_flag_ = 1;             // stay in sync with shared flag
                    return 0;
                }
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
        /* The flag rides process()'s pipelined batch (same frame, no extra
         * round trip). Fall back to a direct GET only if that batch didn't
         * run or failed; if Redis is unreachable, hold the current record
         * state this frame rather than misreading the outage as a stop.     */
        OptionalString v;
        if (rec_flag_prefetch_valid_)
            v = rec_flag_prefetch_;
        else
        {
            try { v = redis_->get("is_recording"); }
            catch (const Error &) { return 0; }
        }
        int rec_flag = 0;
        if (v && !v->empty())
            rec_flag = std::stoi(*v);               // 0 or 1

        /* ── first invocation: establish baseline, possibly join late. ────── */
        static bool first_call = true;
        if (first_call)
        {
            baseline_flag_ = rec_flag;
            first_call     = false;

            if (rec_flag && !is_recording_ && recordCamsIncludesSelf())  // already rolling → join
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
                if (!recordCamsIncludesSelf())      // not this sensor's take
                    return 0;                       // baseline already synced above
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

        /* ── 3.  Policy B: join-only live record gate. ─────────────────────
         * While a take is already rolling (is_recording high, baseline synced),
         * a sensor that sat the start out may JOIN back-to-back when the operator
         * switches the on-camera preview to a view that includes it — cinemate
         * republishes record_cams live, and we pick it up here on the next frame.
         * The +1 lands in cinepi_raw.cpp's start path, opening a fresh clip folder
         * stamped with THIS frame's wall-clock, so the joining sensor's clip is
         * seamless from the switch instant. Join-only: we never drop a sensor
         * mid-take here — leaving a take is only ever via the shared stop edge.
         * The `!is_recording_` guard short-circuits before the Redis read on the
         * already-recording sensor, so this adds no per-frame cost there.        */
        if (rec_flag && baseline_flag_ && !is_recording_ && recordCamsIncludesSelf())
        {
            console->info("Gate opened mid-take → joining back-to-back.");
            setRecording(true);
            is_recording_ = true;
            return +1;
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

        // ── Frame-rate phase lock (sigma-delta VBLANK dither) ───────────────
        // Control law is the pure cinepi::phaseLockStep() in phase_lock_core.hpp
        // (unit-tested); this class only owns the runtime enable, the live-tunable
        // gains, and the per-frame servo state.
        std::atomic_bool        phaseLockEnabled_{false}; // runtime enable (redis fps_phase_lock)
        // Defaults tuned on imx585 mode0 @25fps (Pi-verified: +1066 -> -9 ppm,
        // ~2-3 line dither). Runtime-tunable via Redis (pll_kp/pll_ki/pll_deadband_us).
        cinepi::PhaseLockParams pllParams_{};             // kp / ki / deadbandUs / clampUs
        cinepi::PhaseLockState  pllState_{};              // per-frame servo state

        int baseline_flag_{0};          // remembers last seen is_recording level

        // "is_recording" fetched by process()'s pipelined batch each frame,
        // consumed by triggerRec() immediately after (both capture thread).
        // valid_ is false whenever the batch didn't run or failed this frame.
        OptionalString rec_flag_prefetch_;
        bool rec_flag_prefetch_valid_ = false;

        // Outage-edge logging for the per-frame pipeline (capture thread
        // only): warn once when it starts failing, info once on recovery,
        // debug in between — a PERSISTENT failure must be visible at the
        // default log level without spamming at frame rate.
        bool frame_pipe_failing_ = false;

        // Debounce for the RDB snapshot after control changes (subscriber
        // thread only). bgsave_done_ forces the very first control message
        // to save: a value-initialised steady_clock time_point is the BOOT
        // epoch on Linux, so a bare 60 s comparison would silently skip
        // every save in the first minute of uptime — exactly the boot
        // window where cinemate seeds the launch-config keys.
        bool bgsave_done_ = false;
        std::chrono::steady_clock::time_point last_bgsave_{};

        // Last zoom actually applied to the ISP — the CONTROL_KEY_ZOOM
        // handler's dedup baseline. Written by the Redis subscriber thread,
        // reset from the main loop via resetZoomDedup(), hence atomic.
        std::atomic<double> last_zoom_{1.0};

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
