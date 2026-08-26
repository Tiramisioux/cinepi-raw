#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <time.h>
#include <stdint.h>

#include "core/logging.hpp"

#include <chrono>
#include <iostream>
#include <stdexcept>

#include <mutex>
#include <queue>
#include <thread>

#define CONTROL_TRIGGER_RECORD "rec"

#define CONTROL_KEY_RECORD "is_recording"
#define CONTROL_KEY_ISO "iso"
#define CONTROL_KEY_WB "awb"
#define CONTROL_KEY_COLORGAINS "cg_rb"
#define CONTROL_KEY_SHUTTER_ANGLE "shutter_a"
#define CONTROL_KEY_SHUTTER_SPEED "shutter_s"

#define CONTROL_KEY_FRAMERATE "fps"
#define CONTROL_KEY_WIDTH "width"
#define CONTROL_KEY_HEIGHT "height"
#define CONTROL_KEY_BIT_DEPTH "bit_depth"
#define CONTROL_KEY_PACKING "packing"
#define CONTROL_KEY_MODE "mode"
#define CONTROL_KEY_LORES_WIDTH "lores_width"
#define CONTROL_KEY_LORES_HEIGHT "lores_height"
#define CONTROL_KEY_COMPRESSION "compress"
#define CONTROL_KEY_THUMBNAIL "thumbnail"
#define CONTROL_KEY_THUMBNAIL_SIZE "thumbnail_size"

#define CONTROL_KEY_RAW_CROP "raw_crop"

#define CONTROL_KEY_CAMERAINIT "cam_init"

#define CONTROL_KEY_ZOOM "zoom"

// ── imx585 ClearHDR live knobs (custom V4L2 sensor controls; applied while
//    streaming — only wide_dynamic_range itself needs a process restart) ──
#define CONTROL_KEY_HDR_THRESHOLD_LOW "hdr_threshold_low"   // 0..4095 — HG→LG data-selection threshold, low
#define CONTROL_KEY_HDR_THRESHOLD_HIGH "hdr_threshold_high" // 0..4095 — HG→LG data-selection threshold, high
#define CONTROL_KEY_HDR_BLEND "hdr_blend"           // 0..8 — HG/LG blending mode (driver menu index)
#define CONTROL_KEY_HDR_GAIN_ADDER "hdr_gain_adder" // 0..5 — LG gain adder menu index (default 2 = +12 dB)
#define CONTROL_KEY_HCG "hcg"                       // 0/1 — SDR high conversion gain (imx585; the driver force-disables it in ClearHDR)

// Closed-loop frame-rate phase lock (sigma-delta VBLANK dither). Off by default.
#define CONTROL_KEY_PHASE_LOCK "fps_phase_lock"   // 0/1 enable
#define CONTROL_KEY_PLL_KP "pll_kp"               // proportional gain (damping)
#define CONTROL_KEY_PLL_KI "pll_ki"               // integral gain (removes steady offset)
#define CONTROL_KEY_PLL_DEADBAND "pll_deadband_us" // phase-error deadband (us)

class CinePIState
{
    public:
        CinePIState() : is_recording_(false), clip_number_(0), still_number_(0) {};
        ~CinePIState() {};

        void setRecording(bool state){
            is_recording_ = state;
        }

        bool isRecording(){
            return is_recording_;
        }

        unsigned int getClipNumber(){
            return clip_number_;
        }

        void advanceClipNumber(){
            clip_number_++;
        }

    protected:
        float framerate_;
        bool is_recording_;
        unsigned int iso_;
        unsigned int awb_;
        float shutter_speed_;
        float shutter_angle_;
        unsigned int color_temp_;
        float cg_rb_[2];

        uint16_t width_;
        uint16_t height_;
        int mode_;
        int compression_;

        int thumbnail_;
        int thumbnail_size_;

        unsigned int clip_number_;
        unsigned int still_number_;
};
