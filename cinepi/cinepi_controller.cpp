#include "cinepi_controller.hpp"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <ctime>
#include <iomanip>
#include <sstream>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>

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
// thumbnail_size is a right-shift applied to the lores plane inside
// dng_save() (0 = full lores resolution, 1 = half, 2 = quarter, ...). 0 is
// the default: it is what every size/cost figure in the C9 plan and
// GATES.md assumes (the 1272x720 lores frame, unscaled). The redis value
// found resident pre-feature (PI-008: thumbnail_size=50) predates any
// consumer of this key and is not a default worth preserving.
#define CP_DEF_THUMBNAIL_SIZE 0

/* ── imx585 ClearHDR live knobs ─────────────────────────────────────────────
 * The knobs are custom V4L2 controls on the sensor subdev; their IDs mirror
 * imx585.c (Tiramisioux imx585-v4l2-driver, 6.12.y). They are plain sensor
 * register controls, so they apply live while streaming. Only the ClearHDR
 * enable itself (wide_dynamic_range, set via --hdr sensor at launch) changes
 * the sensor's mode list and therefore needs a process restart.
 */
static constexpr uint32_t IMX585_CID_BASE           = V4L2_CID_USER_BASE + 0x2000;
static constexpr uint32_t IMX585_CID_HDR_DATASEL_TH = IMX585_CID_BASE + 0; /* u16[2], 0..4095 */
static constexpr uint32_t IMX585_CID_HDR_DATASEL_BK = IMX585_CID_BASE + 1; /* menu, 0..8 */
static constexpr uint32_t IMX585_CID_HDR_GAIN_ADDER = IMX585_CID_BASE + 5; /* menu, 0..5 */

/* Probe /dev/v4l-subdevN for the sensor that exposes the ClearHDR controls. */
static int open_imx585_subdev()
{
    for (int i = 0; i < 16; i++) {
        std::string dev = "/dev/v4l-subdev" + std::to_string(i);
        int fd = open(dev.c_str(), O_RDWR, 0);
        if (fd < 0)
            continue;
        struct v4l2_query_ext_ctrl q = {};
        q.id = IMX585_CID_HDR_DATASEL_TH;
        if (!ioctl(fd, VIDIOC_QUERY_EXT_CTRL, &q))
            return fd;
        close(fd);
    }
    return -1;
}

/* Set one ClearHDR control: a u16 pair when `pair` is non-null, else `value`. */
static bool set_imx585_hdr_ctrl(uint32_t id, int32_t value, const uint16_t *pair)
{
    int fd = open_imx585_subdev();
    if (fd < 0)
        return false;

    uint16_t buf[2];
    struct v4l2_ext_control c = {};
    c.id = id;
    if (pair) {
        buf[0] = pair[0];
        buf[1] = pair[1];
        c.size = sizeof(buf);
        c.p_u16 = buf;
    } else {
        c.value = value;
    }

    struct v4l2_ext_controls ctrls = {};
    ctrls.which = V4L2_CTRL_WHICH_CUR_VAL;
    ctrls.count = 1;
    ctrls.controls = &c;
    bool ok = !ioctl(fd, VIDIOC_S_EXT_CTRLS, &ctrls);
    close(fd);
    return ok;
}

/* Parse one hdr_threshold_low/high Redis value, clamped 0..4095; missing or
 * invalid -> 0. IMX585_CID_HDR_DATASEL_TH is a hardware u16[2] pair, so both
 * sides must always be written together even though they are two Redis keys. */
static uint16_t parse_hdr_threshold(const std::optional<std::string>& v)
{
    if (!v || v->empty())
        return 0;
    try {
        return (uint16_t)std::clamp(std::stoi(*v), 0, 4095);
    } catch (...) {
        return 0;
    }
}

/* Build the IMX585_CID_HDR_DATASEL_TH u16[2] from the two Redis keys, in the
 * order the driver writes them (imx585.c:1532-1534, write site :1472-1474):
 *
 *     th[0] -> EXP_TH_H (0x36D0)   high-gain saturation cutoff
 *     th[1] -> EXP_TH_L (0x36D4)   high-gain "low" cutoff
 *
 * so hdr_threshold_HIGH belongs in th[0] and hdr_threshold_LOW in th[1].
 * That matches what both keys have always been documented to mean --
 * hdr_threshold_high is "the raw level above which the sensor reads pure
 * low-gain", which is precisely the HG saturation cutoff.
 *
 * These were passed the other way round until 2026-08-30: every call site
 * built { low, high }, so hdr_threshold_low landed in EXP_TH_H. Confirmed on
 * hardware by writing low=3000/high=500 and reading the registers back --
 * 0x36D0 came back 3000 and 0x36D4 came back 500.
 *
 * Returns false, writing nothing, when the pair would violate the sensor's
 * EXP_TH_H >= EXP_TH_L constraint. The driver is explicit that the spec marks
 * EXP_TH_H < EXP_TH_L as "Prohibited -- the sensor enters an invalid state and
 * only outputs the BLC pedestal", and escaping that state needs a large light
 * transient at the sensor, so refusing the write is much cheaper than making
 * it. Note this is reachable from the documented usage: setting only one of
 * the two keys leaves the other parsing as 0.
 */
static bool build_hdr_threshold_pair(const std::optional<std::string>& low_v,
                                     const std::optional<std::string>& high_v,
                                     uint16_t pair[2])
{
    const uint16_t low = parse_hdr_threshold(low_v);
    const uint16_t high = parse_hdr_threshold(high_v);

    if (high < low)
        return false;

    pair[0] = high;   /* EXP_TH_H */
    pair[1] = low;    /* EXP_TH_L */
    return true;
}

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
        redis_->set(CONTROL_KEY_THUMBNAIL_SIZE, to_string(thumbnail_size_));
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

    // ── imx585 ClearHDR knobs: apply any persisted values at startup, so a
    //    profile selected before this process launched (CineMate `set hdr
    //    profile`) takes effect without an extra pub/sub round-trip.
    //
    //    Gate on ClearHDR being ON. These are HDR-only sensor controls, and the
    //    gain adder writes EXP_GAIN (0x3081). The driver's common_regs reset
    //    EXP_GAIN to 0 for every mode and only common_clearHDR_mode raises it to
    //    +12 dB, so re-applying a persisted hdr_gain_adder here in an SDR launch
    //    would override that reset and boost SDR by up to +29 dB (magenta shadow
    //    noise). When HDR is off we leave the sensor's normal-mode defaults be.
    if (options_->hdr == "sensor" || options_->hdr == "auto") {
        auto low_v = redis_->get(CONTROL_KEY_HDR_THRESHOLD_LOW);
        auto high_v = redis_->get(CONTROL_KEY_HDR_THRESHOLD_HIGH);
        if ((low_v && !low_v->empty()) || (high_v && !high_v->empty())) {
            uint16_t pair[2];
            if (!build_hdr_threshold_pair(low_v, high_v, pair))
                console->warn("ClearHDR threshold restore refused: hdr_threshold_high ({}) is "
                              "below hdr_threshold_low ({}). EXP_TH_H < EXP_TH_L is a prohibited "
                              "sensor state that outputs only the black-level pedestal. Leaving "
                              "the driver's own pair in place. Set both keys, or neither.",
                              parse_hdr_threshold(high_v), parse_hdr_threshold(low_v));
            else if (set_imx585_hdr_ctrl(IMX585_CID_HDR_DATASEL_TH, 0, pair))
                console->info("ClearHDR data-selection threshold restored to "
                              "EXP_TH_H={}, EXP_TH_L={}", pair[0], pair[1]);
            else
                console->warn("ClearHDR threshold restore: no imx585 ClearHDR subdev control found");
        }
        if (auto v = redis_->get(CONTROL_KEY_HDR_BLEND); v && !v->empty()) {
            try {
                int val = std::clamp(std::stoi(*v), 0, 8);
                if (set_imx585_hdr_ctrl(IMX585_CID_HDR_DATASEL_BK, val, nullptr))
                    console->info("ClearHDR blending mode restored to {}", val);
                else
                    console->warn("ClearHDR blend restore: no imx585 ClearHDR subdev control found");
            } catch (...) {}
        }
        if (auto v = redis_->get(CONTROL_KEY_HDR_GAIN_ADDER); v && !v->empty()) {
            try {
                int val = std::clamp(std::stoi(*v), 0, 5);
                if (set_imx585_hdr_ctrl(IMX585_CID_HDR_GAIN_ADDER, val, nullptr))
                    console->info("ClearHDR gain adder restored to menu index {}", val);
                else
                    console->warn("ClearHDR gain adder restore: no imx585 ClearHDR subdev control found");
            } catch (...) {}
        }
    }

    // ── Frame-rate phase-lock config (write defaults if the keys are absent) ──
    if (auto v = redis_->get(CONTROL_KEY_PHASE_LOCK); v && !v->empty()) {
        try { phaseLockEnabled_.store(std::stoi(*v) != 0); } catch (...) {}
    } else {
        redis_->set(CONTROL_KEY_PHASE_LOCK, "0");
    }
    if (auto v = redis_->get(CONTROL_KEY_PLL_KP); v && !v->empty()) {
        try { pllParams_.kp = std::stod(*v); } catch (...) {}
    } else {
        redis_->set(CONTROL_KEY_PLL_KP, std::to_string(pllParams_.kp));
    }
    if (auto v = redis_->get(CONTROL_KEY_PLL_KI); v && !v->empty()) {
        try { pllParams_.ki = std::stod(*v); } catch (...) {}
    } else {
        redis_->set(CONTROL_KEY_PLL_KI, std::to_string(pllParams_.ki));
    }
    if (auto v = redis_->get(CONTROL_KEY_PLL_DEADBAND); v && !v->empty()) {
        try { pllParams_.deadbandUs = std::stod(*v); } catch (...) {}
    } else {
        redis_->set(CONTROL_KEY_PLL_DEADBAND, std::to_string(pllParams_.deadbandUs));
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
        /*
         * FrameWallClock is unavailable on this build, so map the sensor
         * timestamp (info.ts, CLOCK_BOOTTIME ns) to Unix-epoch ns ourselves.
         *
         * The CLOCK_BOOTTIME→CLOCK_REALTIME offset is a system-wide constant,
         * so sample both clocks back-to-back ONCE and cache their difference.
         * Do NOT anchor epoch time to a frame's arrival: that folds this
         * process's frame-handling latency into the offset, and because cam0
         * (sync server) and cam1 (sync client) handle their first frame at
         * different instants they end up with different constants — the
         * "one frame ahead/behind" seen across hardware-synced sensors. A pure
         * clock offset is identical in every process, so synced frames (which
         * share the same sensor timestamp) get identical time-codes.
         */
        static bool    have_offset      = false;
        static int64_t boot_to_epoch_ns = 0;             // REALTIME − BOOTTIME

        if (!have_offset)
        {
            struct timespec bt {}, rt {};
            clock_gettime(CLOCK_BOOTTIME, &bt);
            clock_gettime(CLOCK_REALTIME, &rt);
            int64_t boot_now = static_cast<int64_t>(bt.tv_sec) * 1'000'000'000LL + bt.tv_nsec;
            int64_t real_now = static_cast<int64_t>(rt.tv_sec) * 1'000'000'000LL + rt.tv_nsec;
            boot_to_epoch_ns = real_now - boot_now;
            have_offset = true;
        }
        epoch_ns = static_cast<uint64_t>(static_cast<int64_t>(info.ts) + boot_to_epoch_ns);
    }

    /* ────────────────────────────────────────────────────────────── */
    /*  1. One-off buffer-pool size announcement (queued in step 5)  */
    /* ────────────────────────────────────────────────────────────── */
    const bool announce_buffer_size =
        !buffer_size_sent_ && app_->GetEncoder()->initialized();

    /* ────────────────────────────────────────────────────────────── */
    /*  2. Build live stats                                          */
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
    data["cameraPort"] = options_->CamPort();                  // cam0 / cam1 — disambiguates the shared cp_stats channel

    /* per-camera timestamp key (TOD ns) */
    const char *ts_key = (options_->CamPort() == "cam1")
                           ? "timestamp_cam1"
                           : "timestamp_cam0";

    /* ────────────────────────────────────────────────────────────── */
    /*  3. Feed encoder with µs-since-epoch (for DNG time-code)      */
    /* ────────────────────────────────────────────────────────────── */
    app_->GetEncoder()->setWallClockTimestamp(epoch_ns / 1'000ULL); // µs

    /* ────────────────────────────────────────────────────────────── */
    /*  4. Last encoder BCD time-code (written to Redis in step 5)   */
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

    /* ────────────────────────────────────────────────────────────── */
    /*  5. One pipelined Redis round trip for the whole frame:        */
    /*     stats publish + timestamp/tc SETs (+ one-off buffer_size)  */
    /*     + the fps_user and is_recording GETs that used to be       */
    /*     separate blocking calls here and in triggerRec().          */
    /*     This runs on the capture thread, so collapsing 4-6 round   */
    /*     trips into 1 keeps Redis stalls off the frame path.        */
    /* ────────────────────────────────────────────────────────────── */
    OptionalString fps_user;
    rec_flag_prefetch_valid_ = false;
    try
    {
        auto pipe = redis_->pipeline(false);   // borrow the pooled connection
        pipe.publish(CHANNEL_STATS, data.toStyledString())
            .set(ts_key, std::to_string(epoch_ns))
            .set(tc_key, tc.str());
        if (announce_buffer_size)
            pipe.set("buffer_size",
                     std::to_string(app_->GetEncoder()->maxRamBuffers()));
        pipe.get("fps_user")
            .get("is_recording");
        auto replies = pipe.exec();

        const std::size_t base = announce_buffer_size ? 4 : 3;
        fps_user           = replies.get<OptionalString>(base);
        rec_flag_prefetch_ = replies.get<OptionalString>(base + 1);
        rec_flag_prefetch_valid_ = true;
        if (announce_buffer_size)
            buffer_size_sent_ = true;
        if (frame_pipe_failing_)
        {
            console->info("per-frame Redis pipeline recovered");
            frame_pipe_failing_ = false;
        }
    }
    catch (const Error &err)
    {
        /* Redis briefly unavailable: keep capturing — stats/timecode resume
         * on the next frame, and triggerRec() holds the current record state
         * (see rec_flag_prefetch_valid_). Warn once per outage so a
         * PERSISTENT failure (e.g. a WRONGTYPE reply on every GET while the
         * rest of Redis works) stays visible at the default log level;
         * repeats stay at debug to avoid frame-rate log spam.              */
        if (!frame_pipe_failing_)
        {
            console->warn("per-frame Redis pipeline failed — live stats and the "
                          "record safety-net are degraded until it recovers: {}",
                          err.what());
            frame_pipe_failing_ = true;
        }
        else
            console->debug("per-frame Redis pipeline still failing: {}", err.what());
    }

    /* ────────────────────────────────────────────────────────────── */
    /*  6. Closed-loop frame-rate phase lock. References the Pi wall    */
    /*     clock (FrameWallClock = the audio clock, computed above).    */
    /*     No-op unless enabled; suppressed on the --sync client.       */
    /* ────────────────────────────────────────────────────────────── */
    updatePhaseLock(static_cast<int64_t>(epoch_ns), fps_user);
}


void CinePIController::mainThread(){
    // spdlog::set_level(spdlog::level::debug); 
    console->info("CinePIController Started!");
    auto sub = redis_->subscriber();

    using MessageHandler = std::function<void(const std::optional<std::string>&)>;

    /* Both threshold keys write the same u16[2] sensor control, so whichever
     * one changed, the other is read alongside it and the pair goes out
     * together. Shared so the EXP_TH_H >= EXP_TH_L guard cannot drift between
     * the two handlers. */
    auto apply_hdr_thresholds = [this](const std::optional<std::string>& low_v,
                                       const std::optional<std::string>& high_v) {
        uint16_t pair[2];
        if (!build_hdr_threshold_pair(low_v, high_v, pair)) {
            console->warn("ClearHDR threshold rejected: hdr_threshold_high ({}) is below "
                          "hdr_threshold_low ({}). EXP_TH_H < EXP_TH_L is a prohibited sensor "
                          "state that outputs only the black-level pedestal, and clearing it "
                          "needs a light transient at the sensor. Sensor left unchanged.",
                          parse_hdr_threshold(high_v), parse_hdr_threshold(low_v));
            return;
        }
        if (set_imx585_hdr_ctrl(IMX585_CID_HDR_DATASEL_TH, 0, pair))
            console->info("ClearHDR data-selection threshold set to EXP_TH_H={}, EXP_TH_L={}",
                          pair[0], pair[1]);
        else
            console->warn("ClearHDR threshold: no imx585 ClearHDR subdev control found");
    };

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
            if(r && !r->empty()) { try { pllParams_.kp = std::stod(*r); } catch (...) {} }
        }},
        { CONTROL_KEY_PLL_KI, [this](const std::optional<std::string>& r) {
            if(r && !r->empty()) { try { pllParams_.ki = std::stod(*r); } catch (...) {} }
        }},
        { CONTROL_KEY_PLL_DEADBAND, [this](const std::optional<std::string>& r) {
            if(r && !r->empty()) { try { pllParams_.deadbandUs = std::stod(*r); } catch (...) {} }
        }},
        { CONTROL_KEY_HDR_THRESHOLD_LOW, [apply_hdr_thresholds, this](const std::optional<std::string>& r) {
            if(r && !r->empty())
                apply_hdr_thresholds(r, redis_->get(CONTROL_KEY_HDR_THRESHOLD_HIGH));
        }},
        { CONTROL_KEY_HDR_THRESHOLD_HIGH, [apply_hdr_thresholds, this](const std::optional<std::string>& r) {
            if(r && !r->empty())
                apply_hdr_thresholds(redis_->get(CONTROL_KEY_HDR_THRESHOLD_LOW), r);
        }},
        { CONTROL_KEY_HDR_BLEND, [this](const std::optional<std::string>& r) {
            if(r && !r->empty()) {
                try {
                    int v = std::clamp(std::stoi(*r), 0, 8);
                    if (set_imx585_hdr_ctrl(IMX585_CID_HDR_DATASEL_BK, v, nullptr))
                        console->info("ClearHDR blending mode set to {}", v);
                    else
                        console->warn("ClearHDR blend: no imx585 ClearHDR subdev control found");
                } catch (...) {}
            }
        }},
        { CONTROL_KEY_HDR_GAIN_ADDER, [this](const std::optional<std::string>& r) {
            if(r && !r->empty()) {
                try {
                    int v = std::clamp(std::stoi(*r), 0, 5);
                    if (set_imx585_hdr_ctrl(IMX585_CID_HDR_GAIN_ADDER, v, nullptr))
                        console->info("ClearHDR gain adder set to menu index {}", v);
                    else
                        console->warn("ClearHDR gain adder: no imx585 ClearHDR subdev control found");
                } catch (...) {}
            }
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
            /* last_zoom_ is the last zoom APPLIED to the ISP, not the last
             * value seen — a camera restart resets ScalerCrop, so the main
             * loop clears this baseline (resetZoomDedup) after StartCamera. */
            double last_z = last_zoom_.load();
            double z = std::clamp(std::stod(*r), 0.10, 25.0);   // keep sane range

            console->debug("ZOOM raw='{}'  parsed={:.3f}  prev={:.3f}",
                        *r, z, last_z);

            if (std::abs(z - last_z) < 1e-3) {                  // no real change
                console->debug("… duplicate – ignored");
                return;
            }
            last_zoom_.store(z);
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

        /* Debounced RDB snapshot. This used to run unconditionally, forking
         * redis-server and rewriting dump.rdb on the SD card for EVERY control
         * message — a rotary-encoder burst meant a fork storm while recording.
         * One save per minute keeps operator-seeded keys (the launch-config
         * contract) persistent across power cuts with a ≤60 s window; the
         * distro redis.conf save policy backstops the trailing edge of a
         * burst. bgsave_done_ guarantees the first message saves — see the
         * member note: a zero time_point is the boot epoch, not "long ago". */
        auto now = std::chrono::steady_clock::now();
        if (!bgsave_done_ || now - last_bgsave_ >= std::chrono::seconds(60)) {
            try {
                redis_->bgsave();
                last_bgsave_ = now;
                bgsave_done_ = true;
            } catch (const Error &err) {
                console->debug("bgsave failed: {}", err.what());
            }
        }
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
/*  the Pi wall clock (FrameWallClock, de-jittered, = the audio clock). */
/*  Output is FrameDurationLimits; the                                  */
/*  integer-VBLANK quantisation downstream turns the smoothly-varying   */
/*  request into a first-order sigma-delta dither between two adjacent  */
/*  lines, so the *average* recorded cadence equals the operator's      */
/*  nominal fps exactly — beating the ~half-line (~125 ppm) floor of a  */
/*  single fixed correction factor. VBLANK-only: never touches HMAX, so */
/*  the 4K line-length failure mode cannot recur. Closed-loop, so it    */
/*  idles harmlessly if the sensor is already on target.                */
/*                                                                     */
/*  Role: this is the single absolute disciplinarian. It runs on a lone */
/*  sensor (--sync off) and on the dual master (--sync server), but is   */
/*  suppressed on the --sync client, where rpi.sync owns the VBLANK to   */
/*  hold the relative A->B genlock. Inferred from options_->sync.        */
/* ------------------------------------------------------------------ */
void CinePIController::updatePhaseLock(int64_t refTsNs, const OptionalString &fpsUser)
{
    /* I/O lives here; the control law is the pure phaseLockStep() in
     * phase_lock_core.hpp (unit-tested in tests/phase_lock_core_test.cpp).
     *
     * Target = operator's NOMINAL fps (fps_user), read each frame so an fps change
     * re-arms the lock — fetched by process()'s pipelined batch and passed in, so
     * the freshness is unchanged but the dedicated per-frame GET is gone. The
     * reference clock is the Pi wall clock (FrameWallClock), passed in as refTsNs
     * by process(). The --sync client role suppresses the lock so libcamera
     * rpi.sync owns that sensor's VBLANK on a genlock rig. */
    double target = pllState_.targetFps;
    if (fpsUser && !fpsUser->empty()) {
        try { target = std::stod(*fpsUser); } catch (...) {}
    }

    const bool roleClient = (options_->sync == 2);
    cinepi::PhaseLockResult res = cinepi::phaseLockStep(
        pllState_, pllParams_, phaseLockEnabled_.load(), roleClient,
        is_recording_, target, refTsNs);

    /* Push FrameDurationLimits only on an integer-us change — this is where the
     * integer-VBLANK quantisation downstream becomes the sigma-delta dither. */
    if (res.setControls) {
        long int dv[2] = { res.durUs, res.durUs };
        libcamera::Span<const long int, 2> range(dv, 2);
        libcamera::ControlList cl;
        cl.set(libcamera::controls::FrameDurationLimits, range);
        app_->SetControls(cl);
    }

    /* Telemetry for the test harness (only when the servo actually ran) —
     * both SETs share one pipelined round trip on the capture thread. */
    if (res.servoRan) {
        try {
            auto pipe = redis_->pipeline(false);
            pipe.set("pll_phase_err_us", std::to_string(std::lround(res.phaseErrUs)))
                .set("pll_req_dur_us", std::to_string(res.durUs));
            pipe.exec();
        } catch (const Error &err) {
            console->debug("phase-lock telemetry write failed: {}", err.what());
        }
    }
}
