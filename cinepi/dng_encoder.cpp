/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * Based on mjpeg_encoder.cpp, modifications by Csaba Nagy, Will Whang & Patrik Eriksson
 *
 * dng_encoder.cpp - dng video encoder.
 */

 #include <chrono>                  // encodeThread timing
 #include <iostream>                // debugging, std::cerr
 #include <libcamera/control_ids.h> // metadata.get(controls::...)
 #include <libcamera/formats.h>     // libcamera::formats::
 #include <cmath>
#include <cstring>
#include <cerrno>
 #include <stdexcept>
 #include <iomanip>
 
#include <sstream>
#include <algorithm>
#include <array>
#include <fstream>
#include <regex>
#include <utility>
#include <sched.h>
#include <sys/resource.h>
 
 #include "dng_encoder.hpp"        
 #include "utils.hpp"               
 #include "ifd_builder.hpp"         
 
 #include <sys/mman.h>              
 #include <sys/types.h>
 #include <sys/stat.h>
 #include <fcntl.h>
 #include <unistd.h>

#include "core/still_options.hpp"

#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/time.h>


namespace fs = std::filesystem;

#define ONE_MB 1048576
#define BLOCK_SIZE 4096 

using namespace libcamera;

static char CFA_RGGB[4] = { 0, 1, 1, 2 };
static char CFA_GRBG[4] = { 1, 0, 2, 1 };
static char CFA_BGGR[4] = { 2, 1, 1, 0 };
static char CFA_GBRG[4] = { 1, 2, 0, 1 };

// TIFF photometric interpretation values
constexpr uint16_t PHOTOMETRIC_MINISBLACK = 1;
constexpr uint16_t PHOTOMETRIC_RGB        = 2;
constexpr uint16_t PHOTOMETRIC_CFA        = 32803; // DNG CFA

// TIFF compression
constexpr uint16_t COMPRESSION_NONE = 1;

// Sample format
constexpr uint16_t SAMPLEFORMAT_UINT = 1;


struct BayerFormat
{
	char const *name;
	int bits;
	char const *order;
	bool packed;
	bool compressed;
};

static const std::map<PixelFormat, BayerFormat> bayer_formats =
{
	{ formats::SRGGB10_CSI2P, { "RGGB-10", 10, CFA_RGGB, true, false } },
	{ formats::SGRBG10_CSI2P, { "GRBG-10", 10, CFA_GRBG, true, false } },
	{ formats::SBGGR10_CSI2P, { "BGGR-10", 10, CFA_BGGR, true, false } },
	{ formats::SGBRG10_CSI2P, { "GBRG-10", 10, CFA_GBRG, true, false } },

	{ formats::SRGGB10, { "RGGB-10", 10, CFA_RGGB, false, false } },
	{ formats::SGRBG10, { "GRBG-10", 10, CFA_GRBG, false, false } },
	{ formats::SBGGR10, { "BGGR-10", 10, CFA_BGGR, false, false } },
	{ formats::SGBRG10, { "GBRG-10", 10, CFA_GBRG, false, false } },

	{ formats::SRGGB12_CSI2P, { "RGGB-12", 12, CFA_RGGB, true, false } },
	{ formats::SGRBG12_CSI2P, { "GRBG-12", 12, CFA_GRBG, true, false } },
	{ formats::SBGGR12_CSI2P, { "BGGR-12", 12, CFA_BGGR, true, false } },
	{ formats::SGBRG12_CSI2P, { "GBRG-12", 12, CFA_GBRG, true, false } },

	{ formats::SRGGB12, { "RGGB-12", 12, CFA_RGGB, false, false } },
	{ formats::SGRBG12, { "GRBG-12", 12, CFA_GRBG, false, false } },
	{ formats::SBGGR12, { "BGGR-12", 12, CFA_BGGR, false, false } },
	{ formats::SGBRG12, { "GBRG-12", 12, CFA_GBRG, false, false } },

	{ formats::SRGGB16, { "RGGB-16", 16, CFA_RGGB, false, false } },
	{ formats::SGRBG16, { "GRBG-16", 16, CFA_GRBG, false, false } },
	{ formats::SBGGR16, { "BGGR-16", 16, CFA_BGGR, false, false } },
	{ formats::SGBRG16, { "GBRG-16", 16, CFA_GBRG, false, false } },

	{ formats::R10_CSI2P, { "BGGR-10", 10, CFA_BGGR, true, false } },
	{ formats::R10, { "BGGR-10", 10, CFA_BGGR, false, false } },
	// Currently not in the main libcamera branch
        { formats::R12_CSI2P, { "BGGR-12", 12, CFA_BGGR, true, false } },
	{ formats::R12, { "BGGR-12", 12, CFA_BGGR, false, false } },
    { formats::R16, { "BGGR-16", 16, CFA_BGGR, false, false } },

	/* PiSP compressed formats. */
	{ formats::RGGB_PISP_COMP1, { "RGGB-16-PISP", 16, CFA_RGGB, false, true } },
	{ formats::GRBG_PISP_COMP1, { "GRBG-16-PISP", 16, CFA_GRBG, false, true } },
	{ formats::GBRG_PISP_COMP1, { "GBRG-16-PISP", 16, CFA_GBRG, false, true } },
	{ formats::BGGR_PISP_COMP1, { "BGGR-16-PISP", 16, CFA_BGGR, false, true } },
};

// mono_formats should stay *empty* for these raw formats
static const std::map<PixelFormat,int> mono_formats = {
    /* leave genuine true-mono formats here, e.g. GREY8 if you ever use it */
};


bool mono_ = false;   // add as a private member of DngEncoder

void DngEncoder::setWallClockTimestamp(uint64_t us)
{
    wallclock_ts_us_ = us;
}

void pack_8bit_data(const uint16_t* src, uint8_t* dst, size_t num_pixels) {
    for (size_t i = 0; i < num_pixels; i++) {
        dst[0] = src[i];
    }
}

/* The live 10-bit packer moved to cinepi/dng_pack.hpp as pack_row_10bit() so it
 * can be unit-tested (tests/dng_pack_test.cpp). */

void pack_12bit_data(const uint16_t* src, uint8_t* dst, size_t num_pixels) {
    // 3 bytes can hold 2 12-bit pixels
    // Every iteration of the loop processes 2 pixels (24 bits)
    for (size_t i = 0; i < num_pixels; i += 2) {
        dst[0] = src[i] >> 4;                               // Highest 8 bits of pixel 1
        dst[1] = (src[i] << 4) | (src[i + 1] >> 8);         // Lowest 4 bits of pixel 1 + highest 4 bits of pixel 2
        dst[2] = src[i + 1];                                // Lowest 8 bits of pixel 2

        dst += 3; // Move to the next 3 bytes
    }
}

void pack_14bit_data(const uint16_t* src, uint8_t* dst, size_t num_pixels) {
    // Ensure that we have enough pixels (must be a multiple of 4)
    if (num_pixels % 4 != 0) return;

    // Every iteration of the loop processes 4 pixels (56 bits)
    for (size_t i = 0; i < num_pixels; i += 4) {
        dst[0] = (src[i] >> 6);                                  // Highest 8 bits of pixel 1
        dst[1] = ((src[i] & 0x3F) << 2) | (src[i + 1] >> 12);    // Lowest 6 bits of pixel 1 and highest 2 bits of pixel 2
        dst[2] = (src[i + 1] >> 4) & 0xFF;                       // Middle 8 bits of pixel 2
        dst[3] = ((src[i + 1] & 0xF) << 4) | (src[i + 2] >> 10); // Lowest 4 bits of pixel 2 and highest 4 bits of pixel 3
        dst[4] = (src[i + 2] >> 2) & 0xFF;                       // Middle 8 bits of pixel 3
        dst[5] = ((src[i + 2] & 0x3) << 6) | (src[i + 3] >> 8);  // Lowest 2 bits of pixel 3 and highest 6 bits of pixel 4
        dst[6] = src[i + 3] & 0xFF;                              // Lowest 8 bits of pixel 4

        dst += 7; // Move to the next 7 bytes
    }
}

#include <vector>   // one new header

/* Pure DNG pixel pack/unpack helpers (pack_row_12bit, pack_row_16_to_12bit,
 * pack_row_10bit, unpack_csi2_raw12/raw10, PiSP COMP1 decode) live in a header so
 * they can be unit-tested without libcamera. See tests/dng_pack_test.cpp. */
#include "cinepi/dng_pack.hpp"


struct Matrix
{
Matrix(float m0, float m1, float m2,
       float m3, float m4, float m5,
       float m6, float m7, float m8)
    {
        m[0] = m0, m[1] = m1, m[2] = m2;
        m[3] = m3, m[4] = m4, m[5] = m5;
        m[6] = m6, m[7] = m7, m[8] = m8;
    }
    Matrix(float diag0, float diag1, float diag2) : Matrix(diag0, 0, 0, 0, diag1, 0, 0, 0, diag2) {}
    Matrix() {}
    float m[9];
    Matrix T() const
    {
        return Matrix(m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8]);
    }
    Matrix C() const
    {
        return Matrix(m[4] * m[8] - m[5] * m[7], -(m[3] * m[8] - m[5] * m[6]), m[3] * m[7] - m[4] * m[6],
                      -(m[1] * m[8] - m[2] * m[7]), m[0] * m[8] - m[2] * m[6], -(m[0] * m[7] - m[1] * m[6]),
                      m[1] * m[5] - m[2] * m[4], -(m[0] * m[5] - m[2] * m[3]), m[0] * m[4] - m[1] * m[3]);
    }
    Matrix Adj() const { return C().T(); }
    float Det() const
    {
        return (m[0] * (m[4] * m[8] - m[5] * m[7]) -
                m[1] * (m[3] * m[8] - m[5] * m[6]) +
                m[2] * (m[3] * m[7] - m[4] * m[6]));
    }
    Matrix Inv() const { return Adj() * (1.0 / Det()); }
    Matrix operator*(Matrix const &other) const
    {
        Matrix result;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                result.m[i * 3 + j] =
                    m[i * 3] * other.m[j] + m[i * 3 + 1] * other.m[3 + j] + m[i * 3 + 2] * other.m[6 + j];
        return result;
    }
    Matrix operator*(float const &f) const
    {
        Matrix result;
        for (int i = 0; i < 9; i++)
            result.m[i] = m[i] * f;
        return result;
    }
};

#include <pthread.h>
#include <cerrno>

DngEncoder::DngEncoder(RawOptions const *options)
    : Encoder(options), // Assuming you're calling the base class constructor
      write12bit_(false),
      encoder_initialized_(false),
      encodeCheck_(false),
      resetCount_(false),
      index_(0),
      frames_(0),
      options_(options)
{
    console = spdlog::get("dng_encoder");
    if (!console)
        console = spdlog::stdout_color_mt("dng_encoder");

    if (options_)
    {
        encode_worker_count_ = std::max<uint32_t>(1, options_->encode_workers);
        disk_worker_count_   = std::max<uint32_t>(1, options_->disk_workers);
        encode_affinity_     = options_->encode_affinity;
        disk_affinity_       = options_->disk_affinity;
        encode_nice_         = options_->encode_nice;
        disk_nice_           = options_->disk_nice;
    }
    else
    {
        encode_worker_count_ = 2;
        disk_worker_count_   = 8;
    }

    encode_threads_.reserve(encode_worker_count_);
    for (size_t i = 0; i < encode_worker_count_; ++i)
        encode_threads_.emplace_back(&DngEncoder::encodeThread, this, static_cast<int>(i));

    disk_threads_.reserve(disk_worker_count_);
    for (size_t i = 0; i < disk_worker_count_; ++i)
        disk_threads_.emplace_back(&DngEncoder::diskThread, this, static_cast<int>(i));

    console->info("DngEncoder started with {} encode worker(s) and {} disk worker(s)",
                  encode_worker_count_,
                  disk_worker_count_);
}

DngEncoder::~DngEncoder()
{
    stopThreads();
    drainPooledBuffers();
    console->info("DngEncoder stopped!");
}

void DngEncoder::stopThreads()
{
    bool encode_was_running = !stop_encode_.exchange(true, std::memory_order_acq_rel);
    bool disk_was_running   = !stop_disk_.exchange(true, std::memory_order_acq_rel);

    if (encode_was_running)
        encode_cond_var_.notify_all();
    if (disk_was_running)
        disk_cond_var_.notify_all();

    for (auto &thread : encode_threads_)
    {
        if (thread.joinable())
            thread.join();
    }
    for (auto &thread : disk_threads_)
    {
        if (thread.joinable())
            thread.join();
    }

    encode_threads_.clear();
    disk_threads_.clear();
}

void DngEncoder::configureThreadContext(const std::string &baseName,
                                        size_t index,
                                        size_t total,
                                        const std::optional<std::vector<int>> &affinity,
                                        const std::optional<int> &nice_value)
{
    std::string name = baseName + std::to_string(index);
    if (name.size() >= 16)
        name.resize(15);

    if (pthread_setname_np(pthread_self(), name.c_str()) != 0)
    {
        if (console)
            console->warn("{}: failed to set thread name: {}", name, strerror(errno));
    }

    std::vector<int> cpus_to_pin;
    if (affinity && !affinity->empty())
    {
        /* ── Audio-core isolation guard ───────────────────────────────────
         * cinepi-audio-capture pins itself to the last online core at
         * SCHED_FIFO priority 80 (see cinepi_audio_capture.cpp). DNG encode
         * and disk workers must never share that core, or the USB-audio
         * capture loop stalls and the WAV loses sync. Strip the audio core
         * from any requested set here so a storage recorder profile can never
         * place workers on it (a stale ext4 "2-3" on a 4-core Pi would).
         * No-op when the audio core was not requested. Skipped on <=2 cores,
         * where there is nothing to isolate. */
        std::vector<int> safe_affinity;
        const long online_cpus = sysconf(_SC_NPROCESSORS_ONLN);
        const int audio_core = (online_cpus > 2) ? static_cast<int>(online_cpus) - 1 : -1;
        for (int cpu : *affinity)
            if (cpu != audio_core)
                safe_affinity.push_back(cpu);
        if (safe_affinity.empty())   /* only the audio core was requested */
            for (int cpu = 0; cpu < static_cast<int>(online_cpus) - 1; ++cpu)
                safe_affinity.push_back(cpu);
        if (console && audio_core >= 0 && safe_affinity.size() != affinity->size())
            console->info("{}: excluded audio core {} from requested affinity",
                          name, audio_core);

        if (safe_affinity.size() >= total)
        {
            cpus_to_pin.push_back(safe_affinity[index % safe_affinity.size()]);
        }
        else
        {
            cpus_to_pin.assign(safe_affinity.begin(), safe_affinity.end());
        }

        cpu_set_t cpu_mask;
        CPU_ZERO(&cpu_mask);
        for (int cpu : cpus_to_pin)
            CPU_SET(cpu, &cpu_mask);

        if (pthread_setaffinity_np(pthread_self(), sizeof(cpu_mask), &cpu_mask) != 0)
        {
            if (console)
                console->warn("{}: failed to set CPU affinity: {}", name, strerror(errno));
        }
        else
        {
            std::ostringstream oss;
            for (size_t i = 0; i < cpus_to_pin.size(); ++i)
            {
                if (i)
                    oss << ',';
                oss << cpus_to_pin[i];
            }
            if (console)
                console->debug("{} pinned to CPU(s) {}", name, oss.str());
        }
    }

    if (nice_value)
    {
        if (setpriority(PRIO_PROCESS, 0, *nice_value) != 0)
        {
            if (console)
                console->warn("{}: failed to set nice level {}: {}", name, *nice_value, strerror(errno));
        }
        else if (console)
        {
            console->debug("{} nice level set to {}", name, *nice_value);
        }
    }
}

void DngEncoder::EncodeBuffer(int fd, size_t size, void *mem, StreamInfo const &info, int64_t timestamp_us)
{
    {
        std::lock_guard<std::mutex> lock(encode_mutex_);
    }
    {
        input_done_callback_(nullptr);
        output_ready_callback_(mem, size, timestamp_us, true);
    }       
}

void DngEncoder::EncodeBuffer2(int fd, size_t size, void *mem, StreamInfo const &info, size_t losize, void *lomem, StreamInfo const &loinfo, int64_t timestamp_us, CompletedRequest::ControlList const &metadata)
{
    {
        if (stop_encode_.load(std::memory_order_acquire))
            return;

        std::lock_guard<std::mutex> lock(encode_mutex_);
        EncodeItem item = {
            mem,
            size,
            info,
            lomem,
            losize,
            loinfo,
            metadata,
            timestamp_us,
            index_++,
            options_ ? options_->folder : std::string()
        };
        encode_queue_.push(item);
        frames_in_flight_.fetch_add(1, std::memory_order_relaxed);
        encode_cond_var_.notify_one();
    }
}

/* ────────────────────────────────────────────────────────────── */
/*  Helper – encode float → TIFF RATIONAL                         */
/* ────────────────────────────────────────────────────────────── */
static inline void encode_rational_array(const float *src,
                                         int          count,
                                         int32_t     *dst,
                                         int32_t      scale = 10000)
{
    for (int i = 0; i < count; ++i)
    {
        float v = src[i];
        if (!std::isfinite(v) || scale == 0)
        {
            dst[2*i]   = 0;
            dst[2*i+1] = 1;
        }
        else
        {
            dst[2*i]   = static_cast<int32_t>(std::round(v * scale));
            dst[2*i+1] = scale;
        }
    }
}

/* ────────────────────────────────────────────────────────────── */
/*  Helper – power-of-two align                                   */
/* ────────────────────────────────────────────────────────────── */
static inline uint32_t align_up(uint32_t v, uint32_t a)
{
    return (v + a - 1) & ~(a - 1);
}

/* ────────────────────────────────────────────────────────────── */
/*  Public: setup_encoder                                         */
/* ────────────────────────────────────────────────────────────── */
void DngEncoder::setup_encoder(const libcamera::StreamConfiguration &cfg,
                               const libcamera::StreamConfiguration &lo_cfg,
                               const CompletedRequest::ControlList  &metadata)
{
    /* ──  Pixel format – Bayer only  ──────────────────────────── */
    auto it = bayer_formats.find(cfg.pixelFormat);
    if (it == bayer_formats.end())
        throw std::runtime_error("Unsupported Bayer format " + cfg.pixelFormat.toString());

    const BayerFormat &bf          = it->second;
    raw_packed_in_                 = bf.packed;
    raw_compressed_in_             = bf.compressed;
    dng_info.bits                  = bf.bits;
    dng_info.white                 = (1u << bf.bits) - 1u;
    dng_info.photometric           = PHOTOMETRIC_CFA;
    dng_info.samples_per_pixel     = 1;
    std::memcpy(dng_info.bayer_order, bf.order, 4);
    dng_info.black_level_repeat_dim[0] = 2;
    dng_info.black_level_repeat_dim[1] = 2;

    /* On PiSP every raw stream arrives as a 16-bit container. SDR sensor modes
     * (<=12 significant bits, MSB-aligned) always pack down to 12-bit DNGs;
     * dropping the 4 padding LSBs is lossless there. A true 16-bit sensor mode
     * (imx585 ClearHDR SRGGB16, sensor mode bit depth 16) carries real data in
     * all 16 bits, so it keeps full depth. The bit depth comes from the snapshot
     * taken at reconfigure time, not options_->mode (which the redis thread
     * mutates). BlackLevel is computed per-frame in dng_save() from
     * SensorBlackLevels, scaled to the output white level.
     *
     * (--keep16 used to force full depth for the SDR case too; it was removed
     * because the 4 bits it preserved are padding, so it only ever bought a
     * ~33% larger file carrying the same information. --log-encode is the one
     * output-depth control now.) */
    write12bit_ = (bf.bits == 16) && sensor_mode_bit_depth_ != 16;

    if (write12bit_) {
        dng_info.bits  = 12;
        dng_info.white = (1u << 12) - 1u;
    }

    /* ──  CineMate Log  ───────────────────────────────────────
     * Resolve the curve here, the one place that knows both depths, and let
     * everything downstream key off log_lut_. dng_info.bits/white feed four
     * things at once — the dng_save() branch chain, tag 258, tag 0xC61D and
     * the black-level rescale — so they are the only two values the log path
     * has to override:
     *
     *   bits  = the STORED code depth (12), so tag 258 and the buffer sizing
     *           below describe the packed log codes that are really written.
     *   white = the LINEAR (table-output) white level, because under a
     *           LinearizationTable the level tags live in the table's OUTPUT
     *           domain, not the code domain. That is also exactly what turns
     *           the `* dng_info.white / 65535.f` black rescale at the bottom of
     *           dng_save() into the identity, which is the required bypass:
     *           SensorBlackLevels is already in that same 16-bit linear domain.
     *
     * Scope: 16-bit ClearHDR and 12-bit SDR sensor modes, to 12- or 10-bit
     * codes. Anything else falls through to the normal linear path rather than
     * emitting an untested file. */
    log_lut_       = nullptr;
    log_src_shift_ = 0;
    if (options_ && options_->log_encode)
    {
        std::string err;
        const LogLut *lut  = nullptr;
        const int target   = options_->log_encode;
        const int src_bits = static_cast<int>(sensor_mode_bit_depth_);
        unsigned shift     = 0;

        /* Which curve is keyed on the SENSOR mode depth, not bf.bits. On PiSP
         * every raw stream arrives in a 16-bit container, so a 12-bit mode is
         * SRGGB16 carrying its 12 significant bits MSB-aligned — measured on
         * device: stride is 2 B/px and the recorded BlackLevel 200 lands at code
         * 200, not at 12. bf describes the row LAYOUT, sensor_mode_bit_depth_
         * the DOMAIN the curve was fitted to, and those two disagree exactly
         * there. Getting it wrong is silent: a 16-bit sample indexed into a
         * 4096-entry forward table clamps, pinning the frame at white. */
        if (src_bits != 16 && src_bits != 12)
            err = "needs a 16- or 12-bit sensor mode (this one is " +
                  std::to_string(src_bits) + "-bit)";
        /* Which row shapes: only the ones the loop in dng_save() can normalise
         * to right-justified src_bits, and nothing else. */
        else if (bf.compressed && src_bits != 16)
            err = "COMP1 rows decode to a 16-bit domain, not " + std::to_string(src_bits);
        else if (bf.packed && (bf.bits != 12 || src_bits != 12))
            err = "no CSI2 unpacker for packed " + std::to_string(bf.bits) +
                  "-bit rows in a " + std::to_string(src_bits) + "-bit domain";
        else if (!bf.compressed && !bf.packed && bf.bits != src_bits)
        {
            /* The one legal mismatch is the PiSP SDR container above. */
            if (bf.bits == 16 && src_bits == 12)
                shift = 4;
            else
                err = std::to_string(bf.bits) + "-bit rows cannot carry a " +
                      std::to_string(src_bits) + "-bit domain";
        }

        if (err.empty())
            lut = get_log_lut(src_bits, target, err);

        /* The loaded spec, not the flag, decides the real depths. load_log_lut()
         * picks the file by NAME and verifies its table round-trips, but never
         * cross-checks the depths the file declares against the pair it was
         * asked for — and the row path below only has packers for 12 and 10. So
         * refuse a spec that disagrees, instead of encoding against one domain
         * and labelling the file with another. */
        if (lut)
        {
            const LogLutParams &lp = lut->params();
            if (lp.target_bits != target)
                err = "spec targets " + std::to_string(lp.target_bits) +
                      " bit, expected " + std::to_string(target);
            else if (lp.source_bits != src_bits)
                err = "spec sources " + std::to_string(lp.source_bits) +
                      " bit, expected " + std::to_string(src_bits);
            else if (lp.target_bits != 12 && lp.target_bits != 10)
                err = "no packer for " + std::to_string(lp.target_bits) + "-bit codes";

            if (!err.empty())
                lut = nullptr;
        }

        /* A spec is keyed on the DEPTH PAIR alone — log_lut_spec_filename() builds
         * the name from <src>to<tgt> and nothing else — but its black level is
         * per-sensor. cinemate_log_12to10 assumes 200 (imx585/imx283 3200 in the
         * 16-bit domain); every sensor has a 12-bit mode and imx477's black is
         * 256, imx296's 240. Handing it that spec would build the toe around 200
         * while SensorBlackLevels writes 256 into the file's own BlackLevel tag —
         * curve and tag disagreeing by 56 LSB, exactly where the footroom codes
         * live. Refuse instead, like every other scope guard here.
         *
         * Tolerance is one footroom code (foot/F): below that the toe is
         * misplaced by less than the quantisation it controls, which is also
         * enough slack for per-channel jitter in the reported levels. Making spec
         * selection genuinely sensor-aware is a separate pass — it breaks the
         * "rebuilt table must equal the spec's shipped table" invariant. */
        if (lut)
        {
            const LogLutParams &lp = lut->params();
            auto bl = metadata.get(controls::SensorBlackLevels);
            if (!bl || bl->size() < 4)
            {
                /* Without the reported levels this check cannot run at all, and
                 * the entire reason it exists is that a wrong black is SILENT —
                 * the toe lands in the wrong place and nothing downstream says
                 * so. Refuse, like every other guard here, rather than encode
                 * against a curve nothing has confirmed belongs to this sensor.
                 * Every Pi sensor reports these; a mode that somehow does not
                 * records linear instead of recording something subtly wrong. */
                err = "no SensorBlackLevels reported, cannot verify the curve's black level";
                lut = nullptr;
            }
            else
            {
                int worst_seen = lp.black_level;
                float worst_off = 0.f;
                for (size_t i = 0; i < 4; ++i)
                {
                    const int scaled = log_lut_scale_black(lp, (*bl)[i]);
                    const float off  = std::fabs(static_cast<float>(scaled - lp.black_level));
                    if (off > worst_off) { worst_off = off; worst_seen = scaled; }
                }
                if (worst_off > log_lut_black_tolerance(lp))
                {
                    err = "spec assumes black " + std::to_string(lp.black_level) +
                          " but this sensor reports " + std::to_string(worst_seen) +
                          " at " + std::to_string(lp.source_bits) + " bit";
                    lut = nullptr;
                }
            }
        }

        if (lut)
        {
            log_lut_       = lut;
            log_src_shift_ = shift;
            write12bit_    = false;       /* the log path owns the row conversion */
            dng_info.bits  = lut->params().target_bits;
            dng_info.white = lut->params().white_level;
        }
        else
            console->warn("CineMate Log off for this mode: {}", err);
    }

    /* ──  White-balance gains & CCM  ──────────────────────────── */
    std::fill(std::begin(dng_info.NEUTRAL), std::end(dng_info.NEUTRAL), 1.f);
    Matrix wb(1, 1, 1);

    if (auto cg = metadata.get(controls::ColourGains); cg)
    {
        dng_info.NEUTRAL[0] = 1.f / (*cg)[0];
        dng_info.NEUTRAL[2] = 1.f / (*cg)[1];
        wb = Matrix((*cg)[0], 1, (*cg)[1]);
    }

    Matrix ccm(1.90255, -0.77478, -0.12777,
               -0.31338,  1.88197, -0.56858,
               -0.06001, -0.61785,  1.67786);

    if (auto m = metadata.get(controls::ColourCorrectionMatrix); m)
        ccm = Matrix((*m)[0], (*m)[1], (*m)[2],
                     (*m)[3], (*m)[4], (*m)[5],
                     (*m)[6], (*m)[7], (*m)[8]);

    Matrix rgb2xyz(0.4124564, 0.3575761, 0.1804375,
                   0.2126729, 0.7151522, 0.0721750,
                   0.0193339, 0.1191920, 0.9503041);

    Matrix cam_xyz = (rgb2xyz * ccm * wb).Inv();
    std::copy(std::begin(cam_xyz.m), std::end(cam_xyz.m), dng_info.CAM_XYZ);

    /* ──  Thumbnail (mono 8-bit Y-plane)  ─────────────────────── */
    dng_info.thumbWidth           = lo_cfg.size.width;
    dng_info.thumbHeight          = lo_cfg.size.height;
    dng_info.thumbSamplesPerPixel = 1;
    dng_info.thumbBitsPerSample   = 8;
    dng_info.thumbPhotometric     = PHOTOMETRIC_MINISBLACK;   /* = 1 */

    /* ──  Buffer sizing  ──────────────────────────────────────── */
    /* 64 KB covers the IFD and its out-of-line payloads; log adds an 8 KB
     * LinearizationTable on top of that. Worth stating explicitly: an overflow
     * here does not crash, write_pod() throws and the frame is dropped without
     * a pixel of evidence. */
    const uint32_t frame = ((cfg.size.width * dng_info.bits + 7) / 8) * cfg.size.height;
    const uint32_t tail_slack =
        64 * 1024 + (log_lut_ ? static_cast<uint32_t>(log_lut_->inverse_size() * sizeof(uint16_t)) : 0u);
    dng_info.buffer_size = align_up(frame + tail_slack, ONE_MB);

    /* ──  Static strings & misc  ──────────────────────────────── */
    dng_info.make       = "Raspberry Pi";
    /* Model = the attached sensor (libcamera properties::Model, falling back to
     * the camera id), captured into options_->model in cinepi_raw.cpp. So the DNG
     * carries the real sensor name even when cinepi-raw is run without Cinemate. */
    dng_info.model      = (options_ && !options_->model.empty())
                              ? options_->model
                              : std::string("unknown sensor");
    dng_info.software   = "Libcamera;cinepi-raw";
    dng_info.ucm        = options_->ucm.value_or("cinepi");
    dng_info.serial     = getHwId();
    dng_info.compression = COMPRESSION_NONE;

    make_tag_ = dng_info.make;
    make_tag_.push_back('\0');
    model_tag_ = dng_info.model;
    model_tag_.push_back('\0');
    software_tag_ = dng_info.software;
    software_tag_.push_back('\0');
    ucm_tag_ = dng_info.ucm;
    ucm_tag_.push_back('\0');

    /* ────────────────────────────────────────────────────────── */
    /*  Dynamic RAM limit: 90 % of current MemAvailable          */
    /* ────────────────────────────────────────────────────────── */
    std::ifstream meminfo("/proc/meminfo");
    std::string   line;
    size_t memAvail = 0;
    while (std::getline(meminfo, line))
        if (line.rfind("MemAvailable:", 0) == 0)
        {
            std::istringstream iss(line);
            std::string key, kB;
            iss >> key >> memAvail >> kB;
            memAvail *= 1024;          /* kB → bytes */
            break;
        }

    constexpr double RAM_FRACTION = 0.90;   /* use 90 % of what’s free *now*  */
    max_ram_buffers_ = std::max<size_t>(1,         /* never zero */
                        static_cast<size_t>(RAM_FRACTION * memAvail)
                        / dng_info.buffer_size);

    console->info("RAM pool: up to {} frames  (~{} MB)",
                  max_ram_buffers_, (max_ram_buffers_ * dng_info.buffer_size) >> 20);


    encoder_initialized_ = true;
    console->info("Encoder configured – {}×{} {}-bit, buffer {} MB",
                  cfg.size.width, cfg.size.height,
                  dng_info.bits, dng_info.buffer_size / ONE_MB);
    console->info("DNG writer: raw-only frames; embedded lores thumbnail disabled");
    if (log_lut_)
        console->info("{}  LinearizationTable {} entries", log_lut_->params().describe(),
                      log_lut_->inverse_size());
    if (raw_compressed_in_)
        console->info("PiSP COMP1 raw input detected; decoding to {}-bit DNG rows", dng_info.bits);
}

/* ────────────────────────────────────────────────────────────── */
/*  Private: dng_save – build TIFF in-memory                      */
/* ────────────────────────────────────────────────────────────── */
size_t DngEncoder::dng_save([[maybe_unused]] int                /*thread_num*/,
                            const uint8_t                      *mem_buf,
                            const uint8_t                      *raw,
                            const StreamInfo                   &info,
                            [[maybe_unused]] const uint8_t     *lomem,
                            [[maybe_unused]] const StreamInfo  &loinfo,
                            [[maybe_unused]] size_t             losize,
                            const libcamera::ControlList       &metadata,
                            int64_t                             timestamp_us,
                            int64_t                             tc_frame_count)
{
    thread_local std::vector<uint8_t> rowBuf;
    thread_local std::vector<uint16_t> row16Buf;
    const auto format_it = bayer_formats.find(info.pixel_format);
    if (format_it == bayer_formats.end())
        throw std::runtime_error("Unsupported Bayer format " + info.pixel_format.toString());
    const BayerFormat &bayer_format = format_it->second;

    /* ──  Memory writer / TIFF header  ────────────────────────── */
    MemoryBuffer buf{const_cast<uint8_t*>(mem_buf), 0, 0,
                     static_cast<uint32_t>(dng_info.buffer_size)};
    write_pod (buf, "II", 2);           /* little-endian */
    write_uint16(buf, 42);              /* TIFF magic    */
    write_uint32(buf, 0);               /* IFD-0 offset (patched later) */

    /* ── 1. Raw image copy (packing if 12-bit) ─────────────────── */
    const uint32_t rawOff = buf.offset;

    if (log_lut_)
    {
        /* CineMate Log: linear sensor codes -> log codes, packed at the target
         * depth. The forward LUT already emits right-justified target-depth
         * codes, so the ordinary packer consumes its output with no extra
         * conversion.
         *
         * Every source shape funnels through one uint16 row, right-justified in
         * the curve's own source domain: COMP1 is decompressed, CSI2-packed
         * RAW12 is unpacked, the PiSP SDR container is shifted down off its MSB
         * alignment, and an already-right-justified row is read in place.
         * setup_encoder() refused anything this chain cannot normalise, so the
         * cases below are exhaustive. Encoding row16Buf into itself is
         * deliberate — the map is per-sample and the write to x follows the read
         * of x — and it keeps every converting case to a single scratch row. */
        const int target         = log_lut_->params().target_bits;
        const uint32_t rowPacked = (info.width * target + 7) / 8;
        rowBuf.resize(rowPacked);
        row16Buf.resize(info.width);

        for (uint32_t y = 0; y < info.height; ++y)
        {
            const uint8_t  *srow = raw + y * info.stride;
            const uint16_t *lin;
            if (bayer_format.compressed)
            {
                unpack_pisp_comp1_row_to_16(srow, row16Buf.data(), info.width);
                lin = row16Buf.data();
            }
            else if (bayer_format.packed)
            {
                unpack_csi2_raw12(srow, row16Buf.data(), info.width);
                lin = row16Buf.data();
            }
            else if (log_src_shift_)
            {
                right_justify_row(reinterpret_cast<const uint16_t *>(srow),
                                  row16Buf.data(), info.width, log_src_shift_);
                lin = row16Buf.data();
            }
            else
            {
                lin = reinterpret_cast<const uint16_t *>(srow);
            }

            log_lut_->encode_row(lin, row16Buf.data(), info.width);
            if (target == 12)
                pack_row_12bit(row16Buf.data(), rowBuf.data(), info.width);
            else
                pack_row_10bit(row16Buf.data(), rowBuf.data(), info.width);
            write_pod(buf, rowBuf.data(), rowPacked);
        }
    }
    else if (bayer_format.compressed)
    {
        if (write12bit_)
        {
            const uint32_t rowPacked = (info.width * 12 + 7) / 8;
            rowBuf.resize(rowPacked);

            for (uint32_t y = 0; y < info.height; ++y)
            {
                unpack_pisp_comp1_row_to_packed12(raw + y * info.stride, rowBuf.data(), info.width);
                write_pod(buf, rowBuf.data(), rowPacked);
            }
        }
        else
        {
            const uint32_t rowBytes = info.width * sizeof(uint16_t);
            row16Buf.resize(info.width);

            for (uint32_t y = 0; y < info.height; ++y)
            {
                unpack_pisp_comp1_row_to_16(raw + y * info.stride, row16Buf.data(), info.width);
                write_pod(buf, row16Buf.data(), rowBytes);
            }
        }
    }
    else if (write12bit_)          /* source 16-bit, we emit packed 12-bit */
    {
        const uint32_t rowPacked = (info.width * 12 + 7) / 8;
        rowBuf.resize(rowPacked);

        for (uint32_t y = 0; y < info.height; ++y) {
            const uint16_t *src = reinterpret_cast<const uint16_t *>(
                                    raw + y * info.stride);
            pack_row_16_to_12bit(src, rowBuf.data(), info.width);
            write_pod(buf, rowBuf.data(), rowPacked);
        }
    }
    else if (dng_info.bits == 12)
    {
        const uint32_t rowPacked = (info.width * 12 + 7) / 8;   /* 1.5 B / px */
        rowBuf.resize(rowPacked);
        if (bayer_format.packed)
            row16Buf.resize(info.width);

        for (uint32_t y = 0; y < info.height; ++y)
        {
            const uint16_t *src;
            if (bayer_format.packed)
            {
                /* CSI2-packed 12-bit (SBGGR12_CSI2P — the Pi 4 / VC4 'P' path):
                 * the receiver delivers MIPI CSI-2 RAW12, so unpack to right-
                 * justified 16-bit before re-packing to the contiguous 12-bit DNG
                 * layout. (A verbatim copy or a uint16 reinterpret here gives the
                 * garbled "wrong bit order" raw seen on Pi 4.) */
                unpack_csi2_raw12(raw + y * info.stride, row16Buf.data(), info.width);
                src = row16Buf.data();
            }
            else
            {
                /* Unpacked SBGGR12 (Pi 5 / PiSP 'U'): 16-bit samples, right-
                 * justified in the low 12 bits. */
                src = reinterpret_cast<const uint16_t *>(raw + y * info.stride);
            }
            pack_row_12bit(src, rowBuf.data(), info.width);
            write_pod(buf, rowBuf.data(), rowPacked);
        }
    }
    else if (dng_info.bits == 10)
    {
        /* pack_row_10bit() writes exactly this many bytes: Pass 2 gave it a
         * zero-padded final group that emits only the (n*10+7)/8 bytes those n
         * pixels occupy, replacing the old flat 5 B/group (which also over-READ
         * the source row). So the scratch row needs no rounding-up — same sizing
         * as the log path above. */
        const uint32_t rowPacked = (info.width * 10 + 7) / 8;   /* 1.25 B / px */
        rowBuf.resize(rowPacked);
        if (bayer_format.packed)
            row16Buf.resize(info.width);

        for (uint32_t y = 0; y < info.height; ++y)
        {
            const uint16_t *src;
            if (bayer_format.packed)
            {
                /* CSI2-packed 10-bit (SBGGR10_CSI2P — the Pi 4 / VC4 'P' path):
                 * MIPI CSI-2 RAW10 → unpack to right-justified 16-bit, then
                 * re-pack to the contiguous 10-bit DNG layout. (Same MIPI-vs-
                 * contiguous fix as the 12-bit path above.) */
                unpack_csi2_raw10(raw + y * info.stride, row16Buf.data(), info.width);
                src = row16Buf.data();
            }
            else
            {
                /* Unpacked SBGGR10 (Pi 5 / PiSP 'U'): 16-bit samples, right-
                 * justified in the low 10 bits. */
                src = reinterpret_cast<const uint16_t *>(raw + y * info.stride);
            }
            pack_row_10bit(src, rowBuf.data(), info.width);
            write_pod(buf, rowBuf.data(), rowPacked);
        }
    }
    else
    {
        /* 14- or 16-bit → copy verbatim, one active row each */
        const uint32_t rowBytes = (info.width * dng_info.bits + 7) / 8;
        for (uint32_t y = 0; y < info.height; ++y)
            write_pod(buf, raw + y * info.stride, rowBytes);
    }

    /* exact size we really wrote */
    const uint32_t rawSize = buf.offset - rawOff;



    /* ──  3.  Per-channel black-level  ────────────────────────── */
    uint16_t black[4] {};
    auto ord = bayer_format.order;
    if (log_lut_)
    {
        /* Under a LinearizationTable a reader applies the curve BEFORE reading
         * this tag, so BlackLevel describes the table's OUTPUT — and that output
         * has exactly one black point: inverse[F] == lp.black_level, by
         * construction (log_decode_level(F) is BL + 0; asserted for all three
         * shipped curves in tests/log_lut_test.cpp).
         *
         * So the curve is the authority here, not the metadata. The reported
         * per-channel levels do not survive the encode — all four CFA channels
         * pass through the same single-channel map — and writing them would
         * claim a per-channel pedestal the table has already flattened, up to
         * the one footroom code of slack setup_encoder() allows. That is exactly
         * the shadow range the footroom exists to preserve.
         *
         * It is also the only right answer if SensorBlackLevels goes missing:
         * the 4096-pedestal fallback below is a LINEAR-path assumption and would
         * write 4096 (16->*) or 256 (12->10) where the curve says 3200 or 200.
         * setup_encoder() now refuses the log path in that case, so this is
         * belt-and-braces rather than the only guard. */
        std::fill(std::begin(black), std::end(black),
                  static_cast<uint16_t>(log_lut_->params().black_level));
    }
    else if (auto bl = metadata.get(controls::SensorBlackLevels); bl && bl->size() >= 4)
    {
        for (int i = 0; i < 4; ++i)
        {
            /* map Bayer order→R,G1,G2,B */
            int c = ord[i] == 0 ? 0 :
                    ord[i] == 2 ? 3 :
                    1 + (i & 1);
            black[c] = static_cast<uint16_t>((*bl)[i] *
                                             dng_info.white / 65535.f + 0.5f);
        }
    }
    else
    {
        /* No SensorBlackLevels metadata: assume the common Pi pedestal of 4096
         * in the 16-bit domain (upstream rpicam-apps uses 4096 >> (16 - bits))
         * and scale it to the output white level like the metadata path above:
         * 256 for 12-bit output, 4096 for 16-bit. */
        const uint16_t fallback =
            static_cast<uint16_t>(4096.f * dng_info.white / 65535.f + 0.5f);
        std::fill(std::begin(black), std::end(black), fallback);
    }

    /* ──  4.  Prepare matrices  ───────────────────────────────── */
    int32_t matrixXY[18]; encode_rational_array(dng_info.CAM_XYZ, 9, matrixXY);
    int32_t neutral[6];   encode_rational_array(dng_info.NEUTRAL, 3, neutral);

    /* ──  5.  Build main IFD-0  ──────────────────────────────── */
    uint16_t planar  = 1;
    uint16_t sampFmt = SAMPLEFORMAT_UINT;
    static const uint8_t v[4] = {1, 4, 0, 0};
    IFDBuilder ifd(info.width, info.height);
    ifd.baseOffset = buf.usedSize;

    uint16_t bits       = dng_info.bits;
    uint32_t bitsPacked = bits;        // upper word = 0

    uint32_t typeFull = 0;
    uint16_t phot     = mono_ ? PHOTOMETRIC_MINISBLACK : PHOTOMETRIC_CFA;

    ifd.addEntry(254 , TIFF_LONG , 1, &typeFull);
    ifd.addEntry(256 , TIFF_LONG , 1, &info.width);
    ifd.addEntry(257 , TIFF_LONG , 1, &info.height);
    ifd.addEntry(258 , TIFF_SHORT, 1, &bitsPacked);
    ifd.addEntry(259 , TIFF_SHORT, 1, &dng_info.compression);
    ifd.addEntry(262 , TIFF_SHORT, 1, &phot);
    ifd.addEntry(273 , TIFF_LONG , 1, &rawOff);
    ifd.addEntry(278 , TIFF_LONG , 1, &info.height);
    ifd.addEntry(279 , TIFF_LONG , 1, &rawSize);
    ifd.addEntry(277 , TIFF_SHORT, 1, &dng_info.samples_per_pixel);
    ifd.addEntry(284 , TIFF_SHORT, 1, &planar);
    ifd.addEntry(339 , TIFF_SHORT, 1, &sampFmt);
    ifd.addEntry(0xC612, TIFF_BYTE, 4, v);
    ifd.addEntry(0xC613, TIFF_BYTE, 4, v);

    /* black / white */
    int32_t blackRat[8];
    for (int i = 0; i < 4; ++i) { blackRat[2 * i] = black[i]; blackRat[2 * i + 1] = 1; }
    ifd.addEntry(0xC61A, TIFF_RATIONAL, 4, blackRat);
    uint16_t white16 = static_cast<uint16_t>(dng_info.white);
    ifd.addEntry(0xC61D, TIFF_SHORT, 1, &white16);

    /* LinearizationTable — log only. The inverse table IS the tag's SHORT
     * payload, no conversion. It is what puts the two levels above into the
     * right domain: a reader applies this table first, so BlackLevel/WhiteLevel
     * describe its OUTPUT (linear), not the stored codes. Tag order does not
     * matter here, sortEntries() below puts the directory in ascending order. */
    if (log_lut_)
        ifd.addEntry(0xC618, TIFF_SHORT,
                     static_cast<uint32_t>(log_lut_->inverse_size()), log_lut_->inverse());

    /* colour matrices */
    ifd.addEntry(0xC621, TIFF_SRATIONAL, 9, matrixXY);   /* ColorMatrix1 */
    ifd.addEntry(0xC622, TIFF_SRATIONAL, 9, matrixXY);   /* ColorMatrix2 */
    uint16_t illum = 21;                                 /* D65 */
    ifd.addEntry(0xC65A, TIFF_SHORT, 1, &illum);
    ifd.addEntry(0xC65B, TIFF_SHORT, 1, &illum);
    ifd.addEntry(0xC628, TIFF_RATIONAL, 3, neutral);     /* AsShotNeutral */

    /* CFA pattern */
    ifd.addEntry(0xC619, TIFF_SHORT, 2, dng_info.black_level_repeat_dim);
    ifd.addEntry(0x828D, TIFF_SHORT, 2, dng_info.black_level_repeat_dim);
    ifd.addEntry(0x828E, TIFF_BYTE , 4, dng_info.bayer_order);

    /* strings */
    ifd.addEntry(271, TIFF_ASCII, make_tag_.size(), make_tag_.data());
    ifd.addEntry(272, TIFF_ASCII, model_tag_.size(), model_tag_.data());
    ifd.addEntry(305, TIFF_ASCII, software_tag_.size(), software_tag_.data());
    ifd.addEntry(0xC614, TIFF_ASCII, ucm_tag_.size(), ucm_tag_.data());

    /* ▸ CinemaDNG tag 0xC764  –  FrameRate (SRATIONAL) */
    // Use the configured frame rate. The FrameDuration metadata path that
    // was here previously embedded the sensor's quantised register period
    // (e.g. 25.011 for a 25 fps target) due to VMAX/HMAX rounding. With
    // the IMX585 getBlanking patch the sensor delivers the nominal rate
    // exactly, and using the configured value is correct for all sensors.
    // The denominator of 1000 gives three decimal places of precision for
    // fractional rates such as 23.976.
    double fps = options_->framerate.value_or(DEFAULT_FRAMERATE);
    int32_t fpsRat[2] = {
        static_cast<int32_t>(std::round(fps * 1000)),  // e.g. 25000 for 25 fps
        1000
    };

    ifd.addEntry(0xC764, TIFF_SRATIONAL, 1, fpsRat);


    /* ------------------------------------------------------------------
     *  Wall-clock timestamp — used only for TIFF DateTime tags below.
     *  TC stepping is now done in encodeThread under encode_mutex_ using
     *  each frame's own sensor timestamp; tc_frame_count is pre-computed
     *  and passed in as a parameter.
     * ------------------------------------------------------------------ */
    uint64_t ts_us = wallclock_ts_us_ ? wallclock_ts_us_   // µs since epoch
                                      : static_cast<uint64_t>(timestamp_us);

    struct timeval tv;
    tv.tv_sec  = ts_us / 1'000'000;
    tv.tv_usec = ts_us % 1'000'000;

    struct tm *lt = localtime(&tv.tv_sec);

    /* tc_start_hh_/mm_/ss_ and tc_fps_ were set at clip origin in encodeThread. */
    int ff  = static_cast<int>(tc_frame_count % tc_fps_);
    int64_t total_s = tc_frame_count / tc_fps_;
    int ss  = static_cast<int>(total_s % 60);
    int mm  = static_cast<int>((total_s / 60) % 60);
    int hh  = static_cast<int>((total_s / 3600) % 24);

    /* Add wall-clock origin, propagating carries */
    ss += tc_start_ss_;
    if (ss >= 60) { ss -= 60; mm += 1; }
    mm += tc_start_mm_;
    if (mm >= 60) { mm -= 60; hh += 1; }
    hh = (hh + tc_start_hh_) % 24;

    uint8_t tc[8] = {
        static_cast<uint8_t>(((ff/10)<<4)|(ff%10)),
        static_cast<uint8_t>(((ss/10)<<4)|(ss%10)),
        static_cast<uint8_t>(((mm/10)<<4)|(mm%10)),
        static_cast<uint8_t>(((hh/10)<<4)|(hh%10)),
        0,0,0,0
    };

    /* store time-code & date for other modules */
     std::copy(std::begin(tc), std::end(tc), originationTimeCode.begin());
    originationDate[0] = static_cast<uint16_t>(lt->tm_year + 1900);
    originationDate[1] = static_cast<uint16_t>(lt->tm_mon + 1);
    originationDate[2] = static_cast<uint16_t>(lt->tm_mday);

    ifd.addEntry(0xC763, TIFF_BYTE, 8, tc);

    /* DateTimeOriginal */
    char dateStr[20];
    strftime(dateStr, sizeof(dateStr), "%Y:%m:%d %H:%M:%S", lt);
    ifd.addEntry(0x9003, TIFF_ASCII, 20, dateStr);

    ifd.sortEntries(); ifd.build(buf);

    /* patch TIFF header with IFD-0 offset */
    *reinterpret_cast<uint32_t*>(buf.buffer + 4) = ifd.baseOffset;
    return buf.usedSize;
}

// ──────────────────────────────────────────────────────────────
//  Encoding image buffer
// ──────────────────────────────────────────────────────────────
void DngEncoder::encodeThread(int num)
{
    configureThreadContext("dng-enc-", static_cast<size_t>(num), encode_worker_count_, encode_affinity_, encode_nice_);
    EncodeItem encode_item;

    while (true)
    {
        /* ──  Get the next job from the queue  ───────────────── */
        {
            std::unique_lock<std::mutex> lock(encode_mutex_);
            encode_cond_var_.wait(lock, [this] {
                return stop_encode_.load(std::memory_order_acquire) || !encode_queue_.empty();
            });

            if (stop_encode_.load(std::memory_order_acquire) && encode_queue_.empty())
                break;

            encode_item = encode_queue_.front();
            encode_queue_.pop();

            /* ── TC step: computed here, under encode_mutex_, so frames are
             *    processed in FIFO order and each frame's own sensor timestamp
             *    is used.  This eliminates the wallclock_ts_us_ shared-scalar
             *    race that was producing phantom holes with >1 encode workers. */
            if (!tc_origin_set_)
            {
                /* First frame of clip: capture wall-clock HH:MM:SS origin. */
                int fps_int = 24;
                /* libcamera FrameDuration is in MICROSECONDS (40000 µs @ 25fps),
                 * so fps = 1e6 / fd.  Using 1e9 here gives 25000 (1000× too high):
                 * raw_elapsed = round(40000 * 25000 / 1e6) = 1000 → ~999 phantom
                 * holes per frame.  The pre-fix path derived this same value as
                 * fpsRat[0]/fpsRat[1] = round(1e9/fd)/1000, i.e. exactly 1e6/fd. */
                if (auto fd = encode_item.met.get(controls::FrameDuration); fd && *fd > 0)
                    fps_int = static_cast<int>(1'000'000.0 / static_cast<double>(*fd) + 0.5);
                if (fps_int <= 0) fps_int = 24;

                /* Prefer wall-clock for the HH:MM:SS display origin; fall back
                 * to sensor monotonic (small display-only error if not set yet). */
                uint64_t origin_us = wallclock_ts_us_
                                     ? wallclock_ts_us_
                                     : static_cast<uint64_t>(encode_item.timestamp_us);
                struct timeval tv { static_cast<time_t>(origin_us / 1'000'000ULL),
                                    static_cast<suseconds_t>(origin_us % 1'000'000ULL) };
                struct tm lt_val {};
                localtime_r(&tv.tv_sec, &lt_val);

                /* Sub-second frame offset: how many frames into the current
                 * second did this clip start?  origin_us is a wall-clock µs
                 * value, so origin_us % 1e6 is the fractional-second part.
                 * Seeding tc_frame_count_ with this (instead of 0) gives
                 * frame-accurate absolute TC alignment with external sources.
                 * The carry propagation in dng_save() already handles
                 * tc_frame_count_ ≥ tc_fps_ at the second boundary. */
                uint64_t sub_us     = origin_us % 1'000'000ULL;
                int64_t sub_frames  = static_cast<int64_t>(
                    std::llround(static_cast<double>(sub_us) * fps_int / 1'000'000.0));
                if (sub_frames >= fps_int) sub_frames = fps_int - 1; /* clamp at boundary */

                tc_last_ts_us_  = encode_item.timestamp_us; /* now monotonic µs */
                tc_frame_count_ = sub_frames;
                tc_start_hh_    = lt_val.tm_hour;
                tc_start_mm_    = lt_val.tm_min;
                tc_start_ss_    = lt_val.tm_sec;
                tc_fps_         = fps_int;
                tc_origin_set_  = true;
                encode_item.tc_frame_count = sub_frames;
            }
            else
            {
                /* Delta uses the per-frame sensor timestamp already in the queue.
                 * No shared scalar, no phantom holes regardless of worker count. */
                uint64_t delta_us = static_cast<uint64_t>(encode_item.timestamp_us)
                                    - static_cast<uint64_t>(tc_last_ts_us_);
                int64_t raw_elapsed = static_cast<int64_t>(std::llround(
                    static_cast<double>(delta_us) * tc_fps_ / 1'000'000.0));
                int64_t frames_elapsed = std::max(INT64_C(1), raw_elapsed);
                tc_frame_count_ += frames_elapsed;
                tc_last_ts_us_   = encode_item.timestamp_us; /* monotonic */
                if (raw_elapsed >= 2)
                    dropped_frames_ += raw_elapsed - 1;
                encode_item.tc_frame_count = tc_frame_count_;
            }
        }

        frames_ = encode_item.index;
        console->trace("Thread[{}] encode frame: {}", num, encode_item.index);

        /* ────────────────────────────────────────────────────── */
        /*  RAM back-pressure + aligned allocation               */
        /* ────────────────────────────────────────────────────── */
        uint8_t *mem_buf = nullptr;

        {
            /* wait until a permit is available */
            std::unique_lock<std::mutex> lk(ram_mtx_);
            ram_cv_.wait(lk, [this] { return ram_buffers_ < max_ram_buffers_; });
            ++ram_buffers_;
        }

        mem_buf = acquirePooledBuffer();

        if (!mem_buf)
        {
            if (posix_memalign(reinterpret_cast<void **>(&mem_buf),
                               BLOCK_SIZE,
                               dng_info.buffer_size) != 0)
            {
                /* Allocation failed – release the reserved permit; frame is dropped */
                perror("posix_memalign");
                frames_in_flight_.fetch_sub(1, std::memory_order_relaxed);
                {
                    std::lock_guard<std::mutex> lk(ram_mtx_);
                    if (ram_buffers_ > 0)
                        --ram_buffers_;
                }
                ram_cv_.notify_one();
                continue;
            }
        }

        /* ────────────────────────────────────────────────────── */
        /*  Build the TIFF/DNG into the new buffer               */
        /* ────────────────────────────────────────────────────── */
        auto start_time = std::chrono::high_resolution_clock::now();

        size_t tiff_size = 0;
        try
        {
            tiff_size = dng_save(
                num,
                static_cast<const uint8_t *>(mem_buf),
                static_cast<const uint8_t *>(encode_item.mem),
                encode_item.info,
                static_cast<const uint8_t *>(encode_item.lomem),
                encode_item.loinfo,
                encode_item.losize,
                encode_item.met,
                encode_item.timestamp_us,
                encode_item.tc_frame_count);
        }
        catch (const std::exception &e)
        {
            /* dng_save can throw (e.g. "Unsupported Bayer format", or bad_alloc).
             * Drop this frame WITHOUT killing the encode thread: an uncaught
             * throw here would leave frames_in_flight_ incremented forever and
             * permanently jam the rec-gate (green stuck on, every future
             * recording blocked). Mirror the alloc-fail drop cleanup, and also
             * run the normal-path camera-buffer release tail so the libcamera
             * encode_buffer_queue_ does not leak a buffer. */
            console->error("Thread[{}] dng_save failed for frame {}: {} — frame dropped",
                           num, encode_item.index, e.what());
            frames_in_flight_.fetch_sub(1, std::memory_order_relaxed);
            if (mem_buf)
                releasePooledBuffer(mem_buf);
            {
                std::lock_guard<std::mutex> lk(ram_mtx_);
                if (ram_buffers_ > 0)
                    --ram_buffers_;
            }
            ram_cv_.notify_one();
            input_done_callback_(nullptr);
            output_ready_callback_(encode_item.mem,
                                   encode_item.size,
                                   encode_item.timestamp_us,
                                   true);
            continue;
        }

        /* queue for disk writer */
        {
            DiskItem item = {
                mem_buf,
                tiff_size,
                encode_item.info,
                encode_item.met,
                encode_item.timestamp_us,
                encode_item.index,
                encode_item.folder
            };

            std::lock_guard<std::mutex> lock(disk_mutex_);
            disk_buffer_.push(std::move(item));
            noteBufferDepth(static_cast<int>(disk_buffer_.size()));
            disk_cond_var_.notify_one();
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        console->debug("Thread[{}] {} Time taken for the encode: {} ms, disk queue:{}  Size:{}",
                       num, encode_item.index, duration, disk_buffer_.size(), tiff_size);

        /* mark the camera buffer as reusable */
        input_done_callback_(nullptr);
        output_ready_callback_(encode_item.mem,
                               encode_item.size,
                               encode_item.timestamp_us,
                               true);
    }
}


//Flushing data to disk
void DngEncoder::diskThread(int num)
{
    configureThreadContext("dng-dsk-", static_cast<size_t>(num), disk_worker_count_, disk_affinity_, disk_nice_);
    DiskItem disk_item;

    while (true)
    {
        {
            std::unique_lock<std::mutex> lock(disk_mutex_);
            disk_cond_var_.wait(lock, [this] {
                return stop_disk_.load(std::memory_order_acquire) || !disk_buffer_.empty();
            });

            if (stop_disk_.load(std::memory_order_acquire) && disk_buffer_.empty())
                break;

            disk_item = disk_buffer_.front();
            disk_buffer_.pop();
        }

        std::ostringstream oss;
        const std::string folder = disk_item.folder.empty() ? options_->folder : disk_item.folder;

        oss << options_->mediaDest << '/'
            << folder << '/'
            << folder << '_'
            << std::setw(9) << std::setfill('0') << disk_item.index 
            << ".dng";

        std::string filename = oss.str();
    
        console->trace("Thread[{}]  Save frame to disk: {}", num, disk_item.index);

        console->info("DNG written: {}", filename);
        
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // Use standard buffered IO instead of O_DIRECT
        int fd = open(filename.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);

        if (fd != -1) {
            // Provide sequential access hint
            posix_fadvise(fd, 0, 0, POSIX_FADV_SEQUENTIAL);


            // Always use actual used size returned by dng_save
            ssize_t bytes_written = write(fd, disk_item.mem_buf, disk_item.size);
            bool write_ok = (bytes_written >= 0 &&
                             static_cast<size_t>(bytes_written) == disk_item.size);
            if (!write_ok) {
                int werr = errno;
                write_failures_.fetch_add(1, std::memory_order_relaxed);
                perror("Error writing to file");
                console->error("DNG write FAILED ({}): wrote {} of {} bytes - {}",
                               filename, bytes_written, disk_item.size, strerror(werr));
            }
            // close() can surface deferred write-back errors that write() did
            // not report (common on FUSE/ntfs-3g and network filesystems).
            // Count those too, but only when the write itself looked OK so a
            // single lost frame is not counted twice.
            if (close(fd) != 0 && write_ok) {
                int cerr = errno;
                write_failures_.fetch_add(1, std::memory_order_relaxed);
                console->error("DNG write FAILED on close ({}) - {}",
                               filename, strerror(cerr));
            }
        } else {
            int oerr = errno;
            write_failures_.fetch_add(1, std::memory_order_relaxed);
            perror("Failed to open file for writing");
            console->error("DNG write FAILED to open ({}) - {}",
                           filename, strerror(oerr));
        }

        // Frame fully handled (written or errored) — release in-flight count
        frames_in_flight_.fetch_sub(1, std::memory_order_relaxed);

        // Clean up
        releasePooledBuffer(static_cast<uint8_t *>(disk_item.mem_buf));

        {
            std::lock_guard<std::mutex> lk(ram_mtx_);
            if (ram_buffers_ > 0)
                --ram_buffers_;          // <-- give permit back
            ram_cv_.notify_one();
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        console->debug("Thread[{}] {} Time taken for the disk io: {} milliseconds", num, disk_item.index, duration);
    }
}

uint8_t *DngEncoder::acquirePooledBuffer()
{
    const size_t required_size = dng_info.buffer_size;
    if (required_size == 0)
        return nullptr;

    std::lock_guard<std::mutex> lock(buffer_pool_mutex_);

    if (pooled_buffer_size_ != 0 && pooled_buffer_size_ != required_size)
    {
        for (auto *ptr : buffer_pool_)
            free(ptr);
        buffer_pool_.clear();
        pooled_buffer_size_ = 0;
    }

    if (pooled_buffer_size_ == 0)
        pooled_buffer_size_ = required_size;

    if (buffer_pool_.empty())
        return nullptr;

    uint8_t *buffer = buffer_pool_.back();
    buffer_pool_.pop_back();
    return buffer;
}

void DngEncoder::releasePooledBuffer(uint8_t *buffer)
{
    if (!buffer)
        return;

    const size_t required_size = dng_info.buffer_size;
    if (required_size == 0)
    {
        free(buffer);
        return;
    }

    std::lock_guard<std::mutex> lock(buffer_pool_mutex_);

    bool size_changed = false;

    if (pooled_buffer_size_ != 0 && pooled_buffer_size_ != required_size)
    {
        for (auto *ptr : buffer_pool_)
            free(ptr);
        buffer_pool_.clear();
        pooled_buffer_size_ = 0;
        size_changed = true;
    }

    if (size_changed)
    {
        free(buffer);
        return;
    }

    if (pooled_buffer_size_ == 0)
        pooled_buffer_size_ = required_size;

    if (pooled_buffer_size_ != required_size)
    {
        free(buffer);
        return;
    }

    buffer_pool_.push_back(buffer);
}

void DngEncoder::drainPooledBuffers()
{
    std::vector<uint8_t *> buffers;
    {
        std::lock_guard<std::mutex> lock(buffer_pool_mutex_);
        buffers.swap(buffer_pool_);
        pooled_buffer_size_ = 0;
    }

    for (auto *ptr : buffers)
        free(ptr);
}
