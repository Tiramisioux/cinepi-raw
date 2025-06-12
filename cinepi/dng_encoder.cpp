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
 #include <cstring>
 #include <stdexcept>
 #include <iomanip>
 
 #include <sstream>                 // ostringstream for filenames
 #include <fstream>                 // /proc/meminfo parsing
 #include <regex>                   // extract MemAvailable
 
 #include "dng_encoder.hpp"         // This file's class definition
 #include "utils.hpp"               // You call getHwId() from here
 #include "ifd_builder.hpp"         // Used in dng_save() for TIFF/IFD writing
 
 #include <sys/mman.h>              // O_DIRECT
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
	{ formats::R12_CSI2P, { "BGGR-12", 12, CFA_BGGR, true } },
	{ formats::R12, { "BGGR-12", 12, CFA_BGGR, false, false } },

	/* PiSP compressed formats. */
	{ formats::RGGB_PISP_COMP1, { "RGGB-16-PISP", 16, CFA_RGGB, false, true } },
	{ formats::GRBG_PISP_COMP1, { "GRBG-16-PISP", 16, CFA_GRBG, false, true } },
	{ formats::GBRG_PISP_COMP1, { "GBRG-16-PISP", 16, CFA_GBRG, false, true } },
	{ formats::BGGR_PISP_COMP1, { "BGGR-16-PISP", 16, CFA_BGGR, false, true } },
};

static const std::map<PixelFormat, int> mono_formats = {
    { formats::R12_CSI2P, 12 },
    { formats::R16,       16 }
};


void pack_8bit_data(const uint16_t* src, uint8_t* dst, size_t num_pixels) {
    for (size_t i = 0; i < num_pixels; i++) {
        dst[0] = src[i];
    }
}

void pack_10bit_data(const uint16_t* src, uint8_t* dst, size_t num_pixels) {
    // 5 bytes can hold 4 10-bit pixels
    // Every iteration of the loop processes 4 pixels (40 bits)
    for (size_t i = 0; i < num_pixels; i += 4) {
        dst[0] = src[i] >> 2;                               // Highest 8 bits of pixel 1
        dst[1] = (src[i] << 6) | (src[i + 1] >> 4);         // Lowest 2 bits of pixel 1 + highest 6 bits of pixel 2
        dst[2] = (src[i + 1] << 4) | (src[i + 2] >> 6);     // Lowest 4 bits of pixel 2 + highest 4 bits of pixel 3
        dst[3] = (src[i + 2] << 2) | (src[i + 3] >> 8);     // Lowest 6 bits of pixel 3 + highest 2 bits of pixel 4
        dst[4] = src[i + 3];                                // Lowest 8 bits of pixel 4

        dst += 5; // Move to the next 5 bytes
    }
}

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

DngEncoder::DngEncoder(RawOptions const *options)
    : Encoder(options), // Assuming you're calling the base class constructor
      encoder_initialized_(false),
      encodeCheck_(false),
      abortEncode_(false), 
      abortOutput_(false), 
      resetCount_(false), 
      index_(0), 
      frames_(0), 
      options_(options)
{
    console = spdlog::stdout_color_mt("dng_encoder");

    for (int i = 0; i < NUM_ENC_THREADS; i++){
        encode_thread_[i] = std::thread(std::bind(&DngEncoder::encodeThread, this, i));
    }
    for (int i = 0; i < NUM_DISK_THREADS; i++){
        disk_thread_[i] = std::thread(std::bind(&DngEncoder::diskThread, this, i));
    }

    console->info("DngEncoder started!");
}

DngEncoder::~DngEncoder()
{
    abortEncode_ = true;
    for (int i = 0; i < NUM_ENC_THREADS; i++){
        encode_thread_[i].join();
    }
    for (int i = 0; i < NUM_DISK_THREADS; i++){
        disk_thread_[i].join();
    }

    abortOutput_ = true;
    console->info("DngEncoder stopped!");
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
        std::lock_guard<std::mutex> lock(encode_mutex_);
        EncodeItem item = { mem, size, info, lomem, losize, loinfo, metadata, timestamp_us, index_++ };
        encode_queue_.push(item);
        encode_cond_var_.notify_all();
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
    dng_info.bits                  = bf.bits;
    dng_info.white                 = (1u << bf.bits) - 1u;
    dng_info.photometric           = PHOTOMETRIC_CFA;
    dng_info.samples_per_pixel     = 1;
    std::memcpy(dng_info.bayer_order, bf.order, 4);
    dng_info.black_level_repeat_dim[0] = 2;
    dng_info.black_level_repeat_dim[1] = 2;

    /* ──  Black-level defaults (16-bit = 256 DN)  ─────────────── */
    std::fill(std::begin(dng_info.black_levels),
              std::end(dng_info.black_levels),
              256.f);

    if (auto bl = metadata.get(controls::SensorBlackLevels); bl && bl->size() >= 4)
        std::copy(bl->begin(), bl->end(), dng_info.black_levels);

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
    const uint32_t frame  = (cfg.size.width * cfg.size.height * dng_info.bits) / 8;
    const uint32_t thumb  = lo_cfg.stride * lo_cfg.size.height;
    dng_info.buffer_size  = align_up(frame + thumb + 20 * 1024, ONE_MB);

    /* ──  Static strings & misc  ──────────────────────────────── */
    dng_info.make       = "Raspberry Pi";
    dng_info.model      = "SONY IMX585-AAQJ1";
    dng_info.software   = "Libcamera;cinepi-raw";
    dng_info.ucm        = "CinePi";
    dng_info.serial     = getHwId();
    dng_info.compression = COMPRESSION_NONE;

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
}

/* ────────────────────────────────────────────────────────────── */
/*  Private: dng_save – build TIFF in-memory                      */
/* ────────────────────────────────────────────────────────────── */
size_t DngEncoder::dng_save([[maybe_unused]] int               /*thread_num*/,
                            const uint8_t                     *mem_buf,
                            const uint8_t                     *raw,
                            const StreamInfo                  &info,
                            const uint8_t                     *lomem,
                            const StreamInfo                  &loinfo,
                            [[maybe_unused]] size_t            /*losize*/,
                            const CompletedRequest::ControlList &metadata,
                            [[maybe_unused]] uint64_t          /*fn*/)
{
    /* ──  Memory writer / TIFF header  ────────────────────────── */
    MemoryBuffer buf{const_cast<uint8_t*>(mem_buf), 0, 0,
                     static_cast<uint32_t>(dng_info.buffer_size)};
    write_pod (buf, "II", 2);           /* little-endian */
    write_uint16(buf, 42);              /* TIFF magic    */
    write_uint32(buf, 0);               /* IFD-0 offset (patched later) */

    /* ──  1.  Thumbnail copy (mono 8-bit)  ────────────────────── */
    const uint32_t thumbOff   = buf.offset;
    const uint32_t thumbBytes = dng_info.thumbWidth;          /* 1 byte / px */
    for (uint32_t y = 0; y < dng_info.thumbHeight; ++y)
        write_pod(buf, lomem + y * loinfo.stride, thumbBytes);
    const uint32_t thumbSize = thumbBytes * dng_info.thumbHeight;

    /* ──  2.  Raw image copy (stride-aware)  ──────────────────── */
    const uint32_t rawOff = buf.offset;
    const uint32_t rrb    = (info.width * dng_info.bits + 7) / 8;
    for (uint32_t y = 0; y < info.height; ++y)
        write_pod(buf, raw + y * info.stride, rrb);
    const uint32_t rawSize = rrb * info.height;

    /* ──  3.  Per-channel black-level  ────────────────────────── */
    uint16_t black[4] {};
    auto ord = bayer_formats.find(info.pixel_format)->second.order;
    if (auto bl = metadata.get(controls::SensorBlackLevels); bl && bl->size() >= 4)
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
        std::fill(std::begin(black), std::end(black), 256);

    /* ──  4.  Prepare matrices  ───────────────────────────────── */
    int32_t matrixXY[18]; encode_rational_array(dng_info.CAM_XYZ, 9, matrixXY);
    int32_t neutral[6];   encode_rational_array(dng_info.NEUTRAL, 3, neutral);

    /* ──  5.  Build thumbnail IFD (SubIFD[0])  ───────────────── */
    IFDBuilder sub(dng_info.thumbWidth, dng_info.thumbHeight);
    sub.baseOffset = buf.usedSize;

    uint32_t subType = 1;                 /* reduced-res */
    uint16_t planar  = 1;
    uint16_t sampFmt = SAMPLEFORMAT_UINT;
    static const uint8_t v[4] = {1, 4, 0, 0};

    sub.addEntry(254,  TIFF_LONG , 1, &subType);
    sub.addEntry(256,  TIFF_SHORT, 1, &dng_info.thumbWidth);
    sub.addEntry(257,  TIFF_SHORT, 1, &dng_info.thumbHeight);
    sub.addEntry(258,  TIFF_SHORT, 1, &dng_info.thumbBitsPerSample);
    sub.addEntry(259,  TIFF_SHORT, 1, &dng_info.compression);
    sub.addEntry(262,  TIFF_SHORT, 1, &dng_info.thumbPhotometric);
    sub.addEntry(273,  TIFF_LONG , 1, &thumbOff);
    sub.addEntry(278,  TIFF_SHORT, 1, &dng_info.thumbHeight);
    sub.addEntry(279,  TIFF_LONG , 1, &thumbSize);
    sub.addEntry(277,  TIFF_SHORT, 1, &dng_info.thumbSamplesPerPixel);
    sub.addEntry(284,  TIFF_SHORT, 1, &planar);
    sub.addEntry(339,  TIFF_SHORT, 1, &sampFmt);
    sub.addEntry(0xC612, TIFF_BYTE, 4, v);
    sub.addEntry(0xC613, TIFF_BYTE, 4, v);
    std::string ucm = dng_info.ucm + '\0';
    sub.addEntry(0xC614, TIFF_ASCII, ucm.size(), ucm.data());

    sub.sortEntries(); sub.build(buf);
    const uint32_t subIFDoff = sub.baseOffset;

    /* ──  6.  Build main IFD-0  ───────────────────────────────── */
    IFDBuilder ifd(info.width, info.height);
    ifd.baseOffset = buf.usedSize;

    uint32_t typeFull = 0;
    uint16_t phot     = PHOTOMETRIC_CFA;

    ifd.addEntry(254 , TIFF_LONG , 1, &typeFull);
    ifd.addEntry(256 , TIFF_LONG , 1, &info.width);
    ifd.addEntry(257 , TIFF_LONG , 1, &info.height);
    ifd.addEntry(258 , TIFF_SHORT, 1, &dng_info.bits);
    ifd.addEntry(259 , TIFF_SHORT, 1, &dng_info.compression);
    ifd.addEntry(262 , TIFF_SHORT, 1, &phot);
    ifd.addEntry(273 , TIFF_LONG , 1, &rawOff);
    ifd.addEntry(278 , TIFF_LONG , 1, &info.height);
    ifd.addEntry(279 , TIFF_LONG , 1, &rawSize);
    ifd.addEntry(277 , TIFF_SHORT, 1, &dng_info.samples_per_pixel);
    ifd.addEntry(284 , TIFF_SHORT, 1, &planar);
    ifd.addEntry(339 , TIFF_SHORT, 1, &sampFmt);
    ifd.addEntry(0x014A, TIFF_LONG, 1, &subIFDoff);
    ifd.addEntry(0xC612, TIFF_BYTE, 4, v);
    ifd.addEntry(0xC613, TIFF_BYTE, 4, v);

    /* black / white */
    int32_t blackRat[8];
    for (int i = 0; i < 4; ++i) { blackRat[2 * i] = black[i]; blackRat[2 * i + 1] = 1; }
    ifd.addEntry(0xC61A, TIFF_RATIONAL, 4, blackRat);
    uint16_t white16 = static_cast<uint16_t>(dng_info.white);
    ifd.addEntry(0xC61D, TIFF_SHORT, 1, &white16);

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
    std::string make  = dng_info.make  + '\0';
    std::string model = dng_info.model + '\0';
    std::string soft  = dng_info.software + '\0';
    ifd.addEntry(271, TIFF_ASCII, make .size(), make .data());
    ifd.addEntry(272, TIFF_ASCII, model.size(), model.data());
    ifd.addEntry(305, TIFF_ASCII, soft .size(), soft .data());
    ifd.addEntry(0xC614, TIFF_ASCII, ucm.size(), ucm.data());

    /* frame-rate from metadata */
    int32_t fpsRat[2] = { 25000, 1000 };                 /* default 25 fps */
    if (auto fd = metadata.get(controls::FrameDuration); fd && *fd > 0)
    {
        double fps = 1e9 / static_cast<double>(*fd);
        fpsRat[0] = static_cast<int32_t>(fps * 1000 + 0.5);
        fpsRat[1] = 1000;
    }
    ifd.addEntry(0xC764, TIFF_SRATIONAL, 1, fpsRat);

    /* time-code (BCD) */
    struct timeval tv;  gettimeofday(&tv, nullptr);
    struct tm *lt = localtime(&tv.tv_sec);
    int fps = fpsRat[0] / fpsRat[1];
    int frame = static_cast<int>((tv.tv_usec * fps) / 1'000'000);
    uint8_t tc[8] = {
        static_cast<uint8_t>(((frame     /10)<<4)|(frame     %10)),
        static_cast<uint8_t>(((lt->tm_sec/10)<<4)|(lt->tm_sec%10)),
        static_cast<uint8_t>(((lt->tm_min/10)<<4)|(lt->tm_min%10)),
        static_cast<uint8_t>(((lt->tm_hour/10)<<4)|(lt->tm_hour%10)),
        0,0,0,0
    };
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

//Encoding image buffer
void DngEncoder::encodeThread(int num)
{
    std::chrono::duration<double> encode_time(0);
    EncodeItem encode_item;

    while (true)
    {
        {
            std::unique_lock<std::mutex> lock(encode_mutex_);
            while (true)
            {   
                if (!encode_queue_.empty())
                {
                    encode_item = encode_queue_.front();
                    encode_queue_.pop();
                    break;
                }
                else {
                    encode_cond_var_.wait_for(lock, 500us);
                }
            }
        }

        frames_ = {encode_item.index};
        console->trace("Thread[{}] encode frame: {}", num, encode_item.index);

        /* ──  RAM back-pressure  ───────────────────────── */
        {
            std::unique_lock<std::mutex> lk(ram_mtx_);
            ram_cv_.wait(lk, [this]{ return ram_buffers_ < max_ram_buffers_; });
            ++ram_buffers_;                       /* we’re about to allocate */
        }

        {
            auto start_time = std::chrono::high_resolution_clock::now();

            uint8_t *mem_buf;
            if (posix_memalign((void **)&mem_buf, BLOCK_SIZE, dng_info.buffer_size) != 0) {
                perror("Error allocating aligned memory");
                continue;
            }

            size_t tiff_size = dng_save(
                num,
                static_cast<const uint8_t *>(mem_buf),
                static_cast<const uint8_t *>(encode_item.mem),
                encode_item.info,
                static_cast<const uint8_t *>(encode_item.lomem),
                encode_item.loinfo,
                encode_item.losize,
                encode_item.met,
                encode_item.index);

            DiskItem item = {
                mem_buf,
                tiff_size,
                encode_item.info,
                encode_item.met,
                encode_item.timestamp_us,
                encode_item.index
            };

            {
                std::lock_guard<std::mutex> lock(disk_mutex_);
                disk_buffer_.push(std::move(item));
                disk_cond_var_.notify_all();
            }

            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
            console->info("Thread[{}] {} Time taken for the encode: {} milliseconds, disk buffer count:{} Size:{}",
                         num, encode_item.index, duration, disk_buffer_.size(), tiff_size);
        }

        input_done_callback_(nullptr);
        output_ready_callback_(encode_item.mem, encode_item.size, encode_item.timestamp_us, true);
    }
}

//Flushing data to disk
void DngEncoder::diskThread(int num)
{
    DiskItem disk_item;

    while (true)
    {
        {
            std::unique_lock<std::mutex> lock(disk_mutex_);
            while (true)
            {
                if (!disk_buffer_.empty())
                {
                    disk_item = disk_buffer_.front();
                    disk_buffer_.pop();
                    break;
                }
                else {
                    disk_cond_var_.wait_for(lock, 1ms);
                }
            }
        }

        std::ostringstream oss;
        oss << options_->mediaDest << '/' 
            << options_->folder << '/' 
            << options_->folder << '_'
            << std::setw(9) << std::setfill('0') << disk_item.index 
            << ".dng";

        std::string filename = oss.str();
    
        console->trace("Thread[{}]  Save frame to disk: {}", num, disk_item.index);
        
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // Use standard buffered IO instead of O_DIRECT
        int fd = open(filename.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);

        if (fd != -1) {
            // Provide sequential access hint
            posix_fadvise(fd, 0, 0, POSIX_FADV_SEQUENTIAL);
            posix_fadvise(fd, 0, 0, POSIX_FADV_NOREUSE);

            // Always use actual used size returned by dng_save
            ssize_t bytes_written = write(fd, disk_item.mem_buf, disk_item.size);
            if (bytes_written < 0 || static_cast<size_t>(bytes_written) != disk_item.size) {
                perror("Error writing to file");
            }
            close(fd);
        } else {
            perror("Failed to open file for writing");
        }

        // Clean up
        free(disk_item.mem_buf);

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        console->info("Thread[{}] {} Time taken for the disk io: {} milliseconds", num, disk_item.index, duration);
    }
}