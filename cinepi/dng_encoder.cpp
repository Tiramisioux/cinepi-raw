/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * Based on mjpeg_encoder.cpp, modifications by Csaba Nagy & Will Whang 
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

void DngEncoder::setup_encoder(libcamera::StreamConfiguration const &cfg, libcamera::StreamConfiguration const &lo_cfg, CompletedRequest::ControlList const &metadata)
{
    auto bayer_it = bayer_formats.find(cfg.pixelFormat);
    auto mono_it = mono_formats.find(cfg.pixelFormat);
    

    if (bayer_it != bayer_formats.end()) {
        const BayerFormat &bayer_format = bayer_it->second;
        dng_info.bits = bayer_format.bits;
        console->debug("Bayer format: {} ({})", bayer_format.name, cfg.pixelFormat.toString());
        dng_info.white = (1 << dng_info.bits) - 1;
        dng_info.photometric = PHOTOMETRIC_CFA;
        dng_info.samples_per_pixel = 1;
        memcpy(dng_info.bayer_order, bayer_format.order, 4);
        dng_info.cfa_repeat_pattern_dim[0] = 2;
        dng_info.cfa_repeat_pattern_dim[1] = 2;
        dng_info.black_level_repeat_dim[0] = 2;
        dng_info.black_level_repeat_dim[1] = 2;
        mono_ = false;
        console->debug("Pixel format is Bayer: {} ({}-bit)", bayer_format.name, dng_info.bits);
    }
    else if (auto mit = mono_formats.find(cfg.pixelFormat); mit != mono_formats.end()) {
        dng_info.bits = mit->second;
        dng_info.white = (1 << dng_info.bits) - 1;
        dng_info.photometric = PHOTOMETRIC_MINISBLACK;
        dng_info.samples_per_pixel = 1;
        mono_ = true;
        dng_info.cfa_repeat_pattern_dim[0] = 0;
        dng_info.cfa_repeat_pattern_dim[1] = 0;
        memset(dng_info.bayer_order, 0, 4);
        dng_info.black_level_repeat_dim[0] = 1;
        dng_info.black_level_repeat_dim[1] = 1;
        console->debug("Pixel format is Monochrome: {}-bit", dng_info.bits);
    }
    else {
        throw std::runtime_error("Unsupported pixel format: " + cfg.pixelFormat.toString());
    }
    

    // white level -----------------------------------------------------
    dng_info.white = (1 << dng_info.bits) - 1;

    /* ---------------------------------------------------------------
    * black level – choose a sensible constant                        *
    *  - 12-bit RAW  :  64   (16 DN × 4)                              *
    *  - 16-bit RAW  : 256   (16 DN × 16)   ← IMX585 linear mode      *
    * -------------------------------------------------------------- */
    dng_info.black = 256;            // <<< place it right here

    auto bl = metadata.get(controls::SensorBlackLevels);
    if (bl && bl->size()>=4)
        std::copy(bl->begin(), bl->end(), dng_info.black_levels);
    else std::fill(...,256);


    // AnalogBalance -- Nuetral setup
    std::fill(std::begin(dng_info.NEUTRAL), std::end(dng_info.NEUTRAL), 1);
    std::fill(std::begin(dng_info.ANALOGBALANCE), std::end(dng_info.ANALOGBALANCE), 1);


    // CCM Configuration
    Matrix WB_GAINS(1, 1, 1);
    auto cg = metadata.get(controls::ColourGains);
    if (cg)
    {
        dng_info.NEUTRAL[0] = 1.0 / (*cg)[0];
        dng_info.NEUTRAL[2] = 1.0 / (*cg)[1];
        WB_GAINS = Matrix((*cg)[0], 1, (*cg)[1]);
    }

    // Use a slightly plausible default CCM in case the metadata doesn't have one (it should!).
    Matrix CCM(1.90255, -0.77478, -0.12777,
               -0.31338, 1.88197, -0.56858,
               -0.06001, -0.61785, 1.67786);
    auto ccm = metadata.get(controls::ColourCorrectionMatrix);
    if (ccm)
    {
        CCM = Matrix((*ccm)[0], (*ccm)[1], (*ccm)[2], (*ccm)[3], (*ccm)[4], (*ccm)[5], (*ccm)[6], (*ccm)[7], (*ccm)[8]);
    }
    else
        console->error("WARNING: no CCM metadata found");

    // This maxtrix from http://www.brucelindbloom.com/index.html?Eqn_RGB_XYZ_Matrix.html
    Matrix RGB2XYZ(0.4124564, 0.3575761, 0.1804375,
                   0.2126729, 0.7151522, 0.0721750,
                   0.0193339, 0.1191920, 0.9503041);
    Matrix CAM_XYZ = (RGB2XYZ * CCM * WB_GAINS).Inv();
    std::copy(std::begin(CAM_XYZ.m), std::end(CAM_XYZ.m), std::begin(dng_info.CAM_XYZ));

    // cfa
    dng_info.cfa_repeat_pattern_dim[0] = 2;
    dng_info.cfa_repeat_pattern_dim[1] = 2;
    dng_info.black_level_repeat_dim[0] = 2;
    dng_info.black_level_repeat_dim[1] = 2;

    // offsets
    // dng_info.offset_y_start = options_->rawCrop[0];
    // dng_info.offset_y_end = options_->rawCrop[1];
    // dng_info.offset_x_start = options_->rawCrop[2];
    // dng_info.offset_x_end = options_->rawCrop[3];

    dng_info.offset_y_start = 0;
    dng_info.offset_y_end = 0;
    dng_info.offset_x_start = 0;
    dng_info.offset_x_end = 0;

    // const float bppf = (dng_bits/8);
    // const uint16_t byte_offset_x = (bppf * offset_x_start) / sizeof(uint64_t); 
    // // const uint16_t read_length_x = (1.5 * (info.width - (offset_x_start+offset_x_end))) / sizeof(uint64_t);
    dng_info.t_height = (cfg.size.height - (dng_info.offset_y_start+dng_info.offset_y_end));
    dng_info.t_width = (cfg.size.width - (dng_info.offset_x_start+dng_info.offset_x_end));

    // thumbnail config
    dng_info.thumbType = options_->thumbnail;
    dng_info.thumbWidth = 32;
    dng_info.thumbHeight = 32;
    dng_info.thumbPhotometric = PHOTOMETRIC_MINISBLACK;
    dng_info.thumbBitsPerSample = 8;
    dng_info.thumbSamplesPerPixel = 1;
    unsigned int thumbnail_size = dng_info.thumbWidth * dng_info.thumbHeight;

    switch(dng_info.thumbType){
        case 1:
            dng_info.thumbWidth = lo_cfg.stride;
            dng_info.thumbHeight = lo_cfg.size.height;
            thumbnail_size = dng_info.thumbWidth * dng_info.thumbHeight;
            break;
        case 2:
            dng_info.thumbWidth = lo_cfg.size.width;
            dng_info.thumbHeight = lo_cfg.size.height;
            dng_info.thumbSamplesPerPixel = 3;
            dng_info.thumbPhotometric = PHOTOMETRIC_RGB;
            thumbnail_size = lo_cfg.stride*3*lo_cfg.size.height;
            break;
    }

    // buffer_size calculation
    const unsigned int dng_wrapper_size = 20000; // ~20kb, much smaller in practice.  
    const unsigned int frame_size = (cfg.size.width * cfg.size.height * dng_info.bits) / 8;
    // const unsigned int frame_size = (cfg.stride * cfg.size.height);
    const unsigned long bytes = frame_size + dng_wrapper_size + thumbnail_size;
    dng_info.buffer_size = ((bytes + ONE_MB - 1) / ONE_MB) * ONE_MB;

    // compression and other
    dng_info.compression = COMPRESSION_NONE;

    // extra metadata
    dng_info.make = "EQUNIOX V1";
    dng_info.model = "SONY IMX585-AAQJ1";
    dng_info.serial = getHwId();
    dng_info.software = "Libcamera;cinepi-raw";
    dng_info.ucm = "ALTCINE EQUNIOX";

    // adjust disk_buffer
    const double MAX_RAM_FRACTION = 2.0 / 3.0;
    std::ifstream meminfo("/proc/meminfo");
    std::string content((std::istreambuf_iterator<char>(meminfo)), std::istreambuf_iterator<char>());

    std::regex memAvailRegex(R"(MemAvailable:\s+(\d+)\s+kB)");
    std::smatch match;

    size_t totalRam = 0;
    if (std::regex_search(content, match, memAvailRegex)) {
        totalRam = std::stoull(match[1]) * 1024;  // Convert kilobytes to bytes
    }
    max_buffer_frames = (MAX_RAM_FRACTION * totalRam) / dng_info.buffer_size;

    console->debug("Max Frames in Buffer: {}", max_buffer_frames);

    encoder_initialized_ = true;
    console->info("DngEncoder is setup!");
}


void encode_rational_array(const float *src, int count, int32_t *dst, int32_t scale = 10000)
{
    for (int i = 0; i < count; ++i)
    {
        float val = src[i];
        if (!std::isfinite(val) || scale == 0)
        {
            dst[2 * i + 0] = 0;
            dst[2 * i + 1] = 1;
        }
        else
        {
            dst[2 * i + 0] = static_cast<int32_t>(std::round(val * scale));
            dst[2 * i + 1] = scale;
        }
    }
}

size_t DngEncoder::dng_save(int              thread_num,
                            const uint8_t   *mem_buf,
                            const uint8_t   *raw,
                            const StreamInfo &info,
                            const uint8_t   *lomem,
                            const StreamInfo &loinfo,
                            size_t           losize,
                            const libcamera::ControlList &metadata,
                            uint64_t         fn)
{
    /* ------------------------------------------------------------ *
    * 1.  Set-up memory writer & TIFF header                       *
    * ------------------------------------------------------------ */
    MemoryBuffer memBuf{};
    memBuf.buffer    = const_cast<uint8_t *>(mem_buf);
    memBuf.offset    = 0;
    memBuf.usedSize  = 0;
    memBuf.totalSize = dng_info.buffer_size;

    write_pod (memBuf, "II", 2);      // little-endian
    write_uint16(memBuf, 42);         // TIFF magic
    write_uint32(memBuf, 0);          // first-IFD offset (patched later)

    /* ------------------------------------------------------------ *
    * 2.  Write thumbnail pixels                                   *
    * ------------------------------------------------------------ */
    const uint32_t thumbOffset   = memBuf.offset;
    const uint32_t thumbRowBytes = dng_info.thumbWidth * dng_info.thumbSamplesPerPixel;
    const uint32_t thumbSize     = thumbRowBytes * dng_info.thumbHeight;

    for (uint32_t y = 0; y < dng_info.thumbHeight; ++y)
        write_pod(memBuf, lomem + y * loinfo.stride, thumbRowBytes);

    /* ------------------------------------------------------------ *
    * 3.  Write full-resolution RAW pixels                         *
    * ------------------------------------------------------------ */
    const uint32_t rawOffset   = memBuf.offset;
    const uint32_t rawRowBytes = (dng_info.bits == 12)
                            ? info.width * 2
                            : (info.width * dng_info.bits + 7) / 8;
    const uint32_t rawSize     = rawRowBytes * info.height;

    for (uint32_t y = 0; y < info.height; ++y)
        write_pod(memBuf, raw + y * info.stride, rawRowBytes);

    /* ------------------------------------------------------------ *
    * 4.  Build SubIFD[0]  →  THUMBNAIL                            *
    * ------------------------------------------------------------ */
    IFDBuilder thumbIFD(dng_info.thumbWidth, dng_info.thumbHeight);
    thumbIFD.baseOffset = memBuf.usedSize;

    /* geometry / opcode tags (thumbnail) ------------------------- */
    {
        uint32_t activeArea[4] =
            { dng_info.offset_y_start,
            dng_info.offset_x_start,
            dng_info.offset_y_start + dng_info.thumbHeight,
            dng_info.offset_x_start + dng_info.thumbWidth };
        uint32_t cropOrigin[2] = { 0, 0 };
        uint32_t cropSize [2]  = { dng_info.thumbWidth, dng_info.thumbHeight };
        uint8_t  opcodeDummy   = 0;

        thumbIFD.addEntry(0xC68E, TIFF_LONG, 4, activeArea);
        thumbIFD.addEntry(0xC68C, TIFF_LONG, 2, cropOrigin);
        thumbIFD.addEntry(0xC68D, TIFF_LONG, 2, cropSize);
        thumbIFD.addEntry(0xC68B, TIFF_BYTE, 0, &opcodeDummy);
    }

    uint32_t subfileTypeThumb = 1;          // reduced-res / preview
    uint16_t thumbBps         = dng_info.thumbBitsPerSample;
    uint16_t thumbFormat      = SAMPLEFORMAT_UINT;
    uint16_t planar           = 1;          // chunky
    uint16_t tWidth  = static_cast<uint16_t>(dng_info.thumbWidth);
    uint16_t tHeight = static_cast<uint16_t>(dng_info.thumbHeight);
    uint16_t tRows   = static_cast<uint16_t>(dng_info.thumbHeight);

    thumbIFD.addEntry(254,  TIFF_LONG , 1, &subfileTypeThumb);
    thumbIFD.addEntry(256,  TIFF_SHORT, 1, &tWidth);
    thumbIFD.addEntry(257,  TIFF_SHORT, 1, &tHeight);
    thumbIFD.addEntry(258,  TIFF_SHORT, 1, &thumbBps);
    thumbIFD.addEntry(259,  TIFF_SHORT, 1, &dng_info.compression);
    thumbIFD.addEntry(262,  TIFF_SHORT, 1, &dng_info.thumbPhotometric);
    thumbIFD.addEntry(273,  TIFF_LONG , 1, &thumbOffset);
    thumbIFD.addEntry(278,  TIFF_SHORT, 1, &tRows);
    thumbIFD.addEntry(279,  TIFF_LONG , 1, &thumbSize);
    thumbIFD.addEntry(277,  TIFF_SHORT, 1, &dng_info.thumbSamplesPerPixel);
    thumbIFD.addEntry(284,  TIFF_SHORT, 1, &planar);          // PlanarConfiguration
    thumbIFD.addEntry(339,  TIFF_SHORT, 1, &thumbFormat);     // SampleFormat

    /* usual strings & DNG header tags */
    uint8_t dngVer [4] = { 1, 4, 0, 0 };
    uint8_t dngBack[4] = { 1, 4, 0, 0 };
    std::string make  = dng_info.make    + '\0';
    std::string model = dng_info.model   + '\0';
    std::string soft  = dng_info.software+ '\0';

    thumbIFD.addEntry(271, TIFF_ASCII, make .size(), make .data());
    thumbIFD.addEntry(272, TIFF_ASCII, model.size(), model.data());
    thumbIFD.addEntry(305, TIFF_ASCII, soft .size(), soft .data());
    thumbIFD.addEntry(0xC612, TIFF_BYTE, 4, dngVer);
    thumbIFD.addEntry(0xC613, TIFF_BYTE, 4, dngBack);
    thumbIFD.addEntry(0xC614, TIFF_ASCII, model.size(), model.data());   // UniqueCameraModel

    thumbIFD.sortEntries();
    thumbIFD.build(memBuf);
    const uint32_t thumbIFDOffset = thumbIFD.baseOffset;

    /* ------------------------------------------------------------ *
    * 5.  Build IFD-0  →  FULL-RES RAW                             *
    * ------------------------------------------------------------ */
    IFDBuilder ifd0(info.width, info.height);
    ifd0.baseOffset = memBuf.usedSize;

    /* ---------- Resolve geometry tags ---------- */
    {
        uint32_t activeArea[4] =
            { dng_info.offset_y_start,
            dng_info.offset_x_start,
            dng_info.offset_y_start + info.height,
            dng_info.offset_x_start + info.width };

        uint32_t cropOrigin[2] = { 0, 0 };
        uint32_t cropSize  [2] = { info.width, info.height };
        uint8_t  opcodeDummy   = 0;

        ifd0.addEntry(0xC68E, TIFF_LONG, 4, activeArea);
        ifd0.addEntry(0xC68C, TIFF_LONG, 2, cropOrigin);
        ifd0.addEntry(0xC68D, TIFF_LONG, 2, cropSize);
        ifd0.addEntry(0xC68B, TIFF_BYTE, 0, &opcodeDummy);
    }

    /* image basics ------------------------------------------------ */
    uint16_t sampleFormat = SAMPLEFORMAT_UINT;
    ifd0.addEntry(256, TIFF_LONG , 1, &info.width);
    ifd0.addEntry(257, TIFF_LONG , 1, &info.height);
    ifd0.addEntry(258, TIFF_SHORT, 1, &dng_info.bits);
    ifd0.addEntry(259, TIFF_SHORT, 1, &dng_info.compression);
    ifd0.addEntry(273, TIFF_LONG , 1, &rawOffset);
    ifd0.addEntry(278, TIFF_LONG , 1, &info.height);
    ifd0.addEntry(279, TIFF_LONG , 1, &rawSize);
    ifd0.addEntry(277, TIFF_SHORT, 1, &dng_info.samples_per_pixel);
    ifd0.addEntry(339, TIFF_SHORT, 1, &sampleFormat);

    /* DNG header -------------------------------------------------- */
    ifd0.addEntry(0xC612, TIFF_BYTE, 4, dngVer);
    ifd0.addEntry(0xC613, TIFF_BYTE, 4, dngBack);

    /* black & white levels --------------------------------------- */
    int32_t blackRat[8];
    if (mono_) {
        blackRat[0] = static_cast<int32_t>(dng_info.black);
        blackRat[1] = 1;
        ifd0.addEntry(0xC61A, TIFF_RATIONAL, 1, blackRat);
    } else {
        for (int i = 0; i < 4; ++i) { blackRat[2*i] = dng_info.black; blackRat[2*i+1] = 1; }
        ifd0.addEntry(0xC61A, TIFF_RATIONAL, 4, blackRat);
    }
    uint16_t whiteLevel = static_cast<uint16_t>(dng_info.white);
    ifd0.addEntry(0xC61D, TIFF_SHORT, 1, &whiteLevel);

    /* profile & colour stuff ------------------------------------- */
    int32_t matrix1[18];  encode_rational_array(dng_info.CAM_XYZ, 9, matrix1);
    int32_t neutral[6];   encode_rational_array(dng_info.NEUTRAL, 3, neutral);
    int32_t analog [6];   encode_rational_array(dng_info.ANALOGBALANCE, 3, analog);

    ifd0.addEntry(0xC621, TIFF_SRATIONAL, 9, matrix1);     // ColorMatrix1
    ifd0.addEntry(0xC622, TIFF_SRATIONAL, 9, matrix1);     // ColorMatrix2

    uint16_t illum = 21, illum2 = 21;                      // both D65
    ifd0.addEntry(0xC65A, TIFF_SHORT, 1, &illum);          // CalibrationIlluminant1
    ifd0.addEntry(0xC65B, TIFF_SHORT, 1, &illum2);         // CalibrationIlluminant2

    ifd0.addEntry(0xC628, TIFF_RATIONAL , 3, neutral);     // AsShotNeutral
    ifd0.addEntry(0xC627, TIFF_RATIONAL , 3, analog);      // AnalogBalance

    /* CFA / mono handling ---------------------------------------- */
    uint32_t subfileType0 = 0;                             // full-res
    ifd0.addEntry(254, TIFF_LONG, 1, &subfileType0);

    uint16_t photometric = PHOTOMETRIC_CFA;
    ifd0.addEntry(262, TIFF_SHORT, 1, &photometric);

    if (mono_) {
        uint16_t repMono[2] = { 1, 1 }; uint8_t cfaMono[1] = { 0 };
        ifd0.addEntry(0x828D, TIFF_SHORT, 2, repMono);
        ifd0.addEntry(0x828E, TIFF_BYTE , 1, cfaMono);
        ifd0.addEntry(0xC619, TIFF_SHORT, 2, repMono);
    } else {
        ifd0.addEntry(0xC619, TIFF_SHORT, 2, dng_info.black_level_repeat_dim);
        ifd0.addEntry(0x828D, TIFF_SHORT, 2, dng_info.cfa_repeat_pattern_dim);
        ifd0.addEntry(0x828E, TIFF_BYTE , 4, dng_info.bayer_order);
    }

    /* frame rate -------------------------------------------------- */
    static int32_t frameRate[2];
    frameRate[0] = static_cast<int32_t>(*options_->framerate * 1000);
    frameRate[1] = 1000;
    ifd0.addEntry(0xC764, TIFF_SRATIONAL, 1, frameRate);

    /* camera strings --------------------------------------------- */
    ifd0.addEntry(271, TIFF_ASCII, make .size(), make .data());
    ifd0.addEntry(272, TIFF_ASCII, model.size(), model.data());
    ifd0.addEntry(305, TIFF_ASCII, soft .size(), soft .data());

    /* link thumbnail as SubIFD ----------------------------------- */
    ifd0.addEntry(0x014A, TIFF_LONG, 1, &thumbIFDOffset);  // SubIFDs[0]

    ifd0.sortEntries();
    ifd0.build(memBuf);
    const uint32_t ifd0Offset = ifd0.baseOffset;

    /* ------------------------------------------------------------ *
    * 6.  Patch TIFF header & return size                          *
    * ------------------------------------------------------------ */
    *reinterpret_cast<uint32_t *>(memBuf.buffer + 4) = ifd0Offset;
    return memBuf.usedSize;

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