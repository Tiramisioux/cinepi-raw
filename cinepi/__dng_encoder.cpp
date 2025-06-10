/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * Based on mjpeg_encoder.cpp, modifications by Csaba Nagy & Will Whang 
 *
 * dng_encoder.cpp - dng video encoder.
 */

#include <chrono>
#include <iostream>
#include <libcamera/control_ids.h>
#include <libcamera/formats.h>

#include <tiffio.h>
#include <tiffio.hxx>
#include <sstream>
#include <fstream>
#include <regex>

#include "core/still_options.hpp"
#include "core/stream_info.hpp"

#include "dng_encoder.hpp"
#include <arm_neon.h>
#include "utils.hpp"

// extern "C" {
// #include "lj92.h"
// }

#include "yuv2rgb.hpp"

#include <filesystem>
namespace fs = std::filesystem;

#define ONE_MB 1048576
#define BLOCK_SIZE 4096*16

using namespace libcamera;


static char TIFF_RGGB[4] = { 0, 1, 1, 2 };
static char TIFF_GRBG[4] = { 1, 0, 2, 1 };
static char TIFF_BGGR[4] = { 2, 1, 1, 0 };
static char TIFF_GBRG[4] = { 1, 2, 0, 1 };

struct BayerFormat
{
	char const *name;
	int bits;
	char const *order;
	bool packed;
	bool compressed;
};

// CinemaDNG Tags
ttag_t TIFFTAG_FRAMERATE =  0xC764;
ttag_t TIFFTAG_TIMECODE = 0xC763;
ttag_t TIFFTAG_CAMERALABEL = 0xC7A1;
ttag_t TIFFTAG_REELNAME = 0xC789;
ttag_t TIFFTAG_TSTOP = 0xC772;
char frameRateStr[] = "FrameRate";
char timeCodeStr[] = "TimeCodes";
char cameraLabelStr[] = "CameraLabel";
char reelNameStr[] = "ReelName";
char tStopStr[] = "TStop";
static const TIFFFieldInfo xtiffFieldInfo[] = {
    { TIFFTAG_FRAMERATE, 1, 1, TIFF_SRATIONAL,   FIELD_CUSTOM,
      true, false,  frameRateStr },
    { TIFFTAG_TSTOP, 1, 1, TIFF_RATIONAL,   FIELD_CUSTOM,
      true, false,  tStopStr },
    { TIFFTAG_TIMECODE, 8, 8, TIFF_BYTE,    FIELD_CUSTOM,
      true, false,  timeCodeStr },
    { TIFFTAG_CAMERALABEL, TIFF_VARIABLE, TIFF_VARIABLE, TIFF_ASCII,      FIELD_CUSTOM, 
      true, false, cameraLabelStr },
    { TIFFTAG_REELNAME, TIFF_VARIABLE, TIFF_VARIABLE, TIFF_ASCII,      FIELD_CUSTOM, 
      true, false, reelNameStr }
};


static const std::map<PixelFormat, BayerFormat> bayer_formats =
{
	{ formats::SRGGB10_CSI2P, { "RGGB-10", 10, TIFF_RGGB, true, false } },
	{ formats::SGRBG10_CSI2P, { "GRBG-10", 10, TIFF_GRBG, true, false } },
	{ formats::SBGGR10_CSI2P, { "BGGR-10", 10, TIFF_BGGR, true, false } },
	{ formats::SGBRG10_CSI2P, { "GBRG-10", 10, TIFF_GBRG, true, false } },

	{ formats::SRGGB10, { "RGGB-10", 10, TIFF_RGGB, false, false } },
	{ formats::SGRBG10, { "GRBG-10", 10, TIFF_GRBG, false, false } },
	{ formats::SBGGR10, { "BGGR-10", 10, TIFF_BGGR, false, false } },
	{ formats::SGBRG10, { "GBRG-10", 10, TIFF_GBRG, false, false } },

	{ formats::SRGGB12_CSI2P, { "RGGB-12", 12, TIFF_RGGB, true, false } },
	{ formats::SGRBG12_CSI2P, { "GRBG-12", 12, TIFF_GRBG, true, false } },
	{ formats::SBGGR12_CSI2P, { "BGGR-12", 12, TIFF_BGGR, true, false } },
	{ formats::SGBRG12_CSI2P, { "GBRG-12", 12, TIFF_GBRG, true, false } },

	{ formats::SRGGB12, { "RGGB-12", 12, TIFF_RGGB, false, false } },
	{ formats::SGRBG12, { "GRBG-12", 12, TIFF_GRBG, false, false } },
	{ formats::SBGGR12, { "BGGR-12", 12, TIFF_BGGR, false, false } },
	{ formats::SGBRG12, { "GBRG-12", 12, TIFF_GBRG, false, false } },

	{ formats::SRGGB16, { "RGGB-16", 16, TIFF_RGGB, false, false } },
	{ formats::SGRBG16, { "GRBG-16", 16, TIFF_GRBG, false, false } },
	{ formats::SBGGR16, { "BGGR-16", 16, TIFF_BGGR, false, false } },
	{ formats::SGBRG16, { "GBRG-16", 16, TIFF_GBRG, false, false } },

	{ formats::R10_CSI2P, { "BGGR-10", 10, TIFF_BGGR, true, false } },
	{ formats::R10, { "BGGR-10", 10, TIFF_BGGR, false, false } },
	// Currently not in the main libcamera branch
	//{ formats::R12_CSI2P, { "BGGR-12", 12, TIFF_BGGR, true } },
	{ formats::R12, { "BGGR-12", 12, TIFF_BGGR, false, false } },

	/* PiSP compressed formats. */
	{ formats::RGGB_PISP_COMP1, { "RGGB-16-PISP", 16, TIFF_RGGB, false, true } },
	{ formats::GRBG_PISP_COMP1, { "GRBG-16-PISP", 16, TIFF_GRBG, false, true } },
	{ formats::GBRG_PISP_COMP1, { "GBRG-16-PISP", 16, TIFF_GBRG, false, true } },
	{ formats::BGGR _PISP_COMP1, { "BGGR-16-PISP", 16, TIFF_BGGR, false, true } },
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

#include <arm_neon.h>
#include <stddef.h>
#define ARCH_K8


void pack_16bit_to_12bit(const uint16_t* input, uint8_t* output, int length) {
    for (int i = 0; i < length; i += 16) {
        // Load 16 16-bit elements
        uint16x8x2_t data = vld2q_u16(input + i);
        uint8x8x3_t result;
        result.val[0] = vmovn_u16(vshrq_n_u16(data.val[0], 8)); 
        result.val[1] = vmovn_u16(data.val[0]) + vmovn_u16(vshrq_n_u16(data.val[1], 12));
        result.val[2] = vmovn_u16(vshrq_n_u16(data.val[1],4));
        // Store the 16 12-bit elements 24 bytes packed data
        vst3_u8(output + (i * 3) / 2, result);
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
    // Check the Bayer format
    auto it = bayer_formats.find(cfg.pixelFormat);
    if (it == bayer_formats.end())
        throw std::runtime_error("unsupported Bayer format");
    BayerFormat const &bayer_format = it->second;
    console->debug("Bayer format is {}", bayer_format.name);

    dng_info.bits = bayer_format.bits;
    dng_info.bits = 12;

    // white level
    dng_info.white = (1 << dng_info.bits) - 1;

    // black_level configuartion
    dng_info.black = 4096 * (1 << dng_info.bits) / 65536.0;
    std::fill(std::begin(dng_info.black_levels), std::end(dng_info.black_levels), dng_info.black);

    auto bl = metadata.get(controls::SensorBlackLevels);
    if (bl)
    {
        // levels is in the order R, Gr, Gb, B. Re-order it for the actual bayer order.
        for (int i = 0; i < 4; i++)
        {
            int j = bayer_format.order[i];
            j = j == 0 ? 0 : (j == 2 ? 3 : 1 + !!bayer_format.order[i ^ 1]);
            dng_info.black_levels[j] = (*bl)[i] * (1 << dng_info.bits) / 65536.0;
        }
    }
    else
        console->error("WARNING: no black level found, using default");

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


        LOG(2, "Cam_XYZ default: ");
        LOG(2, CAM_XYZ.m[0] << " " << CAM_XYZ.m[1] << " " << CAM_XYZ.m[2]);
        LOG(2, CAM_XYZ.m[3] << " " << CAM_XYZ.m[4] << " " << CAM_XYZ.m[5]);
        LOG(2, CAM_XYZ.m[6] << " " << CAM_XYZ.m[7] << " " << CAM_XYZ.m[8]);


    // cfa
    dng_info.cfa_repeat_pattern_dim[0] = 2;
    dng_info.cfa_repeat_pattern_dim[1] = 2;
    dng_info.black_level_repeat_dim[0] = 2;
    dng_info.black_level_repeat_dim[1] = 2;
    memcpy(dng_info.bayer_order,bayer_format.order,4);

    dng_info.offset_y_start = 0;
    dng_info.offset_y_end = 0;
    dng_info.offset_x_start = 0;
    dng_info.offset_x_end = 0;

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
    dng_info.make = "Raspberry Pi";
    dng_info.model = options_->model;
    dng_info.serial = getHwId();
    dng_info.software = "Libcamera;cinepi-raw";
    dng_info.ucm = options_->ucm.value_or("CinePI");

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

#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

typedef struct {
    unsigned char* buffer;  // In-memory buffer
    toff_t offset;          // Current offset
    toff_t usedSize;        // Track the maximum offset written to
    toff_t totalSize;       // Total size of the buffer
} MemoryBuffer;


tsize_t myTIFFReadProc(thandle_t fd, tdata_t buf, tsize_t size) {
    MemoryBuffer* memBuf = (MemoryBuffer*) fd;
    if (memBuf->offset + size > memBuf->usedSize) {
        size = memBuf->usedSize - memBuf->offset;
    }
    memcpy(buf, memBuf->buffer + memBuf->offset, size);
    memBuf->offset += size;
    return size;
}

tsize_t myTIFFWriteProc(thandle_t fd, tdata_t buf, tsize_t size) {
    MemoryBuffer* memBuf = (MemoryBuffer*) fd;
    if (memBuf->offset + size > memBuf->totalSize) {
        // If writing the data would overflow the buffer, adjust the size.
        size = memBuf->totalSize - memBuf->offset;
    }
    memcpy(memBuf->buffer + memBuf->offset, buf, size);
    memBuf->offset += size;
    if (memBuf->offset > memBuf->usedSize) {
        memBuf->usedSize = memBuf->offset;
    }
    return size;
}

toff_t myTIFFSeekProc(thandle_t fd, toff_t off, int whence) {
    MemoryBuffer* memBuf = (MemoryBuffer*) fd;
    switch (whence) {
        case SEEK_SET:
            memBuf->offset = off;
            break;
        case SEEK_CUR:
            memBuf->offset += off;
            break;
        case SEEK_END:
            memBuf->offset = memBuf->usedSize + off;
            break;
    }
    return memBuf->offset;
}

toff_t myTIFFSizeProc(thandle_t fd) {
    MemoryBuffer* memBuf = (MemoryBuffer*) fd;
    return memBuf->usedSize;
}

int myTIFFCloseProc(thandle_t fd) {
    // Nothing special to do in the memory buffer context.
    // Memory will be freed after writing to disk, outside of the libTIFF context.
    return 0;
}

#include <math.h>
#include <float.h>
typedef struct {
    unsigned short endian; // Endianness indicator
    unsigned short magic;  // Magic number (42)
    unsigned int offset;   // Offset to the first IFD
} TIFFHeader;

typedef struct {
    unsigned short tag;    // Tag identifier
    unsigned short type;   // Data type
    unsigned int count;    // Number of elements of the specified data type
    unsigned int value;   // elements
} IFDEntry;

int double_to_rational(double x, double *numerator, double *denominator) {
  if (!isfinite(x)) {
    *numerator = *denominator = 0.0;
    if (x > 0.0) *numerator = 1.0;
    if (x < 0.0) *numerator = -1.0;
    return 1;
  }
  int bdigits = DBL_MANT_DIG;
  int expo;
  *denominator = 1.0;
  *numerator = frexp(x, &expo) * pow(2.0, bdigits);
  expo -= bdigits;
  if (expo > 0) {
    *numerator *= pow(2.0, expo);
  }
  else if (expo < 0) {
    expo = -expo;
    if (expo >= DBL_MAX_EXP-1) {
      *numerator /= pow(2.0, expo - (DBL_MAX_EXP-1));
      *denominator *= pow(2.0, DBL_MAX_EXP-1);
      return fabs(*numerator) < 1.0;
    } else {
      *denominator *= pow(2.0, expo);
    }
  }

  while (*numerator && fmod(*numerator,2) == 0 && fmod(*denominator,2) == 0) {
    *numerator /= 2.0;
    *denominator /= 2.0;
  }
  return 0;
}



size_t DngEncoder::dng_save(int thread_num, uint8_t *mem_tiff, uint8_t const *mem, StreamInfo const &info, uint8_t const *lomem, StreamInfo const &loinfo, size_t losize,
              ControlList const &metadata, uint64_t fn)
{
    const DngInfo& constDngInfo = dng_info;

    // get raw unique value as sensor timestamp
    std::array<uint8_t, 8> rawUniq = {};
    if (auto rU = metadata.get(libcamera::controls::SensorTimestamp)) {
        std::copy_n(reinterpret_cast<uint8_t*>(&*rU), rawUniq.size(), rawUniq.begin());
    }

    uint32_t white = (1 << constDngInfo.bits) - 1;

    // get shutter speed
    auto exp = metadata.get(controls::ExposureTime);
    float exp_time = 10000;
    if (exp)
        exp_time = *exp;
    else
        console->error("WARNING: default to exposure time of {}us", exp_time);
    exp_time /= 1e6;

    // get iso
    auto ag = metadata.get(controls::AnalogueGain);
    uint16_t iso = 100;
    if (ag)
        iso = *ag * 100.0;
    else
        console->error("WARNING: default to ISO value of {}", iso);

    console->info("thrd: {} Writing DNG {}", thread_num, fn);
    try
    {
        std::ostringstream oss;
        oss << options_->mediaDest << '/' 
            << options_->folder << '/' 
            << options_->folder << '_'
            << std::setw(9) << std::setfill('0') << fn
            << ".dng";

        std::string filename = oss.str();

        time_t t;
        time(&t);
        struct tm *time_info = localtime(&t);

        // CCM Configuration
        float NEUTRAL[] = { 1, 1, 1 };
        Matrix WB_GAINS(1, 1, 1);
        auto cg = metadata.get(controls::ColourGains);
        if (cg)
        {
            NEUTRAL[0] = 1.0 / (*cg)[0];
            NEUTRAL[2] = 1.0 / (*cg)[1];
            WB_GAINS = Matrix((*cg)[0], 1, (*cg)[1]);
        }

        unsigned int AsShotNeutral[3][2];
        for (int i=0;i<3;i++){
            double tmp_numerator;
            double tmp_denominator;
            double_to_rational(NEUTRAL[i],&tmp_numerator,&tmp_denominator);
            AsShotNeutral[i][0] = tmp_numerator;
            AsShotNeutral[i][1] = tmp_denominator;
        }

        //LOG(1, "Neutral " << NEUTRAL[0] << " " << NEUTRAL[1] << " " << NEUTRAL[2]);
        
        // CCM from calibration json - Daylight
        Matrix CCM_Daylight(1.42939, -0.40792, -0.02147,
                            -0.27719, 1.46177, -0.18458,
                            0.08958, -0.63362, 1.54404);

        // This maxtrix from http://www.brucelindbloom.com/index.html?Eqn_RGB_XYZ_Matrix.html
        Matrix RGB2XYZ(0.4124564, 0.3575761, 0.1804375,
                       0.2126729, 0.7151522, 0.0721750,
                       0.0193339, 0.1191920, 0.9503041);

        Matrix CAM_XYZ_Daylight = (RGB2XYZ * CCM_Daylight * WB_GAINS).Inv();

        int ColorMatrix1[9][2];
        //LOG(2, "Cam_XYZ: ");
        //LOG(2, CAM_XYZ_Daylight.m[0] << " " << CAM_XYZ_Daylight.m[1] << " " << CAM_XYZ_Daylight.m[2]);
        //LOG(2, CAM_XYZ_Daylight.m[3] << " " << CAM_XYZ_Daylight.m[4] << " " << CAM_XYZ_Daylight.m[5]);
        //LOG(2, CAM_XYZ_Daylight.m[6] << " " << CAM_XYZ_Daylight.m[7] << " " << CAM_XYZ_Daylight.m[8]);

        for (int i=0;i<9;i++){
            double tmp_numerator;
            double tmp_denominator;
            double_to_rational(CAM_XYZ_Daylight.m[i],&tmp_numerator,&tmp_denominator);
            ColorMatrix1[i][0] = tmp_numerator;
            ColorMatrix1[i][1] = tmp_denominator;
        }

        Matrix CCM_Tungsten(1.78809, -0.66968, -0.11841,
                            -0.67083, 1.69256, -0.02173,
                            -0.53371, 1.05827, 0.47545);

        Matrix CAM_XYZ_Tungsten  = (RGB2XYZ * CCM_Tungsten * WB_GAINS).Inv();
        int ColorMatrix2[9][2];

        for (int i=0;i<9;i++){
            double tmp_numerator;
            double tmp_denominator;
            double_to_rational(CAM_XYZ_Tungsten.m[i],&tmp_numerator,&tmp_denominator);
            ColorMatrix2[i][0] = tmp_numerator;
            ColorMatrix2[i][1] = tmp_denominator;
        }

        unsigned int BaselineExposure[2] = {1,1};
        unsigned int BlackLevel[4] = {0x32, 0x32, 0x32, 0x32};

        float black = 4096 * (1 << constDngInfo.bits) / 65536.0;
        auto bl = metadata.get(controls::SensorBlackLevels);
        if (bl)
        {
            // levels is in the order R, Gr, Gb, B. Re-order it for the actual bayer order.
            for (int i = 0; i < 4; i++)
            {
                int j = constDngInfo.bayer_order[i];
                j = j == 0 ? 0 : (j == 2 ? 3 : 1 + !!constDngInfo.bayer_order[i ^ 1]);
                BlackLevel[j] = (*bl)[i] * (1 << constDngInfo.bits) / 65536.0;
            }
        }
        else
            LOG_ERROR("WARNING: no black level found, using default");

        //LOG(2, "Black levels " << BlackLevel[0] << " " << BlackLevel[1] << " " << BlackLevel[2] << " "
        //                   << BlackLevel[3] << ", exposure time " << exp_time * 1e6 << "us, ISO " << iso);


        unsigned int ExposureTime[2];
        double tmp_numerator;
        double tmp_denominator;
        double_to_rational(exp_time,&tmp_numerator,&tmp_denominator);

        ExposureTime[0] = tmp_numerator;
        ExposureTime[1] = tmp_denominator;


        char UniqueCameraModel[] = {"CinePi"};
        char CameraSerialNumber[] = {"IMX585"};
        char DateTimeOriginal[20];
        strftime(DateTimeOriginal, 20, "%Y:%m:%d %H:%M:%S", time_info);


        //large arrays that needs offset will be after 512bytes
        unsigned int array_offset = 512;
        unsigned int BlackLevel_Offset = 512;
        array_offset += sizeof(BlackLevel);

        unsigned int ColorMatrix1_Offset = array_offset;
        array_offset += sizeof(ColorMatrix1);

        unsigned int ColorMatrix2_Offset = array_offset;
        array_offset += sizeof(ColorMatrix2);
        array_offset += array_offset % 2;

        unsigned int UniqueCameraModel_Offset = array_offset;
        array_offset += sizeof(UniqueCameraModel);
        array_offset += array_offset % 2;

        unsigned int CameraSerialNumber_Offset = array_offset;
        array_offset += sizeof(CameraSerialNumber);

        unsigned int DateTimeOriginal_Offset = array_offset;
        array_offset += sizeof(DateTimeOriginal);

        unsigned int AsShotNeutral_Offset = array_offset;
        array_offset += sizeof(AsShotNeutral);

        unsigned int BaselineExposure_Offset = array_offset;
        array_offset += sizeof(BaselineExposure);

        unsigned int ExposureTime_Offset = array_offset;
        array_offset += sizeof(ExposureTime);

        unsigned int imageDataOffset = 1024;

        IFDEntry entries[] = {
            {0x00FE, 4, 1, 0},               //Main image
            {0x0100, 4, 1, info.stride/2},
            {0x0101, 4, 1, constDngInfo.t_height},
            {0x0102, 3, 1, constDngInfo.bits},               // Assume 8 bits per sample for simplicity
            {0x0103, 3, 1, 1},               // No compression
            {0x0106, 3, 1, 32803},           // PHOTOMETRIC_CFA
            {0x0111, 4, 1, imageDataOffset}, // StripOffsets
            {0x0112, 3, 1, 1},               // Orientation - The 0th row represents the visual top of the image, and the 0th column represents the visual left-hand side.
            {0x0115, 3, 1, 1},               // SamplesPerPixel
            {0x0117, 4, 1, info.stride/2 * constDngInfo.t_height * constDngInfo.bits /8},  // StripByteCounts
            {0x011C, 3, 1, 1},               // PlanarConfiguration
            {0x0153, 3, 1, 1},               // SampleFormat
            {0x828D, 3, 2, 0x00020002},      // CFARepeatPatternDim
            {0x828E, 1, 4, 0x02010100},      // CFAPattern
            {0x829A, 5, 1, ExposureTime_Offset}, // ExposureTime [Offset]
            {0x8827, 3, 1, iso},                 // ISOSpeedRatings [Offset]
            {0x9003, 2, sizeof(DateTimeOriginal), DateTimeOriginal_Offset},      // DateTimeOriginal [Offset]
            {0xC612, 1, 4, 0x00000401},      // DNGVersion 
            {0xC613, 1, 4, 0x00000401},      // DNGBackwardVersion 
            {0xC614, 2, sizeof(UniqueCameraModel), UniqueCameraModel_Offset},     // UniqueCameraModel [Offset]
            //{0xC616, 1, 3, 0x00010200},            // CFAPlaneColor
            {0xC619, 3, 2, 0x00020002},            // BlackLevelRepeatDim
            {0xC61A, 4, 4, BlackLevel_Offset},     // BlackLevel
            {0xC61D, 3, 1, white},                 // WhiteLevel
            {0xC621, 10, 9, ColorMatrix1_Offset}, // ColorMatrix1 [Offset]
            {0xC622, 10, 9, ColorMatrix2_Offset}, // ColorMatrix2 [Offset]
            {0xC628, 5, 3, AsShotNeutral_Offset}, // AsShotNeutral [Offset]
            //{0xC62A, 10, 1, BaselineExposure_Offset},      // BaselineExposure 
            {0xC62F, 2, sizeof(CameraSerialNumber), CameraSerialNumber_Offset},     // UniqueCameraModel [Offset]
            {0xC65A, 3, 1, 1},                 // CalibrationIlluminant1 - 1 = 
            {0xC65B, 3, 1, 3},                // CalibrationIlluminant2 - 3 = Tungsten
        };

        uint8_t *buffer;
        if (posix_memalign((void **)&buffer, BLOCK_SIZE, dng_info.buffer_size) != 0) {
            perror("Error allocating aligned memory");
            return 0;
        }

        unsigned int offset = 0;

        //Write headers
        TIFFHeader header = {0x4949, 42, 8}; // 'II' for little endian, magic number, offset to first IFD
        memcpy(buffer,(void*)&header, sizeof(TIFFHeader));
        offset += sizeof(TIFFHeader);

        // Write number of entries
        unsigned short numEntries = sizeof(entries) / sizeof(IFDEntry);
        memcpy(buffer+offset,(void*)&numEntries, sizeof(unsigned short));
        offset += sizeof(unsigned short);

        // Write IFD entries
        for (int i = 0; i < numEntries; ++i) {
            //fwrite(&entries[i], sizeof(IFDEntry), 1, fp);
            memcpy(buffer+offset,(void*)&entries[i], sizeof(IFDEntry));
            offset += sizeof(IFDEntry);
        }

        // Write next IFD offset (0, indicating no more IFDs)
        unsigned int nextIFDOffset = 0;
        memcpy(buffer+offset,(void*)&nextIFDOffset, sizeof(unsigned int));
        offset += sizeof(unsigned int);

        //Copy all the large array to IFD
        memcpy(buffer+ColorMatrix1_Offset,(void*)ColorMatrix1, sizeof(ColorMatrix1));
        memcpy(buffer+ColorMatrix2_Offset,(void*)ColorMatrix2, sizeof(ColorMatrix2));
        memcpy(buffer+BlackLevel_Offset,(void*)BlackLevel, sizeof(BlackLevel));
        memcpy(buffer+UniqueCameraModel_Offset,(void*)UniqueCameraModel, sizeof(UniqueCameraModel));
        memcpy(buffer+CameraSerialNumber_Offset,(void*)CameraSerialNumber, sizeof(CameraSerialNumber));
        memcpy(buffer+DateTimeOriginal_Offset,(void*)DateTimeOriginal, sizeof(DateTimeOriginal));
        memcpy(buffer+AsShotNeutral_Offset,(void*)AsShotNeutral, sizeof(AsShotNeutral));
        memcpy(buffer+BaselineExposure_Offset,(void*)BaselineExposure, sizeof(BaselineExposure));
        memcpy(buffer+ExposureTime_Offset,(void*)ExposureTime, sizeof(ExposureTime));

        pack_16bit_to_12bit((uint16_t*)mem, (uint8_t*)buffer+1024, constDngInfo.t_height * (info.stride/2));

        console->info("thrd: {} Flushing DNG to Disk {}", thread_num, fn);

        int fd = open(filename.c_str(), O_WRONLY | O_CREAT | O_DIRECT, 0644);
        if (fd != -1) {

            // Provide sequential access hint
            posix_fadvise(fd, 0, 0, POSIX_FADV_SEQUENTIAL);
            posix_fadvise(fd, 0, 0, POSIX_FADV_NOREUSE);
            if(write(fd, buffer, dng_info.buffer_size) != dng_info.buffer_size) {
                perror("Error writing to file");
            }
            //ftruncate(fd, memBuf.usedSize);
            close(fd);
            free(buffer);

        } else {
            fprintf(stderr, "Failed to open file for writing\n");
        }

        console->info("thrd: {} Writing DNG Done {}", thread_num, fn);

        return dng_info.buffer_size;
    }
    catch (std::exception const &e)
    {
        throw;
    }
}


void DngEncoder::encodeThread(int num)
{
    std::chrono::duration<double> encode_time(0);
    EncodeItem encode_item;

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(2, &cpuset);
    CPU_SET(3, &cpuset);
    CPU_SET(0, &cpuset);
    CPU_SET(1, &cpuset);
    pthread_t current_thread = pthread_self();
    if(pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset)) {
        std::cerr << "Error setting thread affinity" << std::endl;
    }

    console->info("Encode Thread[{}]  Start", num);
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
                else{
                    encode_cond_var_.wait_for(lock, 500us);
                }
            }
        }

        frames_ = {encode_item.index};
        console->info("Thread[{}] encode frame: {}", num, encode_item.index);

        {   
            auto start_time = std::chrono::high_resolution_clock::now();
        
            size_t tiff_size = dng_save(num,NULL,(const uint8_t*)encode_item.mem, encode_item.info, (const uint8_t*)encode_item.lomem, encode_item.loinfo, encode_item.losize, encode_item.met, encode_item.index);
   
            auto end_time = std::chrono::high_resolution_clock::now();

            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
            console->info("Thread[{}] Frame {} Time taken for the encode: {} milliseconds, Size:{} {} Mb/s", num, encode_item.index, duration, tiff_size,  float(tiff_size)/duration/1024/1024*1000);
        }

        {
            input_done_callback_(nullptr);
            output_ready_callback_(encode_item.mem, encode_item.size, encode_item.timestamp_us, true);
        }       

    }
}


//Flushing data to disk
void DngEncoder::diskThread(int num)
{
    DiskItem disk_item;

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(0, &cpuset);
    CPU_SET(1, &cpuset);
    CPU_SET(2, &cpuset);
    CPU_SET(3, &cpuset);

    pthread_t current_thread = pthread_self();    
    if(pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset)) {
        std::cerr << "Error setting thread affinity" << std::endl;
    }

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
                else{
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
    
        console->trace("Thread[{}]  Save frame to disk: {}", num,  disk_item.index);
        
        auto start_time = std::chrono::high_resolution_clock::now();

        auto end_time = std::chrono::high_resolution_clock::now();

        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        console->info("Thread[{}] Frame {} Time taken for the disk io: {} milliseconds {} Mb/s", num, disk_item.index, duration, float(disk_item.size)/duration/1024/1024*1000);

    }
}