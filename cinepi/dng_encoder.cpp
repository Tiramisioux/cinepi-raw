/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * Based on mjpeg_encoder.cpp, modifcations by Csaba Nagy & Will Whang 
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


#include "core/still_options.hpp"
#include "core/stream_info.hpp"

#include "dng_encoder.hpp"
#include <arm_neon.h>
#include "lj92.h"
#include "utils.hpp"

#include "yuv2rgb.hpp"

#include <filesystem>
namespace fs = std::filesystem;

#define BLOCK_SIZE 4096 

using namespace libcamera;


static char TIFF_RGGB[4] = { 0, 1, 1, 2 };
static char TIFF_GRBG[4] = { 1, 0, 2, 1 };
static char TIFF_BGGR[4] = { 2, 1, 1, 0 };
static char TIFF_GBRG[4] = { 1, 2, 0, 1 };

struct BayerFormat
{
    const char *name;
    int bits;
    const char *order;
};

ttag_t TIFFTAG_FRAMERATE =  0xC764;
ttag_t TIFFTAG_TIMECODE = 0xC763;
char frameRateStr[] = "FrameRate";
char timeCodestr[] = "TimeCodes";
static const TIFFFieldInfo xtiffFieldInfo[] = {
    { TIFFTAG_FRAMERATE, 1, 1, TIFF_RATIONAL,   FIELD_CUSTOM,
      true, false,  frameRateStr },
    { TIFFTAG_TIMECODE, 8, 8, TIFF_BYTE,    FIELD_CUSTOM,
      true, false,  timeCodestr },
};


static const std::map<PixelFormat, BayerFormat> bayer_formats =
{
    { formats::SRGGB10_CSI2P, { "RGGB-10", 10, TIFF_RGGB } },
    { formats::SGRBG10_CSI2P, { "GRBG-10", 10, TIFF_GRBG } },
    { formats::SBGGR10_CSI2P, { "BGGR-10", 10, TIFF_BGGR } },
    { formats::SGBRG10_CSI2P, { "GBRG-10", 10, TIFF_GBRG } },
    { formats::SRGGB12_CSI2P, { "RGGB-12", 12, TIFF_RGGB } },
    { formats::SGRBG12_CSI2P, { "GRBG-12", 12, TIFF_GRBG } },
    { formats::SBGGR12_CSI2P, { "BGGR-12", 12, TIFF_BGGR } },
    { formats::SGBRG12_CSI2P, { "GBRG-12", 12, TIFF_GBRG } },
    { formats::SRGGB12, { "RGGB-12", 12, TIFF_RGGB } },
    { formats::SRGGB10, { "RGGB-10", 10, TIFF_RGGB } },
    { formats::SBGGR12, { "BGGR-12", 12, TIFF_BGGR } },
    { formats::SBGGR10, { "BGGR-10", 10, TIFF_BGGR } },
};

extern __attribute__((noinline, section("disasm"))) void unpack10_64(uint64_t *read, uint64_t *buffer, const uint16_t chunk){
    for (uint64_t* write64 = buffer; write64 < buffer + chunk; write64 += 5, read += 5){

        uint64_t a = *(read);
        uint64_t b = *(read + 1);
        uint64_t c = *(read + 2);
        uint64_t d = *(read + 3);
        uint64_t e = *(read + 4);

        *(write64 + 0) = ((a >> 0) & 0x00FF0000FF0000FF) | ((a >>  4) & 0x0F000FFF000FFF00) | ((a << 12) & 0x0000F00000F00000) | (((b << 28) & 0xF0000000) << 32);
        *(write64 + 1) = ((b >> 0) & 0xFF0000FF0000FF00) | ((b >>  4) & 0x000FFF000FFF000F) | ((b << 12) & 0x00F00000F0000000) | (((a >> 56) & 0x0F) << 4);
        *(write64 + 2) = ((c >> 0) & 0x0000FF0000FF0000) | ((c >>  4) & 0x0FFF000FFF000FFF) | ((c << 12) & 0xF00000000000F000) | (((c << 28) & 0xF0000000) >> 32) | (((c >> 20) & 0x000000F0) << 32);
        *(write64 + 3) = ((d >> 0) & 0x0000FF0000FF0000) | ((c >>  4) & 0x0FFF000FFF000FFF) | ((c << 12) & 0xF00000000000F000) | (((c << 28) & 0xF0000000) >> 32) | (((c >> 20) & 0x000000F0) << 32);
        *(write64 + 4) = ((e >> 0) & 0x0000FF0000FF0000) | ((c >>  4) & 0x0FFF000FFF000FFF) | ((c << 12) & 0xF00000000000F000) | (((c << 28) & 0xF0000000) >> 32) | (((c >> 20) & 0x000000F0) << 32);
    }
}


extern __attribute__((noinline, section("disasm"))) void unpack12_64(uint64_t *read, uint64_t *buffer, const uint16_t chunk){
    for (uint64_t* write64 = buffer; write64 < buffer + chunk; write64 += 3, read+=3){

        uint64_t a = *(read);
        uint64_t b = *(read + 1);
        uint64_t c = *(read + 2);

        *(write64 + 0) = ((a >> 0) & 0x00FF0000FF0000FF) | ((a >>  4) & 0x0F000FFF000FFF00) | ((a << 12) & 0x0000F00000F00000) | (((b << 28) & 0xF0000000) << 32);
        *(write64 + 1) = ((b >> 0) & 0xFF0000FF0000FF00) | ((b >>  4) & 0x000FFF000FFF000F) | ((b << 12) & 0x00F00000F0000000) | (((a >> 56) & 0x0F) << 4);
        *(write64 + 2) = ((c >> 0) & 0x0000FF0000FF0000) | ((c >>  4) & 0x0FFF000FFF000FFF) | ((c << 12) & 0xF00000000000F000) | (((c << 28) & 0xF0000000) >> 32) | (((c >> 20) & 0x000000F0) << 32);
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
      compressed(false), // Default value, adjust as needed
      still_capture(false), // Default value, adjust as needed
      encodeCheck_(false), // Default value, adjust as needed
      abortEncode_(false), // Default value, adjust as needed
      abortOutput_(false), // Default value, adjust as needed
      resetCount_(false), // Default value, adjust as needed
      index_(0), // Default value, adjust as needed
      frames_(0), // Default value, adjust as needed
      frameStop_(0), // Default value, adjust as needed
      options_(options),
      disk_buffer_(448)
{
    for (int i = 0; i < 2; i++){
        encode_thread_[i] = std::thread(std::bind(&DngEncoder::encodeThread, this, i));
    }
    for (int i = 0; i < 10; i++){
        disk_thread_[i] = std::thread(std::bind(&DngEncoder::diskThread, this, i));
    }

    LOG(2, "Opened DngEncoder");
}

DngEncoder::~DngEncoder()
{
    abortEncode_ = true;
    for (int i = 0; i < 2; i++){
        encode_thread_[i].join();
    }
    for (int i = 0; i < 10; i++){
        disk_thread_[i].join();
    }

    abortOutput_ = true;
    LOG(2, "DngEncoder closed");
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


#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#define BUFFER_SIZE 9 * 1024 * 1024  // 16MB

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


size_t DngEncoder::dng_save(int thread_num,uint8_t const *mem_tiff, uint8_t const *mem, StreamInfo const &info, uint8_t const *lomem, StreamInfo const &loinfo, size_t losize,
              ControlList const &metadata, std::string const &filename,
              std::string const &cam_name, RawOptions const *options, uint64_t fn)
{
    
    uint8_t rawUniq[8]; 
    memset(rawUniq, 0, sizeof(rawUniq));
    auto rU = metadata.get(libcamera::controls::SensorTimestamp);
    if(rU){
        memcpy(rawUniq, (uint8_t*)&(*rU), sizeof(rawUniq));
    }

    // Check the Bayer format
    auto it = bayer_formats.find(info.pixel_format);
    if (it == bayer_formats.end())
        throw std::runtime_error("unsupported Bayer format");
    BayerFormat const &bayer_format = it->second;
    LOG(2, "Bayer format is " << bayer_format.name);

    // We need to fish out some metadata values for the DNG.
    float black = 4096 * (1 << bayer_format.bits) / 65536.0;
    float black_levels[] = { black, black, black, black };
    auto bl = metadata.get(controls::SensorBlackLevels);
    if (bl)
    {
        // levels is in the order R, Gr, Gb, B. Re-order it for the actual bayer order.
        for (int i = 0; i < 4; i++)
        {
            int j = bayer_format.order[i];
            j = j == 0 ? 0 : (j == 2 ? 3 : 1 + !!bayer_format.order[i ^ 1]);
            black_levels[j] = (*bl)[i] * (1 << bayer_format.bits) / 65536.0;
        }
    }
    else
        LOG_ERROR("WARNING: no black level found, using default");

    auto exp = metadata.get(controls::ExposureTime);
    float exp_time = 10000;
    if (exp)
        exp_time = *exp;
    else
        LOG_ERROR("WARNING: default to exposure time of " << exp_time << "us");
    exp_time /= 1e6;

    auto ag = metadata.get(controls::AnalogueGain);
    uint16_t iso = 100;
    if (ag)
        iso = *ag * 100.0;
    else
        LOG_ERROR("WARNING: default to ISO value of " << iso);

    float NEUTRAL[] = { 1, 1, 1 };
    float ANALOGBALANCE[] = { 1, 1, 1 };
    Matrix WB_GAINS(1, 1, 1);
    auto cg = metadata.get(controls::ColourGains);
    if (cg)
    {
        NEUTRAL[0] = 1.0 / (*cg)[0];
        NEUTRAL[2] = 1.0 / (*cg)[1];
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
        LOG_ERROR("WARNING: no CCM metadata found");

    // This maxtrix from http://www.brucelindbloom.com/index.html?Eqn_RGB_XYZ_Matrix.html
    Matrix RGB2XYZ(0.4124564, 0.3575761, 0.1804375,
                   0.2126729, 0.7151522, 0.0721750,
                   0.0193339, 0.1191920, 0.9503041);
    Matrix CAM_XYZ = (RGB2XYZ * CCM * WB_GAINS).Inv();

    // Finally write the DNG.
    const uint16_t offset_y_start = options_->rawCrop[0];
    const uint16_t offset_y_end = options_->rawCrop[1];
    const uint16_t offset_x_start = options_->rawCrop[2];
    const uint16_t offset_x_end = options_->rawCrop[3];

    // LOG(1, offset_x_start << " " << offset_x_end << " " << offset_y_start << " " << offset_y_end);

    const uint16_t byte_offset_x = (1.5 * offset_x_start) / sizeof(uint64_t); 
    // const uint16_t read_length_x = (1.5 * (info.width - (offset_x_start+offset_x_end))) / sizeof(uint64_t);
    const uint16_t t_height = (info.height - (offset_y_start+offset_y_end));
    const uint16_t t_width = (info.width - (offset_x_start+offset_x_end));

    TIFF *tif = nullptr;
    MemoryBuffer memBuf;
    memBuf.buffer = (unsigned char*)mem_tiff;
    if (!memBuf.buffer) {
        fprintf(stderr, "Failed to allocate memory\n");
        exit(1);
    }
    memBuf.offset = 0;
    memBuf.usedSize = 0;
    memBuf.totalSize = BUFFER_SIZE;

    LOG(2, thread_num << " Writing DNG " << fn);
    try
    {

        const short cfa_repeat_pattern_dim[] = { 2, 2 };
        uint32_t white = (1 << bayer_format.bits) - 1;
        toff_t offset_subifd = 0, offset_exififd = 0;


        tif = TIFFClientOpen("memory", "w", (thandle_t)&memBuf,
                                   myTIFFReadProc, myTIFFWriteProc,
                                   myTIFFSeekProc, myTIFFCloseProc,
                                   myTIFFSizeProc, NULL, NULL);

        if (!tif)
            throw std::runtime_error("could not open file " + fn);
        
        
        uint16_t thumbWidth = 32;
        uint16_t thumbHeight = 32;
        uint16_t thumbPhotometric = PHOTOMETRIC_MINISBLACK;
        uint16_t thumbBitsPerSample = 8;
        uint16_t thumbSamplesPerPixel = 1;

        if(options_->thumbnail == 1){
            thumbWidth = loinfo.stride;
            thumbHeight = loinfo.height;
        } else if(options_->thumbnail == 2){
            thumbWidth = loinfo.width;
            thumbHeight = loinfo.height;
            thumbSamplesPerPixel = 3;
            thumbPhotometric = PHOTOMETRIC_RGB;
        }

        LOG(2, thread_num << " Writing DNG thumbnail" << fn);

        TIFFSetField(tif, TIFFTAG_SUBFILETYPE, 1);
        TIFFSetField(tif, TIFFTAG_IMAGEWIDTH, thumbWidth);
        TIFFSetField(tif, TIFFTAG_IMAGELENGTH, thumbHeight);
        TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE, thumbBitsPerSample);
        TIFFSetField(tif, TIFFTAG_COMPRESSION, COMPRESSION_NONE);
        TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, thumbPhotometric );
        TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, thumbSamplesPerPixel);
        TIFFSetField(tif, TIFFTAG_MAKE, "Raspberry Pi");
        TIFFSetField(tif, TIFFTAG_MODEL, options_->sensor.c_str());
        TIFFSetField(tif, TIFFTAG_DNGVERSION, "\001\004\000\000");
        TIFFSetField(tif, TIFFTAG_DNGBACKWARDVERSION, "\001\001\000\000");
        TIFFSetField(tif, TIFFTAG_UNIQUECAMERAMODEL, options_->serial.c_str());
        TIFFSetField(tif, TIFFTAG_RAWDATAUNIQUEID, rawUniq);
        TIFFSetField(tif, TIFFTAG_ORIENTATION, ORIENTATION_TOPLEFT);
        TIFFSetField(tif, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
        TIFFSetField(tif, TIFFTAG_SOFTWARE, "Libcamera;cinepi-raw");
        TIFFSetField(tif, TIFFTAG_COLORMATRIX1, 9, CAM_XYZ.m);
        TIFFSetField(tif, TIFFTAG_ASSHOTNEUTRAL, 3, NEUTRAL);
        TIFFSetField(tif, TIFFTAG_CALIBRATIONILLUMINANT1, 21);
        TIFFSetField(tif, TIFFTAG_SUBIFD, 1, &offset_subifd);
        TIFFSetField(tif, TIFFTAG_EXIFIFD, offset_exififd);

        if(options_->thumbnail == 0){
            size_t tSize = thumbWidth * thumbHeight;
            uint8_t *thumb = (uint8_t*)malloc(tSize);
            memset(thumb, 0, tSize);
            TIFFWriteRawStrip(tif, 0, thumb, tSize);
            free(thumb);
        } else if(options_->thumbnail == 1){
            uint8_t* Y_data = const_cast<uint8_t*>(lomem);
            TIFFWriteRawStrip(tif, 0, Y_data, thumbWidth * thumbHeight);
        } else if(options_->thumbnail == 2){
            size_t rowSize = loinfo.stride*3;
            size_t thumbSize = rowSize*loinfo.height;
            uint8_t *thumb = (uint8_t*)malloc(thumbSize);
            uint8_t *read = &thumb[0];
            if(nv21_to_rgb(thumb, lomem, loinfo.stride, loinfo.height) != 1){
                throw std::runtime_error("error converting yuv2rgb image data");
            }
            // TIFFWriteRawStrip(tif, 0, read, thumbSize);
            for(unsigned int y = 0; y < loinfo.height; y++){
                if (TIFFWriteScanline(tif, (read + y*rowSize), y, 0) != 1)
                    throw std::runtime_error("error writing DNG image data");
                
            }
            free(thumb);
            LOG(1, "made thumb!");
        }

        TIFFCheckpointDirectory(tif);
        TIFFWriteDirectory(tif);

        LOG(2, thread_num << " Writing DNG scanline end " << fn);
        
        LOG(2, thread_num << " Writing DNG main image " << fn);

        // The main image (actually tends to show up as "sub-image 1").
        TIFFSetField(tif, TIFFTAG_SUBFILETYPE, 0);
        TIFFSetField(tif, TIFFTAG_IMAGEWIDTH, t_width);
        TIFFSetField(tif, TIFFTAG_IMAGELENGTH, t_height);
        TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE, bayer_format.bits);
        TIFFSetField(tif, TIFFTAG_COMPRESSION, COMPRESSION_NONE );
        TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_CFA);
        TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, 1);
        TIFFSetField(tif, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
        TIFFSetField(tif, TIFFTAG_CFAREPEATPATTERNDIM, cfa_repeat_pattern_dim);
#if TIFFLIB_VERSION >= 20201219 // version 4.2.0 or later
        TIFFSetField(tif, TIFFTAG_CFAPATTERN, 4, bayer_format.order);
#else
        TIFFSetField(tif, TIFFTAG_CFAPATTERN, bayer_format.order);
#endif
        TIFFSetField(tif, TIFFTAG_WHITELEVEL, 1, &white);
        const uint16_t black_level_repeat_dim[] = { 2, 2 };
        TIFFSetField(tif, TIFFTAG_BLACKLEVELREPEATDIM, &black_level_repeat_dim);
        TIFFSetField(tif, TIFFTAG_BLACKLEVEL, 4, &black_levels);
        
        TIFFSetField(tif, TIFFTAG_ANALOGBALANCE, 3, ANALOGBALANCE);
        TIFFSetField(tif, TIFFTAG_BASELINEEXPOSURE, 1.0);
        TIFFSetField(tif, TIFFTAG_BASELINENOISE, 1.0);
        TIFFSetField(tif, TIFFTAG_BASELINESHARPNESS, 1.0);
        TIFFSetField(tif, TIFFTAG_BAYERGREENSPLIT, 0);
        TIFFSetField(tif, TIFFTAG_LINEARRESPONSELIMIT, 1.0);
        
        time_t t;
        time(&t);
        struct tm *time_info = localtime(&t);
        TIFFMergeFieldInfo(tif, xtiffFieldInfo, 2);
        const double frameRate = (double)*options_->framerate;
        TIFFSetField(tif, TIFFTAG_FRAMERATE, &frameRate);
        const char timecode[] = { 
            static_cast<char>(fn % static_cast<uint8_t>(frameRate)),
            static_cast<char>(time_info->tm_sec),
            static_cast<char>(time_info->tm_min),
            static_cast<char>(time_info->tm_hour), 
            0, 0, 0, 0 
        };
        TIFFSetField(tif, TIFFTAG_TIMECODE, &timecode);

        LOG(2, thread_num << " Writing DNG main image " << fn);

		// Adjust the memory pointer based on stride and row skips
		mem += (info.stride * offset_y_start);
		LOG(2, thread_num << " Writing buffer " << t_height);

        // (info.stride / sizeof(uint64_t)) - 
		const uint16_t chunk = (info.stride / sizeof(uint64_t));
		LOG(2, thread_num << " Writing DNG chunk: " << chunk << "Stride: " << info.stride);
		std::vector<uint64_t> buffer(chunk);
		for (unsigned int y = 0; y < t_height; y++)
		{
		    uint64_t* read = (uint64_t*)(mem + y * info.stride) + byte_offset_x;  // Combined addition
			unpack12_64(read, buffer.data(), chunk);
		    if (TIFFWriteScanline(tif, (uint8_t *)buffer.data(), y, 0) != 1)
		        throw std::runtime_error("error writing DNG image data");
		}
        
        TIFFCheckpointDirectory(tif);
        offset_subifd = TIFFCurrentDirOffset(tif);
        LOG(2, thread_num << " Writing DNG main dict " << fn);
        TIFFWriteDirectory(tif);

        LOG(2, thread_num << " Writing DNG EXIF " << fn);
        
        TIFFCreateEXIFDirectory(tif);

        LOG(2, thread_num << " Writing DNG TIFFCreateEXIFDirectory " << fn);
        char time_str[32];
        strftime(time_str, 32, "%Y:%m:%d %H:%M:%S", time_info);
        TIFFSetField(tif, EXIFTAG_DATETIMEORIGINAL, time_str);

        TIFFSetField(tif, EXIFTAG_ISOSPEEDRATINGS, 1, &iso);
        TIFFSetField(tif, EXIFTAG_EXPOSURETIME, exp_time);

        TIFFCheckpointDirectory(tif);
        offset_exififd = TIFFCurrentDirOffset(tif);
        TIFFWriteDirectory(tif);
        TIFFSetDirectory(tif, 0);
        TIFFSetField(tif, TIFFTAG_EXIFIFD, offset_exififd);

        LOG(2, thread_num << " Writing DNG DICT " << fn);
        TIFFWriteDirectory(tif);
        TIFFUnlinkDirectory(tif, 2);
        TIFFClose(tif);

        LOG(2, thread_num << " Writing DNG TIFFClose " << fn);

        return memBuf.usedSize;
    }
    catch (std::exception const &e)
    {
        if (tif){
            TIFFClose(tif);
        }
        if (memBuf.buffer) {
            free(memBuf.buffer);
        }
        throw;
    }
}


void DngEncoder::encodeThread(int num)
{
    std::chrono::duration<double> encode_time(0);
    EncodeItem encode_item;
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(3, &cpuset);

    pthread_t current_thread = pthread_self();    
    if(pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset)) {
        std::cerr << "Error setting thread affinity" << std::endl;
    }


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
        LOG(1, "Thread[" << num << "] " << " encode frame: " << encode_item.index);

        {   
            auto start_time = std::chrono::high_resolution_clock::now();
            
            uint8_t *mem_tiff;
            if (posix_memalign((void **)&mem_tiff, BLOCK_SIZE, BUFFER_SIZE) != 0) {
                perror("Error allocating aligned memory");
                return;
            }
            size_t tiff_size = dng_save(num,(const uint8_t*) mem_tiff,(const uint8_t*)encode_item.mem, encode_item.info, (const uint8_t*)encode_item.lomem, encode_item.loinfo, encode_item.losize, encode_item.met, "MEM", "CINEPI-2K", options_, encode_item.index);
            DiskItem item = { mem_tiff, tiff_size, encode_item.info, encode_item.met, encode_item.timestamp_us, encode_item.index};

            std::lock_guard<std::mutex> lock(disk_mutex_);
            disk_buffer_.push_back(std::move(item));
            int amount = disk_buffer_.size();
            disk_cond_var_.notify_all();
            auto end_time = std::chrono::high_resolution_clock::now();

            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
            LOG(1, "Thread[" << num << "] " << encode_item.index << " Time taken for the encode: " << duration << " milliseconds, disk buffer count:"<< amount << " Size:" << tiff_size << std::endl);
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
    CPU_SET(1, &cpuset);
    CPU_SET(2, &cpuset);
    CPU_SET(0, &cpuset);

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
                    disk_buffer_.pop_front();
                    break;
                }
                else{
                    disk_cond_var_.wait_for(lock, 1ms);
                }
            }
        }

        char ft[128];
        snprintf(ft, sizeof(ft), "%s/%s/%s_%09ld.dng", options_->mediaDest.c_str(), options_->folder.c_str(), options_->folder.c_str(), disk_item.index);
        
        std::string filename = std::string(ft);
    
        LOG(1, "Thread[" << num << "] " << " Save frame to disk: " << disk_item.index);
        

            auto start_time = std::chrono::high_resolution_clock::now();
            // Now save the memory buffer to disk
            int fd = open(filename.c_str(), O_WRONLY | O_CREAT | O_DIRECT, 0644);
            if (fd != -1) {

                // Provide sequential access hint
                posix_fadvise(fd, 0, 0, POSIX_FADV_SEQUENTIAL);
                posix_fadvise(fd, 0, 0, POSIX_FADV_NOREUSE);
                if(write(fd, disk_item.mem_tiff, BUFFER_SIZE) != BUFFER_SIZE) {
                    perror("Error writing to file");
                }
                ftruncate(fd, disk_item.size);
                close(fd);

            } else {
                fprintf(stderr, "Failed to open file for writing\n");
            }
            // Clean up
            free(disk_item.mem_tiff);
            auto end_time = std::chrono::high_resolution_clock::now();

            // 3. Calculate and print the difference
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
            LOG(1, "Thread[" << num << "] " << disk_item.index << " Time taken for the disk io: " << duration << " milliseconds" << std::endl);
        

        still_capture = false;
    }
}
