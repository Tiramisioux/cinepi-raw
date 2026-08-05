/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * dng_pack.hpp - pure DNG pixel pack/unpack helpers.
 *
 * These row-level packers/unpackers were extracted VERBATIM from
 * dng_encoder.cpp so they can be unit-tested in isolation (see
 * tests/dng_pack_test.cpp). No libcamera, no other cinepi headers — only the
 * standard library. Behaviour is unchanged; do not edit the packing math here
 * without updating the tests.
 */

#ifndef CINEPI_DNG_PACK_HPP
#define CINEPI_DNG_PACK_HPP

#include <cstdint>
#include <cstddef>
#include <array>
#include <algorithm>
#include <cstring>

/* ────────────────────────────────────────────────────────────── */
/*  Helper: pack a single 16-bit row → 12-bit packed               */
/*  width must be even (IMX585 gives even pixel counts).          */
/* ────────────────────────────────────────────────────────────── */
static inline void pack_row_12bit(const uint16_t *src,
                                  uint8_t       *dst,
                                  uint32_t       width)
{
    for (uint32_t x = 0; x < width; x += 2)
    {
        uint16_t p0 = src[x];
        uint16_t p1 = src[x + 1];
        dst[0] =  p0 >> 4;                     /* upper 8 bits of pixel 0      */
        dst[1] = (p0 << 4) | (p1 >> 8);        /* lower 4 + upper 4            */
        dst[2] =  p1;                          /* lower 8 bits of pixel 1       */
        dst += 3;
    }
}

/* Pack a 16-bit source row to packed 12-bit output while dropping 4 LSBs. */
static inline void pack_row_16_to_12bit(const uint16_t *src,
                                        uint8_t       *dst,
                                        uint32_t       width)
{
    for (uint32_t x = 0; x < width; x += 2)
    {
        const uint16_t p0 = src[x] >> 4;
        const uint16_t p1 = src[x + 1] >> 4;
        dst[0] = p0 >> 4;
        dst[1] = (p0 << 4) | (p1 >> 8);
        dst[2] = p1;
        dst += 3;
    }
}

/* Pack one 4-pixel group of right-justified 10-bit samples into 5 contiguous
 * bytes, MSB-first (the layout DNG expects for BitsPerSample=10). */
static inline void pack_group_10bit(const uint16_t *src, uint8_t *dst)
{
    dst[0] =  src[0] >> 2;                     /* upper 8 bits of pixel 0      */
    dst[1] = (src[0] << 6) | (src[1] >> 4);    /* lower 2 + upper 6            */
    dst[2] = (src[1] << 4) | (src[2] >> 6);    /* lower 4 + upper 4            */
    dst[3] = (src[2] << 2) | (src[3] >> 8);    /* lower 6 + upper 2            */
    dst[4] =  src[3];                          /* lower 8 bits of pixel 3      */
}

/* Pack a row of right-justified 10-bit samples to contiguous 10-bit (4 px in
 * 5 bytes). Moved here from dng_encoder.cpp's pack_10bit_data(); the math per
 * 4-pixel group is unchanged.
 *
 * A width that is not a multiple of 4 packs a ZERO-PADDED final group and emits
 * only the (n*10+7)/8 bytes those n pixels occupy — the original read up to 3
 * uint16 past the end of the source row instead. Every 10-bit sensor mode has a
 * multiple-of-4 width (1332, 1456, 3936, 5568), so recorded DNG output is
 * byte-identical; the tail only removes the over-read. */
static inline void pack_row_10bit(const uint16_t *src,
                                  uint8_t       *dst,
                                  uint32_t       width)
{
    uint32_t x = 0;
    for (; x + 4u <= width; x += 4u, dst += 5)
        pack_group_10bit(src + x, dst);

    const uint32_t remaining = width - x;
    if (remaining > 0)
    {
        uint16_t working[4] {};
        uint8_t  packed[5] {};
        std::copy(src + x, src + width, working);
        pack_group_10bit(working, packed);
        std::memcpy(dst, packed, (static_cast<size_t>(remaining) * 10u + 7u) / 8u);
    }
}

/* Unpack MIPI CSI-2 RAW12 (2 px in 3 bytes) to right-justified 16-bit (0..4095).
 * VC4/Unicam delivers SBGGR12_CSI2P in this layout — verified against real Pi 4
 * IMX477 captures (decoding as contiguous-12 instead gives a checkerboard/
 * "wrong bit order" raw). Byte 2 holds the two low nibbles: the even pixel takes
 * the low nibble, the odd pixel the high nibble. The output is right-justified so
 * it feeds pack_row_12bit() (NOT pack_row_16_to_12bit, which drops 4 LSBs). */
static inline void unpack_csi2_raw12(const uint8_t *src, uint16_t *dst, uint32_t width)
{
    for (uint32_t x = 0; x + 1 < width; x += 2)
    {
        const uint8_t b0 = src[0], b1 = src[1], b2 = src[2];
        dst[x]     = (static_cast<uint16_t>(b0) << 4) |  (b2 & 0x0F);
        dst[x + 1] = (static_cast<uint16_t>(b1) << 4) | ((b2 >> 4) & 0x0F);
        src += 3;
    }
}

/* Unpack MIPI CSI-2 RAW10 (4 px in 5 bytes) to right-justified 16-bit (0..1023).
 * Byte 4 holds the four pixels' low 2 bits, first pixel in the lowest pair. Same
 * MIPI convention as RAW12 above; feeds pack_row_10bit(). Note that this is NOT
 * the inverse of pack_row_10bit — that packs the CONTIGUOUS DNG layout. */
static inline void unpack_csi2_raw10(const uint8_t *src, uint16_t *dst, uint32_t width)
{
    for (uint32_t x = 0; x + 3 < width; x += 4)
    {
        const uint8_t b0 = src[0], b1 = src[1], b2 = src[2], b3 = src[3], b4 = src[4];
        dst[x]     = (static_cast<uint16_t>(b0) << 2) |  (b4 & 0x03);
        dst[x + 1] = (static_cast<uint16_t>(b1) << 2) | ((b4 >> 2) & 0x03);
        dst[x + 2] = (static_cast<uint16_t>(b2) << 2) | ((b4 >> 4) & 0x03);
        dst[x + 3] = (static_cast<uint16_t>(b3) << 2) | ((b4 >> 6) & 0x03);
        src += 5;
    }
}

/*
 * PiSP COMP1 compressed Bayer decode.
 *
 * Pi 5's PiSP frontend may turn requested CSI2 packed raw into PISP_COMP1.
 * The compressed stream stores one 8-pixel block in 8 bytes. Decode back to
 * PiSP's 16-bit working domain, then the regular DNG row packer can emit the
 * same 12-bit DNG payload used for unpacked 16-bit raw. The constants match
 * the Raspberry Pi PiSP pipeline configuration used by Will Whang's IMX585
 * libcamera fork. Decoder logic adapted from Apertar-Core's MIT-licensed
 * CdngEncoder (Copyright (c) 2026 Apertar Studio).
 */
constexpr uint16_t PISP_COMP1_OFFSET = 2048;
constexpr size_t PISP_DEQUANT_LUT_SIZE = 1024;

static inline uint32_t read_le32(const uint8_t *src)
{
    return static_cast<uint32_t>(src[0]) |
           (static_cast<uint32_t>(src[1]) << 8) |
           (static_cast<uint32_t>(src[2]) << 16) |
           (static_cast<uint32_t>(src[3]) << 24);
}

static uint16_t pisp_dequantize_scalar(uint16_t q, int qmode)
{
    switch (qmode)
    {
    case 0:
        return static_cast<uint16_t>((q < 320) ? (16 * q) : (32 * (q - 160)));
    case 1:
        return static_cast<uint16_t>(std::min<uint32_t>(65535u, 64u * q));
    case 2:
        return static_cast<uint16_t>(std::min<uint32_t>(65535u, 128u * q));
    default:
        return static_cast<uint16_t>((q < 94) ? (256 * q) : std::min<uint32_t>(65535u, 512u * (q - 47)));
    }
}

static std::array<uint16_t, PISP_DEQUANT_LUT_SIZE> build_pisp_dequant_lut(int qmode)
{
    std::array<uint16_t, PISP_DEQUANT_LUT_SIZE> lut {};
    for (size_t i = 0; i < lut.size(); ++i)
        lut[i] = pisp_dequantize_scalar(static_cast<uint16_t>(i), qmode);
    return lut;
}

static const std::array<uint16_t, PISP_DEQUANT_LUT_SIZE> PISP_DEQUANT_MODE0 = build_pisp_dequant_lut(0);
static const std::array<uint16_t, PISP_DEQUANT_LUT_SIZE> PISP_DEQUANT_MODE1 = build_pisp_dequant_lut(1);
static const std::array<uint16_t, PISP_DEQUANT_LUT_SIZE> PISP_DEQUANT_MODE2 = build_pisp_dequant_lut(2);
static const std::array<uint16_t, PISP_DEQUANT_LUT_SIZE> PISP_DEQUANT_MODE3 = build_pisp_dequant_lut(3);

static inline uint16_t pisp_dequantize_fast(int q, int qmode)
{
    const size_t idx = static_cast<size_t>(std::clamp(q, 0, static_cast<int>(PISP_DEQUANT_LUT_SIZE - 1)));
    switch (qmode)
    {
    case 0:
        return PISP_DEQUANT_MODE0[idx];
    case 1:
        return PISP_DEQUANT_MODE1[idx];
    case 2:
        return PISP_DEQUANT_MODE2[idx];
    default:
        return PISP_DEQUANT_MODE3[idx];
    }
}

static inline uint16_t add_pisp_comp1_offset(uint16_t value)
{
    return static_cast<uint16_t>(std::min<uint32_t>(65535u, static_cast<uint32_t>(value) + PISP_COMP1_OFFSET));
}

static void pisp_comp1_subblock(uint16_t *dst, uint32_t word)
{
    int q[4] {};
    const int qmode = word & 3;
    if (qmode < 3)
    {
        const int field0 = (word >> 2) & 511;
        const int field1 = (word >> 11) & 127;
        const int field2 = (word >> 18) & 127;
        const int field3 = (word >> 25) & 127;
        if (qmode == 2 && field0 >= 384)
        {
            q[1] = field0;
            q[2] = field1 + 384;
        }
        else
        {
            q[1] = (field1 >= 64) ? field0 : field0 + 64 - field1;
            q[2] = (field1 >= 64) ? field0 + field1 - 64 : field0;
        }
        int p1 = std::max(0, q[1] - 64);
        if (qmode == 2)
            p1 = std::min(384, p1);
        int p2 = std::max(0, q[2] - 64);
        if (qmode == 2)
            p2 = std::min(384, p2);
        q[0] = p1 + field2;
        q[3] = p2 + field3;
    }
    else
    {
        const int pack0 = (word >> 2) & 32767;
        const int pack1 = (word >> 17) & 32767;
        q[0] = (pack0 & 15) + 16 * ((pack0 >> 8) / 11);
        q[1] = (pack0 >> 4) % 176;
        q[2] = (pack1 & 15) + 16 * ((pack1 >> 8) / 11);
        q[3] = (pack1 >> 4) % 176;
    }

    dst[0] = pisp_dequantize_fast(q[0], qmode);
    dst[2] = pisp_dequantize_fast(q[1], qmode);
    dst[4] = pisp_dequantize_fast(q[2], qmode);
    dst[6] = pisp_dequantize_fast(q[3], qmode);
}

static void decode_pisp_comp1_block(const uint8_t *src, uint16_t *dst)
{
    pisp_comp1_subblock(dst, read_le32(src));
    pisp_comp1_subblock(dst + 1, read_le32(src + 4));
    for (int i = 0; i < 8; ++i)
        dst[i] = add_pisp_comp1_offset(dst[i]);
}

static inline void unpack_pisp_comp1_row_to_16(const uint8_t *src, uint16_t *dst, uint32_t width)
{
    const uint32_t full_blocks = width / 8u;
    uint32_t x = 0;
    for (uint32_t block = 0; block < full_blocks; ++block, x += 8u, src += 8u)
        decode_pisp_comp1_block(src, dst + x);

    const uint32_t remaining = width - x;
    if (remaining > 0)
    {
        uint16_t working[8] {};
        decode_pisp_comp1_block(src, working);
        std::copy(working, working + remaining, dst + x);
    }
}

static inline void unpack_pisp_comp1_row_to_packed12(const uint8_t *src, uint8_t *dst, uint32_t width)
{
    const uint32_t full_blocks = width / 8u;
    uint32_t x = 0;
    for (uint32_t block = 0; block < full_blocks; ++block, x += 8u, src += 8u)
    {
        uint16_t working[8];
        decode_pisp_comp1_block(src, working);
        pack_row_16_to_12bit(working, dst + (static_cast<size_t>(x) / 2u) * 3u, 8u);
    }

    const uint32_t remaining = width - x;
    if (remaining > 0)
    {
        uint16_t working[8] {};
        uint8_t packed[12] {};
        decode_pisp_comp1_block(src, working);
        const uint32_t tail = std::min(remaining, 8u);
        pack_row_16_to_12bit(working, packed, tail);
        std::memcpy(dst + (static_cast<size_t>(x) / 2u) * 3u, packed, (tail * 12u + 7u) / 8u);
    }
}

#endif // CINEPI_DNG_PACK_HPP
