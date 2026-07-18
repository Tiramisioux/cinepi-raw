// SPDX-License-Identifier: BSD-2-Clause
//
// Unit tests for the pure DNG pixel pack/unpack helpers (cinepi/dng_pack.hpp).
//
// Pure / self-contained: no libcamera, no Redis. Build & run:
//   c++ -std=c++17 -O2 -I.. tests/dng_pack_test.cpp -o /tmp/dng_pack_test && /tmp/dng_pack_test
// (or via meson: `meson test dng_pack`).
//
// These helpers were extracted verbatim from dng_encoder.cpp. Every expected
// vector below was derived BY HAND from the function bodies (not by running the
// code) so the test is an independent check of the byte math, tier by tier:
//   Tier 1 — contiguous 12-bit packers (pack_row_16_to_12bit, pack_row_12bit)
//   Tier 2 — MIPI CSI-2 unpackers      (unpack_csi2_raw12, unpack_csi2_raw10)
//   Tier 3 — PiSP COMP1 decode         (unpack_pisp_comp1_row_to_16 / _packed12)

#include "cinepi/dng_pack.hpp"

#include <cstdio>
#include <cstdint>
#include <cstddef>
#include <vector>

// ── tiny test harness (same shape as phase_lock_core_test.cpp) ───────────────
static int g_failures = 0;
static int g_checks   = 0;
#define CHECK(cond, msg)                                                        \
    do {                                                                        \
        ++g_checks;                                                             \
        if (!(cond)) {                                                          \
            ++g_failures;                                                       \
            std::printf("  FAIL: %s  (%s:%d)\n", (msg), __FILE__, __LINE__);    \
        }                                                                       \
    } while (0)

// Compare a produced byte buffer to an expected one, printing the first diff.
static bool bytes_equal(const char *what,
                        const uint8_t *got, const uint8_t *exp, size_t n)
{
    for (size_t i = 0; i < n; ++i) {
        if (got[i] != exp[i]) {
            std::printf("  %s: byte %zu got 0x%02X exp 0x%02X\n",
                        what, i, got[i], exp[i]);
            return false;
        }
    }
    return true;
}

// Compare a produced uint16 buffer to an expected one, printing the first diff.
static bool words_equal(const char *what,
                        const uint16_t *got, const uint16_t *exp, size_t n)
{
    for (size_t i = 0; i < n; ++i) {
        if (got[i] != exp[i]) {
            std::printf("  %s: word %zu got 0x%04X exp 0x%04X\n",
                        what, i, got[i], exp[i]);
            return false;
        }
    }
    return true;
}

// ── TIER 1: contiguous 12-bit packers ───────────────────────────────────────

// pack_row_16_to_12bit: src>>4 (drop low 4 bits) -> 12-bit, two per 3 bytes BE.
static void test_pack_row_16_to_12bit() {
    std::printf("test_pack_row_16_to_12bit\n");
    {
        const uint16_t src[2] = { 0xABCD, 0x1234 };
        const uint8_t  exp[3] = { 0xAB, 0xC1, 0x23 };
        uint8_t dst[3] = {0};
        pack_row_16_to_12bit(src, dst, 2);
        CHECK(bytes_equal("anchor ABCD,1234", dst, exp, 3), "pack16 anchor {0xABCD,0x1234}");
    }
    {
        const uint16_t src[2] = { 0xFFFF, 0x0000 };
        const uint8_t  exp[3] = { 0xFF, 0xF0, 0x00 };
        uint8_t dst[3] = {0};
        pack_row_16_to_12bit(src, dst, 2);
        CHECK(bytes_equal("anchor FFFF,0000", dst, exp, 3), "pack16 anchor {0xFFFF,0x0000}");
    }
    {
        const uint16_t src[2] = { 0x0000, 0xFFFF };
        const uint8_t  exp[3] = { 0x00, 0x0F, 0xFF };
        uint8_t dst[3] = {0};
        pack_row_16_to_12bit(src, dst, 2);
        CHECK(bytes_equal("anchor 0000,FFFF", dst, exp, 3), "pack16 anchor {0x0000,0xFFFF}");
    }
    // width=4 -> two independent 3-byte groups.
    {
        const uint16_t src[4] = { 0xABCD, 0x1234, 0xFFFF, 0x0000 };
        const uint8_t  exp[6] = { 0xAB, 0xC1, 0x23, 0xFF, 0xF0, 0x00 };
        uint8_t dst[6] = {0};
        pack_row_16_to_12bit(src, dst, 4);
        CHECK(bytes_equal("width4", dst, exp, 6), "pack16 width=4 two groups");
    }
    // all-zero.
    {
        const uint16_t src[2] = { 0x0000, 0x0000 };
        const uint8_t  exp[3] = { 0x00, 0x00, 0x00 };
        uint8_t dst[3] = {0xEE,0xEE,0xEE};
        pack_row_16_to_12bit(src, dst, 2);
        CHECK(bytes_equal("allzero", dst, exp, 3), "pack16 all-zero");
    }
}

// pack_row_12bit: input already right-justified 12-bit (no >>4), same packing.
static void test_pack_row_12bit() {
    std::printf("test_pack_row_12bit\n");
    {
        const uint16_t src[2] = { 0x0ABC, 0x0123 };
        const uint8_t  exp[3] = { 0xAB, 0xC1, 0x23 };
        uint8_t dst[3] = {0};
        pack_row_12bit(src, dst, 2);
        CHECK(bytes_equal("anchor 0ABC,0123", dst, exp, 3), "pack12 anchor {0x0ABC,0x0123}");
    }
    // max 12-bit values.
    {
        const uint16_t src[2] = { 0x0FFF, 0x0FFF };
        const uint8_t  exp[3] = { 0xFF, 0xFF, 0xFF };
        uint8_t dst[3] = {0};
        pack_row_12bit(src, dst, 2);
        CHECK(bytes_equal("allmax", dst, exp, 3), "pack12 all-max {0x0FFF,0x0FFF}");
    }
    // all-zero, width=4.
    {
        const uint16_t src[4] = { 0, 0, 0, 0 };
        const uint8_t  exp[6] = { 0, 0, 0, 0, 0, 0 };
        uint8_t dst[6] = {0xEE,0xEE,0xEE,0xEE,0xEE,0xEE};
        pack_row_12bit(src, dst, 4);
        CHECK(bytes_equal("allzero4", dst, exp, 6), "pack12 all-zero width=4");
    }
    // Cross-check: pack_row_12bit(v) == pack_row_16_to_12bit(v<<4) for 12-bit v.
    {
        const uint16_t v12[2]  = { 0x0ABC, 0x0123 };
        const uint16_t v16[2]  = { 0x0ABC << 4, 0x0123 << 4 };
        uint8_t a[3] = {0}, b[3] = {0};
        pack_row_12bit(v12, a, 2);
        pack_row_16_to_12bit(v16, b, 2);
        CHECK(bytes_equal("crosscheck", a, b, 3),
              "pack12(v) == pack16(v<<4) — same packing, different justification");
    }
}

// ── TIER 2: MIPI CSI-2 unpackers ─────────────────────────────────────────────

// Build the CSI2 RAW12 byte triple for two right-justified 12-bit values, per
// the layout the unpacker assumes: b0=v0>>4, b1=v1>>4, byte2 = low nibbles
// (v0 low nibble in bits 0-3, v1 low nibble in bits 4-7).
static void csi12_pack(uint16_t v0, uint16_t v1, uint8_t out[3]) {
    out[0] = static_cast<uint8_t>(v0 >> 4);
    out[1] = static_cast<uint8_t>(v1 >> 4);
    out[2] = static_cast<uint8_t>((v0 & 0x0F) | ((v1 & 0x0F) << 4));
}

static void test_unpack_csi2_raw12() {
    std::printf("test_unpack_csi2_raw12\n");
    // Direct byte anchor: {0xAB,0x12,0x3C} -> {0xABC, 0x123}.
    {
        const uint8_t  src[3] = { 0xAB, 0x12, 0x3C };
        const uint16_t exp[2] = { 0x0ABC, 0x0123 };
        uint16_t dst[2] = {0};
        unpack_csi2_raw12(src, dst, 2);
        CHECK(words_equal("anchor", dst, exp, 2), "raw12 anchor {0xAB,0x12,0x3C}");
    }
    // all-zero and all-max (all-max must give the 12-bit ceiling 0xFFF).
    {
        const uint8_t  z[3] = { 0, 0, 0 };
        const uint16_t ez[2] = { 0, 0 };
        uint16_t dz[2] = {0xFFFF,0xFFFF};
        unpack_csi2_raw12(z, dz, 2);
        CHECK(words_equal("zero", dz, ez, 2), "raw12 all-zero");

        const uint8_t  f[3] = { 0xFF, 0xFF, 0xFF };
        const uint16_t ef[2] = { 0x0FFF, 0x0FFF };
        uint16_t df[2] = {0};
        unpack_csi2_raw12(f, df, 2);
        CHECK(words_equal("max", df, ef, 2), "raw12 all-max -> 0x0FFF ceiling");
    }
    // Round-trip: build bytes from known 12-bit values, unpack, recover them.
    {
        const uint16_t vals[4] = { 0x0ABC, 0x0123, 0x0FFF, 0x0000 };
        uint8_t bytes[6];
        csi12_pack(vals[0], vals[1], bytes + 0);
        csi12_pack(vals[2], vals[3], bytes + 3);
        uint16_t dst[4] = {0};
        unpack_csi2_raw12(bytes, dst, 4);
        CHECK(words_equal("roundtrip", dst, vals, 4), "raw12 round-trip width=4");
    }
}

// Build the CSI2 RAW10 5-byte group for four right-justified 10-bit values.
static void csi10_pack(uint16_t v0, uint16_t v1, uint16_t v2, uint16_t v3, uint8_t out[5]) {
    out[0] = static_cast<uint8_t>(v0 >> 2);
    out[1] = static_cast<uint8_t>(v1 >> 2);
    out[2] = static_cast<uint8_t>(v2 >> 2);
    out[3] = static_cast<uint8_t>(v3 >> 2);
    out[4] = static_cast<uint8_t>((v0 & 0x03) | ((v1 & 0x03) << 2) |
                                  ((v2 & 0x03) << 4) | ((v3 & 0x03) << 6));
}

static void test_unpack_csi2_raw10() {
    std::printf("test_unpack_csi2_raw10\n");
    // Hand-derived anchor: {0xFF,0x00,0x55,0xAA,0x93} -> {0x3FF,0x000,0x155,0x2AA}.
    {
        const uint8_t  src[5] = { 0xFF, 0x00, 0x55, 0xAA, 0x93 };
        const uint16_t exp[4] = { 0x03FF, 0x0000, 0x0155, 0x02AA };
        uint16_t dst[4] = {0};
        unpack_csi2_raw10(src, dst, 4);
        CHECK(words_equal("anchor", dst, exp, 4), "raw10 anchor");
    }
    // all-zero and all-max (each 10-bit output must ceiling at 0x3FF).
    {
        const uint8_t  z[5] = { 0, 0, 0, 0, 0 };
        const uint16_t ez[4] = { 0, 0, 0, 0 };
        uint16_t dz[4] = {0xFFFF,0xFFFF,0xFFFF,0xFFFF};
        unpack_csi2_raw10(z, dz, 4);
        CHECK(words_equal("zero", dz, ez, 4), "raw10 all-zero");

        const uint8_t  f[5] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
        const uint16_t ef[4] = { 0x03FF, 0x03FF, 0x03FF, 0x03FF };
        uint16_t df[4] = {0};
        unpack_csi2_raw10(f, df, 4);
        CHECK(words_equal("max", df, ef, 4), "raw10 all-max -> 0x03FF ceiling");
    }
    // Round-trip over two groups (width=8).
    {
        const uint16_t vals[8] = { 0x3FF, 0x000, 0x155, 0x2AA, 0x001, 0x002, 0x3FE, 0x200 };
        uint8_t bytes[10];
        csi10_pack(vals[0], vals[1], vals[2], vals[3], bytes + 0);
        csi10_pack(vals[4], vals[5], vals[6], vals[7], bytes + 5);
        uint16_t dst[8] = {0};
        unpack_csi2_raw10(bytes, dst, 8);
        CHECK(words_equal("roundtrip", dst, vals, 8), "raw10 round-trip width=8");
    }
}

// ── TIER 3: PiSP COMP1 decode ────────────────────────────────────────────────
//
// decode_pisp_comp1_block reads two little-endian 32-bit words. word0 fills
// output lanes 0,2,4,6; word1 fills lanes 1,3,5,7; then +2048 (PISP_COMP1_OFFSET)
// is added to all eight. Expected values below are derived by hand from
// pisp_comp1_subblock + pisp_dequantize_scalar.

static void test_unpack_pisp_comp1_row_to_16() {
    std::printf("test_unpack_pisp_comp1_row_to_16\n");
    // (A) 8 zero bytes -> both words qmode 0, q={0,64,0,0} per subblock.
    //     dequant0(0)=0, dequant0(64)=1024; +2048 offset:
    //     lanes = [2048,2048, 3072,3072, 2048,2048, 2048,2048].
    {
        const uint8_t  src[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
        const uint16_t exp[8] = { 2048, 2048, 3072, 3072, 2048, 2048, 2048, 2048 };
        uint16_t dst[8] = {0};
        unpack_pisp_comp1_row_to_16(src, dst, 8);
        CHECK(words_equal("zeros", dst, exp, 8), "comp1->16 all-zero block");
    }
    // (B) word0=0x00000001 (qmode 1), word1=0 (qmode 0).
    //     word0: q={0,64,0,0}, dequant1(64)=64*64=4096 -> +2048 = 6144 on lane 2.
    //     word1: same as (A) -> lanes 1,3,5,7 = [2048,3072,2048,2048].
    {
        const uint8_t  src[8] = { 0x01, 0, 0, 0, 0, 0, 0, 0 };
        const uint16_t exp[8] = { 2048, 2048, 6144, 3072, 2048, 2048, 2048, 2048 };
        uint16_t dst[8] = {0};
        unpack_pisp_comp1_row_to_16(src, dst, 8);
        CHECK(words_equal("qmode1", dst, exp, 8), "comp1->16 qmode1 word0");
    }
    // (C) both words = 0x00000003 (qmode 3, else/packed branch, all q=0).
    //     dequant3(0)=0 -> every lane = +2048 = 2048.
    {
        const uint8_t  src[8] = { 0x03, 0, 0, 0, 0x03, 0, 0, 0 };
        const uint16_t exp[8] = { 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048 };
        uint16_t dst[8] = {0};
        unpack_pisp_comp1_row_to_16(src, dst, 8);
        CHECK(words_equal("qmode3", dst, exp, 8), "comp1->16 qmode3 else-branch");
    }
    // (D) tail path: width=4 decodes one block, copies first 4 lanes only.
    {
        const uint8_t  src[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
        const uint16_t exp[4] = { 2048, 2048, 3072, 3072 };
        uint16_t dst[4] = {0};
        unpack_pisp_comp1_row_to_16(src, dst, 4);
        CHECK(words_equal("tail", dst, exp, 4), "comp1->16 partial-width tail copies 4 lanes");
    }
}

static void test_unpack_pisp_comp1_row_to_packed12() {
    std::printf("test_unpack_pisp_comp1_row_to_packed12\n");
    // Same decode as (A) above: lanes [2048,2048,3072,3072,2048,2048,2048,2048],
    // then pack_row_16_to_12bit (>>4): [128,128,192,192,128,128,128,128] ->
    //   pair(128,128) -> {0x08,0x00,0x80}
    //   pair(192,192) -> {0x0C,0x00,0xC0}
    {
        const uint8_t src[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
        const uint8_t exp[12] = {
            0x08, 0x00, 0x80,
            0x0C, 0x00, 0xC0,
            0x08, 0x00, 0x80,
            0x08, 0x00, 0x80,
        };
        uint8_t dst[12] = {0};
        unpack_pisp_comp1_row_to_packed12(src, dst, 8);
        CHECK(bytes_equal("zeros", dst, exp, 12), "comp1->packed12 all-zero block");
    }
    // Tail path: width=4 packs first 4 lanes -> 6 bytes.
    {
        const uint8_t src[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
        const uint8_t exp[6] = { 0x08, 0x00, 0x80, 0x0C, 0x00, 0xC0 };
        uint8_t dst[6] = {0};
        unpack_pisp_comp1_row_to_packed12(src, dst, 4);
        CHECK(bytes_equal("tail", dst, exp, 6), "comp1->packed12 partial-width tail");
    }
    // Consistency: packed12 output equals pack_row_16_to_12bit applied to the
    // 16-bit decode of the same block (both go through the same packer).
    {
        const uint8_t src[8] = { 0x01, 0, 0, 0, 0, 0, 0, 0 };
        uint16_t dec[8] = {0};
        unpack_pisp_comp1_row_to_16(src, dec, 8);
        uint8_t viaPack[12] = {0};
        pack_row_16_to_12bit(dec, viaPack, 8);
        uint8_t direct[12] = {0};
        unpack_pisp_comp1_row_to_packed12(src, direct, 8);
        CHECK(bytes_equal("consistency", direct, viaPack, 12),
              "comp1->packed12 == pack16(comp1->16) for the same block");
    }
}

int main() {
    std::printf("=== dng_pack unit tests ===\n");
    // Tier 1
    test_pack_row_16_to_12bit();
    test_pack_row_12bit();
    // Tier 2
    test_unpack_csi2_raw12();
    test_unpack_csi2_raw10();
    // Tier 3
    test_unpack_pisp_comp1_row_to_16();
    test_unpack_pisp_comp1_row_to_packed12();

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    if (g_failures == 0) std::printf("ALL TESTS PASSED\n");
    return g_failures == 0 ? 0 : 1;
}
