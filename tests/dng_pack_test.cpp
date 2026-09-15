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
//   Tier 1 — contiguous packers  (pack_row_16_to_12bit, pack_row_12bit,
//                                 pack_row_10bit, pack_row_16_to_10bit)
//   Tier 2 — MIPI CSI-2 unpackers      (unpack_csi2_raw12, unpack_csi2_raw10)
//   Tier 3 — PiSP COMP1 decode         (unpack_pisp_comp1_row_to_16 / _packed12)

#include "cinepi/dng_pack.hpp"

#include <cstdio>
#include <cstdint>
#include <cstddef>
#include <cstring>
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

// right_justify_row: undo the MSB alignment PiSP puts on a sub-16-bit sensor
// mode. Shares the >>4 with pack_row_16_to_12bit, so cross-check against it.
static void test_right_justify_row() {
    std::printf("test_right_justify_row\n");
    {
        // A 12-bit mode arrives as value << 4; shifting back must be exact over
        // the whole 12-bit domain, including both ends.
        const uint16_t src[4] = { 0x0000, 0x00C0 << 4, 0x0ABC << 4, 0x0FFF << 4 };
        const uint16_t exp[4] = { 0x0000, 0x00C0,      0x0ABC,      0x0FFF };
        uint16_t dst[4] = {0xEEEE,0xEEEE,0xEEEE,0xEEEE};
        right_justify_row(src, dst, 4, 4);
        CHECK(std::memcmp(dst, exp, sizeof exp) == 0, "shift=4 recovers the 12-bit code");
    }
    {
        // shift=0 is the identity, which is how the caller expresses "already
        // right-justified" without branching.
        const uint16_t src[3] = { 0x0000, 0x1234, 0xFFFF };
        uint16_t dst[3] = {0xEEEE,0xEEEE,0xEEEE};
        right_justify_row(src, dst, 3, 0);
        CHECK(std::memcmp(dst, src, sizeof src) == 0, "shift=0 is the identity");
    }
    {
        // In place, because the encoder normalises through one scratch row.
        uint16_t buf[2] = { 0x0ABC << 4, 0x0FFF << 4 };
        const uint16_t exp[2] = { 0x0ABC, 0x0FFF };
        right_justify_row(buf, buf, 2, 4);
        CHECK(std::memcmp(buf, exp, sizeof exp) == 0, "safe when dst aliases src");
    }
    {
        // Cross-check: pack_row_16_to_12bit drops the same 4 LSBs on its way into
        // the packed layout, so right-justifying first then packing right-
        // justified 12 must give the identical bytes.
        const uint16_t v16[2] = { 0x0ABC << 4, 0x0123 << 4 };
        uint16_t just[2];
        uint8_t a[3] = {0}, b[3] = {0};
        right_justify_row(v16, just, 2, 4);
        pack_row_12bit(just, a, 2);
        pack_row_16_to_12bit(v16, b, 2);
        CHECK(bytes_equal("crosscheck", a, b, 3),
              "right_justify + pack12 == pack16 — same 4 bits dropped");
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

// The inverse of pack_row_10bit lives HERE rather than in dng_pack.hpp: every
// function in that header has a production caller, and nothing in cinepi-raw
// ever decodes a contiguous-10 DNG row. Keeping it test-local also makes the
// round-trip an INDEPENDENT check — it walks the 40-bit MSB-first stream one bit
// at a time, straight from the DNG layout, instead of re-using the packer's
// shift algebra. (unpack_csi2_raw10 in the header is NOT this inverse: that
// decodes the MIPI layout, which parks the low bits in a shared trailing byte.)
static void unpack_row_10bit(const uint8_t *src, uint16_t *dst, uint32_t width) {
    for (uint32_t x = 0; x < width; ++x) {
        uint16_t v = 0;
        for (size_t b = 0; b < 10; ++b) {
            const size_t p = static_cast<size_t>(x) * 10 + b;   // MSB-first bit index
            v = static_cast<uint16_t>((v << 1) | ((src[p >> 3] >> (7 - (p & 7))) & 1));
        }
        dst[x] = v;
    }
}

// pack_10bit_data() exactly as it stood in dng_encoder.cpp before Pass 2 moved
// it. Kept only to prove the moved packer is byte-identical on the multiple-of-4
// widths every real 10-bit sensor mode uses (1332, 1456, 3936, 5568).
static void pack_row_10bit_premove(const uint16_t *src, uint8_t *dst, size_t num_pixels) {
    for (size_t i = 0; i < num_pixels; i += 4) {
        dst[0] = src[i] >> 2;
        dst[1] = (src[i] << 6) | (src[i + 1] >> 4);
        dst[2] = (src[i + 1] << 4) | (src[i + 2] >> 6);
        dst[3] = (src[i + 2] << 2) | (src[i + 3] >> 8);
        dst[4] = src[i + 3];
        dst += 5;
    }
}

// pack_row_10bit: right-justified 10-bit in, contiguous MSB-first 4px/5B out.
static void test_pack_row_10bit() {
    std::printf("test_pack_row_10bit\n");
    // Anchor, derived by hand from the 40-bit stream:
    //   3FF 000 155 2AA = 1111111111 0000000000 0101010101 1010101010
    //   regrouped by 8  = 11111111 11000000 00000101 01010110 10101010
    {
        const uint16_t src[4] = { 0x3FF, 0x000, 0x155, 0x2AA };
        const uint8_t  exp[5] = { 0xFF, 0xC0, 0x05, 0x56, 0xAA };
        uint8_t dst[5] = {0};
        pack_row_10bit(src, dst, 4);
        CHECK(bytes_equal("anchor 3FF,000,155,2AA", dst, exp, 5), "pack10 anchor group");
    }
    // all-max (40 set bits) and all-zero.
    {
        const uint16_t f[4]  = { 0x3FF, 0x3FF, 0x3FF, 0x3FF };
        const uint8_t  ef[5] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
        uint8_t df[5] = {0};
        pack_row_10bit(f, df, 4);
        CHECK(bytes_equal("max", df, ef, 5), "pack10 all-max -> 40 set bits");

        const uint16_t z[4]  = { 0, 0, 0, 0 };
        const uint8_t  ez[5] = { 0, 0, 0, 0, 0 };
        uint8_t dz[5] = {0xEE,0xEE,0xEE,0xEE,0xEE};
        pack_row_10bit(z, dz, 4);
        CHECK(bytes_equal("zero", dz, ez, 5), "pack10 all-zero");
    }
    // width=8 -> two independent 5-byte groups. Second group by hand:
    //   001 002 3FE 200 = 0000000001 0000000010 1111111110 1000000000
    //                   = 00000000 01000000 00101111 11111010 00000000
    {
        const uint16_t src[8]  = { 0x3FF, 0x000, 0x155, 0x2AA,
                                   0x001, 0x002, 0x3FE, 0x200 };
        const uint8_t  exp[10] = { 0xFF, 0xC0, 0x05, 0x56, 0xAA,
                                   0x00, 0x40, 0x2F, 0xFA, 0x00 };
        uint8_t dst[10] = {0};
        pack_row_10bit(src, dst, 8);
        CHECK(bytes_equal("width8", dst, exp, 10), "pack10 width=8 two groups");
    }
    // Round-trip every 10-bit code: unpack(pack(x)) == x.
    {
        std::vector<uint16_t> vals(1024), back(1024);
        for (uint32_t i = 0; i < 1024; ++i) vals[i] = static_cast<uint16_t>(i);
        std::vector<uint8_t> packed(1024 * 10 / 8);
        pack_row_10bit(vals.data(), packed.data(), 1024);
        unpack_row_10bit(packed.data(), back.data(), 1024);
        CHECK(words_equal("roundtrip", back.data(), vals.data(), 1024),
              "pack10 round-trip over all 1024 codes");
    }
    // THE PASS-2 GATE, at unit level: byte-identical to the pre-move packer on a
    // real multiple-of-4 mode width.
    {
        const uint32_t W = 1332;                       // imx477 10-bit mode width
        std::vector<uint16_t> src(W);
        for (uint32_t i = 0; i < W; ++i)
            src[i] = static_cast<uint16_t>((i * 7919u + 12345u) & 0x3FF);
        std::vector<uint8_t> got(W * 10 / 8, 0), exp(W * 10 / 8, 0);
        pack_row_10bit(src.data(), got.data(), W);
        pack_row_10bit_premove(src.data(), exp.data(), W);
        CHECK(bytes_equal("vs pre-move", got.data(), exp.data(), got.size()),
              "pack10 byte-identical to the pre-move packer at width=1332");
    }
    // Tail: a width that is not a multiple of 4 packs a ZERO-PADDED final group
    // and emits only the (n*10+7)/8 bytes those n pixels occupy. By hand:
    //   n=1 {3FF}         -> 1111111111 + pad          -> FF C0        (2 B)
    //   n=2 {3FF,155}     -> ... 0101010101 + pad      -> FF D5 50     (3 B)
    //   n=3 {3FF,155,2AA} -> ... 1010101010 + pad      -> FF D5 5A A8  (4 B)
    {
        const uint16_t src[3]  = { 0x3FF, 0x155, 0x2AA };
        const uint8_t  exp1[2] = { 0xFF, 0xC0 };
        const uint8_t  exp2[3] = { 0xFF, 0xD5, 0x50 };
        const uint8_t  exp3[4] = { 0xFF, 0xD5, 0x5A, 0xA8 };
        uint8_t d[5];

        std::memset(d, 0xEE, sizeof d);
        pack_row_10bit(src, d, 1);
        CHECK(bytes_equal("tail1", d, exp1, 2), "pack10 width=1 tail");
        CHECK(d[2] == 0xEE && d[3] == 0xEE && d[4] == 0xEE,
              "pack10 width=1 writes exactly 2 bytes");

        std::memset(d, 0xEE, sizeof d);
        pack_row_10bit(src, d, 2);
        CHECK(bytes_equal("tail2", d, exp2, 3), "pack10 width=2 tail");
        CHECK(d[3] == 0xEE && d[4] == 0xEE, "pack10 width=2 writes exactly 3 bytes");

        std::memset(d, 0xEE, sizeof d);
        pack_row_10bit(src, d, 3);
        CHECK(bytes_equal("tail3", d, exp3, 4), "pack10 width=3 tail");
        CHECK(d[4] == 0xEE, "pack10 width=3 writes exactly 4 bytes");
    }
    // The tail must not fold in the word PAST the row. Same 3-pixel row, but now
    // followed by all-ones: the pre-move packer read src[3] and produced 0xAB in
    // the last byte; the zero-padded tail gives 0xA8. (The probe reads only
    // in-bounds memory, so the test itself is well-defined.)
    {
        const uint16_t buf[8] = { 0x3FF, 0x155, 0x2AA, 0x3FF,
                                  0x3FF, 0x3FF, 0x3FF, 0x3FF };
        const uint8_t  exp[4] = { 0xFF, 0xD5, 0x5A, 0xA8 };
        uint8_t d[4] = {0};
        pack_row_10bit(buf, d, 3);
        CHECK(bytes_equal("no over-read", d, exp, 4),
              "pack10 width=3 ignores the word past the row (over-read fixed)");
    }
    // Sweep: the packer writes EXACTLY (width*10+7)/8 bytes at every width, not
    // the rounded-up ((width+3)/4)*5 the pre-move body needed. dng_encoder.cpp
    // sizes both the linear and the log 10-bit scratch row on that expression
    // alone, with no slack, so a width where the packer ran one byte over would
    // be a heap overflow rather than a cosmetic waste. Guard bytes, every width
    // through two full groups past the largest remainder case.
    {
        bool clean = true;
        size_t first_bad = 0;
        for (uint32_t w = 1; w <= 64; ++w) {
            const size_t exact = (static_cast<size_t>(w) * 10u + 7u) / 8u;
            const size_t slack = ((static_cast<size_t>(w) + 3u) / 4u) * 5u;
            std::vector<uint16_t> src(w);
            for (uint32_t x = 0; x < w; ++x)
                src[x] = static_cast<uint16_t>((x * 37u) & 0x3FF);
            std::vector<uint8_t> dst(exact + 8, 0xEE);
            pack_row_10bit(src.data(), dst.data(), w);
            for (size_t i = exact; i < dst.size(); ++i)
                if (dst[i] != 0xEE) { clean = false; if (!first_bad) first_bad = w; break; }
            if (slack < exact) { clean = false; if (!first_bad) first_bad = w; }
        }
        CHECK(clean, "pack10 writes exactly (w*10+7)/8 bytes at every width 1..64");
        if (!clean)
            std::printf("  first bad width: %zu\n", first_bad);
    }
}

// pack_row_16_to_10bit: src>>6 (drop low 6 bits) -> contiguous 10-bit, 4px/5B.
// The 10-bit sibling of pack_row_16_to_12bit, for a native 10-bit sensor mode
// arriving MSB-aligned in PiSP's 16-bit container.
static void test_pack_row_16_to_10bit() {
    std::printf("test_pack_row_16_to_10bit\n");
    // Anchor, derived by hand: {0xABCD,0x1234,0xFFFF,0x0000} >>6 gives the
    // right-justified 10-bit quad {0x2AF, 0x048, 0x3FF, 0x000}. Then
    // pack_group_10bit's five bytes are
    //   b0 = 2AF>>2                    = 0xAB
    //   b1 = (2AF<<6)|(048>>4)  = C0|04 = 0xC4
    //   b2 = (048<<4)|(3FF>>6)  = 80|0F = 0x8F
    //   b3 = (3FF<<2)|(000>>8)  = FC|00 = 0xFC
    //   b4 =  000                      = 0x00
    {
        const uint16_t src[4] = { 0xABCD, 0x1234, 0xFFFF, 0x0000 };
        const uint8_t  exp[5] = { 0xAB, 0xC4, 0x8F, 0xFC, 0x00 };
        uint8_t dst[5] = {0};
        pack_row_16_to_10bit(src, dst, 4);
        CHECK(bytes_equal("anchor", dst, exp, 5), "pack16to10 anchor quad");
    }
    // Alternating full-scale / zero: {3FF,000,3FF,000}.
    //   b0 = FF, b1 = C0|0 = C0, b2 = 0|0F = 0F, b3 = FC|0 = FC, b4 = 00
    {
        const uint16_t src[4] = { 0xFFFF, 0x0000, 0xFFFF, 0x0000 };
        const uint8_t  exp[5] = { 0xFF, 0xC0, 0x0F, 0xFC, 0x00 };
        uint8_t dst[5] = {0};
        pack_row_16_to_10bit(src, dst, 4);
        CHECK(bytes_equal("alternating", dst, exp, 5), "pack16to10 3FF,0,3FF,0");
    }
    // The six dropped bits really are dropped: samples differing only below
    // bit 6 must pack identically. This is the whole lossless-padding claim.
    {
        const uint16_t a[4] = { 0x2AF << 6, 0x048 << 6, 0x3FF << 6, 0x000 << 6 };
        const uint16_t b[4] = { static_cast<uint16_t>((0x2AF << 6) | 0x3F),
                                static_cast<uint16_t>((0x048 << 6) | 0x01),
                                static_cast<uint16_t>((0x3FF << 6) | 0x2A),
                                static_cast<uint16_t>((0x000 << 6) | 0x3F) };
        uint8_t da[5] = {0}, db[5] = {0};
        pack_row_16_to_10bit(a, da, 4);
        pack_row_16_to_10bit(b, db, 4);
        CHECK(bytes_equal("low-6 ignored", db, da, 5),
              "pack16to10 ignores the six padding LSBs");
    }
    // Cross-check against the composition it replaces: for any 10-bit v,
    // pack_row_16_to_10bit(v<<6) == pack_row_10bit(v). Mirrors the existing
    // pack_row_12bit / pack_row_16_to_12bit cross-check above.
    {
        std::vector<uint16_t> v10(1024), v16(1024);
        for (uint32_t i = 0; i < 1024; ++i) {
            v10[i] = static_cast<uint16_t>(i);
            v16[i] = static_cast<uint16_t>(i << 6);
        }
        const size_t n = (1024u * 10u + 7u) / 8u;
        std::vector<uint8_t> viaShift(n, 0), viaPack(n, 0);
        pack_row_16_to_10bit(v16.data(), viaShift.data(), 1024);
        pack_row_10bit(v10.data(), viaPack.data(), 1024);
        CHECK(bytes_equal("cross-check", viaShift.data(), viaPack.data(), n),
              "pack16to10(v<<6) == pack10(v) across all 1024 codes");
    }
    // Cross-check the other seam: right_justify_row(...,6) then pack_row_10bit
    // is the un-fused form of the same operation.
    {
        const uint16_t src[8] = { 0xFFFF, 0x0000, 0xABCD, 0x1234,
                                  0x8000, 0x7FFF, 0x0040, 0x003F };
        uint8_t fused[10] = {0}, staged[10] = {0};
        uint16_t mid[8] = {0};
        pack_row_16_to_10bit(src, fused, 8);
        right_justify_row(src, mid, 8, 6);
        pack_row_10bit(mid, staged, 8);
        CHECK(bytes_equal("fused==staged", fused, staged, 10),
              "pack16to10 == right_justify_row(6) + pack10");
    }
    // Tail: width not a multiple of 4 must zero-pad the final group and emit
    // exactly (w*10+7)/8 bytes, never reading the word past the row. Same
    // contract pack_row_10bit has, and dng_save() sizes the scratch row on
    // that expression with no slack.
    {
        const uint16_t buf[8] = { 0x3FF << 6, 0x155 << 6, 0x2AA << 6, 0x3FF << 6,
                                  0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF };
        const uint8_t  exp[4] = { 0xFF, 0xD5, 0x5A, 0xA8 };   // same as pack10's tail3
        uint8_t d[5];
        std::memset(d, 0xEE, sizeof d);
        pack_row_16_to_10bit(buf, d, 3);
        CHECK(bytes_equal("tail3", d, exp, 4), "pack16to10 width=3 tail");
        CHECK(d[4] == 0xEE, "pack16to10 width=3 writes exactly 4 bytes");
    }
    // Byte-count sweep with guard bytes, every width through two full groups
    // past the largest remainder case.
    {
        bool clean = true;
        size_t first_bad = 0;
        for (uint32_t w = 1; w <= 64; ++w) {
            const size_t exact = (static_cast<size_t>(w) * 10u + 7u) / 8u;
            std::vector<uint16_t> src(w);
            for (uint32_t x = 0; x < w; ++x)
                src[x] = static_cast<uint16_t>(((x * 37u) & 0x3FF) << 6);
            std::vector<uint8_t> dst(exact + 8, 0xEE);
            pack_row_16_to_10bit(src.data(), dst.data(), w);
            for (size_t i = exact; i < dst.size(); ++i)
                if (dst[i] != 0xEE) { clean = false; if (!first_bad) first_bad = w; break; }
        }
        CHECK(clean, "pack16to10 writes exactly (w*10+7)/8 bytes at every width 1..64");
        if (!clean)
            std::printf("  first bad width: %zu\n", first_bad);
    }
    // Every live 10-bit mode width is a multiple of 4, so the tail is a
    // robustness path rather than a hot one: imx477 1332, imx296 1456,
    // imx283 3936 / 5568, imx585 3840.
    {
        bool ok = true;
        for (uint32_t w : {1332u, 1456u, 3840u, 3936u, 5568u})
            if (w % 4u != 0u) ok = false;
        CHECK(ok, "every shipped 10-bit mode width is a multiple of 4");
    }
}

// round_16_to_10bit / pack_row_16_to_10bit_rounded: the rounding repack the
// COMP1 path needs. Distinct from pack_row_16_to_10bit's bare >> 6 ONLY when
// the low six bits are non-zero, which on a real row means a lossy COMP1
// reconstruction rather than known-zero padding.
static void test_round_16_to_10bit() {
    std::printf("test_round_16_to_10bit\n");
    // On an exact v<<6 the two agree — this is what lets the same 10-bit DNG
    // layout serve both the unpacked and the COMP1 path.
    {
        bool same = true;
        for (uint32_t v = 0; v < 1024; ++v)
            if (round_16_to_10bit(static_cast<uint16_t>(v << 6)) != v) { same = false; break; }
        CHECK(same, "round_16_to_10bit(v<<6) == v for all 1024 codes");
    }
    // Rounds at the half-step, rather than truncating.
    {
        CHECK(round_16_to_10bit(64 * 5 +  0) == 5, "round: exact stays put");
        CHECK(round_16_to_10bit(64 * 5 + 31) == 5, "round: below half rounds down");
        CHECK(round_16_to_10bit(64 * 5 + 32) == 6, "round: at half rounds up");
        CHECK(round_16_to_10bit(64 * 5 + 63) == 6, "round: above half rounds up");
        // The bare >> 6 truncates all four to 5 — that difference is the point.
        CHECK(static_cast<uint16_t>((64 * 5 + 63) >> 6) == 5,
              "the truncating shift really does differ here");
    }
    // 65535 + 32 would wrap a uint16, and >> 6 of the unwrapped sum is 1024 —
    // one past the 10-bit white level. Both hazards are handled.
    {
        CHECK(round_16_to_10bit(65535) == 1023, "round_16_to_10bit clamps 65535 to white");
        CHECK(round_16_to_10bit(65504) == 1023, "round_16_to_10bit: 1023<<6 stays 1023");
    }
}

static void test_pack_row_16_to_10bit_rounded() {
    std::printf("test_pack_row_16_to_10bit_rounded\n");
    // Byte-for-byte identical to pack_row_16_to_10bit whenever the source is a
    // clean v<<6 row, across every width including the ragged tails.
    {
        bool same = true;
        size_t bad_w = 0;
        for (uint32_t w = 1; w <= 33 && same; ++w) {
            std::vector<uint16_t> src(w);
            for (uint32_t x = 0; x < w; ++x)
                src[x] = static_cast<uint16_t>(((x * 37u) & 0x3FF) << 6);
            const size_t n = (static_cast<size_t>(w) * 10u + 7u) / 8u;
            std::vector<uint8_t> a(n, 0), b(n, 0);
            pack_row_16_to_10bit(src.data(), a.data(), w);
            pack_row_16_to_10bit_rounded(src.data(), b.data(), w);
            for (size_t i = 0; i < n; ++i)
                if (a[i] != b[i]) { same = false; bad_w = w; break; }
        }
        CHECK(same, "rounded == truncating on exact v<<6 rows, widths 1..33");
        if (!same) std::printf("  first differing width: %zu\n", bad_w);
    }
    // And genuinely different once the padding bits carry something: a row of
    // v<<6 | 32 must round every sample UP by one code.
    {
        const uint16_t src[4] = { static_cast<uint16_t>((100 << 6) | 32),
                                  static_cast<uint16_t>((200 << 6) | 32),
                                  static_cast<uint16_t>((300 << 6) | 63),
                                  static_cast<uint16_t>((400 << 6) | 32) };
        const uint16_t exp[4] = { 101, 201, 301, 401 };
        uint8_t dst[5] = {0};
        uint16_t back[4] = {0};
        pack_row_16_to_10bit_rounded(src, dst, 4);
        unpack_row_10bit(dst, back, 4);
        CHECK(words_equal("rounds up", back, exp, 4),
              "rounded packer lifts half-step samples to the next code");
    }
    // Same byte-count contract as its truncating sibling: exactly (w*10+7)/8
    // bytes at every width, never a byte more.
    {
        bool clean = true;
        for (uint32_t w = 1; w <= 64; ++w) {
            const size_t exact = (static_cast<size_t>(w) * 10u + 7u) / 8u;
            std::vector<uint16_t> src(w);
            for (uint32_t x = 0; x < w; ++x)
                src[x] = static_cast<uint16_t>((x * 1013u) & 0xFFFF);
            std::vector<uint8_t> dst(exact + 8, 0xEE);
            pack_row_16_to_10bit_rounded(src.data(), dst.data(), w);
            for (size_t i = exact; i < dst.size(); ++i)
                if (dst[i] != 0xEE) { clean = false; break; }
        }
        CHECK(clean, "rounded packer writes exactly (w*10+7)/8 bytes, widths 1..64");
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

// Cross-tier: the whole Pi 4 / VC4 10-bit path in miniature — sensor delivers
// MIPI CSI-2 RAW10, the encoder unpacks it to right-justified 16-bit and repacks
// it into the contiguous DNG layout. Both layouts must carry the same codes.
static void test_csi2_raw10_to_contiguous() {
    std::printf("test_csi2_raw10_to_contiguous\n");
    const uint16_t vals[8] = { 0x3FF, 0x000, 0x155, 0x2AA, 0x001, 0x002, 0x3FE, 0x200 };
    uint8_t csi[10];
    csi10_pack(vals[0], vals[1], vals[2], vals[3], csi + 0);
    csi10_pack(vals[4], vals[5], vals[6], vals[7], csi + 5);

    uint16_t mid[8] = {0};
    unpack_csi2_raw10(csi, mid, 8);
    uint8_t contig[10] = {0};
    pack_row_10bit(mid, contig, 8);
    uint16_t back[8] = {0};
    unpack_row_10bit(contig, back, 8);
    CHECK(words_equal("csi2->contig", back, vals, 8),
          "CSI-2 RAW10 -> contiguous DNG preserves all eight codes");

    // The two layouts are genuinely different byte orders — if they were the
    // same, the round-trip above would prove nothing.
    bool differs = false;
    for (size_t i = 0; i < sizeof csi; ++i)
        if (csi[i] != contig[i]) { differs = true; break; }
    CHECK(differs, "CSI-2 RAW10 and contiguous-10 are distinct byte layouts");
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

// unpack_pisp_comp1_row_to_packed10: COMP1 -> packed 10-bit, the branch a
// 10-bit imx519 mode takes on a Pi 5 (cinemate resolves packing 'P' for that
// sensor, and libcamera's PiSP handler turns a CSI2-packed request into COMP1).
//
// The checks below also PIN THE MEASUREMENT that decided this path exists at
// all — see unpack_pisp_comp1_row_to_packed10()'s comment in dng_pack.hpp. If a
// libpisp change ever alters the dequant curves, these fail and the trade has
// to be re-argued rather than silently re-taken.
static void test_unpack_pisp_comp1_row_to_packed10() {
    std::printf("test_unpack_pisp_comp1_row_to_packed10\n");

    // The lattice claim. Every value the decoder can emit passes through
    // pisp_dequantize_fast() (index-clamped) and then the +2048 offset, so
    // iterating the LUT domain covers a SUPERSET of the reachable set — which
    // makes an "always on the 64-grid" result here strictly stronger than one
    // measured over the reachable set alone.
    {
        int off_grid[4] = {0, 0, 0, 0};
        int clamped[4]  = {0, 0, 0, 0};
        for (int qmode = 0; qmode < 4; ++qmode)
            for (int q = 0; q < 1024; ++q) {
                const uint16_t d = add_pisp_comp1_offset(pisp_dequantize_fast(q, qmode));
                if (d == 65535) { clamped[qmode]++; continue; }   // saturation, not a level
                if (d % 64) off_grid[qmode]++;
            }
        CHECK(off_grid[1] == 0, "qmode 1 decodes only onto the 64-grid");
        CHECK(off_grid[2] == 0, "qmode 2 decodes only onto the 64-grid");
        CHECK(off_grid[3] == 0, "qmode 3 decodes only onto the 64-grid");
        CHECK(off_grid[0] >  0, "qmode 0 is the one mode with sub-64 detail");
        CHECK(clamped[2] > 0 && clamped[3] > 0,
              "qmodes 2 and 3 do reach the 65535 clamp");
        if (off_grid[1] || off_grid[2] || off_grid[3])
            std::printf("  off-grid counts: q1=%d q2=%d q3=%d\n",
                        off_grid[1], off_grid[2], off_grid[3]);
    }

    // The result. For every value the decoder can emit, the rounded 10-bit
    // sample implies the SAME original sensor code as the 12-bit sample it
    // replaces — so the two extra bits the 12-bit file was spending carry
    // nothing recoverable. (A reader scales a 12-bit sample by /4 to reach the
    // 10-bit domain; ties go half-up in both, which is why they never split.)
    {
        int disagreements = 0;
        for (int qmode = 0; qmode < 4; ++qmode)
            for (int q = 0; q < 1024; ++q) {
                const uint16_t d    = add_pisp_comp1_offset(pisp_dequantize_fast(q, qmode));
                const int      r10  = round_16_to_10bit(d);
                const int      from12 = std::min(1023,
                                   static_cast<int>((static_cast<double>(d >> 4) / 4.0) + 0.5));
                if (r10 != from12) ++disagreements;
            }
        CHECK(disagreements == 0,
              "rounded 10-bit implies the same code as 12-bit, over the whole decode domain");
        if (disagreements) std::printf("  disagreements: %d\n", disagreements);
    }

    // Anchor, derived by hand. src all zero -> both words qmode 0, decode
    // [2048,2048,3072,3072,2048,2048,2048,2048] (test A above), which rounds to
    // the 10-bit quad pair {32,32,48,48} and {32,32,32,32}. pack_group_10bit:
    //   {32,32,48,48}: b0=32>>2=0x08, b1=(32<<6)|(32>>4)=0x00|0x02=0x02,
    //                  b2=(32<<4)|(48>>6)=0x00|0x00=0x00,
    //                  b3=(48<<2)|(48>>8)=0xC0, b4=48=0x30
    //   {32,32,32,32}: b0=0x08, b1=0x02, b2=0x00, b3=(32<<2)=0x80, b4=0x20
    {
        const uint8_t src[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
        const uint8_t exp[10] = { 0x08, 0x02, 0x00, 0xC0, 0x30,
                                  0x08, 0x02, 0x00, 0x80, 0x20 };
        uint8_t dst[10] = {0};
        unpack_pisp_comp1_row_to_packed10(src, dst, 8);
        CHECK(bytes_equal("zeros", dst, exp, 10), "comp1->packed10 all-zero block");
    }

    // A qmode-0 block that lands OFF the 64-grid, so the rounding is doing real
    // work. word0 = 0x00060000 sets field1=64, field2=1 -> q={1,0,0,0}, and
    // dequant0(1)=16, so lane 0 decodes to 2048+16 = 2064: a quarter of a code
    // above 32, a level the sensor could never have sent.
    {
        const uint8_t src[8] = { 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00 };
        uint16_t dec[8] = {0};
        unpack_pisp_comp1_row_to_16(src, dec, 8);
        CHECK(dec[0] == 2064, "comp1 qmode-0 block decodes lane 0 off the 64-grid");
        CHECK(dec[0] % 64 != 0, "  ...and that value really is off-grid");

        uint8_t  packed[10] = {0};
        uint16_t back[8]    = {0};
        unpack_pisp_comp1_row_to_packed10(src, packed, 8);
        unpack_row_10bit(packed, back, 8);
        CHECK(back[0] == 32, "off-grid 2064 rounds to code 32, as the 12-bit file implies");
    }

    // The case where truncating and rounding actually diverge: field2=2 gives
    // q[0]=2, dequant0(2)=32, so lane 0 decodes to 2080 — exactly half a code
    // above 32. A bare >> 6 would write 32; rounding writes 33, which is what
    // the 12-bit sample (130, i.e. 32.5) rounds to. 49.92% of reachable
    // qmode-0 values sit on this wrong side of a truncating shift.
    {
        const uint8_t src[8] = { 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00 };
        uint16_t dec[8] = {0};
        unpack_pisp_comp1_row_to_16(src, dec, 8);
        CHECK(dec[0] == 2080, "comp1 qmode-0 half-step block decodes to 2080");
        CHECK(static_cast<uint16_t>(dec[0] >> 6) == 32, "  a truncating >>6 would write 32");

        uint8_t  packed[10] = {0};
        uint16_t back[8]    = {0};
        unpack_pisp_comp1_row_to_packed10(src, packed, 8);
        unpack_row_10bit(packed, back, 8);
        CHECK(back[0] == 33, "  the rounding packer writes 33 instead");
    }

    // Consistency with the staged form: the row function must equal
    // pack_row_16_to_10bit_rounded() applied to the block's own 16-bit decode.
    {
        const uint8_t src[16] = { 0x01, 0, 0, 0, 0, 0, 0, 0,
                                  0x00, 0x00, 0x06, 0x00, 0x03, 0, 0, 0 };
        uint16_t dec[16] = {0};
        unpack_pisp_comp1_row_to_16(src, dec, 16);
        uint8_t viaPack[20] = {0};
        pack_row_16_to_10bit_rounded(dec, viaPack, 16);
        uint8_t direct[20] = {0};
        unpack_pisp_comp1_row_to_packed10(src, direct, 16);
        CHECK(bytes_equal("consistency", direct, viaPack, 20),
              "comp1->packed10 == pack10_rounded(comp1->16) over two blocks");
    }

    // Tail: a width that is not a whole block must pack only those pixels and
    // emit exactly (w*10+7)/8 bytes, with no read past the single source block.
    {
        const uint8_t src[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
        const uint8_t exp[5] = { 0x08, 0x02, 0x00, 0xC0, 0x30 };   // first quad only
        uint8_t dst[6];
        std::memset(dst, 0xEE, sizeof dst);
        unpack_pisp_comp1_row_to_packed10(src, dst, 4);
        CHECK(bytes_equal("tail4", dst, exp, 5), "comp1->packed10 width=4 tail");
        CHECK(dst[5] == 0xEE, "comp1->packed10 width=4 writes exactly 5 bytes");
    }
    // Byte-count sweep across every width through two blocks, guard bytes after.
    {
        bool clean = true;
        size_t first_bad = 0;
        std::vector<uint8_t> src(24, 0x5A);
        for (uint32_t w = 1; w <= 24; ++w) {
            const size_t exact = (static_cast<size_t>(w) * 10u + 7u) / 8u;
            std::vector<uint8_t> dst(exact + 8, 0xEE);
            unpack_pisp_comp1_row_to_packed10(src.data(), dst.data(), w);
            for (size_t i = exact; i < dst.size(); ++i)
                if (dst[i] != 0xEE) { clean = false; if (!first_bad) first_bad = w; break; }
        }
        CHECK(clean, "comp1->packed10 writes exactly (w*10+7)/8 bytes at every width 1..24");
        if (!clean) std::printf("  first bad width: %zu\n", first_bad);
    }
    // The size claim that motivated the branch: 1.25 B/px against the 12-bit
    // path's 1.5, i.e. one sixth off the raw plane.
    {
        const uint32_t w = 3840;
        const uint32_t b10 = (w * 10 + 7) / 8, b12 = (w * 12 + 7) / 8;
        CHECK(b10 * 6 == b12 * 5, "packed10 row is exactly 5/6 of the packed12 row");
    }
}

int main() {
    std::printf("=== dng_pack unit tests ===\n");
    // Tier 1
    test_pack_row_16_to_12bit();
    test_right_justify_row();
    test_pack_row_12bit();
    test_pack_row_10bit();
    test_pack_row_16_to_10bit();
    test_round_16_to_10bit();
    test_pack_row_16_to_10bit_rounded();
    // Tier 2
    test_unpack_csi2_raw12();
    test_unpack_csi2_raw10();
    test_csi2_raw10_to_contiguous();
    // Tier 3
    test_unpack_pisp_comp1_row_to_16();
    test_unpack_pisp_comp1_row_to_packed12();
    test_unpack_pisp_comp1_row_to_packed10();

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    if (g_failures == 0) std::printf("ALL TESTS PASSED\n");
    return g_failures == 0 ? 0 : 1;
}
