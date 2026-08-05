// SPDX-License-Identifier: BSD-2-Clause
//
// Unit tests for the CineMate Log curve engine (cinepi/log_lut.hpp).
//
// Pure / self-contained: no libcamera, no Redis, no jsoncpp. Build & run:
//   c++ -std=c++17 -O2 -Wall -Wextra -I. tests/log_lut_test.cpp -o /tmp/log_lut_test && /tmp/log_lut_test
// (or via meson: `meson test log_lut`).
//
// The point of this test is that the C++ curve reproduces
// resources/log_luts/gen_cinemate_log.py BIT-FOR-BIT — if it does not, a recorded
// DNG carries a LinearizationTable that does not invert its own pixels and the
// footage decodes wrong, silently. So every expected value below was lifted from
// the generator / the shipped JSON specs, not from running this C++:
//
//   Tier 1 — curve shape       (boundaries, monotonicity, footroom)
//   Tier 2 — golden values     (spot entries + a hash of every table entry)
//   Tier 3 — round-trip error  (inverse[forward[L]] vs L, in milli-stops)
//   Tier 4 — params validation (malformed specs must be rejected, not built)
//
// Regenerate the golden constants with:
//   cd resources/log_luts && python3 <the gen_golden.py snippet in git history>
// or simply re-derive them from cinemate_log_*.json's linearization_table.

#include "cinepi/log_lut.hpp"
// The encode hook hands LUT output straight to this packer; test_encode_row
// proves the two agree on the sample domain.
#include "cinepi/dng_pack.hpp"

#include <cfenv>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

// ── tiny test harness (same shape as dng_pack_test.cpp) ──────────────────────
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

// ── the three shipped specs, params copied from resources/log_luts/*.json ────
struct Case {
    const char *name;
    LogLutParams p;
    uint64_t fwd_hash;          // FNV-1a/64 over the whole forward table
    uint64_t inv_hash;          // FNV-1a/64 over the whole inverse table
    double max_roundtrip_mstop; // tight regression bound; the feature budget is 20
};

// mu, BL, WL, src, tgt, F, foot
static const Case kCases[] = {
    { "16to12", { 10000.0, 3200, 65535, 16, 12, 32, 487 },
      0x7A45FDCDFFFAF83AULL, 0x7121F2369923A580ULL,  4.0 },
    { "16to10", { 10000.0, 3200, 65535, 16, 10, 32, 487 },
      0x82361D8633988071ULL, 0xE6083833522B394EULL, 14.0 },
    { "12to10", {  1500.0,  200,  4095, 12, 10, 32, 200 },
      0x0AC947B367CCB93AULL, 0x8EAE6265303D80A2ULL, 11.0 },
};

// The documented feature budget from the offline validation (VERIFIED.md §3).
static const double kBudgetMilliStops = 20.0;

// FNV-1a/64 over each entry's two little-endian bytes. Matches the Python that
// produced the constants above; a single wrong entry anywhere changes it.
static uint64_t fnv1a(const uint16_t *v, size_t n)
{
    uint64_t h = 0xCBF29CE484222325ULL;
    for (size_t i = 0; i < n; ++i) {
        h = (h ^ static_cast<uint8_t>(v[i] & 0xFF)) * 0x100000001B3ULL;
        h = (h ^ static_cast<uint8_t>(v[i] >> 8))   * 0x100000001B3ULL;
    }
    return h;
}

// ── TIER 0: the rounding mode the curve depends on ───────────────────────────
//
// The generator rounds with numpy (ties-to-even). std::nearbyint only matches
// under FE_TONEAREST. Nothing in cinepi-raw changes the mode, but if a linked
// library ever did, every table entry would shift silently — so assert it.
static void test_rounding_mode()
{
    std::printf("test_rounding_mode\n");
    CHECK(std::fegetround() == FE_TONEAREST, "default FP rounding is FE_TONEAREST");
    CHECK(std::nearbyint(0.5) == 0.0, "nearbyint(0.5) == 0 (ties-to-even, not 1)");
    CHECK(std::nearbyint(1.5) == 2.0, "nearbyint(1.5) == 2 (ties-to-even)");
    CHECK(std::nearbyint(2.5) == 2.0, "nearbyint(2.5) == 2 (ties-to-even, not 3)");
}

// ── TIER 1: curve shape ──────────────────────────────────────────────────────
static void test_shape(const Case &c)
{
    std::printf("test_shape %s\n", c.name);
    LogLut lut;
    CHECK(lut.build(c.p), "build succeeds");
    if (!lut.valid())
        return;

    const LogLutParams &p = c.p;
    const uint16_t *fwd = lut.forward();
    const uint16_t *inv = lut.inverse();
    const int F = p.footroom_codes, CMAX = p.code_max();

    CHECK(lut.forward_size() == (size_t{1} << p.source_bits), "forward table is 2^source_bits");
    CHECK(lut.inverse_size() == (size_t{1} << p.target_bits), "inverse table is 2^target_bits");

    // Boundaries: black maps to the first picture code, white to the last, and
    // both invert exactly. These four are what a DNG reader keys off.
    CHECK(fwd[p.black_level] == F,    "forward[BL] == F");
    CHECK(inv[F] == p.black_level,    "inverse[F] == BL");
    CHECK(fwd[p.white_level] == CMAX, "forward[WL] == CMAX");
    CHECK(inv[CMAX] == p.white_level, "inverse[CMAX] == WL");

    // Footroom: the codes below F cover [BL-foot, BL) and must decode below black,
    // so sub-black noise survives instead of being rectified into code 0.
    CHECK(fwd[p.black_level - 1] == F - 1, "forward[BL-1] == F-1 (top footroom bin)");
    CHECK(fwd[p.black_level - p.footroom_lsb] == 0, "forward[BL-foot] == 0 (bottom footroom bin)");
    CHECK(fwd[0] == 0, "forward[0] == 0");
    bool foot_below_black = true;
    for (int cde = 0; cde < F; ++cde)
        if (inv[cde] >= p.black_level)
            foot_below_black = false;
    CHECK(foot_below_black, "every footroom code decodes strictly below BL");
    CHECK(inv[0] >= p.black_level - p.footroom_lsb, "inverse[0] >= BL-foot (footroom bin centre)");

    // Monotonic non-decreasing in both directions — required for the curve to be
    // invertible at all, and DNG readers assume it of a LinearizationTable.
    size_t fwd_break = 0, inv_break = 0;
    for (size_t i = 1; i < lut.forward_size(); ++i)
        if (fwd[i] < fwd[i - 1] && !fwd_break) fwd_break = i;
    for (size_t i = 1; i < lut.inverse_size(); ++i)
        if (inv[i] < inv[i - 1] && !inv_break) inv_break = i;
    if (fwd_break) std::printf("  forward drops at %zu: %u -> %u\n",
                               fwd_break, fwd[fwd_break - 1], fwd[fwd_break]);
    if (inv_break) std::printf("  inverse drops at %zu: %u -> %u\n",
                               inv_break, inv[inv_break - 1], inv[inv_break]);
    CHECK(fwd_break == 0, "forward table is monotonic non-decreasing");
    CHECK(inv_break == 0, "inverse table is monotonic non-decreasing");

    // The tables must agree with the scalar curve they are built from.
    CHECK(fwd[p.black_level + 1] == log_encode_code(p.black_level + 1, p), "table == scalar encode");
    CHECK(inv[F + 1] == log_decode_level(F + 1, p), "table == scalar decode");
    CHECK(lut.encode(p.white_level) == CMAX, "encode() clamps/looks up at WL");
    CHECK(lut.encode(0xFFFFFFFFu) == CMAX, "encode() clamps an out-of-range input");
}

// ── TIER 2: golden values from the shipped JSON specs ────────────────────────
//
// inverse[] IS the spec's linearization_table, so these are literal copies from
// resources/log_luts/cinemate_log_*.json. forward[] is not in the JSON; those came
// from running the generator's enc().
struct Golden { int index; uint16_t value; };

static const Golden kInv16to12[] = {
    {0,2721},{1,2736},{15,2949},{31,3192},{32,3200},{33,3200},{100,3201},{2047,3794},{4094,65394},{4095,65535} };
static const Golden kFwd16to12[] = {
    {0,0},{2713,0},{3199,31},{3200,32},{3201,98},{3208,396},{3300,1283},{34367,3789},{65534,4095},{65535,4095} };
static const Golden kInv16to10[] = {
    {0,2721},{1,2736},{15,2949},{31,3192},{32,3200},{33,3200},{100,3205},{511,3729},{1022,64958},{1023,65535} };
static const Golden kFwd16to10[] = {
    {0,0},{2713,0},{3199,31},{3200,32},{3201,48},{3208,121},{3300,337},{34367,948},{65534,1023},{65535,1023} };
static const Golden kInv12to10[] = {
    {0,3},{1,9},{15,97},{31,197},{32,200},{33,200},{100,202},{511,286},{1022,4066},{1023,4095} };
static const Golden kFwd12to10[] = {
    {0,0},{199,31},{200,32},{201,76},{208,223},{300,530},{2047,922},{2147,929},{4094,1023},{4095,1023} };

static void check_golden(const char *what, const char *case_name,
                         const uint16_t *tbl, const Golden *g, size_t n)
{
    for (size_t i = 0; i < n; ++i) {
        if (tbl[g[i].index] != g[i].value) {
            std::printf("  %s %s[%d]: got %u exp %u\n",
                        case_name, what, g[i].index, tbl[g[i].index], g[i].value);
            ++g_failures;
            ++g_checks;
            return;
        }
    }
    ++g_checks;
}

static void test_golden(const Case &c, const Golden *fwd_g, const Golden *inv_g, size_t n)
{
    std::printf("test_golden %s\n", c.name);
    LogLut lut;
    if (!lut.build(c.p))
        return;

    check_golden("forward", c.name, lut.forward(), fwd_g, n);
    check_golden("inverse", c.name, lut.inverse(), inv_g, n);

    // Whole-table hashes: the spot checks above localise a break, these catch one
    // anywhere. A mismatch means the C++ curve diverged from the generator.
    const uint64_t fh = fnv1a(lut.forward(), lut.forward_size());
    const uint64_t ih = fnv1a(lut.inverse(), lut.inverse_size());
    if (fh != c.fwd_hash)
        std::printf("  %s forward hash: got 0x%016llX exp 0x%016llX\n",
                    c.name, (unsigned long long)fh, (unsigned long long)c.fwd_hash);
    if (ih != c.inv_hash)
        std::printf("  %s inverse hash: got 0x%016llX exp 0x%016llX\n",
                    c.name, (unsigned long long)ih, (unsigned long long)c.inv_hash);
    CHECK(fh == c.fwd_hash, "forward table matches the generator entry-for-entry");
    CHECK(ih == c.inv_hash, "inverse table matches the shipped linearization_table");
}

// ── TIER 3: round-trip error ─────────────────────────────────────────────────
//
// Same metric as cinemate-log/analyze_dng.py: error in the above-black excess,
// floored at 1 LSB, expressed in milli-stops. Swept uniformly over [BL, WL]
// rather than weighted by a real frame's histogram, so it is the harsher read.
static void test_roundtrip(const Case &c)
{
    std::printf("test_roundtrip %s\n", c.name);
    LogLut lut;
    if (!lut.build(c.p))
        return;

    const LogLutParams &p = c.p;
    const uint16_t *fwd = lut.forward(), *inv = lut.inverse();
    double worst = 0.0;
    int worst_L = 0;
    for (int L = p.black_level; L <= p.white_level; ++L) {
        const double rec = std::max(1.0, static_cast<double>(inv[fwd[L]]) - p.black_level);
        const double ref = std::max(1.0, static_cast<double>(L) - p.black_level);
        const double e = std::fabs(std::log2(rec / ref)) * 1000.0;
        if (e > worst) { worst = e; worst_L = L; }
    }
    std::printf("  worst %.2f milli-stops at L=%d (budget %.0f, bound %.0f)\n",
                worst, worst_L, kBudgetMilliStops, c.max_roundtrip_mstop);
    CHECK(worst < kBudgetMilliStops, "round-trip within the feature budget");
    CHECK(worst < c.max_roundtrip_mstop, "round-trip within this curve's measured bound");
}

// ── TIER 3b: the encode hook's row path ──────────────────────────────────────
//
// Pass 4 rests on one claim that neither the curve tests nor the packer tests
// cover on their own: a forward code is ALREADY a right-justified target-depth
// sample, so the DNG writer can hand LUT output straight to pack_row_12bit with
// no shifting. If that were wrong the recording would be silently mis-scaled by
// 16x, and every test above would still pass. So encode a row through the real
// LUT, pack it with the real packer, and unpack it with an inverse derived from
// the DNG contiguous-12 bit layout rather than from pack_row_12bit's algebra.
static void unpack_row_12bit(const uint8_t *src, uint16_t *dst, uint32_t width)
{
    // Contiguous 12-bit, MSB-first: 2 px per 3 bytes.
    //   px0 = b0[7:0] b1[7:4]      px1 = b1[3:0] b2[7:0]
    for (uint32_t x = 0; x + 1 < width; x += 2, src += 3) {
        dst[x]     = static_cast<uint16_t>((static_cast<uint16_t>(src[0]) << 4) | (src[1] >> 4));
        dst[x + 1] = static_cast<uint16_t>((static_cast<uint16_t>(src[1] & 0x0F) << 8) | src[2]);
    }
}

// Same claim at the 10-bit target Pass 5 adds. Walks the MSB-first bitstream one
// bit at a time straight from the DNG contiguous-10 layout, so it shares no
// algebra with pack_row_10bit and the round-trip stays an independent check.
// (dng_pack_test.cpp has its own copy for the same reason; dng_pack.hpp
// deliberately does not export one — nothing in cinepi-raw decodes such a row.)
static void unpack_row_10bit(const uint8_t *src, uint16_t *dst, uint32_t width)
{
    for (uint32_t x = 0; x < width; ++x) {
        uint16_t v = 0;
        for (size_t b = 0; b < 10; ++b) {
            const size_t p = static_cast<size_t>(x) * 10 + b;   // MSB-first bit index
            v = static_cast<uint16_t>((v << 1) | ((src[p >> 3] >> (7 - (p & 7))) & 1));
        }
        dst[x] = v;
    }
}

static void test_encode_row(const Case &c)
{
    std::printf("test_encode_row %s\n", c.name);
    LogLut lut;
    if (!lut.build(c.p))
        return;

    const LogLutParams &p = c.p;
    const int cmax = p.code_max();

    // A linear ramp over the source range, with WL forced into the last sample
    // so the row spans code 0..CMAX. Without that the ramp stops short of white
    // (64512 of 65535 at 16 bit) and the packer is never exercised on 0xFFF —
    // the one code where a bad shift or mask is most likely to show up.
    const uint32_t kWidth = 64;
    std::vector<uint16_t> src(kWidth), dst(kWidth);
    const unsigned step = (1u << p.source_bits) / kWidth;
    for (uint32_t x = 0; x < kWidth; ++x)
        src[x] = static_cast<uint16_t>(x * step);
    src[kWidth - 1] = static_cast<uint16_t>(p.white_level);

    lut.encode_row(src.data(), dst.data(), kWidth);
    size_t mismatch = 0;
    for (uint32_t x = 0; x < kWidth; ++x)
        if (dst[x] != lut.encode(src[x]))
            ++mismatch;
    CHECK(mismatch == 0, "encode_row matches per-sample encode()");
    CHECK(dst.front() == 0 && dst.back() == cmax, "the test row spans code 0..CMAX");

    // In-place is safe — the encoder decompresses COMP1 and log-encodes through
    // one scratch row, so dst aliases src on every compressed frame.
    std::vector<uint16_t> inplace = src;
    lut.encode_row(inplace.data(), inplace.data(), kWidth);
    CHECK(inplace == dst, "encode_row is safe when dst aliases src");

    // The composition, at whichever target depth the DNG writer dispatches to.
    {
        std::vector<uint8_t>  packed((kWidth * p.target_bits + 7) / 8);
        std::vector<uint16_t> back(kWidth);
        if (p.target_bits == 12) {
            pack_row_12bit(dst.data(), packed.data(), kWidth);
            unpack_row_12bit(packed.data(), back.data(), kWidth);
            CHECK(back == dst, "pack_row_12bit round-trips the log codes unshifted");
        } else {
            pack_row_10bit(dst.data(), packed.data(), kWidth);
            unpack_row_10bit(packed.data(), back.data(), kWidth);
            CHECK(back == dst, "pack_row_10bit round-trips the log codes unshifted");
        }
    }

    // ── the PiSP source container, for the 12-bit sensor modes Pass 5 admits ──
    //
    // A 12-bit mode does not arrive as SBGGR12. On PiSP it is SRGGB16 holding
    // value << 4, so the row path shifts it back down before indexing a
    // 4096-entry forward table. The 16-bit curves need no shift — there the
    // container IS the source domain.
    if (p.source_bits == 12) {
        std::vector<uint16_t> msb(kWidth), just(kWidth), viaShift(kWidth), direct(kWidth);
        for (uint32_t x = 0; x < kWidth; ++x)
            msb[x] = static_cast<uint16_t>(src[x] << 4);      // what the DMA buffer holds

        right_justify_row(msb.data(), just.data(), kWidth, 4);
        CHECK(just == src, "right_justify_row(v << 4, 4) recovers the sensor code");

        lut.encode_row(just.data(), viaShift.data(), kWidth);
        lut.encode_row(src.data(), direct.data(), kWidth);
        CHECK(viaShift == direct, "the shifted container encodes like the raw sensor code");

        // And the shift is load-bearing. Feeding the container straight in makes
        // encode() clamp every over-range sample to the last forward entry — the
        // silent all-white frame the guard in setup_encoder() exists to prevent.
        // Both the count and the code are derived from the function bodies
        // (encode() clamps to forward_.size()-1; forward[WL] is CMAX), not typed in.
        std::vector<uint16_t> unshifted(kWidth);
        lut.encode_row(msb.data(), unshifted.data(), kWidth);
        const uint16_t top_index = static_cast<uint16_t>((1u << p.source_bits) - 1u);
        size_t expect = 0, got = 0;
        for (uint32_t x = 0; x < kWidth; ++x) {
            if (msb[x] >= top_index) ++expect;
            if (unshifted[x] == lut.encode(top_index)) ++got;
        }
        CHECK(expect > 0 && got == expect,
              "skipping the shift clamps every over-range sample to CMAX");
        CHECK(unshifted != direct, "so the unshifted row is NOT the correct encoding");
    }
}

// ── TIER 3b: does this curve belong to this sensor? ──────────────────────────
//
// Specs are found by depth pair only, so cinemate_log_12to10 (black 200) is what
// ANY 12-bit mode gets — including sensors whose black is not 3200 in the 16-bit
// domain. The reported levels below are the rpi.black_level values shipped in
// libcamera/src/ipa/rpi/pisp/data/*.json, not invented.
static void test_black_level_guard()
{
    std::printf("test_black_level_guard\n");

    struct Sensor { const char *name; float bl16; };
    static const Sensor kSensors[] = {
        { "imx585/imx283",           3200.f },   // what the 12to10 spec assumes
        { "imx290/296/415/462",      3840.f },
        { "imx219/477/519/708/...",  4096.f },
    };

    const LogLutParams &p12 = kCases[2].p;      // 12to10, black 200
    CHECK(p12.source_bits == 12 && p12.black_level == 200, "using the 12to10 spec");

    for (const Sensor &s : kSensors) {
        // Independent derivation of the scaling: integer rational arithmetic
        // instead of the float expression the function uses.
        const int src_white = (1 << p12.source_bits) - 1;
        const int expect = static_cast<int>(
            (static_cast<long long>(s.bl16) * src_white * 2 + 65535) / (2LL * 65535));
        const int got = log_lut_scale_black(p12, s.bl16);
        CHECK(got == expect, "scaled black matches an independent rational derivation");

        const float off = std::fabs(static_cast<float>(got - p12.black_level));
        const bool accepted = off <= log_lut_black_tolerance(p12);
        // Only the sensor the spec was fitted to may be accepted.
        const bool is_own_sensor = (s.bl16 == 3200.f);
        CHECK(accepted == is_own_sensor,
              is_own_sensor ? "the spec's own sensor is accepted"
                            : "a sensor with a different black level is refused");
    }

    // The tolerance is one footroom code, derived from the params.
    CHECK(log_lut_black_tolerance(p12) == static_cast<float>(p12.footroom_lsb) / p12.footroom_codes,
          "tolerance is foot/F");
    // A drift smaller than one footroom code must NOT trip the guard: the toe
    // moves less than the quantisation it controls, and channels jitter.
    {
        const float tol = log_lut_black_tolerance(p12);
        const int near_miss = p12.black_level + static_cast<int>(tol) - 1;
        CHECK(std::fabs(static_cast<float>(near_miss - p12.black_level)) <= tol,
              "sub-footroom-code drift is tolerated");
    }

    // 16-bit sources are the identity — only imx585 has a 16-bit mode and the
    // 16to12/16to10 specs are fitted to exactly its black level.
    for (int i = 0; i < 2; ++i) {
        const LogLutParams &p16 = kCases[i].p;
        CHECK(log_lut_scale_black(p16, 3200.f) == p16.black_level,
              "16-bit source: reported black scales to the spec's black unchanged");
    }
}

// ── TIER 4: params validation ────────────────────────────────────────────────
static void test_validation()
{
    std::printf("test_validation\n");
    const LogLutParams good = kCases[0].p;
    CHECK(good.valid(), "the shipped 16to12 params validate");

    LogLut lut;
    CHECK(!lut.build(LogLutParams{}), "default-constructed params are rejected");
    CHECK(!lut.valid(), "a rejected build leaves the LUT empty");

    auto bad = [&](LogLutParams p, const char *msg) {
        CHECK(!p.valid(), msg);
        LogLut l;
        CHECK(!l.build(p), "build() refuses it too");
    };
    { LogLutParams p = good; p.source_bits = 17;    bad(p, "source_bits > 16 rejected"); }
    { LogLutParams p = good; p.target_bits = 0;     bad(p, "target_bits < 1 rejected"); }
    { LogLutParams p = good; p.target_bits = 16; p.source_bits = 12;
                                                   bad(p, "target deeper than source rejected"); }
    { LogLutParams p = good; p.mu = 0.0;           bad(p, "mu == 0 rejected"); }
    { LogLutParams p = good; p.white_level = 3200; bad(p, "white_level <= black_level rejected"); }
    { LogLutParams p = good; p.white_level = 70000;bad(p, "white_level beyond source range rejected"); }
    { LogLutParams p = good; p.black_level = -1;   bad(p, "negative black_level rejected"); }
    { LogLutParams p = good; p.footroom_codes = 4095;
                                                   bad(p, "footroom_codes leaving TOP < 1 rejected"); }
    { LogLutParams p = good; p.footroom_lsb = 3201;bad(p, "footroom reaching below zero rejected"); }

    // Footroom disabled (v1 behaviour) is a legal curve, not an error.
    LogLutParams v1 = good;
    v1.footroom_codes = 0;
    v1.footroom_lsb = 0;
    LogLut l1;
    CHECK(v1.valid() && l1.build(v1), "footroom_codes == 0 (v1 curve) still builds");
    if (l1.valid()) {
        CHECK(l1.forward()[good.black_level] == 0, "v1: forward[BL] == 0 (no footroom)");
        CHECK(l1.forward()[good.black_level - 1] == 0, "v1: sub-black crushes to 0");
        CHECK(l1.inverse()[0] == good.black_level, "v1: inverse[0] == BL");
    }
}

int main()
{
    std::printf("=== log_lut unit tests ===\n");
    test_rounding_mode();

    for (const Case &c : kCases) {
        test_shape(c);
        test_roundtrip(c);
        test_encode_row(c);
    }
    test_golden(kCases[0], kFwd16to12, kInv16to12, 10);
    test_golden(kCases[1], kFwd16to10, kInv16to10, 10);
    test_golden(kCases[2], kFwd12to10, kInv12to10, 10);

    test_black_level_guard();
    test_validation();

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    if (g_failures == 0) std::printf("ALL TESTS PASSED\n");
    return g_failures == 0 ? 0 : 1;
}
