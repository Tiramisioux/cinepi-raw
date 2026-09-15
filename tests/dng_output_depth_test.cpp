// SPDX-License-Identifier: BSD-2-Clause
//
// Unit tests for the linear DNG output-depth rule (cinepi/dng_output_depth.hpp).
//
// Pure / self-contained: no libcamera, no Redis. Build & run:
//   c++ -std=c++17 -O2 -I.. tests/dng_output_depth_test.cpp -o /tmp/dng_output_depth_test && /tmp/dng_output_depth_test
// (or via meson: `meson test dng_output_depth`).
//
// The rule this pins is the one that made native 10-bit sensor modes cost
// 1.5 B/px for 1.25 B/px of information: on a Pi 5 every raw stream arrives in
// a 16-bit container, so "anything that isn't 16 packs to 12" silently claimed
// the 10-bit modes too. The cases below are written as a truth table over
// (container, sensor depth, trusted, packed, compressed) so that a future edit
// to the predicate has to break a named case rather than drift quietly.
//
// Two properties here are regression guards rather than new behaviour, and
// both were live bugs during development:
//   * A 12-bit COMP1 row must KEEP pack12 — dng_save()'s compressed branch
//     reads that flag to choose packed-12 over a 2 B/px verbatim write. (A
//     10-bit COMP1 row takes pack10 instead, and that branch reads it the same
//     way; see test_comp1.)
//   * The two flags must never both be set, at any input.

#include "cinepi/dng_output_depth.hpp"

#include <cstdio>
#include <vector>

// ── tiny test harness (same shape as ccmp_gate_test.cpp) ─────────────────────
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

static bool same(const DngOutputDepth &d, unsigned bits, unsigned shift,
                 bool pack12, bool pack10)
{
    if (d.bits == bits && d.shift == shift && d.pack12 == pack12 && d.pack10 == pack10)
        return true;
    std::printf("  got bits=%u shift=%u pack12=%d pack10=%d;"
                " expected bits=%u shift=%u pack12=%d pack10=%d\n",
                d.bits, d.shift, (int)d.pack12, (int)d.pack10,
                bits, shift, (int)pack12, (int)pack10);
    return false;
}

// Pi 5 / PiSP: every raw stream is an unpacked 16-bit container, so the sensor
// mode's depth is the only thing that says how many bits are real.
static void test_pi5_unpacked() {
    std::printf("test_pi5_unpacked\n");
    // The defect this whole change exists for: a native 10-bit mode.
    CHECK(same(resolve_dng_output_depth(16, 10, true, false, false), 10, 6, false, true),
          "pi5 10-bit SDR -> 10-bit codes, shift 6");
    // The long-standing 12-bit case must be byte-identical to before.
    CHECK(same(resolve_dng_output_depth(16, 12, true, false, false), 12, 4, true, false),
          "pi5 12-bit SDR -> 12-bit codes, shift 4");
    // A genuine 16-bit ClearHDR mode keeps every bit.
    CHECK(same(resolve_dng_output_depth(16, 16, true, false, false), 16, 0, false, false),
          "pi5 16-bit ClearHDR -> container verbatim");
}

// Pi 4 / VC4: rows arrive at their native depth, already right-justified or
// CSI2-packed. Narrowing must not fire at all — a shift here blacks the frame.
static void test_pi4_vc4() {
    std::printf("test_pi4_vc4\n");
    CHECK(same(resolve_dng_output_depth(10, 10, true, false, false), 10, 0, false, false),
          "vc4 unpacked 10-bit -> native, no shift");
    CHECK(same(resolve_dng_output_depth(10, 10, true, true, false), 10, 0, false, false),
          "vc4 CSI2-packed 10-bit -> native, no shift");
    CHECK(same(resolve_dng_output_depth(12, 12, true, false, false), 12, 0, false, false),
          "vc4 unpacked 12-bit -> native, no shift");
    CHECK(same(resolve_dng_output_depth(12, 12, true, true, false), 12, 0, false, false),
          "vc4 CSI2-packed 12-bit -> native, no shift");
}

// REGRESSION GUARD. dng_save() dispatches on bayer_format.compressed BEFORE
// either flag, and its compressed branch reads pack12/pack10 to choose among
// unpack_pisp_comp1_row_to_packed12(), _packed10() and a 2 B/px verbatim
// write. Clearing pack12 for a 12-bit COMP1 row would turn that take into a
// 16-bit file.
//
// A 10-bit COMP1 row takes pack10 as of 2026-09-15, which is the one case here
// that changed: the repack was measured rather than guessed at (the numbers are
// in dng_output_depth.hpp and dng_pack.hpp, pinned by tests/dng_pack_test.cpp).
static void test_comp1() {
    std::printf("test_comp1\n");
    CHECK(same(resolve_dng_output_depth(16, 12, true, false, true), 12, 4, true, false),
          "COMP1 12-bit KEEPS pack12 (else the take becomes 16-bit)");
    CHECK(same(resolve_dng_output_depth(16, 10, true, false, true), 10, 6, false, true),
          "COMP1 10-bit takes pack10 — the measured repack, not the padded 12");
    CHECK(same(resolve_dng_output_depth(16, 16, true, false, true), 16, 0, false, false),
          "COMP1 16-bit ClearHDR -> container verbatim");
    // The 12-bit COMP1 case and the 10-bit one must not collapse into each
    // other: they differ in every field, which is what makes the branch real.
    CHECK(!same(resolve_dng_output_depth(16, 10, true, false, true), 12, 4, true, false),
          "COMP1 10-bit is no longer the 12-bit answer");
}

// An untrusted snapshot describes the REQUEST, not this stream. Fail toward the
// larger file: keep the container rather than narrow on a value we cannot
// believe. Narrowing wrongly destroys real bits; not narrowing costs padding.
static void test_untrusted() {
    std::printf("test_untrusted\n");
    for (unsigned depth : {8u, 10u, 12u, 14u, 16u}) {
        const DngOutputDepth d = resolve_dng_output_depth(16, depth, false, false, false);
        CHECK(same(d, 16, 0, false, false), "untrusted -> container verbatim, no narrowing");
    }
}

// Unrecognised depths keep the long-standing 12-bit behaviour rather than
// deriving a packer nothing has tested. 0 is "unset"; 8 and 14 are reachable
// only through a stray redis write.
static void test_unrecognised_depths() {
    std::printf("test_unrecognised_depths\n");
    CHECK(same(resolve_dng_output_depth(16, 0, true, false, false), 12, 4, true, false),
          "depth 0 (unset) -> 12-bit, as before");
    CHECK(same(resolve_dng_output_depth(16, 8, true, false, false), 12, 4, true, false),
          "depth 8 -> 12-bit, no 8-bit packer is derived");
    CHECK(same(resolve_dng_output_depth(16, 14, true, false, false), 12, 4, true, false),
          "depth 14 -> 12-bit, no 14-bit packer is derived");
    CHECK(same(resolve_dng_output_depth(16, 99, true, false, false), 12, 4, true, false),
          "nonsense depth -> 12-bit, not derived");
}

// Invariants that must hold at EVERY input, swept exhaustively over the real
// domain. The mutual-exclusion one is the tripwire dng_encoder.cpp also checks
// at runtime; the shift one is what keeps a packer from being handed a row it
// cannot justify.
static void test_invariants() {
    std::printf("test_invariants\n");
    bool both_set = false, bad_shift = false, bad_white = false, narrowed_up = false;
    for (unsigned container : {10u, 12u, 16u})
        for (unsigned depth = 0; depth <= 20; ++depth)
            for (int trusted = 0; trusted < 2; ++trusted)
                for (int packed = 0; packed < 2; ++packed)
                    for (int compressed = 0; compressed < 2; ++compressed) {
                        const DngOutputDepth d = resolve_dng_output_depth(
                            container, depth, trusted, packed, compressed);
                        if (d.pack12 && d.pack10) both_set = true;
                        // A narrowing flag implies exactly its own depth and a
                        // shift that lands the sensor's bits right-justified.
                        if (d.pack12 && (d.bits != 12 || d.shift != container - 12))
                            bad_shift = true;
                        if (d.pack10 && (d.bits != 10 || d.shift != container - 10))
                            bad_shift = true;
                        // Never widen: the strip can never claim more bits than
                        // the container physically carries.
                        if (d.bits > container) narrowed_up = true;
                        if (d.white() != (1u << d.bits) - 1u) bad_white = true;
                    }
    CHECK(!both_set,    "pack12 and pack10 are never both set, at any input");
    CHECK(!bad_shift,   "a set flag always implies its own depth and shift");
    CHECK(!narrowed_up, "output depth never exceeds the container depth");
    CHECK(!bad_white,   "white() is always (1 << bits) - 1");
}

// The shipped modes, named, so the table is readable as camera behaviour and
// not only as booleans. Sizes are the raw readout the encoder actually sees.
static void test_shipped_modes() {
    std::printf("test_shipped_modes\n");
    struct Mode { const char *what; unsigned depth; unsigned expect_bits; };
    const std::vector<Mode> pi5 = {
        { "imx585 3840x2160 10-bit SDR",  10, 10 },
        { "imx585 3840x2160 12-bit SDR",  12, 12 },
        { "imx585 3840x2200 16-bit HDR",  16, 16 },
        { "imx283 3936x2176 10-bit",      10, 10 },
        { "imx283 5568x3664 12-bit",      12, 12 },
        { "imx477 1332x990 10-bit",       10, 10 },
        { "imx296 1456x1088 10-bit",      10, 10 },
    };
    for (const Mode &m : pi5) {
        const DngOutputDepth d = resolve_dng_output_depth(16, m.depth, true, false, false);
        CHECK(d.bits == m.expect_bits, m.what);
    }
    // The saving that motivated the change, stated as bytes rather than a flag:
    // 3840x2160 at 10 vs 12 bits.
    const unsigned w = 3840, h = 2160;
    const unsigned row10 = (w * 10 + 7) / 8, row12 = (w * 12 + 7) / 8;
    CHECK(row10 * h == 10368000u, "3840x2160 at 10-bit is 10,368,000 B");
    CHECK(row12 * h == 12441600u, "3840x2160 at 12-bit is 12,441,600 B");
    CHECK(row12 * h - row10 * h == 2073600u, "the saving is 2,073,600 B/frame (16.7%)");
}

int main() {
    std::printf("=== dng_output_depth unit tests ===\n");
    test_pi5_unpacked();
    test_pi4_vc4();
    test_comp1();
    test_untrusted();
    test_unrecognised_depths();
    test_invariants();
    test_shipped_modes();

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    if (g_failures == 0) std::printf("ALL TESTS PASSED\n");
    return g_failures == 0 ? 0 : 1;
}
