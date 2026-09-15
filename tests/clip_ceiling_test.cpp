/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * clip_ceiling_test.cpp - the ClearHDR clamp detector, against real takes.
 *
 * Includes only cinepi/clip_ceiling.hpp and cinepi/dng_pack.hpp, so it builds
 * with nothing but the standard library and exercises the same code that ships.
 * Build & run:
 *   c++ -std=c++17 -O2 -I.. tests/clip_ceiling_test.cpp -o /tmp/clip_ceiling_test && /tmp/clip_ceiling_test
 * (or via meson: `meson test clip_ceiling`).
 *
 * THE POINT OF THIS TEST. The detector decides whether to overwrite a DNG's
 * WhiteLevel, and both ways of being wrong are real damage: miss a clamp and
 * the highlight stays magenta (the bug), invent one and every highlight above
 * the invented level is clipped to white (a worse bug, on footage that was
 * fine). So the fixtures are not synthetic frames — they are the per-channel
 * code histograms of ELEVEN DNGs this camera actually recorded, dumped straight
 * out of the files, carrying each take's own LinearizationTable where it had
 * one. Nine are ClearHDR and clamped, spanning every mode the camera offers: 4K
 * and binned HD, stored at 10, 12 and 16 bits, log on and log off. Two are
 * native 10-bit SDR that genuinely reach their white and must be left alone —
 * the control group, and the one that catches a detector which has started
 * inventing clamps on healthy footage.
 *
 * A NOTE ON READING THESE FILES, because it cost an afternoon. The strips are
 * DNG's own CONTIGUOUS big-endian bit packing (pixel pairs across 3 bytes at 12
 * bits, quads across 5 at 10), NOT the MIPI CSI-2 layout the sensor delivers.
 * Decoding a 10-bit strip as MIPI produces a plausible-looking histogram that
 * clips exactly at full scale — because an all-ones clipped region reads as
 * all-ones under either interpretation — so the error hides precisely where it
 * matters most. The giveaway is same-colour neighbour distance: 0.5 under the
 * right unpacking, 271 under the wrong one.
 *
 * BOTH ENCODINGS ARE HERE ON PURPOSE. The first version of the detector counted
 * stored codes with code-space thresholds, passed every 12-bit fixture, and
 * REFUSED on a 16-bit frame whose clamp is obvious to the eye — because a clamp
 * that spans ~20 codes through the log curve spans ~1000 in the linear
 * container. The fix was to count LINEARISED values with relative thresholds.
 * domain_transfer_is_invariant() below is the standing guard for that: the same
 * physical clamp, expressed through a compressive curve and directly, must give
 * the same answer. A future "simplification" back to code-space thresholds
 * fails there and in the lin_* fixtures.
 *
 * The expected ceilings are measurements, not hand-tuned constants. Each was
 * read off the recorded file independently of this code, and two were confirmed
 * end-to-end by patching only tag 0xC61D in a byte copy: F06 takes LibRaw's
 * cast over the blown area from 12.0% to 0.0%, F03 from 13.0% to 0.6%.
 *
 * The synthetic cases cover what no take in hand contains, and one is a
 * REGRESSION GUARD rather than a feature test: the tungsten case fails against
 * a min-over-max convergence rule. lessons/hardware-log.md records that exact
 * rule shipping in clip_plateau.hpp, surviving a fix to its sibling, and
 * costing a week of debugging at the wrong layer.
 */

#include "cinepi/clip_ceiling.hpp"
#include "cinepi/dng_pack.hpp"

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace
{

int g_failures = 0;

void check(bool cond, const std::string &name, const std::string &detail = "")
{
    std::cout << (cond ? "  ok   " : "  FAIL ") << name;
    if (!detail.empty())
        std::cout << "  " << detail;
    std::cout << "\n";
    if (!cond)
        ++g_failures;
}

/* Where the recorded histograms live. meson passes the source dir; the
 * fallback lets the test run from the repo root with a plain c++ line. */
#ifndef CLIP_CEILING_DATA_DIR
#define CLIP_CEILING_DATA_DIR "tests/data"
#endif

static const uint8_t kRGGB[4] = { kClipChanR, kClipChanG, kClipChanG, kClipChanB };

/* One take's per-channel code histogram plus its level tags and, where the
 * file had one, its LinearizationTable. Histograms are sparse `code count`
 * pairs — the 16-bit takes occupy only ~15k of 65536 bins. */
struct Fixture
{
    unsigned code_max = 0, white = 0, black = 0;
    std::vector<uint16_t> lut;
    std::vector<std::pair<unsigned, uint32_t>> ch[kClipChanCount];
    bool ok = false;
};

Fixture read_fixture(const std::string &name, std::string &err)
{
    Fixture fx;
    const std::string path = std::string(CLIP_CEILING_DATA_DIR) + "/clip_ceiling_" + name + ".hist";
    std::ifstream f(path);
    if (!f.good()) { err = "cannot open fixture '" + path + "'"; return fx; }

    std::string line;
    while (std::getline(f, line))
    {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream is(line);
        std::string tag;
        is >> tag;
        if (tag == "code_max")    { is >> fx.code_max; continue; }
        if (tag == "white_level") { is >> fx.white;    continue; }
        if (tag == "black_level") { is >> fx.black;    continue; }
        if (tag == "lut")
        {
            unsigned v;
            while (is >> v) fx.lut.push_back(static_cast<uint16_t>(v));
            continue;
        }
        const int c = tag == "R" ? kClipChanR : tag == "G" ? kClipChanG : tag == "B" ? kClipChanB : -1;
        if (c < 0) { err = "unexpected row '" + tag + "' in " + path; return fx; }
        unsigned code; uint32_t n;
        while (is >> code >> n) fx.ch[c].push_back({ code, n });
    }
    if (!fx.white) { err = "no white_level in " + path; return fx; }
    fx.ok = true;
    return fx;
}

void load(ClipCeilingDetector &d, const Fixture &fx)
{
    d.reset(fx.white, fx.black, kRGGB, fx.lut.empty() ? nullptr : fx.lut.data(), fx.lut.size());
    for (int c = 0; c < kClipChanCount; ++c)
        for (size_t i = 0; i < fx.ch[c].size(); ++i)
            for (uint32_t n = 0; n < fx.ch[c][i].second; ++n)
                d.add(fx.ch[c][i].first, static_cast<uint8_t>(c));
}

/* Read back the contiguous big-endian bit layout dng_pack.hpp's packers emit —
 * DNG's own, MSB-first across the row, which is NOT the MIPI CSI-2 layout the
 * sensor delivers. Written out here rather than imported so the comparison in
 * shift_matches_the_real_packers() is against the file's real bytes and not
 * circular. */
void unpack_contiguous_12bit(const uint8_t *src, uint16_t *dst, uint32_t width)
{
    for (uint32_t x = 0; x + 2u <= width; x += 2u, src += 3)
    {
        dst[x]      = static_cast<uint16_t>((src[0] << 4) | (src[1] >> 4));
        dst[x + 1u] = static_cast<uint16_t>(((src[1] & 0x0F) << 8) | src[2]);
    }
}
void unpack_contiguous_10bit(const uint8_t *src, uint16_t *dst, uint32_t width)
{
    for (uint32_t x = 0; x + 4u <= width; x += 4u, src += 5)
    {
        dst[x]      = static_cast<uint16_t>((src[0] << 2) | (src[1] >> 6));
        dst[x + 1u] = static_cast<uint16_t>(((src[1] & 0x3F) << 4) | (src[2] >> 4));
        dst[x + 2u] = static_cast<uint16_t>(((src[2] & 0x0F) << 6) | (src[3] >> 2));
        dst[x + 3u] = static_cast<uint16_t>(((src[3] & 0x03) << 8) | src[4]);
    }
}

/* Adds `count` samples of `channel` spread evenly over [lo, hi] — how a noisy
 * band is described below without transcribing a histogram. */
void band(ClipCeilingDetector &d, uint8_t channel, unsigned lo, unsigned hi, unsigned long count)
{
    const unsigned span = hi - lo + 1u;
    for (unsigned long i = 0; i < count; ++i)
        d.add(lo + static_cast<unsigned>(i % span), channel);
}

void real_takes()
{
    /* name, whether the take is clamped, and the ceiling measured off that DNG.
     * log_f06 and log_f13 are the same operating point minutes apart and agree
     * to the value; log_f07 is a different session and sits far lower, which is
     * exactly why this is measured per take rather than tabulated per mode. */
    struct Case { const char *name; bool clamped; unsigned ceiling; const char *note; };
    const Case cases[] = {
        /* Nine ClearHDR takes spanning every mode the camera offers — 4K and
         * binned HD, stored at 10, 12 and 16 bits, log on and log off. All nine
         * are clamped and all nine render magenta as recorded. One set of
         * constants has to accept all of them, because the clamp's width in
         * CODE space differs by 50x across these encodings while the physical
         * clamp does not. */
        { "clearhdr_4k_log12_a", true,  58352, "4K, 16-bit ClearHDR log-encoded to 12: clamp at 89.0%" },
        { "clearhdr_4k_log12_b", true,  58352, "same operating point, minutes later — agrees to the value" },
        { "clearhdr_4k_log12_c", true,  48172, "another session: 73.5%, same detector, no retune" },
        { "clearhdr_4k_log12_d", true,  54261, "later session again: 82.8%" },
        { "clearhdr_hd_log12",   true,  35941, "BINNED HD: 54.8%, the lowest clamp on record" },
        { "clearhdr_4k_log10",   true,  58440, "log-encoded to 10 bits instead of 12" },
        { "clearhdr_hd_log10",   true,  54960, "binned HD, log to 10 bits" },
        { "clearhdr_4k_lin16_a", true,  57443, "16-bit linear, log OFF — the encoding v1 refused" },
        { "clearhdr_4k_lin16_b", true,  58505, "16-bit linear, second take" },
        /* The control group: native 10-bit SDR genuinely reaches its white, so
         * there is nothing to declare and the detector must stay out of the
         * way. If these ever start reporting a ceiling, the detector has begun
         * inventing clamps on healthy footage, which is the worse failure. */
        { "sdr_4k_10bit_a",      false, 0,     "native 10-bit SDR: reaches 1023, must stay untouched" },
        { "sdr_4k_10bit_b",      false, 0,     "native 10-bit SDR, second take" },
    };

    for (const Case &c : cases)
    {
        std::string err;
        Fixture fx = read_fixture(c.name, err);
        if (!fx.ok) { check(false, std::string("fixture ") + c.name, err); continue; }

        ClipCeilingDetector d;
        load(d, fx);
        const ClipCeilingDetector::Result r = d.detect();
        const std::string tag = std::string(c.name) + ": ";

        check(r.found == c.clamped, tag + (c.clamped ? "clamp detected" : "no clamp claimed"),
              std::string(c.note) + (r.found ? "" : std::string("  [why: ") + (r.why ? r.why : "?") + "]"));
        if (c.clamped && r.found)
        {
            check(r.ceiling == c.ceiling, tag + "ceiling is the measured clamp body",
                  "got " + std::to_string(r.ceiling) + ", measured " + std::to_string(c.ceiling));
            /* Never claim a ceiling at or above the container — that would be a
             * WhiteLevel raise, which is meaningless. */
            check(r.ceiling < fx.white, tag + "ceiling is below the declared white",
                  std::to_string(r.ceiling) + " < " + std::to_string(fx.white));
        }
    }
}

void domain_transfer_is_invariant()
{
    /* THE GUARD FOR THE BUG THAT SHIPPED. One physical clamp, described twice:
     * once as linear values, once as codes through a compressive curve that
     * squeezes the same light into a fifth of the code space. A detector whose
     * thresholds live in code space gives two different answers here — which is
     * exactly how v1 passed every 12-bit take and refused a 16-bit one. */
    const unsigned WHITE = 65535, BLACK = 3200;
    const unsigned CLAMP = 57400;                  /* where the light stops     */
    const unsigned SPREAD = 900;                   /* the body's width in light */

    /* A compressive curve: code c (0..4095) -> linear. Deliberately steep at the
     * top so the clamp body lands on only a handful of codes, like the real log
     * curve does. */
    std::vector<uint16_t> lut(4096);
    for (unsigned c = 0; c < 4096; ++c)
    {
        const double x = static_cast<double>(c) / 4095.0;
        const double lin = BLACK + (WHITE - BLACK) * (std::exp(x * 4.0) - 1.0) / (std::exp(4.0) - 1.0);
        lut[c] = static_cast<uint16_t>(lin + 0.5);
    }
    /* Nearest code for a linear value, so the same light can be fed both ways. */
    auto code_for = [&lut](unsigned lin) {
        unsigned best = 0; long bestd = 1L << 30;
        for (unsigned c = 0; c < 4096; ++c)
        {
            const long dd = std::labs(static_cast<long>(lut[c]) - static_cast<long>(lin));
            if (dd < bestd) { bestd = dd; best = c; }
        }
        return best;
    };

    ClipCeilingDetector direct, viacurve;
    direct.reset(WHITE, BLACK, kRGGB);
    viacurve.reset(WHITE, BLACK, kRGGB, lut.data(), lut.size());

    for (int c = 0; c < kClipChanCount; ++c)
    {
        for (unsigned long i = 0; i < 400000; ++i)
        {
            const unsigned lin = CLAMP - SPREAD / 2u + static_cast<unsigned>(i % SPREAD);
            direct.add(lin, static_cast<uint8_t>(c));
            viacurve.add(code_for(lin), static_cast<uint8_t>(c));
        }
        for (unsigned long i = 0; i < 200000; ++i)
        {
            const unsigned lin = 8000u + static_cast<unsigned>(i % 12000u);
            direct.add(lin, static_cast<uint8_t>(c));
            viacurve.add(code_for(lin), static_cast<uint8_t>(c));
        }
    }

    const ClipCeilingDetector::Result a = direct.detect(), b = viacurve.detect();
    check(a.found && b.found, "domain transfer: the same clamp is found in both encodings",
          std::string("direct ") + (a.found ? "found" : (a.why ? a.why : "?")) +
          ", via curve " + (b.found ? "found" : (b.why ? b.why : "?")));
    if (a.found && b.found)
    {
        const long diff = std::labs(static_cast<long>(a.ceiling) - static_cast<long>(b.ceiling));
        check(diff * 100 <= static_cast<long>(a.ceiling) * 2,
              "domain transfer: both encodings agree on the ceiling within 2%",
              "direct " + std::to_string(a.ceiling) + " vs via curve " + std::to_string(b.ceiling));
    }
}

void tungsten_two_channel_clamp()
{
    /* THE REGRESSION GUARD. A tungsten source pins R and G on the clamp while
     * B never approaches it. Under min-over-max this frame reads as "nothing is
     * clamped" and the lamp stays magenta. Under second-over-max it is caught. */
    ClipCeilingDetector d;
    d.reset(4095, 200, kRGGB);
    band(d, kClipChanR, 2990, 3010, 200000);
    band(d, kClipChanG, 2990, 3010, 200000);
    band(d, kClipChanB, 1100, 1300, 200000);
    band(d, kClipChanR,  300, 1000, 400000);
    band(d, kClipChanG,  300, 1000, 400000);
    band(d, kClipChanB,  300, 1000, 400000);

    const ClipCeilingDetector::Result r = d.detect();
    check(r.found, "tungsten: two-channel clamp is detected",
          r.found ? "" : std::string("why: ") + (r.why ? r.why : "?"));
    check(r.found && r.ceiling >= 2990 && r.ceiling <= 3010,
          "tungsten: ceiling lands in the clamp body", "got " + std::to_string(r.ceiling));
    check(r.second * 100 >= r.peak * 97,
          "tungsten: it is the SECOND channel that converges, not the min",
          "peak " + std::to_string(r.peak) + ", second " + std::to_string(r.second));
}

void neutral_wall_is_not_a_clamp()
{
    /* A bright neutral subject with nothing above it. The sensor's own channel
     * imbalance keeps raw green ~1.7x raw red (that ratio IS AsShotNeutral), so
     * the channels do NOT converge and this must be refused — adopting it would
     * clip a perfectly good wall to white. */
    ClipCeilingDetector d;
    d.reset(4095, 200, kRGGB);
    band(d, kClipChanG, 2950, 3050, 400000);
    band(d, kClipChanR, 1700, 1800, 400000);
    band(d, kClipChanB, 1700, 1800, 400000);

    const ClipCeilingDetector::Result r = d.detect();
    check(!r.found, "neutral wall: refused, channels never converge",
          r.found ? "wrongly claimed " + std::to_string(r.ceiling)
                  : std::string("why: ") + (r.why ? r.why : "?"));
}

void full_scale_is_not_a_clamp()
{
    /* Data reaching the declared white was not clamped short of it. This is what
     * keeps the detector inert on healthy non-ClearHDR modes. */
    ClipCeilingDetector d;
    d.reset(1023, 50, kRGGB);
    for (int c = 0; c < kClipChanCount; ++c)
    {
        band(d, static_cast<uint8_t>(c), 1023, 1023, 300000);
        band(d, static_cast<uint8_t>(c),   60,  900, 300000);
    }
    const ClipCeilingDetector::Result r = d.detect();
    check(!r.found, "full scale: refused, data reaches the container top",
          r.found ? "wrongly claimed " + std::to_string(r.ceiling)
                  : std::string("why: ") + (r.why ? r.why : "?"));
}

void tiny_specular_is_not_a_clamp()
{
    /* A converged specular glint, dense enough per value to register as
     * populated but far too small an AREA to be the merge ceiling. */
    ClipCeilingDetector d;
    d.reset(4095, 200, kRGGB);
    for (int c = 0; c < kClipChanCount; ++c)
    {
        band(d, static_cast<uint8_t>(c), 2999, 3001, 400);
        band(d, static_cast<uint8_t>(c),  300, 1000, 400000);
    }
    const ClipCeilingDetector::Result r = d.detect();
    check(!r.found, "tiny specular: refused, clamp body too small",
          r.found ? "wrongly claimed " + std::to_string(r.ceiling)
                  : std::string("why: ") + (r.why ? r.why : "?"));
}

void trailing_highlight_is_not_a_clamp()
{
    /* A converged band with real population ABOVE it: a clamp terminates the
     * distribution, a bright subject trails past it. Refuse. */
    ClipCeilingDetector d;
    d.reset(4095, 200, kRGGB);
    for (int c = 0; c < kClipChanCount; ++c)
    {
        band(d, static_cast<uint8_t>(c), 2400, 2500, 300000);
        band(d, static_cast<uint8_t>(c), 2600, 3200,  60000);
        band(d, static_cast<uint8_t>(c),  300, 1000, 300000);
    }
    const ClipCeilingDetector::Result r = d.detect();
    check(!r.found, "trailing highlight: refused, histogram does not terminate",
          r.found ? "wrongly claimed " + std::to_string(r.ceiling)
                  : std::string("why: ") + (r.why ? r.why : "?"));
}

void dark_frame_says_nothing()
{
    /* A lens-cap take: everything at the pedestal. There is no ceiling to find
     * and claiming one would destroy the next take shot in the same mode. */
    ClipCeilingDetector d;
    d.reset(65535, 3200, kRGGB);
    for (int c = 0; c < kClipChanCount; ++c)
        band(d, static_cast<uint8_t>(c), 3190, 3260, 600000);
    const ClipCeilingDetector::Result r = d.detect();
    check(!r.found, "dark frame: refused, nothing near a ceiling",
          r.found ? "wrongly claimed " + std::to_string(r.ceiling)
                  : std::string("why: ") + (r.why ? r.why : "?"));
}

void a_converged_midtone_is_not_a_clamp()
{
    /* The floor rule, exercised on its own. A big, converged, perfectly
     * terminating population -- everything the other rules ask for -- but
     * sitting at a fifth of the range. No ClearHDR clamp has ever been
     * measured below ~0.55 of the container, so this is a misread, and
     * adopting it would declare four fifths of the frame blown.
     *
     * Distinct from dark_frame_says_nothing(): that one has nothing above the
     * floor at all and refuses trivially. This one has a real candidate and
     * must still be turned down. */
    ClipCeilingDetector d;
    d.reset(65535, 3200, kRGGB);
    for (int c = 0; c < kClipChanCount; ++c)
    {
        band(d, static_cast<uint8_t>(c), 12800, 13400, 500000);   /* ~0.2 of range */
        band(d, static_cast<uint8_t>(c),  3300,  9000, 200000);
    }
    const ClipCeilingDetector::Result r = d.detect();
    check(!r.found, "converged midtone: refused, candidate ceiling implausibly low",
          r.found ? "wrongly claimed " + std::to_string(r.ceiling)
                  : std::string("why: ") + (r.why ? r.why : "?"));
}

void empty_frame_says_nothing()
{
    ClipCeilingDetector d;
    d.reset(4095, 200, kRGGB);
    const ClipCeilingDetector::Result r = d.detect();
    check(!r.found && r.why != nullptr, "empty frame: refused with a reason",
          r.why ? r.why : "(no reason given)");
}

void add_row_matches_add()
{
    std::vector<uint16_t> even(64, 0), odd(64, 0);
    for (size_t x = 0; x < 64; ++x)
    {
        even[x] = static_cast<uint16_t>((x & 1u) ? 3000 : 2000);  /* G : R */
        odd[x]  = static_cast<uint16_t>((x & 1u) ? 1000 : 3000);  /* B : G */
    }
    /* Enough row pairs to clear kMinSamples. This test shipped comparing
     * 128 samples, which is under that floor -- detect() returned before it
     * computed a peak at all and the assertion passed on 0 == 0, checking
     * nothing. Same failure the shift test was caught on; it is easy to write
     * and invisible once written, so both now assert they are non-vacuous. */
    ClipCeilingDetector a, b;
    a.reset(4095, 200, kRGGB);
    b.reset(4095, 200, kRGGB);
    for (unsigned pair = 0; pair < 64; ++pair)
        for (unsigned y = 0; y < 2; ++y)
            a.add_row((y & 1u) ? odd.data() : even.data(), 64, y);
    for (unsigned pair = 0; pair < 64; ++pair)
        for (size_t x = 0; x < 64; ++x)
        {
            b.add(even[x], (x & 1u) ? kClipChanG : kClipChanR);
            b.add(odd[x],  (x & 1u) ? kClipChanB : kClipChanG);
        }
    const ClipCeilingDetector::Result ra = a.detect(), rb = b.detect();
    check(ra.peak > 0 && ra.second > 0, "add_row: the comparison is not vacuous",
          "peak " + std::to_string(ra.peak) + ", second " + std::to_string(ra.second));
    check(ra.sampled == rb.sampled && ra.peak == rb.peak && ra.second == rb.second,
          "add_row derives the CFA phase from row parity",
          "sampled " + std::to_string(ra.sampled) + "/" + std::to_string(rb.sampled) +
          ", peak " + std::to_string(ra.peak) + "/" + std::to_string(rb.peak));
}

void shift_matches_the_real_packers()
{
    /* add_row()'s `shift` claims to reproduce what dng_save()'s 16->12 and
     * 16->10 branches drop on the way into the file. That is a claim about
     * OTHER code, so check it against that code: pack a row with the shipping
     * packer, unpack it back, and require the detector to have binned exactly
     * those values. If a packer ever starts rounding instead of truncating
     * (pack_row_16_to_10bit_rounded already does, on the COMP1 path), this
     * fails instead of silently measuring a domain the file does not use. */
    const uint32_t width = 64;
    struct Case { const char *name; unsigned shift; unsigned white; };
    const Case cases[] = { { "16->12", 4, 4095 }, { "16->10", 6, 1023 } };

    for (const Case &c : cases)
    {
        std::vector<uint16_t> src(width);
        for (uint32_t x = 0; x < width; ++x)
            src[x] = static_cast<uint16_t>(1000u + x * 977u);

        std::vector<uint8_t>  packed((width * (c.shift == 4 ? 12u : 10u) + 7u) / 8u, 0u);
        std::vector<uint16_t> back(width, 0);
        if (c.shift == 4)
        {
            pack_row_16_to_12bit(src.data(), packed.data(), width);
            unpack_contiguous_12bit(packed.data(), back.data(), width);
        }
        else
        {
            pack_row_16_to_10bit(src.data(), packed.data(), width);
            unpack_contiguous_10bit(packed.data(), back.data(), width);
        }

        /* Enough rows to clear the detector's minimum sample count — below it
         * detect() returns before computing a peak at all, and the comparison
         * would pass on 0 == 0 without having compared anything. */
        ClipCeilingDetector a, b;
        a.reset(c.white, 0, kRGGB);
        b.reset(c.white, 0, kRGGB);
        for (unsigned y = 0; y < 128; ++y)
        {
            a.add_row(src.data(),  width, y, c.shift);
            b.add_row(back.data(), width, y);
        }
        const ClipCeilingDetector::Result ra = a.detect(), rb = b.detect();
        check(ra.peak > 0 && ra.second > 0,
              std::string("shift ") + c.name + ": the comparison is not vacuous",
              "peak " + std::to_string(ra.peak) + ", second " + std::to_string(ra.second));
        check(ra.sampled == rb.sampled && ra.peak == rb.peak && ra.second == rb.second,
              std::string("shift ") + c.name + ": matches the shipping packer",
              "peak " + std::to_string(ra.peak) + " vs " + std::to_string(rb.peak));
    }
}

void out_of_range_values_are_dropped()
{
    /* A sample wider than the container it was packed into is an upstream bug;
     * folding it onto the top would fabricate the full-scale evidence rule 3
     * keys on, turning a clamped frame into an unclamped one. */
    ClipCeilingDetector d;
    d.reset(1023, 50, kRGGB);
    d.add(5000, kClipChanR);
    check(d.detect().sampled == 0, "values above white_level are dropped, not clamped");
}

} // namespace

int main()
{
    std::cout << "clip_ceiling_test\n";

    std::cout << " recorded takes\n";
    real_takes();

    std::cout << " synthetic cases\n";
    domain_transfer_is_invariant();
    tungsten_two_channel_clamp();
    neutral_wall_is_not_a_clamp();
    full_scale_is_not_a_clamp();
    tiny_specular_is_not_a_clamp();
    trailing_highlight_is_not_a_clamp();
    dark_frame_says_nothing();
    a_converged_midtone_is_not_a_clamp();
    empty_frame_says_nothing();
    add_row_matches_add();
    shift_matches_the_real_packers();
    out_of_range_values_are_dropped();

    std::cout << (g_failures ? "FAILED" : "PASSED") << " (" << g_failures << " failures)\n";
    return g_failures ? 1 : 0;
}
