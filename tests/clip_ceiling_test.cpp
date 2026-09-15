/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * clip_ceiling_test.cpp - the ClearHDR clamp detector, against real takes.
 *
 * Includes only cinepi/clip_ceiling.hpp, so it builds with nothing but the
 * standard library and exercises the same code that ships. Build & run:
 *   c++ -std=c++17 -O2 -I.. tests/clip_ceiling_test.cpp -o /tmp/clip_ceiling_test && /tmp/clip_ceiling_test
 * (or via meson: `meson test clip_ceiling`).
 *
 * THE POINT OF THIS TEST. The detector decides whether to overwrite a DNG's
 * WhiteLevel, and both ways of being wrong are real damage: miss a clamp and
 * the highlight stays magenta (the bug), invent one and every highlight above
 * the invented level is clipped to white (a worse bug, on footage that was
 * fine). So the fixtures are not synthetic frames — they are the per-channel
 * code histograms of four DNGs this camera actually recorded, dumped straight
 * out of the files, three ClearHDR takes that ARE clamped and one 10-bit take
 * that is not.
 *
 * The expected ceilings are measurements, not hand-tuned constants. Each was
 * read off the recorded file independently of this code (histogram mode of the
 * clamp body), and for F06 it was confirmed end-to-end: patching tag 0xC61D to
 * the linearised value of code 4041 takes LibRaw's colour cast over the blown
 * area from 12.0% to 0.0%.
 *
 * The synthetic cases below cover what no take in hand happens to contain, and
 * one of them is a REGRESSION GUARD, not a feature test: the tungsten case
 * fails against a min-over-max convergence rule. lessons/hardware-log.md
 * records that exact rule shipping in clip_plateau.hpp, surviving a fix to its
 * sibling, and costing a week of debugging at the wrong layer. It must not be
 * reintroduced here.
 */

#include "cinepi/clip_ceiling.hpp"
#include "cinepi/dng_pack.hpp"

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

/* One take's per-channel histogram, as dumped from its DNG. */
struct Fixture
{
    unsigned              code_max = 0;
    std::vector<uint32_t> ch[kClipChanCount];
    bool                  ok = false;
};

/* Parse the .hist dump: '#' comments, a `code_max N` line, then one
 * `R|G|B c0 c1 ...` line per channel with code_max+1 counts. */
Fixture read_fixture(const std::string &name, std::string &err)
{
    Fixture fx;
    const std::string path = std::string(CLIP_CEILING_DATA_DIR) + "/clip_ceiling_" + name + ".hist";
    std::ifstream f(path);
    if (!f.good())
    {
        err = "cannot open fixture '" + path + "'";
        return fx;
    }

    std::string line;
    while (std::getline(f, line))
    {
        if (line.empty() || line[0] == '#')
            continue;
        std::istringstream is(line);
        std::string tag;
        is >> tag;
        if (tag == "code_max")
        {
            is >> fx.code_max;
            continue;
        }
        int c = tag == "R" ? kClipChanR : tag == "G" ? kClipChanG : tag == "B" ? kClipChanB : -1;
        if (c < 0)
        {
            err = "unexpected row '" + tag + "' in " + path;
            return fx;
        }
        uint32_t v;
        while (is >> v)
            fx.ch[c].push_back(v);
    }

    for (int c = 0; c < kClipChanCount; ++c)
        if (fx.ch[c].size() != static_cast<size_t>(fx.code_max) + 1u)
        {
            err = "channel " + std::to_string(c) + " of " + path + " has " +
                  std::to_string(fx.ch[c].size()) + " entries, expected " +
                  std::to_string(fx.code_max + 1u);
            return fx;
        }

    fx.ok = true;
    return fx;
}

/* Replaying a histogram costs one add() per SAMPLE if done naively, which is
 * 8 million calls per fixture. The detector only ever reads its bins, so the
 * bins are filled directly through the same public add() by repeating each
 * code — but in bulk, via a tiny helper that keeps the test honest about
 * using only the public surface while staying fast enough to run in CI. */
void load(ClipCeilingDetector &d, const Fixture &fx)
{
    static const uint8_t kRGGB[4] = { kClipChanR, kClipChanG, kClipChanG, kClipChanB };
    d.reset(fx.code_max, kRGGB);
    for (int c = 0; c < kClipChanCount; ++c)
        for (unsigned code = 0; code <= fx.code_max; ++code)
            for (uint32_t n = 0; n < fx.ch[c][code]; ++n)
                d.add(code, static_cast<uint8_t>(c));
}

/* Read back the contiguous big-endian bit layout dng_pack.hpp's packers emit —
 * DNG's own, MSB-first across the row, which is NOT the MIPI CSI-2 layout the
 * sensor delivers. Written out here rather than imported because the point of
 * shift_matches_the_real_packers() is to compare the detector against what
 * actually lands in the file; reusing a shared helper for both sides would
 * make that comparison circular. This layout is the one recorded DNGs decode
 * with, confirmed against the fixture files. */
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

/* ── synthetic frame builder ──────────────────────────────────────────────
 * Adds `count` samples of `channel` spread evenly over [lo, hi], which is
 * how a noisy band is described below without transcribing a histogram. */
void band(ClipCeilingDetector &d, uint8_t channel, unsigned lo, unsigned hi, unsigned long count)
{
    const unsigned span = hi - lo + 1u;
    for (unsigned long i = 0; i < count; ++i)
        d.add(lo + static_cast<unsigned>(i % span), channel);
}

void real_takes()
{
    /* name, the ceiling measured off that DNG, and whether it is clamped.
     * F06 and F13 are the same operating point minutes apart and agree to the
     * code; F07 is a different session and sits far lower, which is exactly
     * why this is measured per take rather than tabulated per mode. */
    struct Case { const char *name; bool clamped; unsigned ceiling; const char *note; };
    const Case cases[] = {
        { "clearhdr4k_f06", true,  4041, "12-bit ClearHDR 4K, clamp at 89.0% of declared white" },
        { "clearhdr4k_f13", true,  4041, "same operating point as F06, taken minutes later" },
        { "clearhdr4k_f07", true,  3951, "different session: clamp at 73.5%, same detector" },
        { "linear10b_f02",  false, 0,    "native 10-bit 4K: reaches 1023, must stay untouched" },
    };

    for (const Case &c : cases)
    {
        std::string err;
        Fixture fx = read_fixture(c.name, err);
        if (!fx.ok)
        {
            check(false, std::string("fixture ") + c.name, err);
            continue;
        }

        ClipCeilingDetector d;
        load(d, fx);
        const ClipCeilingDetector::Result r = d.detect();

        const std::string tag = std::string(c.name) + ": ";
        check(r.found == c.clamped, tag + (c.clamped ? "clamp detected" : "no clamp claimed"),
              std::string(c.note) + (r.found ? "" : std::string("  [why: ") + (r.why ? r.why : "?") + "]"));
        if (c.clamped && r.found)
            check(r.ceiling == c.ceiling, tag + "ceiling is the measured clamp body",
                  "got " + std::to_string(r.ceiling) + ", measured " + std::to_string(c.ceiling));

        /* Whatever it decides, it may never claim a ceiling at or above the
         * container: that would be a WhiteLevel raise, which is meaningless. */
        if (r.found)
            check(r.ceiling < fx.code_max, tag + "ceiling is below full scale",
                  std::to_string(r.ceiling) + " < " + std::to_string(fx.code_max));
    }
}

void tungsten_two_channel_clamp()
{
    /* THE REGRESSION GUARD. A tungsten source pins R and G on the clamp while
     * B never approaches it. Under min-over-max this frame reads as "nothing
     * is clamped" and the lamp stays magenta — the bug hardware-log.md
     * describes twice. Under second-over-max it is caught. */
    ClipCeilingDetector d;
    static const uint8_t kRGGB[4] = { kClipChanR, kClipChanG, kClipChanG, kClipChanB };
    d.reset(4095, kRGGB);

    band(d, kClipChanR, 2990, 3010, 200000);   /* pinned on the clamp        */
    band(d, kClipChanG, 2990, 3010, 200000);   /* pinned on the same clamp   */
    band(d, kClipChanB, 1100, 1300, 200000);   /* nowhere near it            */
    band(d, kClipChanR,  100, 1000, 400000);   /* the rest of the scene      */
    band(d, kClipChanG,  100, 1000, 400000);
    band(d, kClipChanB,  100, 1000, 400000);

    const ClipCeilingDetector::Result r = d.detect();
    check(r.found, "tungsten: two-channel clamp is detected",
          r.found ? "" : std::string("why: ") + (r.why ? r.why : "?"));
    check(r.found && r.ceiling >= 2990 && r.ceiling <= 3010,
          "tungsten: ceiling lands in the clamp body",
          "got " + std::to_string(r.ceiling));

    /* Spell out what makes it pass, so a future "simplification" back to
     * min-over-max fails here with an explanation rather than a bare number:
     * the third channel really is far below, and the rule ignores it. */
    check(r.second + ClipCeilingDetector::band_width(4095) >= r.peak,
          "tungsten: it is the SECOND channel that converges, not the min",
          "peak " + std::to_string(r.peak) + ", second " + std::to_string(r.second));
}

void neutral_wall_is_not_a_clamp()
{
    /* A bright neutral subject with nothing above it. The sensor's own channel
     * imbalance keeps raw green ~1.7x raw red (that ratio IS AsShotNeutral),
     * so the channels do NOT converge and this must be refused. Adopting it
     * would clip a perfectly good wall to white. */
    ClipCeilingDetector d;
    static const uint8_t kRGGB[4] = { kClipChanR, kClipChanG, kClipChanG, kClipChanB };
    d.reset(4095, kRGGB);

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
    /* Data that reaches the container's top was not clamped short of it, so
     * the nominal WhiteLevel is already right and must be left alone. This is
     * what keeps the detector inert on healthy non-ClearHDR modes. */
    ClipCeilingDetector d;
    static const uint8_t kRGGB[4] = { kClipChanR, kClipChanG, kClipChanG, kClipChanB };
    d.reset(1023, kRGGB);

    for (int c = 0; c < kClipChanCount; ++c)
    {
        band(d, static_cast<uint8_t>(c), 1023, 1023, 300000);
        band(d, static_cast<uint8_t>(c),   40,  900, 300000);
    }

    const ClipCeilingDetector::Result r = d.detect();
    check(!r.found, "full scale: refused, data reaches the container top",
          r.found ? "wrongly claimed " + std::to_string(r.ceiling)
                  : std::string("why: ") + (r.why ? r.why : "?"));
}

void tiny_specular_is_not_a_clamp()
{
    /* A converged specular glint, dense enough per code to register as
     * populated but far too small an AREA to be the merge ceiling. Anchoring
     * WhiteLevel on it would clip the whole frame to white. Deliberately only
     * three codes wide: spread any thinner and it stops being populated at
     * all, and the refusal would come from a different rule than the one this
     * case exists to exercise. */
    ClipCeilingDetector d;
    static const uint8_t kRGGB[4] = { kClipChanR, kClipChanG, kClipChanG, kClipChanB };
    d.reset(4095, kRGGB);

    for (int c = 0; c < kClipChanCount; ++c)
    {
        band(d, static_cast<uint8_t>(c), 2999, 3001, 300);   /* the glint  */
        band(d, static_cast<uint8_t>(c),  100, 1000, 400000);
    }

    const ClipCeilingDetector::Result r = d.detect();
    check(!r.found, "tiny specular: refused, clamp body too small",
          r.found ? "wrongly claimed " + std::to_string(r.ceiling)
                  : std::string("why: ") + (r.why ? r.why : "?"));
}

void trailing_highlight_is_not_a_clamp()
{
    /* A converged band with real population ABOVE it: a clamp terminates the
     * histogram, a bright subject trails past it. Refuse. */
    ClipCeilingDetector d;
    static const uint8_t kRGGB[4] = { kClipChanR, kClipChanG, kClipChanG, kClipChanB };
    d.reset(4095, kRGGB);

    for (int c = 0; c < kClipChanCount; ++c)
    {
        band(d, static_cast<uint8_t>(c), 2400, 2500, 300000);   /* the body  */
        band(d, static_cast<uint8_t>(c), 2501, 3200,  60000);   /* the tail  */
        band(d, static_cast<uint8_t>(c),  100, 1000, 300000);
    }

    const ClipCeilingDetector::Result r = d.detect();
    check(!r.found, "trailing highlight: refused, histogram does not terminate",
          r.found ? "wrongly claimed " + std::to_string(r.ceiling)
                  : std::string("why: ") + (r.why ? r.why : "?"));
}

void empty_frame_says_nothing()
{
    ClipCeilingDetector d;
    static const uint8_t kRGGB[4] = { kClipChanR, kClipChanG, kClipChanG, kClipChanB };
    d.reset(4095, kRGGB);
    const ClipCeilingDetector::Result r = d.detect();
    check(!r.found && r.why != nullptr, "empty frame: refused with a reason",
          r.why ? r.why : "(no reason given)");
}

void add_row_matches_add()
{
    /* add_row() is what the encoder actually calls; it must derive the same
     * channel assignment from row parity that add() is told explicitly. */
    static const uint8_t kRGGB[4] = { kClipChanR, kClipChanG, kClipChanG, kClipChanB };
    std::vector<uint16_t> even(64, 0), odd(64, 0);
    for (size_t x = 0; x < 64; ++x)
    {
        even[x] = static_cast<uint16_t>((x & 1u) ? 3000 : 2000);  /* G : R */
        odd[x]  = static_cast<uint16_t>((x & 1u) ? 1000 : 3000);  /* B : G */
    }

    ClipCeilingDetector a, b;
    a.reset(4095, kRGGB);
    b.reset(4095, kRGGB);

    for (unsigned y = 0; y < 2; ++y)
        a.add_row((y & 1u) ? odd.data() : even.data(), 64, y);

    for (size_t x = 0; x < 64; ++x)
    {
        b.add(even[x], (x & 1u) ? kClipChanG : kClipChanR);
        b.add(odd[x],  (x & 1u) ? kClipChanB : kClipChanG);
    }

    const ClipCeilingDetector::Result ra = a.detect(), rb = b.detect();
    check(ra.sampled == rb.sampled && ra.peak == rb.peak && ra.second == rb.second,
          "add_row derives the CFA phase from row parity",
          "sampled " + std::to_string(ra.sampled) + "/" + std::to_string(rb.sampled));
}

void shift_matches_the_real_packers()
{
    /* add_row()'s `shift` claims to reproduce what dng_save()'s 16->12 and
     * 16->10 branches drop on the way into the file. That is a claim about
     * OTHER code, so check it against that code rather than against the
     * constants 4 and 6: pack a row with the shipping packer, unpack it back,
     * and require the detector to have binned exactly those codes. If a packer
     * ever starts rounding instead of truncating (pack_row_16_to_10bit_rounded
     * already does, on the COMP1 path), this fails instead of silently
     * measuring a domain the file does not use. */
    static const uint8_t kRGGB[4] = { kClipChanR, kClipChanG, kClipChanG, kClipChanB };
    const uint32_t width = 64;

    struct Case { const char *name; unsigned shift; unsigned code_max; };
    const Case cases[] = { { "16->12", 4, 4095 }, { "16->10", 6, 1023 } };

    for (const Case &c : cases)
    {
        std::vector<uint16_t> src(width);
        for (uint32_t x = 0; x < width; ++x)
            src[x] = static_cast<uint16_t>(1000u + x * 977u);   /* spread over the container */

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
        a.reset(c.code_max, kRGGB);
        b.reset(c.code_max, kRGGB);
        for (unsigned y = 0; y < 128; ++y)
        {
            a.add_row(src.data(),  width, y, c.shift);   /* what the encoder does */
            b.add_row(back.data(), width, y);            /* what is in the file   */
        }

        const ClipCeilingDetector::Result ra = a.detect(), rb = b.detect();
        check(ra.peak > 0 && ra.second > 0,
              std::string("shift ") + c.name + ": the comparison is not vacuous",
              "peak " + std::to_string(ra.peak) + ", second " + std::to_string(ra.second));
        check(ra.sampled == rb.sampled && ra.peak == rb.peak && ra.second == rb.second,
              std::string("shift ") + c.name + ": matches the shipping packer",
              "peak " + std::to_string(ra.peak) + " vs " + std::to_string(rb.peak) +
              ", sampled " + std::to_string(ra.sampled) + " vs " + std::to_string(rb.sampled));
    }
}

void out_of_range_codes_are_dropped()
{
    /* A sample wider than the container it was packed into is an upstream
     * bug; folding it onto the top code would fabricate the full-scale
     * evidence rule 3 keys on, turning a clamped frame into an unclamped one. */
    static const uint8_t kRGGB[4] = { kClipChanR, kClipChanG, kClipChanG, kClipChanB };
    ClipCeilingDetector d;
    d.reset(1023, kRGGB);
    d.add(5000, kClipChanR);
    check(d.detect().sampled == 0, "codes above code_max are dropped, not clamped");
}

} // namespace

int main()
{
    std::cout << "clip_ceiling_test\n";

    std::cout << " recorded takes\n";
    real_takes();

    std::cout << " synthetic cases\n";
    tungsten_two_channel_clamp();
    neutral_wall_is_not_a_clamp();
    full_scale_is_not_a_clamp();
    tiny_specular_is_not_a_clamp();
    trailing_highlight_is_not_a_clamp();
    empty_frame_says_nothing();
    add_row_matches_add();
    shift_matches_the_real_packers();
    out_of_range_codes_are_dropped();

    std::cout << (g_failures ? "FAILED" : "PASSED") << " (" << g_failures << " failures)\n";
    return g_failures ? 1 : 0;
}
