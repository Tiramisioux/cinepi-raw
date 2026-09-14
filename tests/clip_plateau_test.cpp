/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * clip_plateau_test.cpp - the ClearHDR merge-clamp plateau detector.
 *
 * Includes only cinepi/clip_plateau.hpp, so it builds with nothing but the
 * standard library and exercises the same code that ships. Build & run:
 *
 *   c++ -std=c++17 -Wall -Wextra -O1 -I. tests/clip_plateau_test.cpp -o /tmp/clip_plateau_test && /tmp/clip_plateau_test
 *
 * (run from the repo root; or via meson: `meson test clip_plateau`).
 *
 * THE POINT OF THIS TEST. clip_plateau.hpp exists because the 16-bit
 * ClearHDR merge clamp is not a constant -- the same take plateaued at raw
 * code 54100 at analogue gain code 71 and 48600 at code 80, thirteen minutes
 * apart (2026-09-13 hardware log) -- so the anchor has to be read off the
 * quad-convergence signature every frame instead of tabulated per binning the
 * way the 12-bit fix's CcmpAnchor::clip_code is. Every check below drives the
 * detector with synthetic quads whose composition is stated up front, so a
 * reader can see what the detector is supposed to separate before it is
 * asked to separate it: real plateau quads, bright-but-not-converged quads
 * (an ordinary saturated colour, never equal-code), dark equal quads (mean
 * nothing), and single-pixel outliers (never a whole converged quad).
 *
 * The 12-bit case (item 4) is the strongest evidence the detector's logic is
 * right, not just internally consistent: fed nothing but the geometry of the
 * ALREADY hardware-confirmed 12-bit fix (converged quads spread 2948-3054,
 * from the 2026-09-06/07 log), it has to land on anchor 2900 -- the number a
 * human derived by hand and confirmed on hardware -- without being told what
 * that number is.
 */

#include "cinepi/clip_plateau.hpp"

#include <iostream>
#include <string>

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

std::string fmt(unsigned long v)
{
    return std::to_string(v);
}

/* ── 1: a realistic 16-bit frame -- plateau, bright non-converged, dark,
 * and single-pixel-outlier quads all in one histogram ────────────────────── */
void run_realistic_16bit()
{
    std::cout << "\n16-bit: a realistic mixed frame\n";

    ClipPlateauDetector d;
    check(d.configure(16), "configure(16)");

    /* 1000 converged quads (mn == mx, trivially >= 0.9 of themselves), the
     * plateau's body, spread uniformly across the measured 500-code width. */
    constexpr int kPlateau = 1000;
    for (int i = 0; i < kPlateau; ++i)
    {
        const unsigned level = 54000u + static_cast<unsigned>((static_cast<long long>(i) * 500) / (kPlateau - 1));
        d.add(level, level);
    }

    /* 5000 bright quads spread across the whole code range, but mn = 0.55*mx
     * -- an ordinary saturated colour under the shipping gains, never
     * equal-code -- so every one must fail convergence regardless of level. */
    constexpr int kBright = 5000;
    for (int i = 0; i < kBright; ++i)
    {
        const unsigned mx = static_cast<unsigned>((static_cast<long long>(i) * 65535) / (kBright - 1));
        const unsigned mn = (mx * 55) / 100;
        d.add(mn, mx);
    }

    /* 300 dark equal quads: converged (mn == mx) but below full_scale/4, so
     * "equal" means nothing here and they must not reach the histogram. */
    constexpr int kDark = 300;
    for (int i = 0; i < kDark; ++i)
    {
        const unsigned level = static_cast<unsigned>((static_cast<long long>(i) * 16383) / (kDark - 1));
        d.add(level, level);
    }

    /* 20 quads with a single hot sample: one photosite pinned near the top,
     * the other three at a normal level -- a dead/hot pixel or a specular
     * point, not a clamp, and mn/mx = 0.61 fails convergence. */
    for (int i = 0; i < 20; ++i)
        d.add(40000, 65300);

    ClipPlateauDetector::Result r;
    const bool detected = d.detect(r);

    check(detected, "a plateau is detected");
    check(r.sampled == static_cast<unsigned long>(kPlateau + kBright + kDark + 20), "sampled counts every add()",
          "sampled=" + fmt(r.sampled));
    check(r.converged == kPlateau, "converged counts only the plateau quads -- bright/dark/hot excluded",
          "converged=" + fmt(r.converged));
    check(r.floor >= 53900 && r.floor <= 54100, "floor lands in the measured plateau body",
          "floor=" + fmt(r.floor));
    check(r.anchor == r.floor - (r.floor * 15) / 1000, "anchor is exactly floor - 1.5% (integer)",
          "floor=" + fmt(r.floor) + " anchor=" + fmt(r.anchor));
}

/* ── 2: too little evidence, either way ──────────────────────────────────── */
void run_insufficient_evidence()
{
    std::cout << "\ninsufficient evidence\n";

    {
        ClipPlateauDetector d;
        d.configure(16);
        /* All bright, none converged: an ordinary well-exposed colour image,
         * nothing clamped anywhere in it. */
        for (int i = 0; i < 2000; ++i)
        {
            const unsigned mx = 20000u + static_cast<unsigned>(i);
            d.add((mx * 55) / 100, mx);
        }
        ClipPlateauDetector::Result r;
        check(!d.detect(r), "only non-converged quads -> not detected");
        check(r.converged == 0, "and converged is reported as zero", "converged=" + fmt(r.converged));
    }
    {
        ClipPlateauDetector d;
        d.configure(16);
        /* 10 genuinely converged quads -- a real but tiny blown speck, too
         * little to trust a percentile from. */
        for (int i = 0; i < 10; ++i)
            d.add(54000, 54000);
        ClipPlateauDetector::Result r;
        check(!d.detect(r), "10 converged quads (< 64) -> not detected");
        check(r.converged == 10, "and converged still reports the true count", "converged=" + fmt(r.converged));
    }
}

/* ── 3: reset() actually clears state, not just the visible symptom ─────── */
void run_reset()
{
    std::cout << "\nreset()\n";

    ClipPlateauDetector d;
    d.configure(16);
    for (int i = 0; i < 500; ++i)
        d.add(54000, 54000);

    d.reset();

    ClipPlateauDetector::Result empty;
    check(!d.detect(empty), "immediately after reset, nothing is detected");
    check(empty.converged == 0 && empty.sampled == 0, "and both counters read zero",
          "converged=" + fmt(empty.converged) + " sampled=" + fmt(empty.sampled));

    /* A fresh plateau at a completely different level must be read cleanly,
     * with no trace of the pre-reset histogram (which would pull the floor
     * down towards 54000 if any of it survived). */
    for (int i = 0; i < 200; ++i)
        d.add(40000, 40000);

    ClipPlateauDetector::Result r;
    check(d.detect(r), "a plateau added after reset is detected");
    check(r.converged == 200, "with only the post-reset quads counted", "converged=" + fmt(r.converged));
    check(r.floor >= 39936 && r.floor <= 40000, "and the floor reflects the new level, not the old one",
          "floor=" + fmt(r.floor));
}

/* ── 4: the 12-bit case has to land on the hardware-confirmed anchor ────── */
void run_12bit_matches_hardware()
{
    std::cout << "\n12-bit: lands on the hardware-confirmed anchor without being told it\n";

    ClipPlateauDetector d;
    check(d.configure(12), "configure(12)");

    /* The 2026-09-06/07 measured geometry: converged quads spread uniformly
     * over the blown area's range, p1 2948, peak 3054. This is the SAME
     * shape of evidence that led a human to anchor 2900 by hand; the
     * detector has to reach the same place from the raw numbers alone. */
    constexpr int kQuads = 600;
    for (int i = 0; i < kQuads; ++i)
    {
        const unsigned level = 2948u + static_cast<unsigned>((static_cast<long long>(i) * (3054 - 2948)) / (kQuads - 1));
        d.add(level, level);
    }

    ClipPlateauDetector::Result r;
    check(d.detect(r), "detected");
    check(r.floor >= 2944 && r.floor <= 2952, "floor within 4 codes of the measured p1 2948",
          "floor=" + fmt(r.floor));
    check(r.anchor >= 2890 && r.anchor <= 2915, "anchor lands in the hardware-confirmed 2900 band",
          "anchor=" + fmt(r.anchor));
}

/* ── 5: the three boundaries the skip rules turn on ──────────────────────── */
void run_boundaries()
{
    std::cout << "\nbin/skip boundaries\n";

    {
        ClipPlateauDetector d;
        d.configure(16);
        d.add(16384, 16384); /* exactly full_scale/4 (65536/4): must count */
        ClipPlateauDetector::Result r;
        d.detect(r);
        check(r.converged == 1, "a quad at exactly full_scale/4 counts", "converged=" + fmt(r.converged));
    }
    {
        ClipPlateauDetector d;
        d.configure(16);
        d.add(16383, 16383); /* one code below full_scale/4: dark, must not count */
        ClipPlateauDetector::Result r;
        d.detect(r);
        check(r.converged == 0, "one code below full_scale/4 does not count", "converged=" + fmt(r.converged));
    }
    {
        ClipPlateauDetector d;
        d.configure(16);
        d.add(45000, 50000); /* mn*10 == mx*9 exactly (45000*10 == 50000*9): must count */
        ClipPlateauDetector::Result r;
        d.detect(r);
        check(r.converged == 1, "mn*10 == mx*9 exactly counts (the skip test is strict '<')",
              "converged=" + fmt(r.converged));
        check(r.sampled == 1, "and sampled counts the call regardless", "sampled=" + fmt(r.sampled));
    }
}

} // namespace

int main()
{
    std::cout << "clip_plateau_test\n";

    run_realistic_16bit();
    run_insufficient_evidence();
    run_reset();
    run_12bit_matches_hardware();
    run_boundaries();

    std::cout << "\n" << (g_failures ? "FAILED " : "PASSED ") << g_failures << " failure(s)\n";
    return g_failures ? 1 : 0;
}
