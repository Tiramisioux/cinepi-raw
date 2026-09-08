/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * ccmp_log_compose_test.cpp - CCMP12 decompand composed with CineMate Log,
 * against its golden tables.
 *
 * Includes only cinepi/log_lut.hpp (which pulls in cinepi/ccmp_lut.hpp), so it
 * builds with nothing but the standard library and exercises the same code
 * that ships — no libcamera, no jsoncpp, no spec-file I/O.
 *
 * THE POINT OF THIS TEST. 12-bit ClearHDR (CCMP) is not a linear log source —
 * see log_source_is_companded() in cinepi/log_lut.hpp — so `--log-encode 10`
 * on that source cannot use the `12to10` spec. It composes instead: decompand
 * to 16-bit linear first (the already-gated CCMP curve), then apply the
 * already-gated 16to10 log curve. innomaker585/ccmp12-lut/tools/compose_log.py
 * is the golden reference for that composition; this C++ is a reimplementation
 * (LogLut::build_ccmp_composed()), and the one cheap proof that a
 * reimplementation has not acquired a transcription error is that it
 * reproduces the golden tables BYTE FOR BYTE. The .txt files in
 * resources/ccmp_log_luts are those tables, emitted by the Python and copied
 * in unmodified.
 *
 * The 16to10 LogLutParams below are copied from resources/log_luts/
 * cinemate_log_16to10.json, same as the "16to10" case in tests/log_lut_test.cpp
 * — not re-derived here, so this test cannot silently drift from the spec that
 * ships. So: no expected TABLE value in this file is hand-written.
 */

#include "cinepi/log_lut.hpp"

#include <cstdio>
#include <cstdlib>
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

/* Where the golden tables live. meson passes the source dir; the fallback lets
 * the test run from a plain `c++ tests/ccmp_log_compose_test.cpp` in the repo
 * root — same convention as ccmp_lut_test.cpp's CCMP_GOLDEN_DIR. */
#ifndef CCMP_LOG_GOLDEN_DIR
#define CCMP_LOG_GOLDEN_DIR "resources/ccmp_log_luts"
#endif

/* Parse the generator's .txt: '#' comment lines, then whitespace-separated
 * decimal entries. Returns empty on any read failure — the caller reports it. */
std::vector<uint16_t> read_golden(const std::string &name, std::string &err)
{
    const std::string path = std::string(CCMP_LOG_GOLDEN_DIR) + "/" + name;
    std::ifstream f(path);
    if (!f.good())
    {
        err = "cannot open golden table '" + path + "'";
        return {};
    }

    std::vector<uint16_t> out;
    std::string line;
    while (std::getline(f, line))
    {
        if (!line.empty() && line[0] == '#')
            continue;
        std::istringstream ls(line);
        long v;
        while (ls >> v)
        {
            if (v < 0 || v > 65535)
            {
                err = "golden table '" + path + "' has an out-of-range entry " + std::to_string(v);
                return {};
            }
            out.push_back(static_cast<uint16_t>(v));
        }
    }
    return out;
}

/* The shipped 16to10 spec's own params — resources/log_luts/cinemate_log_16to10.json.
 * mu, BL, WL, src, tgt, F, foot. */
const LogLutParams kLog16to10 = { 10000.0, 3200, 65535, 16, 10, 32, 487 };

/* One CCMP mode: compose, then compare with its golden. */
void test_against_golden(double binning, const std::string &golden, int mode)
{
    const std::string tag = "[mode " + std::to_string(mode) + " b=" +
                            std::to_string(static_cast<long long>(binning)) + "]";

    CcmpParams cp;
    if (!ccmp_params_for_binning(binning, cp))
    {
        check(false, tag + " CCMP params for binning");
        return;
    }
    CcmpLut decompand;
    std::string cerr;
    if (!decompand.build(cp, &cerr))
    {
        check(false, tag + " CCMP build", cerr);
        return;
    }

    LogLut composed;
    const bool built = composed.build_ccmp_composed(kLog16to10, decompand);
    check(built, tag + " composed build");
    if (!built)
        return;

    std::string gerr;
    const std::vector<uint16_t> want = read_golden(golden, gerr);
    if (want.empty())
    {
        check(false, tag + " read golden", gerr);
        return;
    }

    /* 1. THE ACCEPTANCE TEST — byte for byte against the Python generator. */
    check(want.size() == composed.forward_size(), tag + " golden has the same entry count",
          std::to_string(want.size()) + " against " + std::to_string(composed.forward_size()));

    if (want.size() == composed.forward_size())
    {
        size_t first_bad = want.size();
        int worst = 0;
        for (size_t i = 0; i < want.size(); ++i)
        {
            const int d = static_cast<int>(composed.forward()[i]) - static_cast<int>(want[i]);
            if (d != 0 && first_bad == want.size())
                first_bad = i;
            if (std::abs(d) > std::abs(worst))
                worst = d;
        }
        std::string detail = std::to_string(want.size()) + " entries identical";
        if (first_bad != want.size())
            detail = "first mismatch at CCMP code " + std::to_string(first_bad) + ": built " +
                     std::to_string(composed.forward()[first_bad]) + " vs golden " +
                     std::to_string(want[first_bad]) + ", worst delta " + std::to_string(worst);
        check(first_bad == want.size(), tag + " table is byte-identical to the golden", detail);
    }

    /* 2. Monotone. Both stages (CCMP decompand, log encode) are monotone
     *    increasing, so the composition must be too. */
    bool mono = true;
    for (size_t i = 1; i < composed.forward_size() && mono; ++i)
        mono = composed.forward()[i] >= composed.forward()[i - 1];
    check(mono, tag + " composed table is monotone non-decreasing");

    /* 3. Range — a valid target-depth code, nothing silently clamped past it. */
    const int cmax = kLog16to10.code_max();
    bool in_range = true;
    for (size_t i = 0; i < composed.forward_size() && in_range; ++i)
        in_range = composed.forward()[i] <= static_cast<uint16_t>(cmax);
    check(in_range, tag + " every composed code is <= code_max", std::to_string(cmax));

    /* 4. THE ANCHOR. CCMP's own reported black (stored code == its own
     *    BlackLevel tag, 200) sits on CCMP's identity segment, so it decodes to
     *    L-above-black == 0 — meaning the composed code at C=200 must be
     *    exactly the log curve's OWN black code, F (footroom_codes). This is
     *    arithmetic (mu-law encode of x=0 is F by construction, see
     *    log_encode_code()), not a fit — the cheapest possible check that the
     *    pedestal arithmetic (target.black_level + decompand) is not off by
     *    the sensor's own black level. */
    const int black_code = static_cast<int>(cp.pedestal + 0.5);
    check(black_code >= 0 && static_cast<size_t>(black_code) < composed.forward_size() &&
          composed.forward()[static_cast<size_t>(black_code)] == static_cast<uint16_t>(kLog16to10.footroom_codes),
          tag + " composed code at CCMP's own black (" + std::to_string(black_code) +
          ") is the log curve's footroom code",
          "F=" + std::to_string(kLog16to10.footroom_codes) + ", got " +
          (static_cast<size_t>(black_code) < composed.forward_size() ?
           std::to_string(composed.forward()[static_cast<size_t>(black_code)]) : "out of range"));

    /* 5. The DNG-facing side (inverse_/params_) is exactly the TARGET curve,
     *    unmodified — this is the whole point of precomposition: the file
     *    carries the already-shipped, already-gated 16to10 table, and a reader
     *    needs no CCMP awareness at all. */
    check(composed.params().source_bits == kLog16to10.source_bits &&
          composed.params().target_bits == kLog16to10.target_bits &&
          composed.params().black_level == kLog16to10.black_level &&
          composed.params().white_level == kLog16to10.white_level,
          tag + " params() is the 16to10 target spec, unmodified");
    check(composed.inverse_size() == (static_cast<size_t>(1) << kLog16to10.target_bits),
          tag + " inverse_ is the target curve's own DNG LinearizationTable size",
          std::to_string(composed.inverse_size()));
}

} // namespace

int main()
{
    std::cout << "CCMP12 x CineMate Log composition — against the golden tables\n\n";

    test_against_golden(1.0, "ccmp12_to_log10_mode3_fullres.txt", 3);
    std::cout << "\n";
    test_against_golden(4.0, "ccmp12_to_log10_mode2_binned.txt", 2);
    std::cout << "\n";

    /* ── cross-mode invariant ──────────────────────────────────────────────── */

    /* ONE GENERATOR, TWO TABLES, same reason ccmp_lut_test.cpp pins this for the
     * decompand alone: if these ever compare equal the binning selection has
     * collapsed and one mode is being composed through the other's curve. */
    CcmpParams pf, pb;
    CcmpLut lf, lb;
    std::string e1, e2;
    LogLut cf, cb;
    const bool got = ccmp_params_for_binning(1.0, pf) && ccmp_params_for_binning(4.0, pb) &&
                     lf.build(pf, &e1) && lb.build(pb, &e2) &&
                     cf.build_ccmp_composed(kLog16to10, lf) && cb.build_ccmp_composed(kLog16to10, lb);
    check(got, "both composed tables build", e1 + e2);
    if (got)
    {
        int worst = 0;
        for (size_t i = 0; i < cf.forward_size(); ++i)
            worst = std::max(worst, std::abs(static_cast<int>(cf.forward()[i]) -
                                             static_cast<int>(cb.forward()[i])));
        check(worst > 0, "the binned and full-res composed tables differ",
              "max |diff| " + std::to_string(worst) + " codes");
    }

    /* AN UNMEASURED BINNING MUST BE REFUSED, not silently composed against a
     * fabricated decompand — same reason ccmp_lut_test.cpp pins this. */
    CcmpParams unmeasured_p;
    CcmpLut unmeasured_lut;
    check(!ccmp_params_for_binning(2.0, unmeasured_p) && !unmeasured_lut.build(unmeasured_p),
          "an unmeasured binning factor never reaches build_ccmp_composed");

    /* An invalid target spec must be refused too, not silently built against a
     * malformed curve. */
    {
        CcmpParams p;
        ccmp_params_for_binning(1.0, p);
        CcmpLut decompand;
        decompand.build(p);
        LogLutParams bad = kLog16to10;
        bad.mu = 0.0;   /* valid() requires mu > 0 */
        LogLut composed;
        check(!composed.build_ccmp_composed(bad, decompand),
              "an invalid target LogLutParams is refused");
        check(!composed.valid(), "a failed composed build leaves the LUT empty");
    }

    std::cout << "\n" << (g_failures ? std::to_string(g_failures) + " FAILED\n" : "all passed\n");
    return g_failures ? 1 : 0;
}
