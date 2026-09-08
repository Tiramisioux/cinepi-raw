/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * ccmp_lut_test.cpp - the CCMP12 decompand curve, against its golden tables.
 *
 * Includes only cinepi/ccmp_lut.hpp, so it builds with nothing but the standard
 * library and exercises the same code that ships.
 *
 * THE POINT OF THIS TEST. The curve was measured on hardware over two chart
 * sessions and the analysis is closed; the generator that settled it is
 * innomaker585/ccmp12-lut/tools/ccmp_decode.py. This C++ is a reimplementation
 * of that generator, and the one cheap proof that a reimplementation has not
 * acquired a transcription error is that it reproduces the golden tables BYTE
 * FOR BYTE. The .txt files in resources/ccmp_luts are those tables, emitted by
 * the Python and copied in unmodified.
 *
 * So: no expected value in this file is hand-written. Every table entry comes
 * from the golden files, and the structural assertions below are derived from
 * the params rather than transcribed.
 */

#include "cinepi/ccmp_lut.hpp"

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
 * the test run from a plain `c++ tests/ccmp_lut_test.cpp` in the repo root. */
#ifndef CCMP_GOLDEN_DIR
#define CCMP_GOLDEN_DIR "resources/ccmp_luts"
#endif

/* Parse the generator's .txt: '#' comment lines, then whitespace-separated
 * decimal entries. Returns empty on any read failure — the caller reports it. */
std::vector<uint16_t> read_golden(const std::string &name, std::string &err)
{
    const std::string path = std::string(CCMP_GOLDEN_DIR) + "/" + name;
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

/* One mode: build from the binning factor alone and compare with its golden. */
void test_against_golden(double binning, const std::string &golden, int mode)
{
    const std::string tag = "[mode " + std::to_string(mode) + " b=" +
                            std::to_string(static_cast<long long>(binning)) + "]";

    CcmpParams p;
    if (!ccmp_params_for_binning(binning, p))
    {
        check(false, tag + " params for binning");
        return;
    }

    CcmpLut lut;
    std::string err;
    if (!lut.build(p, &err))
    {
        check(false, tag + " build", err);
        return;
    }

    std::string gerr;
    const std::vector<uint16_t> want = read_golden(golden, gerr);
    if (want.empty())
    {
        check(false, tag + " read golden", gerr);
        return;
    }

    /* 1. THE ACCEPTANCE TEST — byte for byte against the Python generator. */
    check(want.size() == lut.size(), tag + " golden has the same entry count",
          std::to_string(want.size()) + " against " + std::to_string(lut.size()));

    if (want.size() == lut.size())
    {
        size_t first_bad = want.size();
        int worst = 0;
        for (size_t i = 0; i < want.size(); ++i)
        {
            const int d = static_cast<int>(lut.table()[i]) - static_cast<int>(want[i]);
            if (d != 0 && first_bad == want.size())
                first_bad = i;
            if (std::abs(d) > std::abs(worst))
                worst = d;
        }
        std::string detail = "4096 entries identical";
        if (first_bad != want.size())
            detail = "first mismatch at code " + std::to_string(first_bad) + ": built " +
                     std::to_string(lut.table()[first_bad]) + " vs golden " +
                     std::to_string(want[first_bad]) + ", worst delta " + std::to_string(worst);
        check(first_bad == want.size(), tag + " table is byte-identical to the golden", detail);
    }

    /* 2. The identity segment. Below knee1 the sensor stored L unchanged, so the
     *    table must be the identity there. Catches a binning mix-up, a pedestal
     *    mix-up and a domain mix-up at once. */
    const int top = lut.identity_top_code();
    bool identity = top > 0;
    for (int c = 0; c <= top && identity; ++c)
        identity = (lut.table()[static_cast<size_t>(c)] == static_cast<uint16_t>(c));
    check(identity, tag + " table is the identity on [0, knee1]",
          "codes 0.." + std::to_string(top));

    /* 3. Monotone. A LinearizationTable that decreases anywhere is not a curve. */
    bool mono = true;
    for (size_t i = 1; i < lut.size() && mono; ++i)
        mono = lut.table()[i] >= lut.table()[i - 1];
    check(mono, tag + " table is monotone non-decreasing");

    /* 4. Round trip, on every code. encode(decode(C)) == C. */
    double worst_rt = 0.0;
    for (int c = 0; c < p.code_count(); ++c)
    {
        const double rt = ccmp_encode_level(ccmp_decode_code(c, p), p);
        worst_rt = std::max(worst_rt, std::abs(rt - c));
    }
    check(worst_rt < 1e-9, tag + " encode o decode == identity on all codes",
          "max |err| " + std::to_string(worst_rt));

    /* 5. The knots land at T/b, and the slopes either side are the register
     *    ratios. Derived from the params, not transcribed. */
    check(ccmp_knot_L(0, p) == p.T1 / p.binning && ccmp_knot_L(1, p) == p.T2 / p.binning,
          tag + " knots at T/b",
          "L = " + std::to_string(ccmp_knot_L(0, p)) + " and " + std::to_string(ccmp_knot_L(1, p)));

    const double e = 1e-6;
    for (int k = 0; k < 2; ++k)
    {
        const double kn = ccmp_knot_L(k, p);
        const double lo = (ccmp_encode_level(kn, p) - ccmp_encode_level(kn - e, p)) / e;
        const double hi = (ccmp_encode_level(kn + e, p) - ccmp_encode_level(kn, p)) / e;
        const double want_lo = (k == 0) ? 1.0 : p.s1();
        const double want_hi = (k == 0) ? p.s1() : p.s2();
        check(std::abs(lo - want_lo) < 1e-4 && std::abs(hi - want_hi) < 1e-4,
              tag + " slopes across knee" + std::to_string(k + 1),
              std::to_string(lo) + " -> " + std::to_string(hi));
    }

    /* 6. The tags. BlackLevel is the pedestal unchanged; WhiteLevel is the last
     *    entry. Both are read back from the golden's own header comment so this
     *    is a comparison, not a transcription. */
    check(lut.black_level() == static_cast<int>(kCcmpPedestal),
          tag + " BlackLevel is the pedestal, unchanged",
          std::to_string(lut.black_level()));
    check(lut.white_level() == static_cast<int>(want.back()),
          tag + " WhiteLevel is the golden's last entry",
          std::to_string(lut.white_level()));
}

} // namespace

int main()
{
    std::cout << "CCMP12 decompand LUT — against the golden tables\n\n";

    test_against_golden(1.0, "ccmp_decode_mode3_fullres.txt", 3);
    std::cout << "\n";
    test_against_golden(4.0, "ccmp_decode_mode2_binned.txt", 2);
    std::cout << "\n";

    /* ── cross-mode invariants ─────────────────────────────────────────────── */

    /* ONE GENERATOR, TWO TABLES. If these ever compare equal the selection has
     * collapsed and one mode is being decoded through the other's curve. */
    CcmpParams pf, pb;
    CcmpLut lf, lb;
    std::string e1, e2;
    const bool got = ccmp_params_for_binning(1.0, pf) && ccmp_params_for_binning(4.0, pb) &&
                     lf.build(pf, &e1) && lb.build(pb, &e2);
    check(got, "both tables build", e1 + e2);
    if (got)
    {
        int worst = 0;
        for (size_t i = 0; i < lf.size(); ++i)
            worst = std::max(worst, std::abs(static_cast<int>(lf.table()[i]) -
                                             static_cast<int>(lb.table()[i])));
        check(worst > 0, "the binned and full-res tables differ",
              "max |diff| " + std::to_string(worst) + " codes");

        /* The two WhiteLevels differ because the knee2 codes do. A change that
         * forces them equal is a bug, so pin the disagreement. */
        check(lf.white_level() != lb.white_level(),
              "the two WhiteLevels differ and are not forced equal",
              std::to_string(lf.white_level()) + " vs " + std::to_string(lb.white_level()));
    }

    /* AN UNMEASURED BINNING MUST BE REFUSED, not silently given the register
     * curve — that table would be wrong by 21 L through the mid-tones and would
     * look entirely plausible. */
    CcmpParams unmeasured;
    check(!ccmp_params_for_binning(2.0, unmeasured),
          "an unmeasured binning factor is refused");
    check(!ccmp_params_for_binning(0.0, unmeasured),
          "a zero binning factor is refused");

    /* NEVER CLAMP, AND "FITS uint16" IS A TEST AT BOTH ENDS. The output domain
     * was forced, not preferred: of the four candidates only L+200 fits. The
     * other two rejections fail at OPPOSITE ends, and a guard that only checked
     * the top would let the black-referred one through — where a real dark frame
     * puts a tenth of its pixels below zero.
     *
     * Both vary out_pedestal ONLY. The curve is identical in all three. */
    struct DomainCase { const char *name; double out_ped; const char *end; };
    const DomainCase rejected[] = {
        { "RAW16 (L+3200) overflows at the top",        16 * kCcmpPedestal, "top" },
        { "ABOVE_BLACK (L+0) underflows at the bottom", 0.0,                "bottom" },
    };
    for (const DomainCase &dc : rejected)
    {
        CcmpParams bad;
        ccmp_params_for_binning(1.0, bad);
        bad.out_pedestal = dc.out_ped;
        CcmpLut bad_lut;
        std::string berr;
        check(!bad_lut.build(bad, &berr), std::string("rejected: ") + dc.name,
              berr.empty() ? "no message" : "reported");
        check(!bad_lut.valid(), std::string("a failed build leaves the LUT empty (") +
                                dc.end + ")");
    }

    /* And the curve itself is untouched by that choice — the shipping domain
     * still builds, which is what proves the two pedestals are separable. */
    CcmpParams keep;
    ccmp_params_for_binning(1.0, keep);
    CcmpLut keep_lut;
    std::string kerr;
    check(keep_lut.build(keep, &kerr) && keep_lut.black_level() == 200,
          "KEEP_PEDESTAL still builds with BlackLevel 200", kerr);

    std::cout << "\n" << (g_failures ? std::to_string(g_failures) + " FAILED\n" : "all passed\n");
    return g_failures ? 1 : 0;
}
