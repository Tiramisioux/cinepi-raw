/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * ccmp_lut.hpp - imx585 CCMP decompand curve + the DNG LinearizationTable.
 *
 * Pure table math: no libcamera, no Redis, no JSON — only the standard library,
 * so tests/ccmp_lut_test.cpp exercises the *same* code that ships.
 *
 * WHAT THIS IS FOR. In 12-bit ClearHDR the imx585 companands on-sensor: it maps
 * the 16-bit ClearHDR signal through a three-segment piecewise-linear curve and
 * stores 12-bit codes. A DNG carrying those codes as if they were linear renders
 * with the mid-tones crushed magenta — the highlights white-balance, everything
 * below does not, because the defect is the transfer curve and not the gains. A
 * LinearizationTable undoes it in the file, so a converter sees linear data and
 * no post step is needed.
 *
 * This is NOT the CineMate Log curve and shares nothing with it. Log companding
 * is a mu-law curve the user opts into for grading; this is a sensor compression
 * the user did not choose and cannot decline. They compose (a log 12-bit ClearHDR
 * recording must decompand first, then log-encode) but that precomposition lives
 * wherever the two branches meet, not here.
 *
 * THE MODEL, as measured on hardware over two chart sessions:
 *
 *     stored_code = ccmp(b*L)/b + P      L = linear above black, in 16-bit
 *                                            ClearHDR LSB
 *     ccmp(x) = x                        x <= T1
 *             = T1 + (x-T1)*s1           T1 < x <= T2
 *             = C2 + (x-T2)*s2           x >  T2,  C2 = T1 + (T2-T1)*s1
 *
 *     table[C] = floor( ccmp_inv(b*(C-P))/b + 200 + 0.5 )      C = 0..4095
 *
 * innomaker585/ccmp12-lut/tools/ccmp_decode.py is the single source of truth for
 * the curve; this is an operation-for-operation port of its build(). The golden
 * tables it emits are in that workspace's evidence/tables/ and this header must
 * reproduce them byte for byte — tests/ccmp_lut_test.cpp asserts exactly that.
 *
 * ── FOUR DETAILS ARE LOAD-BEARING AND CHANGE THE TABLE IF "CLEANED UP" ────────
 *
 *   1. ONE GENERATOR, TWO TABLES, SELECTED ON BINNING. The compander's input is
 *      the BINNED signal, so a 2x2-binned mode puts the knees 4x lower in the
 *      delivered-linear domain. At code 400 the decode is 200 full-res and
 *      3387.5 binned. A LinearizationTable takes no mode parameter, so the two
 *      modes need two different tables. Getting the selection backwards is wrong
 *      by 2.6x at knee1 and does not look obviously wrong.
 *
 *   2. floor(x + 0.5), NOT nearbyint(). This is the opposite of log_lut.hpp,
 *      which needs banker's rounding to match numpy — here the generator uses
 *      an explicit floor(x+0.5) precisely because ties are common: every entry
 *      on the binned mode's top segment is a half-integer (16*C - 2812.5), and
 *      ties-to-even would alternate the step 15/17 and put a visible ripple in
 *      the slope.
 *
 *   3. NEVER CLAMP. A table entry is a TIFF SHORT. If the output domain does not
 *      fit uint16 the build FAILS — it does not clip. A clipped highlight or a
 *      clipped shadow is exactly the defect this table exists to remove, and
 *      "fits uint16" is a test at BOTH ends: the black-referred domain fails at
 *      the bottom, not the top.
 *
 *   4. THE ANCHOR IS PER MODE AND IS NOT DERIVABLE. T1 carries a measured
 *      correction (see kT1Effective) that does not scale with b, so it cannot be
 *      computed from the register value. A binning factor with no measured
 *      anchor is an unvalidated mode and build() refuses it rather than silently
 *      falling back to the register curve, which is wrong by 21 L on the middle
 *      segment.
 *
 * ── THE OUTPUT DOMAIN ─────────────────────────────────────────────────────────
 *
 * KEEP_PEDESTAL: the table outputs L + 200, so BlackLevel stays 200 and only
 * WhiteLevel is rewritten. Of the four candidates considered this is the only
 * one that fits uint16 at both ends for both modes. Under a LinearizationTable a
 * reader applies the curve BEFORE reading the level tags, so both describe the
 * table's OUTPUT domain, not the stored codes.
 *
 * The two WhiteLevels differ (63265 full-res, 62704 binned) because the knee2
 * codes do. DO NOT force them equal.
 */

#ifndef CINEPI_CCMP_LUT_HPP
#define CINEPI_CCMP_LUT_HPP

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

/* ── the registers, read back from the driver with ClearHDR live ─────────────
 *
 * The ACMP menu index IS the register value and the ratio is 1/2^idx, so the
 * slopes are exact powers of two and every division by them is exact. */
inline constexpr double kCcmpT1Register = 500.0;    /* CCMP threshold 1        */
inline constexpr double kCcmpT2Register = 11500.0;  /* CCMP threshold 2        */
inline constexpr int    kCcmpAcmp1Index = 6;        /* -> s1 = 1/64            */
inline constexpr int    kCcmpAcmp2Index = 4;        /* -> s2 = 1/16            */
inline constexpr double kCcmpPedestal   = 200.0;    /* BlackLevel, all 6 modes */
inline constexpr int    kCcmpBits       = 12;       /* BitsPerSample           */

/* ── the middle segment's ANCHOR, measured ────────────────────────────────────
 *
 * The middle and high segments both hang off one number,
 *
 *     a1 = P + T1*(1 - s1)/b
 *
 * and P, T1 and b enter them ONLY through it — so on those segments the three
 * are DEGENERATE and no data from them can separate a pedestal error from a
 * threshold error from a binning error. What the data DOES determine, to
 * +/-0.03 codes and reproducibly on both chart sessions, is a1 itself. The
 * register curve is short by 0.3336 codes (b=1) and 0.2321 (b=4), which is
 * 21 L and 15 L on the 1/64 segment.
 *
 * Carried here as an effective T1 with P held at the measured tag 200 and b at
 * the design value. THAT ASSIGNMENT IS A CHOICE, NOT A MEASUREMENT — the same
 * a1 is a pedestal of 200.33/200.23 or a b of 0.9993/3.992, and the table is
 * bit-identical either way. It is written on T1 because that is the only one of
 * the three the identity segment does not contradict.
 *
 * ** The correction does NOT scale as 1/b ** — 0.33 against 0.23 is a ratio of
 * 1.4, where a pure T1 error would give 4 and a pure pedestal error 1. So it is
 * not one physical parameter shared by the two modes; it is two measured
 * anchors, one per table, and neither can be derived from the other. */
struct CcmpAnchor
{
    double binning;   /* b — pixels summed per output sample: 1 or 4 */
    double t1_eff;    /* the effective T1 that delivers the measured a1 */

    /* THE FLOOR OF THE CLEARHDR CLAMP ZONE, in this binning's own codes.
     * Preview-only: nothing in the DNG path reads it. See
     * CcmpPreviewColour::sensor_clip_code for what the zone is and why the
     * FLOOR is the anchor rather than the peak code.
     *
     * It has to be per binning because the code a given scene level lands on
     * is per binning — the compander is applied to b*L and divided back by b,
     * so the same clamp sits at 2900-ish at b=1 and 2582-ish at b=4. Anchoring
     * both on one number is why HD stayed magenta after full res was fixed. */
    unsigned clip_code;
};

inline constexpr CcmpAnchor kT1Effective[] = {
    /* clip_code b=1: MEASURED on takes CINEPI_26-09-06_210738 and _214831 —
     * magenta-area codes p1 2974 / 2948, peak 3054 / 3027 — then dropped below
     * both p1 values for margin. Confirmed on hardware: full res renders blown
     * highlights neutral at this anchor.
     *
     * clip_code b=4: DERIVED, then CONFIRMED IN PLACE — not measured from a
     * binned take, so it is still the softer of the two numbers. It applies the
     * peak-to-floor ratio measured at b=1 (1.0378) to the one binned peak the
     * log had (2723), plus the same 48-code margin, which puts the anchor-to-
     * peak band at 0.082 stops against 0.078 at b=1.
     *
     * Confirmed on hardware 2026-09-07: binned ClearHDR renders blown
     * highlights neutral, and the stage reports `highest uncorrected 2581`
     * against this anchor of 2582 — i.e. everything at or above it is fully
     * corrected and the uncorrected ceiling sits exactly one code below, which
     * is the signature of an anchor placed correctly rather than one that
     * merely happens to be low enough. A direct measurement (record a binned
     * 12-bit ClearHDR take with a blown highlight, read the codes under the
     * magenta area of its embedded thumbnail, as was done for b=1) would still
     * be worth taking if this mode is ever re-tuned. */
    { 1.0, 500.3389, 2900 },   /* full res 3856x2180 */
    { 4.0, 500.9431, 2582 },   /* 2x2 binned 1928x1090 */
};

/* Curve parameters for one mode. `binning` is the only input that varies. */
struct CcmpParams
{
    double binning = 0.0;                  /* b — 1 (full res) or 4 (2x2)      */
    double T1 = 0.0;                       /* effective threshold 1 (anchored) */
    double T2 = kCcmpT2Register;

    /* TWO PEDESTALS, AND THEY ARE NOT THE SAME NUMBER even though both are 200
     * in the shipping configuration.
     *
     *   pedestal      P — the curve's own origin. The compander is BLACK-
     *                 REFERRED, so this is subtracted from the stored code
     *                 before the inverse and it is part of the TRANSFER. It is
     *                 measured (200 on all six modes) and changing it changes
     *                 the curve.
     *   out_pedestal  what the table ADDS on the way out, i.e. which domain the
     *                 table's output lives in. This is a CHOICE among four
     *                 candidates, forced to L+200 by the uint16 constraint —
     *                 the black-referred domain (0) underflows at the bottom and
     *                 the raw-16 domain (3200) overflows at the top.
     *
     * Conflating them silently couples the BlackLevel tag to the transfer: an
     * edit meaning "write a different BlackLevel" would move the curve instead. */
    double pedestal = kCcmpPedestal;       /* P — part of the transfer         */
    double out_pedestal = kCcmpPedestal;   /* the output domain — a choice     */

    /* Preview-only, carried here so it travels with the rest of the per-binning
     * anchor. See CcmpAnchor::clip_code. */
    unsigned clip_code = 0;

    int s1_index = kCcmpAcmp1Index;
    int s2_index = kCcmpAcmp2Index;
    int bits = kCcmpBits;

    double s1() const { return std::ldexp(1.0, -s1_index); }
    double s2() const { return std::ldexp(1.0, -s2_index); }

    /* ccmp() at the second knot — the value the high segment extrapolates from. */
    double C2() const { return T1 + (T2 - T1) * s1(); }

    int code_count() const { return (bits >= 1 && bits <= 16) ? (1 << bits) : 0; }

    bool valid() const
    {
        if (bits < 1 || bits > 16)
            return false;
        if (!(binning > 0.0))
            return false;
        if (!(T1 > 0.0) || !(T2 > T1))
            return false;
        if (pedestal < 0.0 || out_pedestal < 0.0)
            return false;
        if (s1_index < 0 || s1_index > 16 || s2_index < 0 || s2_index > 16)
            return false;
        /* The compander expands going up: 1 -> s1 -> s2 with s1 < s2 < 1. A
         * spec that violates that is not this sensor's curve. */
        if (!(s1() < s2() && s2() < 1.0))
            return false;
        return true;
    }

    std::string describe() const
    {
        return "CCMP12 decompand  b=" + std::to_string(static_cast<long long>(binning)) +
               "  T1=" + std::to_string(T1) + "  T2=" + std::to_string(static_cast<long long>(T2)) +
               "  s1=1/" + std::to_string(1 << s1_index) +
               "  s2=1/" + std::to_string(1 << s2_index) +
               "  P=" + std::to_string(static_cast<long long>(pedestal));
    }
};

/* The measured curve for a binning factor. Returns false when no anchor has been
 * measured for it — see load-bearing detail 4: falling back to the register T1
 * would emit a plausible table that is wrong by 21 L through the mid-tones. */
inline bool ccmp_params_for_binning(double binning, CcmpParams &params)
{
    for (const CcmpAnchor &a : kT1Effective)
    {
        if (a.binning == binning)
        {
            CcmpParams p;
            p.binning = a.binning;
            p.T1 = a.t1_eff;
            p.clip_code = a.clip_code;
            params = p;
            return p.valid();
        }
    }
    return false;
}

/* ── the compander, in its own (pre-binning) domain ──────────────────────────── */

inline double ccmp_forward_x(double x, const CcmpParams &p)
{
    if (x <= p.T1)
        return x;
    if (x <= p.T2)
        return p.T1 + (x - p.T1) * p.s1();
    return p.C2() + (x - p.T2) * p.s2();
}

inline double ccmp_inverse_y(double y, const CcmpParams &p)
{
    if (y <= p.T1)
        return y;
    if (y <= p.C2())
        return p.T1 + (y - p.T1) / p.s1();
    return p.T2 + (y - p.C2()) / p.s2();
}

/* ── the delivered transfer: linear-above-black <-> stored 12-bit code ──────── */

/* L (16-bit ClearHDR LSB, above black) -> stored code. */
inline double ccmp_encode_level(double L, const CcmpParams &p)
{
    return ccmp_forward_x(p.binning * L, p) / p.binning + p.pedestal;
}

/* Stored code -> L. This is the table's payload before the pedestal is added. */
inline double ccmp_decode_code(double C, const CcmpParams &p)
{
    return ccmp_inverse_y(p.binning * (C - p.pedestal), p) / p.binning;
}

/* Where the knees land, in both domains. */
inline double ccmp_knot_L(int which, const CcmpParams &p)
{
    return (which == 0 ? p.T1 : p.T2) / p.binning;
}

inline double ccmp_knee_code(int which, const CcmpParams &p)
{
    return ccmp_encode_level(ccmp_knot_L(which, p), p);
}

/* ── the table ────────────────────────────────────────────────────────────────
 *
 * Built once per binning factor and held for the process lifetime (8 KB). */
class CcmpLut
{
public:
    /* Returns false and leaves the LUT empty on malformed params or a domain
     * that does not fit uint16. `err` is set on failure; NOTHING IS CLAMPED. */
    bool build(const CcmpParams &p, std::string *err = nullptr)
    {
        table_.clear();
        params_ = CcmpParams{};

        if (!p.valid())
        {
            if (err)
                *err = "invalid CCMP params: " + p.describe();
            return false;
        }

        const int n = p.code_count();
        std::vector<double> exact(static_cast<size_t>(n));
        for (int c = 0; c < n; ++c)
            exact[static_cast<size_t>(c)] = ccmp_decode_code(c, p) + p.out_pedestal;

        /* Both ends, before any entry is written. The tolerance is 1e-6 of one
         * code so a float ulp at the top cannot be reported as a clipped
         * highlight, which would be a lie. */
        const double lo = exact.front();
        const double top = exact.back();
        if (lo < -1e-6 || top > 65535.0 + 1e-6)
        {
            if (err)
                *err = "CCMP decompand table does not fit uint16 for b=" +
                       std::to_string(static_cast<long long>(p.binning)) + ": range " +
                       std::to_string(lo) + ".." + std::to_string(top) +
                       ". A LinearizationTable entry is a TIFF SHORT and clamping "
                       "would lose the very range this table exists to recover.";
            return false;
        }

        table_.resize(static_cast<size_t>(n));
        for (int c = 0; c < n; ++c)
            table_[static_cast<size_t>(c)] =
                static_cast<uint16_t>(std::floor(exact[static_cast<size_t>(c)] + 0.5));

        params_ = p;
        return true;
    }

    bool valid() const { return !table_.empty(); }
    const CcmpParams &params() const { return params_; }

    const uint16_t *table() const { return table_.data(); }
    size_t size() const { return table_.size(); }

    /* The two level tags, in the table's OUTPUT domain — so this is the OUTPUT
     * pedestal, not the curve's P. WhiteLevel is the last entry, which is where
     * the decoded top code lands. */
    int black_level() const { return static_cast<int>(std::floor(params_.out_pedestal + 0.5)); }
    int white_level() const { return table_.empty() ? 0 : static_cast<int>(table_.back()); }
    /* The preview's highlight anchor for this binning — see CcmpAnchor. */
    unsigned clip_code() const { return params_.clip_code; }

    /* The last code the curve is the identity on. Below this the sensor stored
     * L unchanged, so the table must too — a one-line check that catches a
     * binning mix-up, a pedestal mix-up and a domain mix-up at once. */
    int identity_top_code() const
    {
        return table_.empty() ? -1 : static_cast<int>(std::floor(ccmp_knee_code(0, params_)));
    }

private:
    CcmpParams params_;
    std::vector<uint16_t> table_;
};

/* ── the process-wide cache ───────────────────────────────────────────────────
 *
 * Keyed on binning, built on first use. A redis mode switch reconfigures the
 * encoder with a different binning mid-run, so this cannot be a startup
 * singleton. Defined in ccmp_lut.cpp; the header stays dependency-free so the
 * test can build against the math alone. */
const CcmpLut *get_ccmp_lut(double binning, std::string &err);

#endif /* CINEPI_CCMP_LUT_HPP */
