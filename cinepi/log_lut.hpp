/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * log_lut.hpp - CineMate Log v1.1 curve + dense forward/inverse tables.
 *
 * Pure table math: no libcamera, no Redis, no JSON — only the standard library,
 * so tests/log_lut_test.cpp exercises the *same* code that ships. Spec loading
 * (jsoncpp) lives in log_lut.cpp so the test executable needs no dependencies.
 *
 * The curve is mu-law companding of the signal above black plus a small linear
 * FOOTROOM segment below it, so sub-black sensor noise survives instead of being
 * half-wave-rectified into code 0:
 *
 *   codes [0, F)     linear ramp over [BL-foot, BL)   -- sub-black noise floor
 *   codes [F, CMAX]  mu-law over [BL, WL]             -- the picture
 *
 *   encode L>=BL: C = F + nearbyint(TOP*log1p(mu*x)/log1p(mu)), x=clamp((L-BL)/(WL-BL),0,1)
 *   encode L< BL: C = clamp(floor((L-(BL-foot))/foot*F), 0, F-1)
 *   decode C>=F : L = BL + (pow(1+mu,(C-F)/TOP)-1)/mu*(WL-BL)
 *   decode C< F : L = (BL-foot) + (C+0.5)/F*foot
 *   where CMAX = 2^target-1 and TOP = CMAX-F.
 *
 * resources/log_luts/gen_cinemate_log.py is the single source of truth for the
 * curve; this is a operation-for-operation port of its build(). Three details
 * are load-bearing and change the emitted table if "cleaned up":
 *
 *   1. floor() on encode below black, bin CENTRE (C+0.5) on decode. round() plus
 *      the bin edge biases the reconstructed black point (measured +1.5 LSB).
 *   2. nearbyint(), not round(). The generator uses numpy's banker's rounding
 *      (ties-to-even); std::round is ties-away-from-zero and will not reproduce
 *      the table. std::nearbyint under the default FE_TONEAREST matches.
 *   3. The division-then-multiply order (`/log1p(mu)*TOP`, `/foot*F`, `/F*foot`).
 *      Reassociating changes the last bit on some entries.
 *
 * The inverse table is exactly the DNG LinearizationTable (tag 0xC618): it is
 * uint16_t code->linear, which is the tag's SHORT payload with no conversion.
 * BlackLevel/WhiteLevel stay in the LINEAR (table-output) domain — they are not
 * rescaled to the stored code range.
 */

#ifndef CINEPI_LOG_LUT_HPP
#define CINEPI_LOG_LUT_HPP

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

/* Curve parameters — the `params` block of a resources/log_luts spec. BL/WL and
 * the footroom extent are per-sensor data; the runtime flag picks target_bits. */
struct LogLutParams
{
    double mu = 0.0;          /* mu-law compression, ~= SPAN / linear-toe size    */
    int black_level = 0;      /* BL, in source linear LSB                         */
    int white_level = 0;      /* WL, in source linear LSB                         */
    int source_bits = 0;      /* linear source depth (forward table is 2^this)    */
    int target_bits = 0;      /* log code depth   (inverse table is 2^this)       */
    int footroom_codes = 0;   /* F — codes reserved below black                   */
    int footroom_lsb = 0;     /* foot — how far below BL those codes reach        */

    /* Derived. Only meaningful once valid(); the shift is clamped so a malformed
     * spec cannot reach UB before validation runs. */
    int code_max() const { return (target_bits >= 1 && target_bits <= 16) ? (1 << target_bits) - 1 : 0; }
    int span() const { return white_level - black_level; }
    int top() const { return code_max() - footroom_codes; }
    bool has_footroom() const { return footroom_codes > 0 && footroom_lsb > 0; }

    bool valid() const
    {
        if (source_bits < 1 || source_bits > 16 || target_bits < 1 || target_bits > 16)
            return false;
        if (target_bits > source_bits)          /* log-encoding never expands     */
            return false;
        if (!(mu > 0.0))
            return false;
        if (black_level < 0 || white_level <= black_level)
            return false;
        if (white_level > (1 << source_bits) - 1)
            return false;
        if (footroom_codes < 0 || footroom_codes >= code_max())  /* TOP >= 1      */
            return false;
        if (footroom_lsb < 0 || footroom_lsb > black_level)      /* not below 0   */
            return false;
        return true;
    }

    std::string describe() const
    {
        return "CineMate Log " + std::to_string(source_bits) + "->" + std::to_string(target_bits) +
               " bit  mu=" + std::to_string(static_cast<long long>(mu)) +
               "  black=" + std::to_string(black_level) + "  white=" + std::to_string(white_level) +
               "  footroom=" + std::to_string(footroom_codes) + "/" + std::to_string(footroom_lsb);
    }
};

/* Linear source level -> log code. Mirrors gen_cinemate_log.py enc(). */
inline int log_encode_code(double L, const LogLutParams &p)
{
    const int cmax = p.code_max();
    const int F = p.footroom_codes;
    const double top = p.top();

    double C;
    if (p.has_footroom() && L < p.black_level)
    {
        const double foot = p.footroom_lsb;
        C = std::floor((L - (p.black_level - foot)) / foot * F);
        C = std::min(std::max(C, 0.0), static_cast<double>(F - 1));
    }
    else
    {
        /* Degenerate footroom (F>0 but foot==0) lands here too and maps every
         * sub-black level to code F, i.e. black — the generator would divide by
         * zero, so there is nothing to match. No shipped spec hits this. */
        double x = (L - p.black_level) / p.span();
        x = std::min(std::max(x, 0.0), 1.0);
        C = std::nearbyint(std::log1p(p.mu * x) / std::log1p(p.mu) * top) + F;
    }
    return static_cast<int>(std::min(std::max(C, 0.0), static_cast<double>(cmax)));
}

/* Log code -> linear source level. Mirrors gen_cinemate_log.py dec(); the result
 * for code in [0, 2^target) is the DNG LinearizationTable. */
inline int log_decode_level(int code, const LogLutParams &p)
{
    const int F = p.footroom_codes;
    const double top = p.top();
    const double C = code;

    double L;
    if (p.has_footroom() && code < F)
    {
        const double foot = p.footroom_lsb;
        L = (p.black_level - foot) + (C + 0.5) / F * foot;
    }
    else
    {
        const double up = (std::pow(1.0 + p.mu, std::max(0.0, C - F) / top) - 1.0) / p.mu;
        L = p.black_level + up * p.span();
    }
    L = std::nearbyint(L);
    return static_cast<int>(std::min(std::max(L, 0.0), static_cast<double>(p.white_level)));
}

/* Dense tables built once at startup: forward[2^source_bits] for the encode hot
 * path, inverse[2^target_bits] for tag 0xC618. 16-bit source costs 128 KB — hold
 * one instance, never a static const in a header (one copy per TU). */
class LogLut
{
public:
    /* Returns false and leaves the LUT empty if the params are malformed. */
    bool build(const LogLutParams &p)
    {
        forward_.clear();
        inverse_.clear();
        params_ = LogLutParams{};
        if (!p.valid())
            return false;
        params_ = p;

        forward_.resize(static_cast<size_t>(1) << p.source_bits);
        for (size_t i = 0; i < forward_.size(); ++i)
            forward_[i] = static_cast<uint16_t>(log_encode_code(static_cast<double>(i), p));

        inverse_.resize(static_cast<size_t>(1) << p.target_bits);
        for (size_t c = 0; c < inverse_.size(); ++c)
            inverse_[c] = static_cast<uint16_t>(log_decode_level(static_cast<int>(c), p));
        return true;
    }

    bool valid() const { return !forward_.empty(); }
    const LogLutParams &params() const { return params_; }

    const uint16_t *forward() const { return forward_.data(); }
    const uint16_t *inverse() const { return inverse_.data(); }
    size_t forward_size() const { return forward_.size(); }
    size_t inverse_size() const { return inverse_.size(); }

    /* Encode hot path. Clamps rather than trusting the caller's depth so a row
     * that overshoots the source range cannot walk off the table. */
    uint16_t encode(unsigned linear) const
    {
        return forward_[std::min(static_cast<size_t>(linear), forward_.size() - 1)];
    }

    /* Encode a whole row. Output codes are right-justified in target_bits, which
     * is exactly what dng_pack.hpp's pack_row_12bit/pack_row_10bit consume — the
     * log path needs no depth-converting packer.
     *
     * dst MAY alias src: the map is per-sample and the write to x follows the
     * read of x. The encoder relies on that to decompress COMP1 and log-encode
     * through a single scratch row. */
    void encode_row(const uint16_t *src, uint16_t *dst, size_t width) const
    {
        for (size_t x = 0; x < width; ++x)
            dst[x] = encode(src[x]);
    }

private:
    LogLutParams params_;
    std::vector<uint16_t> forward_;
    std::vector<uint16_t> inverse_;
};

/* ── spec loading (log_lut.cpp — needs jsoncpp) ─────────────────────────────── */

/* Canonical spec basename for a depth pair, e.g. "cinemate_log_16to12.json". */
std::string log_lut_spec_filename(int source_bits, int target_bits);

/* First readable spec for the depth pair, searching $CINEPI_LOG_LUT_DIR, the
 * installed datadir, the Pi's repo clone, then ./resources/log_luts. "" if none. */
std::string find_log_lut_spec(int source_bits, int target_bits);

/* Parse one spec file. Fills `params`; if `table` is non-null it also receives the
 * spec's embedded linearization_table. Returns false with `err` set on failure. */
bool load_log_lut_spec(const std::string &path, LogLutParams &params,
                       std::vector<uint16_t> *table, std::string &err);

/* Find + parse + build, then verify the rebuilt inverse table matches the spec's
 * embedded linearization_table entry-for-entry. A mismatch means this C++ curve
 * has drifted from the generator and the DNG's tag would not match its pixels, so
 * it is a hard failure, not a warning. */
bool load_log_lut(int source_bits, int target_bits, LogLut &lut, std::string &err);

/* ── the process-wide cache ─────────────────────────────────────────────────── */
/*
 * There is no single startup load, because the flag only fixes the TARGET depth.
 * The SOURCE depth is a property of the configured camera mode — it is not known
 * when the options are parsed, and a redis mode switch reconfigures the encoder
 * with a different one mid-run. So the LUT is keyed by the pair and built on
 * first use: the startup probe warms whatever the shipped specs cover, and the
 * encoder asks for the pair it actually has.
 */

/* Source depths a shipped spec may exist for, most-likely first. */
extern const int kLogLutSourceBits[2];

/* Cached LUT for a depth pair, built on first use and kept for the process
 * lifetime (a 16-bit source costs 128 KB + 8 KB). Returns nullptr with `err` set
 * when no spec ships for the pair or the build failed; the failure is cached too,
 * so a missing spec is not re-searched once per frame. Thread-safe: encode
 * workers call this concurrently, and the returned pointer stays valid. */
const LogLut *get_log_lut(int source_bits, int target_bits, std::string &err);

/* Warm the cache for every source depth at `target_bits` and describe what came
 * back, for the startup log. Returns the number of usable source depths; on 0,
 * `summary` explains why. */
int preload_log_luts(int target_bits, std::string &summary);

#endif /* CINEPI_LOG_LUT_HPP */
