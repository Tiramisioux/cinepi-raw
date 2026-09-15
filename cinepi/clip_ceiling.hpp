/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * clip_ceiling.hpp - measure where the frame's data actually stops, so
 *                    WhiteLevel can say so.
 *
 * Pure counting: no libcamera, no app headers, integer arithmetic only, so
 * tests/clip_ceiling_test.cpp exercises the *same* code that ships.
 *
 * WHAT THIS IS FOR — AND IT IS NOT THE PREVIEW'S PROBLEM. In ClearHDR the
 * imx585's HG/LG merge clamps well below the container: measured on recorded
 * DNGs, three 4K takes stopped at 89%, 89% and 73.5% of the declared
 * WhiteLevel. The DNG nevertheless declares WhiteLevel at the curve's nominal
 * full scale (log_lut params().white_level, or the CCMP table's last entry),
 * because that is what the TABLE's output domain is — correct as a statement
 * about the curve, wrong as a statement about the data.
 *
 * The consequence is not subtle and it is not a preview artefact. No pixel
 * ever reaches the declared WhiteLevel, so every converter concludes the frame
 * contains no clipped highlight and skips highlight reconstruction. A blown
 * highlight then renders MAGENTA, for a reason worth writing down because it
 * is entirely mechanical:
 *
 *     AsShotNeutral is ~(0.59, 1, 0.59), so white balance multiplies R and B
 *     by ~1.7 and GREEN BY EXACTLY 1.0. In a blown area all three channels sit
 *     at the clamp — say 0.89 of the declared white. R and B are scaled to
 *     1.51 and clip to 1.0; green is scaled to 0.89 and stays there. Red and
 *     blue pinned, green 11% short: that is magenta, by construction, and no
 *     amount of grading removes it because the file says nothing is clipped.
 *
 * Declaring the measured clamp as WhiteLevel puts the three channels back on
 * the same ceiling and the highlight renders neutral. Verified on
 * CINEPI_26-09-15_172306_F06 frame 59 by patching only tag 0xC61D in a byte
 * copy: LibRaw's colour cast over the blown area goes 12.0% -> 0.0%, and a
 * direct render goes sRGB (255,157,249) -> (255,245,249).
 *
 * WHAT THIS CANNOT DO. It fixes the COLOUR of a clamped highlight, not its
 * DETAIL. The merge stopped; there is no roll-off in the signal to recover.
 * A featureless pink blob becomes a featureless white blob, which is what a
 * blown highlight is supposed to look like. lessons/hardware-log.md's
 * 2026-09-14 "the merge ceiling is hard at ~0.55 of the container" entry is
 * the standing statement of that limit, and nothing here changes it.
 *
 * ── FOUR RULES ARE LOAD-BEARING ──────────────────────────────────────────────
 *
 *   1. CHANNEL CONVERGENCE IS THE SIGNATURE, AND IT IS SECOND-OVER-MAX.
 *      A large bright area alone does not mean a clamp — it could be a wall.
 *      What distinguishes a clamp is that the colour channels PIN TOGETHER:
 *      a neutral subject's raw green sits ~1.7x its raw red (that ratio IS
 *      AsShotNeutral), so channels landing on the SAME code is not something
 *      a subject does by accident.
 *
 *      But the test must be SECOND-over-max, not min-over-max. A tungsten
 *      source pins R and G on one code while B never approaches the ceiling;
 *      requiring all three to agree throws that frame away and reports "no
 *      clamp" on a visibly clamped lamp. That exact mistake is recorded twice
 *      in lessons/hardware-log.md (2026-09-14, "the min-over-max bug was never
 *      fixed in clip_plateau.hpp"), where it cost a week of chasing the wrong
 *      layer. It is fixed here at birth.
 *
 *   2. IT MUST BE THE SECOND-BRIGHTEST CHANNEL, NOT THE SECOND-BRIGHTEST
 *      SAMPLE. A Bayer quad carries two greens, always near-equal, so second-
 *      over-max across four raw samples is ~1.0 for ANY bright quad and the
 *      test fires on everything. The two greens are folded into one channel
 *      before the comparison — which is why this counts per CFA channel rather
 *      than per raw sample.
 *
 *   3. DATA THAT REACHES FULL SCALE IS NOT CLAMPED. If the brightest channel
 *      tops out at the container's own maximum, the sensor did reach the top
 *      and the nominal WhiteLevel is already right — report nothing and let
 *      the caller keep it. This is what makes the detector inert on every
 *      healthy non-ClearHDR mode (both 10-bit 4K reference takes hit 1023
 *      exactly) rather than something that has to be gated per mode.
 *
 *   4. THE CEILING IS THE PLATEAU'S BODY, NOT ITS TOP. The clamp is not one
 *      code, it is a noisy band ~0.5% of full scale wide (F06: body 4034-4053,
 *      mode 4041, stray samples to 4054). Anchoring on the extreme top leaves
 *      the body below WhiteLevel and a residual cast with it — measured 4.1%
 *      against 0.0% for the mode. So the answer is the MODE within the band,
 *      and samples above it are clipped, which is correct: they are saturated.
 *
 * ── WHY MEASURED PER TAKE, AND ONLY ONCE ─────────────────────────────────────
 *
 * The clamp is an operating point, not a sensor constant — F06 and F13 both
 * stopped at 58352 while F07, a different session, stopped at 48172, and
 * lessons/hardware-log.md's 2026-09-14 entry has it moving with gain at full
 * res while sitting rock-still across three gain codes when binned. No model
 * predicts it, so it is read off the frame.
 *
 * It is read off ONE frame — the take's first — and held. That is not laziness:
 * WhiteLevel sets the normalisation denominator, so changing it mid-take shifts
 * the whole frame's exposure (65535 -> 58352 is 1.13x, ~0.18 EV). A detector
 * that re-decided per frame would put a visible brightness step in the middle
 * of a graded clip, which is a worse defect than the one it fixes and a new
 * one. Constant-per-take is the requirement; frame 0 is the only measurement
 * point the encoder has, because EncodeBuffer() is called only while recording
 * (cinepi_raw.cpp) and there is no pre-roll frame to look at.
 *
 * The cost of that choice is a take whose first frame is unclamped but which
 * blows later: it keeps the nominal WhiteLevel and those highlights stay
 * magenta. Accepted deliberately — it is the pre-existing behaviour, so the
 * change cannot make any take worse than it is today.
 */

#ifndef CINEPI_CLIP_CEILING_HPP
#define CINEPI_CLIP_CEILING_HPP

#include <cstddef>
#include <cstdint>
#include <vector>

/* Channel indices the CFA map resolves to. Two greens fold onto one. */
enum : uint8_t { kClipChanR = 0, kClipChanG = 1, kClipChanB = 2, kClipChanCount = 3 };

/*
 * Counts output codes per CFA channel over one frame and reports where the
 * data stops, if it stops short of the container.
 *
 * Usage per take: reset(), add_row() for a sample of the frame's rows, then
 * detect(). One instance is used by one encode thread for one frame; it is not
 * thread-safe and does not need to be.
 */
class ClipCeilingDetector
{
public:
    struct Result
    {
        /* True only when every rule below passed. When false the caller must
         * keep the nominal WhiteLevel — `why` says which rule refused. */
        bool     found   = false;
        /* The clamp's body, in the SAME domain as the codes that were added
         * (i.e. stored output codes). Mode of the band, per rule 4. */
        unsigned ceiling = 0;
        /* Highest populated code of the brightest channel — the band's top
         * edge, for the log line, not the answer. */
        unsigned peak    = 0;
        /* Second-brightest channel's top code, so a log line can show how
         * close the convergence test ran. */
        unsigned second  = 0;
        /* Samples inside the band, and samples seen overall. */
        unsigned long band_samples = 0;
        unsigned long sampled      = 0;
        /* Why found is false; nullptr when it is true. Static storage. */
        const char *why = nullptr;
    };

    /*
     * code_max: the largest code the container can hold, i.e. (1<<bits)-1 for
     *           the STORED depth. cfa: which channel each position of the
     *           2x2 Bayer quad carries, indexed [(y&1)*2 + (x&1)], using the
     *           kClipChan* values above (both green positions map to
     *           kClipChanG — see rule 2).
     */
    void reset(unsigned code_max, const uint8_t cfa[4])
    {
        code_max_ = code_max;
        for (int i = 0; i < 4; ++i)
            cfa_[i] = cfa[i];
        for (int c = 0; c < kClipChanCount; ++c)
            hist_[c].assign(static_cast<size_t>(code_max) + 1u, 0u);
        sampled_ = 0;
    }

    /*
     * Count one row. `y` is the row's index in the frame (its parity selects
     * the CFA phase, so rows may be sampled at any stride as long as the true
     * index is passed).
     *
     * `shift` is how many bits the packer drops on this row's way into the
     * file — 4 for dng_save()'s 16->12 branch, 6 for 16->10, 0 wherever the
     * row is already at the stored depth. It exists because the answer has to
     * be in the STORED code domain (that is what WhiteLevel is compared
     * against), while the row in hand is often still in the source container.
     * pack_row_16_to_12bit()/pack_row_16_to_10bit() drop exactly these bits by
     * plain truncation, and tests/clip_ceiling_test.cpp asserts this agrees
     * with them rather than assuming it.
     *
     * Codes above code_max are ignored rather than clamped: a sample that does
     * not fit the container it was packed into is a bug upstream, and folding
     * it onto the top code would fabricate exactly the full-scale evidence
     * rule 3 keys on.
     */
    void add_row(const uint16_t *row, size_t n, unsigned y, unsigned shift = 0)
    {
        const uint8_t *phase = &cfa_[(y & 1u) * 2u];
        for (size_t x = 0; x < n; ++x)
        {
            const unsigned code = static_cast<unsigned>(row[x]) >> shift;
            if (code > code_max_)
                continue;
            ++hist_[phase[x & 1u]][code];
            ++sampled_;
        }
    }

    /* Single-sample form, for tests and for callers with no contiguous row. */
    void add(unsigned code, uint8_t channel)
    {
        if (code > code_max_ || channel >= kClipChanCount)
            return;
        ++hist_[channel][code];
        ++sampled_;
    }

    Result detect() const
    {
        Result r;
        r.sampled = sampled_;

        /* Too little evidence to conclude anything. */
        if (sampled_ < kMinSamples || code_max_ == 0)
        {
            r.why = "too few samples";
            return r;
        }

        /* A code counts as populated only above a floor scaled to the sample
         * count, so a hot pixel or a stray cosmic ray cannot define the top. */
        const unsigned long populated = min_populated(sampled_);

        /* Each channel's top populated code. */
        unsigned top[kClipChanCount] = { 0, 0, 0 };
        for (int c = 0; c < kClipChanCount; ++c)
            top[c] = top_populated(hist_[c], populated);

        /* Sort the three descending — only the order matters, so three
         * compares beat pulling in <algorithm>. */
        unsigned m0 = top[0], m1 = top[1], m2 = top[2];
        if (m1 > m0) { unsigned t = m0; m0 = m1; m1 = t; }
        if (m2 > m0) { unsigned t = m0; m0 = m2; m2 = t; }
        if (m2 > m1) { unsigned t = m1; m1 = m2; m2 = t; }
        r.peak   = m0;
        r.second = m1;

        /* The band: the clamp's noisy body hanging below the top edge. */
        const unsigned width = band_width(code_max_);

        /* Rule 3 — the data reached the container's top, so it was never
         * clamped and the nominal WhiteLevel already describes it. */
        if (m0 + width >= code_max_)
        {
            r.why = "data reaches full scale";
            return r;
        }

        /* A ceiling implausibly far down is more likely a misread than a
         * clamp, and adopting one would clip most of the frame to white.
         * The lowest clamp on record is ~0.55 of the container (binned
         * ClearHDR); a third of full scale leaves that room and still
         * refuses nonsense. */
        if (m0 < code_max_ / 3u)
        {
            r.why = "candidate ceiling implausibly low";
            return r;
        }

        /* Rule 1 — second-over-max on the CHANNEL tops. At a clamp the
         * channels pin together; a subject reaching this level alone does
         * not, because the sensor's own channel imbalance (~1.7x green over
         * red for a neutral) keeps them apart. */
        if (m1 + width < m0)
        {
            r.why = "channels did not converge";
            return r;
        }

        /* The body must be a real area, not a handful of samples. */
        const unsigned low = m0 - width;
        unsigned long band = 0;
        for (int c = 0; c < kClipChanCount; ++c)
            for (unsigned code = low; code <= m0; ++code)
                band += hist_[c][code];
        r.band_samples = band;
        if (band < sampled_ / kMinBandDivisor)
        {
            r.why = "clamp body too small";
            return r;
        }

        /* Nothing meaningful may sit above the top edge: a clamp terminates
         * the histogram, a bright subject trails off past it. */
        unsigned long above = 0;
        for (int c = 0; c < kClipChanCount; ++c)
            for (unsigned code = m0 + 1u; code <= code_max_; ++code)
                above += hist_[c][code];
        if (above > sampled_ / kMaxAboveDivisor)
        {
            r.why = "histogram does not terminate at the peak";
            return r;
        }

        /* Rule 4 — the answer is the body's mode, not its top edge. A tie
         * keeps the LOWEST code, which is the safe direction: a ceiling one
         * code low clips a saturated sample that was already saturated, one
         * code high leaves the body under WhiteLevel and the cast with it. */
        unsigned long best = 0;
        unsigned mode = m0;
        for (unsigned code = low; code <= m0; ++code)
        {
            unsigned long n = 0;
            for (int c = 0; c < kClipChanCount; ++c)
                n += hist_[c][code];
            if (n > best) { best = n; mode = code; }
        }

        r.ceiling = mode;
        r.found   = true;
        return r;
    }

    /* Exposed so the test can assert against the same numbers the code uses
     * rather than transcribing them. */
    static unsigned band_width(unsigned code_max)
    {
        const unsigned w = code_max / kBandDivisor;
        return w ? w : 1u;
    }
    static unsigned long min_populated(unsigned long sampled)
    {
        const unsigned long n = sampled / kPopulatedDivisor;
        return n > kMinPopulatedFloor ? n : kMinPopulatedFloor;
    }

private:
    static unsigned top_populated(const std::vector<uint32_t> &h, unsigned long floor)
    {
        for (size_t code = h.size(); code-- > 0;)
            if (h[code] >= floor)
                return static_cast<unsigned>(code);
        return 0;
    }

    /* Band width as a fraction of full scale: the measured body is ~0.5%
     * wide (F06: 4034-4053 of 4095), so 1% covers it with margin. */
    static constexpr unsigned      kBandDivisor        = 100;
    /* A code is "populated" at >= sampled/20000, floored at 16 so a small
     * test frame still behaves. */
    static constexpr unsigned long kPopulatedDivisor   = 20000;
    static constexpr unsigned long kMinPopulatedFloor  = 16;
    /* The body must cover >= 0.25% of the samples. */
    static constexpr unsigned long kMinBandDivisor     = 400;
    /* Above the top edge, < 0.01% of the samples may remain. */
    static constexpr unsigned long kMaxAboveDivisor    = 10000;
    /* Below this many samples the fractions above stop meaning anything. */
    static constexpr unsigned long kMinSamples         = 4096;

    std::vector<uint32_t> hist_[kClipChanCount];
    uint8_t               cfa_[4] = { kClipChanR, kClipChanG, kClipChanG, kClipChanB };
    unsigned              code_max_ = 0;
    unsigned long         sampled_  = 0;
};

#endif /* CINEPI_CLIP_CEILING_HPP */
