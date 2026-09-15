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
 * DNGs, five ClearHDR takes stopped between 73.5% and 89.3% of their declared
 * WhiteLevel, while both 10-bit reference takes reached theirs exactly. The DNG
 * nevertheless declares WhiteLevel at the curve's nominal full scale — correct
 * as a statement about the transfer curve, wrong as a statement about the data.
 *
 * The consequence is not subtle and it is not a preview artefact. No pixel ever
 * reaches the declared WhiteLevel, so every converter concludes the frame
 * contains no clipped highlight and skips highlight reconstruction. A blown
 * highlight then renders MAGENTA, for a reason worth writing down because it is
 * entirely mechanical:
 *
 *     AsShotNeutral is ~(0.59, 1, 0.59), so white balance multiplies R and B
 *     by ~1.7 and GREEN BY EXACTLY 1.0. In a blown area all three channels sit
 *     at the clamp — say 0.88 of the declared white. R and B are scaled past
 *     1.0 and clip; green is scaled to 0.88 and stays there. Red and blue
 *     pinned, green 12% short: that is magenta, by construction, and no amount
 *     of grading removes it because the file says nothing is clipped.
 *
 * Declaring the measured clamp as WhiteLevel puts the three channels back on
 * the same ceiling and the highlight renders neutral. Verified both ways on
 * recorded frames by patching only tag 0xC61D in a byte copy:
 *   - F06 (12-bit ClearHDR + log): LibRaw's cast over the blown area 12.0% ->
 *     0.0%; direct render sRGB (255,157,249) -> (255,245,249).
 *   - F03 (16-bit ClearHDR, log off): cast 13.0% -> 0.6%; a magenta blob
 *     becomes a white lamp with its warm rim intact.
 *
 * WHAT THIS CANNOT DO. It fixes the COLOUR of a clamped highlight, not its
 * DETAIL. The merge stopped; there is no roll-off in the signal to recover. A
 * featureless pink blob becomes a featureless white blob, which is what a blown
 * highlight is supposed to look like. lessons/hardware-log.md's 2026-09-14 "the
 * merge ceiling is hard at ~0.55 of the container" entry is the standing
 * statement of that limit, and nothing here changes it.
 *
 * ── IT COUNTS LINEARISED VALUES, NOT STORED CODES. THIS IS THE WHOLE DESIGN ──
 *
 * The first version of this file counted STORED CODES and expressed its
 * thresholds as fractions of the code container. It worked on 12-bit ClearHDR
 * and REFUSED on 16-bit ClearHDR, on a frame whose clamp is as obvious to the
 * eye as any in the set. The reason is worth stating plainly because it is the
 * kind of mistake that looks like a tuning problem and is not:
 *
 *     A clamp is a fixed spread in the LIGHT. How wide that spread looks in
 *     CODE space depends entirely on the transfer curve. Through the log curve
 *     the 16-bit clamp body occupies ~20 codes of 4096; in the linear 16-bit
 *     container the same physical clamp occupies ~1000 codes of 65536 — fifty
 *     times wider, with a long thin tail above it. Any threshold written as
 *     "N codes" or "1% of full scale" is therefore a statement about ONE
 *     encoding, and silently wrong in the other. Measured: the code-space rule
 *     allowed 0.01% of samples above the peak and the 16-bit tail was 0.08%.
 *
 * So the table (the DNG LinearizationTable, when there is one) is applied HERE,
 * on the way into the histogram, and every rule below is relative:
 * convergence within 3% of the peak, a body +/-5% wide, a tail measured against
 * the BODY rather than against the frame. In the linear domain those fractions
 * mean the same thing whatever curve produced the file, and the same constants
 * accept all five recorded ClearHDR takes across both encodings while still
 * refusing both SDR ones.
 *
 * It also means Result::ceiling comes out already in the tag's own domain: a
 * reader applies the LinearizationTable before reading the level tags, so
 * WhiteLevel describes the table's OUTPUT, which is exactly what was counted.
 * The caller writes it straight to 0xC61D with no conversion.
 *
 * ── THREE MORE RULES ARE LOAD-BEARING ────────────────────────────────────────
 *
 *   1. CHANNEL CONVERGENCE IS THE SIGNATURE, AND IT IS SECOND-OVER-MAX.
 *      A large bright area alone does not mean a clamp — it could be a wall.
 *      What distinguishes a clamp is that the colour channels PIN TOGETHER: a
 *      neutral subject's raw green sits ~1.7x its raw red (that ratio IS
 *      AsShotNeutral), so channels landing on the SAME value is not something a
 *      subject does by accident.
 *
 *      But the test must be SECOND-over-max, not min-over-max. A tungsten
 *      source pins R and G on the clamp while B never approaches it; requiring
 *      all three to agree throws that frame away and reports "no clamp" on a
 *      visibly clamped lamp. That exact mistake is recorded twice in
 *      lessons/hardware-log.md (2026-09-14, "the min-over-max bug was never
 *      fixed in clip_plateau.hpp"), where it cost a week of debugging at the
 *      wrong layer. It is fixed here at birth and guarded by a test.
 *
 *   2. IT MUST BE THE SECOND-BRIGHTEST CHANNEL, NOT THE SECOND-BRIGHTEST
 *      SAMPLE. A Bayer quad carries two greens, always near-equal, so
 *      second-over-max across four raw samples is ~1.0 for ANY bright quad and
 *      the test fires on everything. The two greens fold into one channel.
 *
 *   3. DATA THAT REACHES FULL SCALE IS NOT CLAMPED. If the brightest channel
 *      tops out at the declared white, the sensor did reach it and the nominal
 *      WhiteLevel is already right — report nothing. This is what keeps the
 *      detector inert on every healthy non-ClearHDR mode (both 10-bit 4K
 *      reference takes hit 1023 exactly) rather than something that has to be
 *      gated per mode.
 *
 * ── WHY MEASURED PER TAKE, AND ONLY ONCE ─────────────────────────────────────
 *
 * The clamp is an operating point, not a sensor constant — two takes minutes
 * apart both stopped at 58352 while a third from another session stopped at
 * 48172, and lessons/hardware-log.md's 2026-09-14 entry has it moving with gain
 * at full res while sitting rock-still across three gain codes when binned. No
 * model predicts it, so it is read off the frame.
 *
 * It is read off ONE frame — the take's first — and held. That is not laziness:
 * WhiteLevel sets the normalisation denominator, so changing it mid-take shifts
 * the whole frame's exposure (65535 -> 58352 is 1.13x, ~0.18 EV) and puts a
 * visible brightness step in the middle of a graded clip, which is a worse
 * defect than the one it fixes and a new one. Constant-per-take is the
 * requirement; frame 0 is the only measurement point the encoder has, because
 * EncodeBuffer() is called only while recording (cinepi_raw.cpp).
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
 * Counts LINEARISED sample values per CFA channel over one frame and reports
 * where the data stops, if it stops short of the declared white.
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
        /* True only when every rule passed. When false the caller must keep the
         * nominal WhiteLevel — `why` says which rule refused. */
        bool     found   = false;
        /* The clamp's body, ALREADY IN THE TAG'S DOMAIN (the LinearizationTable's
         * output, or the stored code when there is no table). Write it straight
         * to 0xC61D. */
        unsigned ceiling = 0;
        /* Brightest channel's peak, and the second brightest's, so a log line
         * can show how close the convergence test ran. */
        unsigned peak    = 0;
        unsigned second  = 0;
        /* Samples in the clamp body, above it, and seen overall. */
        unsigned long body_samples = 0;
        unsigned long tail_samples = 0;
        unsigned long sampled      = 0;
        /* Why found is false; nullptr when it is true. Static storage. */
        const char *why = nullptr;
    };

    /*
     * white_level / black_level : the nominal tag values for this take, in the
     *          LINEARISED domain — i.e. exactly what dng_save() would otherwise
     *          write. The histogram spans [0, white_level].
     * cfa    : which channel each position of the 2x2 Bayer quad carries,
     *          indexed [(y&1)*2 + (x&1)], using the kClipChan* values (BOTH
     *          green positions map to kClipChanG — see rule 2).
     * lut / lut_size : the live DNG LinearizationTable, or nullptr when the
     *          stored codes are already linear. Applied on the way into the
     *          histogram — see "IT COUNTS LINEARISED VALUES" above, which is
     *          the single most important thing about this file.
     */
    void reset(unsigned white_level, unsigned black_level, const uint8_t cfa[4],
               const uint16_t *lut = nullptr, size_t lut_size = 0)
    {
        white_ = white_level;
        black_ = black_level < white_level ? black_level : 0u;
        lut_      = lut;
        lut_size_ = lut ? lut_size : 0u;
        for (int i = 0; i < 4; ++i)
            cfa_[i] = cfa[i];
        for (int c = 0; c < kClipChanCount; ++c)
            hist_[c].assign(static_cast<size_t>(white_) + 1u, 0u);
        sampled_ = 0;
    }

    /*
     * Count one row of STORED codes. `y` is the row's index in the frame (its
     * parity selects the CFA phase, so rows may be sampled at any stride as
     * long as the true index is passed).
     *
     * `shift` is how many bits the packer drops on this row's way into the file
     * — 4 for dng_save()'s 16->12 branch, 6 for 16->10, 0 wherever the row is
     * already at the stored depth. pack_row_16_to_12bit()/pack_row_16_to_10bit()
     * drop exactly these bits by plain truncation, and
     * tests/clip_ceiling_test.cpp asserts this agrees with them rather than
     * assuming it.
     *
     * Values landing above white_level after linearisation are ignored rather
     * than clamped: folding them onto the top would fabricate exactly the
     * full-scale evidence rule 3 keys on.
     */
    void add_row(const uint16_t *row, size_t n, unsigned y, unsigned shift = 0)
    {
        const uint8_t *phase = &cfa_[(y & 1u) * 2u];
        for (size_t x = 0; x < n; ++x)
            bump(static_cast<unsigned>(row[x]) >> shift, phase[x & 1u]);
    }

    /* Single-sample form, for tests and for callers with no contiguous row. */
    void add(unsigned code, uint8_t channel)
    {
        if (channel < kClipChanCount)
            bump(code, channel);
    }

    Result detect() const
    {
        Result r;
        r.sampled = sampled_;

        if (sampled_ < kMinSamples || white_ <= black_)
        {
            r.why = "too few samples";
            return r;
        }

        /* A value counts as populated only above a floor scaled to the sample
         * count, so a hot pixel cannot define the top. */
        const unsigned long populated = min_populated(sampled_);

        /* The lowest value a clamp may plausibly sit at. The lowest on record
         * is ~0.55 of the container (binned ClearHDR); a third of the range
         * above black leaves that room and still refuses nonsense. */
        const unsigned floor_val = black_ + (white_ - black_) / 3u;

        /* Per channel: the top populated value, and the peak of the
         * distribution above floor_val — the candidate clamp body. The PEAK is
         * what the rules compare, not the top: the top is in the tail, and in a
         * linear container that tail is long. */
        unsigned top[kClipChanCount] = { 0, 0, 0 };
        unsigned peak[kClipChanCount] = { 0, 0, 0 };
        unsigned highest_top = 0;
        for (int c = 0; c < kClipChanCount; ++c)
        {
            top[c]  = top_populated(hist_[c], populated);
            peak[c] = peak_above(hist_[c], floor_val, top[c]);
            if (top[c] > highest_top)
                highest_top = top[c];
        }

        /* Rule 3 — the data reached the declared white, so it was never clamped
         * short of it and the nominal WhiteLevel already describes it. */
        if (static_cast<unsigned long>(highest_top) * 100ul >=
            static_cast<unsigned long>(white_) * kFullScalePct)
        {
            r.why = "data reaches full scale";
            return r;
        }

        /* Order the three channel peaks descending. */
        unsigned m0 = peak[0], m1 = peak[1], m2 = peak[2];
        if (m1 > m0) { unsigned t = m0; m0 = m1; m1 = t; }
        if (m2 > m0) { unsigned t = m0; m0 = m2; m2 = t; }
        if (m2 > m1) { unsigned t = m1; m1 = m2; m2 = t; }
        r.peak   = m0;
        r.second = m1;

        if (m0 <= floor_val)
        {
            r.why = "candidate ceiling implausibly low";
            return r;
        }

        /* Rule 1 — second-over-max on the channel PEAKS, as a fraction of the
         * brightest. At a clamp the channels pin together; a subject reaching
         * this level alone does not, because the sensor's own channel imbalance
         * (~1.7x green over red for a neutral) keeps them apart. */
        if (static_cast<unsigned long>(m1) * 100ul <
            static_cast<unsigned long>(m0) * kConvergePct)
        {
            r.why = "channels did not converge";
            return r;
        }

        /* The body: +/-kBandPct around the peak. Relative, not a code count —
         * see the header's central point. */
        const unsigned long lo_l = static_cast<unsigned long>(m0) * (100ul - kBandPct) / 100ul;
        const unsigned long hi_l = static_cast<unsigned long>(m0) * (100ul + kBandPct) / 100ul;
        const unsigned lo = static_cast<unsigned>(lo_l);
        const unsigned hi = hi_l > white_ ? white_ : static_cast<unsigned>(hi_l);

        unsigned long body = 0, tail = 0;
        for (int c = 0; c < kClipChanCount; ++c)
        {
            const std::vector<uint32_t> &h = hist_[c];
            for (unsigned v = lo; v <= hi; ++v)
                body += h[v];
            for (unsigned v = hi + 1u; v <= white_; ++v)
                tail += h[v];
        }
        r.body_samples = body;
        r.tail_samples = tail;

        if (body < sampled_ / kMinBodyDivisor)
        {
            r.why = "clamp body too small";
            return r;
        }

        /* A clamp TERMINATES the distribution; a bright subject trails past it.
         * Measured against the BODY, not against the frame: the body is the
         * thing that has to end, and a frame-relative budget is another
         * disguised statement about the encoding. */
        if (tail > body / kTailDivisor)
        {
            r.why = "histogram does not terminate at the peak";
            return r;
        }

        /* The answer is the body's mode. A tie keeps the LOWEST value, which is
         * the safe direction: one step low clips a sample that was already
         * saturated, one step high leaves the body under WhiteLevel and the
         * cast with it. */
        unsigned long best = 0;
        unsigned mode = m0;
        for (unsigned v = lo; v <= hi; ++v)
        {
            unsigned long n = 0;
            for (int c = 0; c < kClipChanCount; ++c)
                n += hist_[c][v];
            if (n > best) { best = n; mode = v; }
        }

        r.ceiling = mode;
        r.found   = true;
        return r;
    }

    /* Exposed so the test asserts against the same numbers the code uses
     * rather than transcribing them. */
    static unsigned long min_populated(unsigned long sampled)
    {
        const unsigned long n = sampled / kPopulatedDivisor;
        return n > kMinPopulatedFloor ? n : kMinPopulatedFloor;
    }

private:
    void bump(unsigned code, uint8_t channel)
    {
        const unsigned v = (lut_ && code < lut_size_) ? lut_[code] : code;
        if (v > white_)
            return;
        ++hist_[channel][v];
        ++sampled_;
    }

    static unsigned top_populated(const std::vector<uint32_t> &h, unsigned long floor)
    {
        for (size_t v = h.size(); v-- > 0;)
            if (h[v] >= floor)
                return static_cast<unsigned>(v);
        return 0;
    }

    static unsigned peak_above(const std::vector<uint32_t> &h, unsigned from, unsigned to)
    {
        unsigned long best = 0;
        unsigned at = 0;
        for (unsigned v = from; v <= to && v < h.size(); ++v)
            if (h[v] > best) { best = h[v]; at = v; }
        return at;
    }

    /* The second-brightest channel peak must be within this percentage of the
     * brightest for the quad population to count as converged. */
    static constexpr unsigned long kConvergePct       = 97;
    /* Half-width of the clamp body, as a percentage of the peak. Measured
     * bodies: ~1.7% (16-bit linear) and ~4.3% (12-bit log, linearised). */
    static constexpr unsigned long kBandPct           = 5;
    /* At or above this percentage of the declared white, the data reached it. */
    static constexpr unsigned long kFullScalePct      = 99;
    /* A value is "populated" at >= sampled/20000, floored at 16 so a small test
     * frame still behaves. */
    static constexpr unsigned long kPopulatedDivisor  = 20000;
    static constexpr unsigned long kMinPopulatedFloor = 16;
    /* The body must cover >= 0.25% of the samples. */
    static constexpr unsigned long kMinBodyDivisor    = 400;
    /* Above the body, at most body/20 (5% of the body) may remain. */
    static constexpr unsigned long kTailDivisor       = 20;
    /* Below this many samples the fractions above stop meaning anything. */
    static constexpr unsigned long kMinSamples        = 4096;

    std::vector<uint32_t> hist_[kClipChanCount];
    uint8_t               cfa_[4] = { kClipChanR, kClipChanG, kClipChanG, kClipChanB };
    const uint16_t       *lut_      = nullptr;
    size_t                lut_size_ = 0;
    unsigned              white_    = 0;
    unsigned              black_    = 0;
    unsigned long         sampled_  = 0;
};

#endif /* CINEPI_CLIP_CEILING_HPP */
