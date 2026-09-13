/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * clip_plateau.hpp - find the ClearHDR merge-clamp floor from the raw itself.
 *
 * Pure counting: no libcamera, no app headers, integer arithmetic only, so
 * tests/clip_plateau_test.cpp exercises the *same* code that ships.
 *
 * WHAT THIS IS FOR. The imx585's HG/LG merge clamps digitally, and the four
 * Bayer samples of a blown quad CONVERGE as it does -- a physical signature no
 * neutral or coloured subject produces by accident (ccmp_preview.hpp's
 * desaturateHighlight() comment has the full argument). That signature is
 * cheap to test per quad: take the min and max of its four samples: dark
 * quads are trivially "equal" and mean nothing, so they are excluded; a
 * genuinely converged, bright quad is the clamp.
 *
 * WHY MEASURED, NOT TABULATED. 12-bit ClearHDR gets away with a fixed anchor
 * per binning (CcmpAnchor::clip_code in ccmp_lut.hpp) because the compander
 * output code for a given physical clamp is deterministic. 16-bit is linear
 * -- there is no compander to fix the code -- and the clamp itself is not a
 * sensor constant: the SAME take, same mode, blend, gain adder, shutter and
 * frame rate, plateaued at code 54100 at analogue gain code 71 and at 48600
 * at code 80, thirteen minutes apart. A table keyed on binning has nothing to
 * key 16-bit on; gain changes live over Redis and nothing here has a model of
 * the sensor's merge as a function of it. The frame carries the answer every
 * time, so this reads it off the frame every time, cheaply enough to run
 * every frame at full lores resolution.
 *
 * THE HISTOGRAM, AND WHY A PERCENTILE RATHER THAN A MINIMUM. A blown area is
 * not one code, it is a soft plateau ~0.8% of full scale wide (measured: body
 * 54000-54500 at code 71). A handful of outlier converged quads well below
 * the real plateau -- noise, a stray reflection -- would drag a plain minimum
 * down and anchor too low; the 1st percentile of converged quad-max is robust
 * to a few of those while still tracking the true floor, which is the same
 * reasoning the 12-bit fix's anchor margin was measured with (see
 * ccmp_lut.hpp's kT1Effective comment).
 *
 * Integer arithmetic throughout: this runs per Bayer quad of the lores frame,
 * every frame, on the post-processing thread.
 */

#ifndef CINEPI_CLIP_PLATEAU_HPP
#define CINEPI_CLIP_PLATEAU_HPP

/*
 * Detects a ClearHDR merge-clamp plateau from per-quad (min, max) samples.
 *
 * Usage per frame: reset(), add() every sampled quad, then detect(). Not
 * thread-safe against concurrent add()/detect() -- the caller (one
 * post-processing stage) serialises all three under its own mutex, the same
 * way CcmpPreviewRenderer does.
 */
class ClipPlateauDetector
{
public:
    struct Result
    {
        /* Lower edge of the bin holding the 1st percentile of converged
         * quad-max -- the floor of the plateau's body, not its peak. */
        unsigned floor = 0;
        /* floor, backed off by the 12-bit fix's measured margin (1.5%) so the
         * ramp clears the body rather than ending exactly at its lower rim. */
        unsigned anchor = 0;
        /* How many sampled quads passed both the dark and convergence tests
         * -- i.e. how many the histogram and the percentile are built from.
         * Filled in whether or not detect() found a plateau. */
        unsigned long converged = 0;
        /* Every quad add() saw this call, converged or not. */
        unsigned long sampled = 0;
        /* Top code of the highest non-empty bin -- the frame's actual peak
         * among converged quads, for the log line, not used by the anchor. */
        unsigned peak_code = 0;
    };

    /* At least this many converged quads before a floor means anything --
     * below it a percentile is noise, not a plateau. */
    static constexpr unsigned long kMinConverged = 64;
    static constexpr unsigned kBinCount = 1024;

    /* code_bits: 12 or 16, the sensor code width add()'s mn/mx are in (i.e.
     * already shifted past raw_shift). Anything else is refused -- a shift
     * this class was not told about is a silent wrong-domain read, the same
     * reasoning ccmp_preview.hpp's configure() range-checks raw_shift for.
     * Resets all state; returns false and leaves nothing configured on a bad
     * code_bits. */
    bool configure(unsigned code_bits)
    {
        if (code_bits != 12 && code_bits != 16)
            return false;

        /* 1024 bins over `code_bits` significant bits: 64 codes/bin at
         * 16-bit, 4 at 12-bit. full_scale is the code COUNT (4096 / 65536),
         * not the top code (4095 / 65535), so full_scale/4 and full_scale/bin
         * width both land on exact, testable boundaries. */
        shift_ = code_bits - 10;
        full_scale_ = 1u << code_bits;
        reset();
        return true;
    }

    void reset()
    {
        for (unsigned b = 0; b < kBinCount; ++b)
            bins_[b] = 0;
        converged_ = 0;
        sampled_ = 0;
    }

    /* mn, mx: min and max of one Bayer quad's four samples, in sensor codes
     * (post raw_shift). Counts every call in sampled_; a quad that is dark
     * (trivially equal, means nothing) or not converged (a genuinely bright,
     * un-clamped subject: a neutral patch sits at ~0.55 under the shipping
     * gains, nowhere near this) is excluded from the histogram and from
     * converged_. */
    void add(unsigned mn, unsigned mx)
    {
        ++sampled_;

        if (mx < (full_scale_ >> 2))
            return;
        if (mn * 10 < mx * 9)
            return;

        ++converged_;
        unsigned bin = mx >> shift_;
        if (bin >= kBinCount)
            bin = kBinCount - 1; /* mx is caller-supplied; clip rather than overrun on a bad code_bits/mx pairing */
        ++bins_[bin];
    }

    /* False (converged_ < kMinConverged) leaves floor/anchor/peak_code at 0
     * in `out` -- converged/sampled are always filled, even on failure, so a
     * caller logging every frame has a real "0 converged quads" to print
     * instead of stale numbers from the last blown frame. */
    bool detect(Result &out) const
    {
        out.converged = converged_;
        out.sampled = sampled_;

        if (converged_ < kMinConverged)
            return false;

        /* Smallest bin b with cum[b] >= 1% of converged_, done as
         * cum[b]*100 >= converged_ so the percentile needs no division and
         * therefore no rounding decision -- see the file comment on why the
         * 1st percentile, not the minimum. */
        unsigned long cum = 0;
        unsigned floor_bin = 0;
        bool floor_found = false;
        unsigned top_bin = 0;

        for (unsigned b = 0; b < kBinCount; ++b)
        {
            if (bins_[b] == 0)
                continue;
            top_bin = b;
            cum += bins_[b];
            if (!floor_found && cum * 100 >= converged_)
            {
                floor_bin = b;
                floor_found = true;
            }
        }
        /* converged_ >= kMinConverged > 0 guarantees at least one non-empty
         * bin, so cum reaches converged_ (>= 1% of itself) by the last one;
         * floor_found cannot still be false here. */

        const unsigned floor = floor_bin << shift_;
        out.floor = floor;
        /* anchor = floor - 1.5%, the 12-bit fix's measured margin (p1 2948 ->
         * anchor 2900): back off the floor rather than the peak, because the
         * ramp has to clear the plateau's BODY -- see ccmp_lut.hpp's
         * kT1Effective comment for the hardware history of getting this
         * backwards. Integer division: floor*15/1000 truncates by under one
         * code versus the real 1.5%, well inside the one-bin tolerance this
         * detector is checked to (see evidence/detector_sim.py). */
        out.anchor = floor - (floor * 15) / 1000;
        out.peak_code = ((top_bin + 1) << shift_) - 1;
        return true;
    }

private:
    unsigned shift_ = 6;
    unsigned full_scale_ = 1u << 16;
    unsigned long bins_[kBinCount] = {};
    unsigned long converged_ = 0;
    unsigned long sampled_ = 0;
};

#endif /* CINEPI_CLIP_PLATEAU_HPP */
