/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * clip_convergence.hpp - decide that a quad is inside the ClearHDR merge clamp
 * from the CHANNELS THEMSELVES, not from the level they sit at.
 *
 * WHY THIS EXISTS, AND WHY THE LEVEL CANNOT DO IT. The clamp was chased with a
 * per-binning anchor code three times (2900/2582, then 2582 -> 2344) and it was
 * wrong again every time, because the code the clamp lands on is not a property
 * of the sensor: it moves with analogue gain, with the HG/LG blend, and with the
 * scene. Measured on this rig, same mode and settings: 54100 at gain code 71 and
 * 48600 at code 80; in 12-bit HD, ~2298 where the shipped anchor said 2582.
 *
 * The measurement that ended it — three takes, developed and probed per region,
 * second-brightest channel over brightest, in linear above black:
 *
 *     region                              2nd/max   level   verdict
 *     pink lamp core, 12-bit HD           0.996     0.56    clamped
 *     orange surround of that same lamp   0.912     0.35    real colour
 *     pink sky, 16-bit 4K                 0.998     0.83    clamped
 *     GENUINELY WHITE subject, 16-bit     0.659     0.74    real white
 *
 * Row four is the whole argument. That white sits at the SAME level as the
 * clamp on row three, so no anchor, however well measured, can separate them —
 * but the ratio does, 0.659 against 0.998.
 *
 * WHY THE RATIO MEANS CLAMPED. Under the shipping AWB gains a neutral subject
 * arrives at the sensor with its channels FAR apart: R = G/1.8 and B = G/1.7,
 * i.e. a second-over-max of about 0.59. Channels only converge when something
 * downstream pins them together, and in ClearHDR that something is the HG/LG
 * merge clamping digitally. So convergence at a high level IS the clamp, and it
 * stays the clamp whatever code it happens to land on today.
 *
 * SECOND-OVER-MAX, NOT MIN-OVER-MAX. The first version of this test asked for
 * all three channels to agree and it missed the 12-bit HD case completely: a
 * tungsten lamp pins R and G at one code while B, which never got near the
 * ceiling, sits at 0.79. The pixel is still magenta — the gains push the two
 * pinned channels apart from a green that cannot move — so the test has to fire
 * on the TOP TWO agreeing and ignore the third.
 *
 * THE LEVEL GATE IS STILL NEEDED, for the opposite end: at black every channel
 * agrees with every other one, so a dark frame is trivially "converged". The
 * gate is what keeps this off the shadows, not what identifies the clamp.
 *
 * ONE PIXEL IS NEVER A CLAMP. Even with the gate raised, a per-pixel rule on
 * noisy data fires on a few hundred scattered pixels per frame — measured at
 * 245 and 498 on the two 16-bit takes — and scattered single pixels driven to
 * white are exactly as objectionable as the pink they replaced. A clamp is a
 * REGION: it covers a lamp, a window, a sky. So the callers take the WEAKEST
 * blend across each 2x2 output block and apply that to all four pixels, which
 * costs nothing (both render loops already walk in 2x2 blocks for YUV420
 * chroma) and takes the isolated count to zero on all three takes, for about
 * 1% of coverage at the blob edge. Use clip_convergence_block() rather than
 * applying the per-pixel value directly.
 *
 * This is a MONITORING correction. It has no counterpart in the recorded DNG
 * and must not grow one: the file's job is to carry the clamped channels as
 * they were, so a grade can decide what to do with them.
 */

#ifndef CINEPI_CLIP_CONVERGENCE_HPP
#define CINEPI_CLIP_CONVERGENCE_HPP

#include <algorithm>

struct ClipConvergence
{
    /* Ramp on the top-two agreement. Below ratio_lo nothing happens, at
     * ratio_hi the pixel is driven fully neutral. The measured separation is
     * 0.912/0.939-p95 for real colour against 0.992-p5 for the clamp, so the
     * ramp sits in the gap rather than on either population. */
    float ratio_lo = 0.98f;
    float ratio_hi = 0.997f;

    /* Ramp on the brightest channel, normalised linear above black. Keeps the
     * rule off the shadows, where channels agree for a different reason. The
     * dimmest clamp measured is 0.56 of full scale, so this sits well under
     * both measured clamps while keeping the rule out of the mid-tones, where
     * an ordinary colour can have its top two channels close together and a
     * single noisy pixel then fires on its own. The first shipped values were
     * 0.15/0.25 and that is exactly what went wrong: white speckle in every
     * HDR mode. */
    float level_lo = 0.40f;
    float level_hi = 0.50f;

    bool enabled = true;
};

/* 0 = leave the pixel alone, 1 = drive it fully neutral. r/g/b are linear above
 * black, normalised to full scale — the domain the preview renderer already
 * works in after its decompand table. */
inline float clip_convergence_blend(float r, float g, float b, const ClipConvergence &c)
{
    if (!c.enabled)
        return 0.f;

    /* max and second-max of three, without sorting all of them. */
    float mx = r, second = g;
    if (second > mx)
        std::swap(mx, second);
    if (b > mx)
    {
        second = mx;
        mx = b;
    }
    else if (b > second)
        second = b;

    if (!(mx > 0.f) || mx <= c.level_lo)
        return 0.f;

    const float ratio = second / mx;
    if (ratio <= c.ratio_lo)
        return 0.f;

    const float r_span = c.ratio_hi - c.ratio_lo;
    const float l_span = c.level_hi - c.level_lo;
    const float s_ratio = r_span > 1e-6f ? std::min(1.f, (ratio - c.ratio_lo) / r_span) : 1.f;
    const float s_level = l_span > 1e-6f ? std::min(1.f, (mx - c.level_lo) / l_span) : 1.f;
    return s_ratio * s_level;
}

/* The weakest of a 2x2 output block — see "ONE PIXEL IS NEVER A CLAMP". Applied
 * to the COMBINED blend (anchor and convergence both), because the anchor
 * speckles for the same reason whenever the clamp code straddles it. */
inline float clip_convergence_block(float s0, float s1, float s2, float s3)
{
    return std::min(std::min(s0, s1), std::min(s2, s3));
}

#endif /* CINEPI_CLIP_CONVERGENCE_HPP */
