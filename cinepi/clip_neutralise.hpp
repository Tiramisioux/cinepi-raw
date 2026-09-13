/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * clip_neutralise.hpp - neutralise the ClearHDR merge-clamp zone in place,
 * over an already-rendered lores YUV frame.
 *
 * Pure pixel math: no libcamera, no Redis, no app headers — only the standard
 * library, ccmp_preview.hpp (the geometry and the nearest-quad mapping) and
 * clip_plateau.hpp (the detector), so tests/ccmp_preview_test.cpp's
 * run_neutraliser() exercises the *same* code that ships.
 *
 * WHAT THIS IS FOR. 16-bit ClearHDR is delivered linear, so unlike 12-bit
 * there is no compander to fix and the ISP's own lores render is correct
 * everywhere except one zone: the imx585's HG/LG merge still clamps
 * digitally, the four Bayer samples of a blown quad still converge on one
 * code, and the shipping white-balance gains still turn that equal-code
 * plateau pink the same way they turn 12-bit's magenta (ccmp_preview.hpp's
 * desaturateHighlight() comment has the full argument for why an equal-code
 * quad cannot render neutral under real gains). Re-rendering the whole frame
 * the way the 12-bit path does would throw away a correct ISP image — denoise,
 * sharpening, the tuned CCM and gamma — to fix one zone (see PLAN.md §3.4 for
 * why that was rejected). So this works the other way around: keep the ISP's
 * YUV, and push ONLY the pixels the raw says are clamped back to neutral,
 * in place.
 *
 * THE TRIGGER IS STILL RAW SATURATION, READ FROM THE SAME QUAD THE PIXEL CAME
 * FROM. Testing the rendered YUV for "bright" cannot tell a clipped highlight
 * from a saturated colour that never clipped — the same reasoning
 * desaturateHighlight() is built on, just applied to a pixel this class did
 * not render itself. ccmp_preview_src_row/col() (ccmp_preview.hpp) give the
 * identical nearest-quad footprint the ISP's own downscale uses, to first
 * approximation, so the raw sample read here is the one that produced the
 * YUV pixel being corrected.
 *
 * THE ANCHOR IS NOT BAKED IN HERE. Where the clamp code sits is not this
 * class's problem — see clip_plateau.hpp for why it moves with gain and has
 * to be measured per frame. setAnchor() takes whatever the caller decided
 * (measured, overridden, or 0 for "nothing is clipped right now"); this class
 * only knows how to ramp a pixel toward neutral once given one.
 *
 * Not thread-safe against concurrent configure()/setAnchor()/setWhite() and
 * apply(); the caller (one post-processing stage) serialises all of them
 * under its own mutex, the same way CcmpPreviewRenderer's caller does.
 */

#ifndef CINEPI_CLIP_NEUTRALISE_HPP
#define CINEPI_CLIP_NEUTRALISE_HPP

#include <algorithm>
#include <cstdint>
#include <string>

#include "ccmp_preview.hpp"
#include "clip_plateau.hpp"

class HighlightNeutraliser
{
public:
    /* Refuses mono: a mono sensor has no CFA, no measured AWB gains and no
     * CCM, so there is no colour cast for this class to remove — the stage
     * keeps the ISP preview for mono and says so, rather than calling
     * configure() at all, but refusing here too means a caller mistake fails
     * loudly instead of silently blending luminance toward "neutral", which
     * is a meaningless operation on a channel that was never a colour. */
    bool configure(const CcmpPreviewGeometry &geom, std::string *err = nullptr)
    {
        ready_ = false;

        if (!geom.valid())
        {
            if (err)
                *err = "invalid preview geometry";
            return false;
        }
        if (geom.mono)
        {
            if (err)
                *err = "mono has no CFA and no colour cast to neutralise";
            return false;
        }

        geom_ = geom;
        ready_ = true;
        return true;
    }

    /* anchor_code 0 disables the correction: apply() then leaves the buffer
     * byte-identical to the ISP's own render, which is what makes "0" a safe
     * default before the first measurement of a take and a safe fallback the
     * moment the anchor is reset (see ccmpPreviewStage.cpp's AnalogueGain
     * handling). rolloff <= 0 is treated the same way — a zero-width ramp is
     * not a ramp. */
    void setAnchor(unsigned anchor_code, double rolloff)
    {
        anchor_ = anchor_code;
        if (anchor_code > 0 && rolloff > 1e-6)
        {
            lo_ = static_cast<float>(static_cast<double>(anchor_code) * (1.0 - rolloff));
            scale_ = static_cast<float>(1.0 / (static_cast<double>(anchor_code) * rolloff));
        }
        else
        {
            lo_ = 0.f;
            scale_ = 0.f;
        }
    }

    /* The Y code fully-desaturated pixels ramp toward: 255 for a full-range
     * lores, 235 for limited — the caller reads this off the stream's own
     * ColorSpace, the same way CcmpPreviewColour::rec709 is. Chroma always
     * ramps toward 128 regardless of range. */
    void setWhite(uint8_t y_white) { white_ = y_white; }

    unsigned anchor() const { return anchor_; }

    /* The peak raw code seen, and the highest code that did NOT fully
     * desaturate, since the last resetMaxCode() — same pair, same meaning as
     * CcmpPreviewRenderer's: with the anchor placed correctly the second sits
     * just under the first; when it tracks the peak instead the anchor is too
     * high. See that class's comment and the 2026-09-06/07 hardware-log entry
     * this observability was added to answer. */
    unsigned maxCodeSeen() const { return max_code_; }
    unsigned maxUndesaturatedCode() const { return max_undesat_; }
    /* Quads the correction ran to completion on (s >= 0.99) since the last
     * reset. Zero while a blown area is in frame means the anchor is too
     * high. */
    unsigned long fullyDesaturated() const { return full_desat_; }
    void resetMaxCode() const { max_code_ = 0; full_desat_ = 0; max_undesat_ = 0; }

    /*
     * raw: the Bayer plane behind the frame `yuv` was already rendered from —
     * same buffer, same geometry ccmp_preview_src_row/col() were configured
     * with. yuv: the ISP's own planar YUV420 (I420) lores render, corrected
     * IN PLACE — Y at yuv, U at +stride*height, V after that, chroma stride
     * half of luma's, exactly CcmpPreviewRenderer::render()'s layout.
     *
     * det: fed one (min, max) per quad this pass touches, if non-null, so the
     * SAME pass that neutralises this frame also gathers the evidence for
     * next frame's anchor — see ccmpPreviewStage.cpp's Process() for why it
     * is always the PREVIOUS frame's anchor in force here, never this frame's
     * still-being-measured one. Pass nullptr to correct without measuring
     * (the override-anchor and low-gain-skip cases).
     */
    void apply(const uint8_t *raw, uint8_t *yuv, ClipPlateauDetector *det) const
    {
        if (!ready_ || !raw || !yuv)
            return;

        const unsigned ow = geom_.out_width, oh = geom_.out_height;
        const size_t ys = geom_.out_stride;
        const size_t cs = ys / 2;

        uint8_t *yp = yuv;
        uint8_t *up = yuv + ys * oh;
        uint8_t *vp = up + cs * (oh / 2);

        /* Two output rows at a time, matching CcmpPreviewRenderer::render():
         * each chroma sample covers a 2x2 luma block, so its blend has to be
         * the strongest of the four before it is applied once per block. */
        for (unsigned oy = 0; oy < oh; oy += 2)
        {
            const unsigned sy0 = ccmp_preview_src_row(geom_, oy);
            const unsigned sy1 = ccmp_preview_src_row(geom_, oy + 1);
            uint8_t *y0 = yp + static_cast<size_t>(oy) * ys;
            uint8_t *y1 = yp + static_cast<size_t>(oy + 1) * ys;
            uint8_t *u = up + static_cast<size_t>(oy / 2) * cs;
            uint8_t *v = vp + static_cast<size_t>(oy / 2) * cs;

            for (unsigned ox = 0; ox < ow; ox += 2)
            {
                const unsigned sx0 = ccmp_preview_src_col(geom_, ox);
                const unsigned sx1 = ccmp_preview_src_col(geom_, ox + 1);

                const float s0 = sampleAndBlend(raw, sx0, sy0, det, y0[ox]);
                const float s1 = sampleAndBlend(raw, sx1, sy0, det, y0[ox + 1]);
                const float s2 = sampleAndBlend(raw, sx0, sy1, det, y1[ox]);
                const float s3 = sampleAndBlend(raw, sx1, sy1, det, y1[ox + 1]);

                const float smax = std::max(std::max(s0, s1), std::max(s2, s3));
                u[ox / 2] = blend8(u[ox / 2], 128, smax);
                v[ox / 2] = blend8(v[ox / 2], 128, smax);
            }
        }
    }

private:
    /* Reads one raw quad, feeds the detector if attached, blends `y` toward
     * white_ by this quad's desaturation factor, and returns that factor so
     * the caller can fold it into the chroma block's maximum. */
    float sampleAndBlend(const uint8_t *raw, unsigned sx, unsigned sy, ClipPlateauDetector *det, uint8_t &y) const
    {
        const uint16_t *r0 = reinterpret_cast<const uint16_t *>(raw + static_cast<size_t>(sy) * geom_.raw_stride);
        const uint16_t *r1 = reinterpret_cast<const uint16_t *>(raw + static_cast<size_t>(sy + 1) * geom_.raw_stride);

        const unsigned c0 = r0[sx] >> geom_.raw_shift;
        const unsigned c1 = r0[sx + 1] >> geom_.raw_shift;
        const unsigned c2 = r1[sx] >> geom_.raw_shift;
        const unsigned c3 = r1[sx + 1] >> geom_.raw_shift;

        const unsigned mx = std::max(std::max(c0, c1), std::max(c2, c3));
        const unsigned mn = std::min(std::min(c0, c1), std::min(c2, c3));

        if (det)
            det->add(mn, mx);

        if (mx > max_code_)
            max_code_ = mx;

        /* clamp((mx - lo_) / (anchor_ - lo_), 0, 1). anchor_ == 0 or a
         * zero-width ramp both land here with scale_ == 0 and lo_ == 0, so
         * mx > lo_ is true for any mx > 0 but the multiply by scale_ still
         * forces s to exactly 0 -- the buffer comes out byte-identical
         * without a second special case. */
        float s = 0.f;
        if (mx > lo_)
            s = std::min(1.f, (static_cast<float>(mx) - lo_) * scale_);

        if (s >= 0.99f)
            ++full_desat_;
        else if (mx > max_undesat_)
            max_undesat_ = mx;

        y = blend8(y, white_, s);
        return s;
    }

    static uint8_t blend8(uint8_t v, uint8_t target, float s)
    {
        const float out = static_cast<float>(v) + s * (static_cast<float>(target) - static_cast<float>(v));
        const int i = static_cast<int>(out + 0.5f);
        return static_cast<uint8_t>(i < 0 ? 0 : (i > 255 ? 255 : i));
    }

    bool ready_ = false;
    CcmpPreviewGeometry geom_;
    unsigned anchor_ = 0;   /* 0 = off; see setAnchor()                      */
    float lo_ = 0.f;        /* raw code where the ramp starts                */
    float scale_ = 0.f;     /* 1/(anchor*rolloff); 0 = correction off        */
    uint8_t white_ = 235;   /* Y a fully desaturated pixel ramps to          */

    /* Observability only, never read by apply() itself — see the accessors'
     * comments. mutable so apply() stays const, matching
     * CcmpPreviewRenderer's max_code_ trio and its reasoning. */
    mutable unsigned max_code_ = 0;
    mutable unsigned long full_desat_ = 0;
    mutable unsigned max_undesat_ = 0;
};

#endif /* CINEPI_CLIP_NEUTRALISE_HPP */
