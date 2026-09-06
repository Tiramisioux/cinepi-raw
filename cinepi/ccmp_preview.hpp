/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * ccmp_preview.hpp - render the lores preview from the raw Bayer, decompanded.
 *
 * Pure pixel math: no libcamera, no Redis, no app headers — only the standard
 * library and ccmp_lut.hpp, so tests/ccmp_preview_test.cpp exercises the *same*
 * code that ships.
 *
 * WHAT THIS IS FOR. The CCMP12 fix in dng_encoder.cpp is metadata: it writes a
 * LinearizationTable into the DNG and never touches a pixel. Only a DNG reader
 * ever applies it. The preview does not read DNGs — it is the PiSP back end's
 * YUV output, and the back end was handed the companded 12-bit codes and told
 * they were linear. So the HDMI and MJPEG previews still render exactly the
 * "before" image: mid-tones crushed magenta.
 *
 * It is a colour cast and not merely flat contrast because the compander is
 * level-dependent, so it changes channel RATIOS. Measured on the chart, R/G
 * swings 51% (binned) / 31% (full res) across the neutral ramp against a 1.3%
 * floor on the linear modes. No white balance can flatten a ratio that moves
 * with level — the same sentence as ccmp_lut.hpp, for the same reason.
 *
 * So the preview has to come from the raw Bayer instead, with the decompand
 * applied to the codes before anything else touches them. That is all this
 * file does:
 *
 *     code -> CcmpLut -> linear -> white balance -> CCM -> gamma -> YUV420
 *
 * The recorded DNG is not involved and does not change. This is a second,
 * independent consumer of the same verified table.
 *
 * ── THREE DETAILS ARE LOAD-BEARING ────────────────────────────────────────────
 *
 *   1. DECOMPAND BEFORE THE GAINS, ALWAYS. The whole defect is that the ISP
 *      applies gains to companded codes. Doing the same here in the wrong order
 *      reproduces the bug faithfully. The table is applied to the raw code; the
 *      white balance multiplies the LINEAR value that comes out.
 *
 *   2. THE RAW CONTAINER IS MSB-ALIGNED. PiSP delivers the 12-bit mode as
 *      SRGGB16 carrying 12 significant bits at the TOP of the 16-bit word, so
 *      the sensor's own code is `px >> 4` — the same shift pack_row_16_to_12bit
 *      does in dng_pack.hpp. Indexing a 4096-entry table with the unshifted
 *      word runs off the end; indexing it with the wrong shift silently reads
 *      the wrong part of the curve. `raw_shift` is explicit for that reason and
 *      configure() range-checks every code it can produce.
 *
 *   3. THE TABLE'S OUTPUT IS L + 200, NOT L. CcmpLut emits the black-referred
 *      level plus the output pedestal, because that is the domain the DNG level
 *      tags describe. A preview wants scene-linear, so black_level() comes off
 *      before the normalisation. Skipping it lifts the shadows by 200/63065 and
 *      -- worse -- puts a constant into the ratio the white balance then scales,
 *      which is a colour cast in the blacks: the defect, one order of magnitude
 *      down.
 *
 * SCOPE. The caller gates this. It is correct only for a source that actually
 * companded — 12-bit ClearHDR — and decompanding a mode that did not is the
 * same defect with the sign flipped (ccmp_lut.hpp, and dng_encoder.cpp's own
 * scope comment).
 */

#ifndef CINEPI_CCMP_PREVIEW_HPP
#define CINEPI_CCMP_PREVIEW_HPP

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "ccmp_lut.hpp"

/* Which colour a Bayer quad position carries. Matches dng_encoder.cpp's CFA
 * arrays (0 = R, 1 = G, 2 = B) so one convention serves both consumers. */
enum : unsigned
{
    CCMP_PREVIEW_R = 0,
    CCMP_PREVIEW_G = 1,
    CCMP_PREVIEW_B = 2,
};

struct CcmpPreviewGeometry
{
    unsigned raw_width = 0;
    unsigned raw_height = 0;
    size_t raw_stride = 0;    /* bytes per raw row                              */
    unsigned raw_shift = 4;   /* >> to reach the sensor's own code (detail 2)   */

    /* Quad positions in raster order: (0,0) (1,0) (0,1) (1,1). Ignored when
     * mono is set. */
    unsigned cfa[4] = { CCMP_PREVIEW_R, CCMP_PREVIEW_G, CCMP_PREVIEW_G, CCMP_PREVIEW_B };

    /* No CFA to demosaic — the mono sensor's R16/R12. quadRgb averages the
     * whole quad into one luminance instead of splitting it by cfa[], and
     * setColour treats white balance and the CCM as identity: a mono tuning
     * has no measured gains or matrix for either to come from. */
    bool mono = false;

    unsigned out_width = 0;
    unsigned out_height = 0;
    size_t out_stride = 0;    /* bytes per Y row; chroma rows are half of it    */

    bool valid() const
    {
        /* Even in both axes: a Bayer quad needs sx+1 and sy+1 in range, and
         * YUV420 needs whole chroma pairs. */
        return raw_width >= 2 && raw_height >= 2 && (raw_width & 1u) == 0 && (raw_height & 1u) == 0 &&
               raw_stride >= static_cast<size_t>(raw_width) * 2 && raw_shift <= 8 &&
               out_width >= 2 && out_height >= 2 && (out_width & 1u) == 0 && (out_height & 1u) == 0 &&
               out_stride >= out_width;
    }
};

struct CcmpPreviewColour
{
    /* The AWB gains the pipeline is already using, i.e. metadata ColourGains.
     * They were solved on companded stats so they are not the gains a linear
     * pipeline would have picked — but the chart evidence renders neutral under
     * exactly these (the DNG's AsShotNeutral is 1/gain), because once the
     * transfer is undone the residual is a gain error and a gain error IS what
     * a white balance corrects. */
    double r_gain = 1.0;
    double b_gain = 1.0;

    /* White-balanced camera RGB -> linear sRGB, row-major. Same matrix and same
     * default as dng_encoder.cpp, so the preview and the file agree on hue. */
    double ccm[9] = { 1.90255, -0.77478, -0.12777,
                      -0.31338, 1.88197, -0.56858,
                      -0.06001, -0.61785, 1.67786 };

    /* 1.0 puts the table's WhiteLevel at diffuse white. The chart sits where
     * that predicts — the five mid-range neutral patches land at 2861..10960 of
     * 63065, i.e. 4.5%..17%, and 18% grey is 17%. So the default is not a
     * guess, it is where the measured chart falls. */
    double exposure = 1.0;
    double gamma = 2.2;

    /* The top fraction of RAW full scale over which a pixel ramps to fully
     * desaturated — see desaturateHighlight(). 0.02 means the correction is
     * inert until a channel is within 2% of the clip code. Widen it for a
     * softer approach to white; 0 disables the correction and puts the
     * clipped-channel magenta back. */
    double highlight_rolloff = 0.02;

    /* THE FLOOR OF THE CLAMP ZONE, as a raw code — the reference
     * `highlight_rolloff` is a fraction OF, and the level at and above which a
     * pixel is treated as clipped. Not the decompand table's top code, and —
     * the part that cost a round on hardware — NOT the highest code the sensor
     * emits either.
     *
     * The table spans the compander's whole 0..4095 output domain because that
     * is what the curve is defined over. The imx585 in 12-bit ClearHDR does not
     * fill it: the HG/LG merge clamps digitally and the channels converge as it
     * does, so a blown area arrives with R, G and B on very nearly the SAME
     * code. That matters because an equal-code quad is exactly what this
     * renderer cannot pass through cleanly — with the shipping gains and CCM,
     * green solves NEGATIVE for an equal-code quad at any level, clamps to 0,
     * and the pixel goes magenta. So the whole convergence zone renders
     * magenta, not just its top.
     *
     * THE ZONE IS SOFT AND SITS BELOW THE PEAK CODE. Measured on two takes:
     *
     *   CINEPI_26-09-06_210738  magenta codes p1 2974, p50 3000, peak 3054
     *   CINEPI_26-09-06_214831  magenta codes p1 2948, p50 2974, peak 3027
     *
     * with the stage's own "peak raw code" line reporting 3050..3061 live. A
     * first attempt anchored this at 3040 — just under the peak — on the
     * assumption that the clamp was one hard code. It desaturated the blown
     * area by a median of 0.000 and changed nothing on hardware, because the
     * BODY of the zone is 60-80 codes below the peak. Anchor on the floor of
     * the zone, not its top: the ramp ends AT this code, so anything above it
     * is fully desaturated and anything the clamp has flattened is caught.
     *
     * WHY 2900. It is below the p1 of both takes (2948 and 2974) with margin
     * for the zone moving with the ClearHDR blend and gain-adder knobs, which
     * the operator can change. The margin is nearly free: codes 2900..3054 span
     * 0.078 stops of scene, so the band this claims is one that only clamp
     * transition lives in, and dropping the anchor from 2974 to 2900 widens the
     * pixels touched from 0.143% to 0.146% of the frame. Both takes end up
     * 99.8%+ of the blown area fully desaturated.
     *
     * Overridable from the post-process file (`sensorClipCode`) so a rig whose
     * ClearHDR knobs land the clamp elsewhere can be corrected without a
     * rebuild; 0 means "the decompand table's top code", i.e. the pre-fix
     * behaviour. ccmpPreviewStage logs the peak code AND how much of the frame
     * actually reached full desaturation — the second is the number that says
     * whether this anchor is low enough, and the first alone is what made the
     * 3040 attempt look right. */
    unsigned sensor_clip_code = 2900;

    /* Which YUV matrix the display expects. The caller reads it off the lores
     * stream's ColorSpace rather than assuming; getting it wrong is a small but
     * real hue shift, and a preview that exists to judge colour should not have
     * one. */
    bool rec709 = false;
};

/*
 * Renders one frame. Configure once per camera configuration, setColour per
 * frame (it is nine multiplies), render per frame.
 *
 * Not thread-safe against configure/setColour; render() is const and the
 * caller (one post-processing stage) is single-threaded per instance.
 */
class CcmpPreviewRenderer
{
public:
    /* Folds the decompand table, the black subtraction and the normalisation
     * into one 4096-entry float table. Returns false with `err` set if the
     * geometry is malformed or the table cannot serve the codes this raw
     * container can produce. */
    bool configure(const CcmpPreviewGeometry &geom, const CcmpLut &lut, std::string *err = nullptr)
    {
        ready_ = false;

        if (!geom.valid())
        {
            if (err)
                *err = "invalid preview geometry";
            return false;
        }
        if (!lut.valid())
        {
            if (err)
                *err = "CCMP decompand table is empty";
            return false;
        }

        /* Detail 2: every code the container can deliver has to be in the
         * table. 12 significant bits in a 16-bit word shifted down by 4 gives
         * 0..4095; anything else is a container this renderer has not been
         * told about, and reading past the table is how that would present. */
        const size_t max_code = static_cast<size_t>(0xFFFFu >> geom.raw_shift);
        if (max_code >= lut.size())
        {
            if (err)
                *err = "raw_shift " + std::to_string(geom.raw_shift) + " yields codes up to " +
                       std::to_string(max_code) + " but the decompand table holds " +
                       std::to_string(lut.size()) +
                       ". The container is not the 12-in-16 MSB-aligned one this assumes.";
            return false;
        }

        const double black = lut.black_level();
        const double span = static_cast<double>(lut.white_level()) - black;
        if (!(span > 0.0))
        {
            if (err)
                *err = "CCMP table has a non-positive black-to-white span";
            return false;
        }

        /* Detail 3: black off first, then normalise. */
        lin_.resize(lut.size());
        for (size_t c = 0; c < lut.size(); ++c)
            lin_[c] = static_cast<float>((static_cast<double>(lut.table()[c]) - black) / span);

        geom_ = geom;
        ready_ = true;
        return true;
    }

    /* Bakes the white balance, the CCM and the exposure into one matrix, and
     * the gamma into a table. Detail 1 lives here: the gains are part of the
     * matrix, which is applied to values that have ALREADY been through lin_. */
    void setColour(const CcmpPreviewColour &colour)
    {
        /* Mono has no AWB gains and no CCM — the tuning file carries neither,
         * so colour.r_gain/b_gain/ccm here are whatever the non-mono default
         * happens to be, not a measurement. Gate on geom_.mono, the format
         * fact, rather than on the gain values themselves (a color sensor can
         * legitimately report unity gains too). Exposure still applies, so a
         * neutral quad stays neutral and only brightens/darkens. */
        if (geom_.mono)
        {
            for (int row = 0; row < 3; ++row)
                for (int col = 0; col < 3; ++col)
                    m_[row * 3 + col] = (row == col) ? static_cast<float>(colour.exposure) : 0.f;
        }
        else
        {
            const double gains[3] = { colour.r_gain, 1.0, colour.b_gain };
            for (int row = 0; row < 3; ++row)
                for (int col = 0; col < 3; ++col)
                    m_[row * 3 + col] =
                        static_cast<float>(colour.ccm[row * 3 + col] * gains[col] * colour.exposure);
        }

        /* Only on change: this is called per frame and the table is 4096 pow()
         * calls, while gamma comes from the post-process file and does not move
         * between frames. The gains and the CCM above do, which is why they are
         * not cached with it. */
        const double g = (colour.gamma > 0.01) ? colour.gamma : 2.2;
        if (g != gamma_built_)
        {
            for (size_t i = 0; i < kGammaSize; ++i)
            {
                const double x = static_cast<double>(i) / static_cast<double>(kGammaSize - 1);
                gamma_[i] = static_cast<float>(std::pow(x, 1.0 / g));
            }
            gamma_built_ = g;
        }

        /* 0 disables the correction: it makes the blend factor identically 0,
         * so the inner loop needs no special case and the A/B against the
         * uncorrected render is one number in the post-process file. */
        /* The clip reference, in the same normalised domain `peak` is measured
         * in. lin_ is built by configure(), which always runs first, so the
         * table lookup here is safe. Out of range (or 0) falls back to the
         * table's top, which is the pre-fix behaviour. */
        double hl_ref = 1.0;
        if (colour.sensor_clip_code > 0 && colour.sensor_clip_code < lin_.size())
            hl_ref = lin_[colour.sensor_clip_code];
        if (!(hl_ref > 0.0))
            hl_ref = 1.0;
        hl_ref_ = static_cast<float>(hl_ref);

        if (colour.highlight_rolloff > 1e-6)
        {
            /* Ramp from (1 - rolloff) * clip up to the clip itself, so a pixel
             * AT the clip is fully desaturated whatever the reference is. */
            hl_lo_ = static_cast<float>(hl_ref * (1.0 - colour.highlight_rolloff));
            hl_scale_ = static_cast<float>(1.0 / (hl_ref * colour.highlight_rolloff));
        }
        else
        {
            hl_lo_ = 1.f;
            hl_scale_ = 0.f;
        }

        if (colour.rec709)
            setYuvCoeffs(0.2126, 0.7152, 0.0722);
        else
            setYuvCoeffs(0.299, 0.587, 0.114);
    }

    bool ready() const { return ready_; }
    const CcmpPreviewGeometry &geometry() const { return geom_; }

    /* The normalised level the highlight correction is referenced to, and the
     * highest raw code seen since the last resetMaxCode(). The second is what
     * says whether the first matches the sensor. */
    float highlightReference() const { return hl_ref_; }
    unsigned maxCodeSeen() const { return max_code_; }
    /* Quads the highlight correction ran to completion on since the last
     * reset. Zero while blown highlights are in frame means sensor_clip_code
     * is anchored too high. */
    unsigned long fullyDesaturated() const { return full_desat_; }
    void resetMaxCode() const { max_code_ = 0; full_desat_ = 0; }

    /*
     * raw: the Bayer plane, `raw_stride` bytes per row, 16-bit samples.
     * yuv: planar YUV420 (I420) — Y at yuv, U at +stride*height, V after that.
     *
     * Nearest-neighbour: one Bayer quad per output pixel. The lores stream is
     * always a large downscale of the raw (720-high against 1090 or 2180), so
     * there is no case where this upsamples, and a box filter would cost more
     * than it buys on a monitoring image.
     */
    void render(const uint8_t *raw, uint8_t *yuv) const
    {
        if (!ready_ || !raw || !yuv)
            return;

        const unsigned ow = geom_.out_width, oh = geom_.out_height;
        const size_t ys = geom_.out_stride;
        const size_t cs = ys / 2;

        uint8_t *yp = yuv;
        uint8_t *up = yuv + ys * oh;
        uint8_t *vp = up + cs * (oh / 2);

        /* Two output rows at a time so each chroma sample is the box mean of
         * the 2x2 luma block it covers, which is what YUV420 means. */
        for (unsigned oy = 0; oy < oh; oy += 2)
        {
            const unsigned sy0 = srcRow(oy);
            const unsigned sy1 = srcRow(oy + 1);
            uint8_t *y0 = yp + static_cast<size_t>(oy) * ys;
            uint8_t *y1 = yp + static_cast<size_t>(oy + 1) * ys;
            uint8_t *u = up + static_cast<size_t>(oy / 2) * cs;
            uint8_t *v = vp + static_cast<size_t>(oy / 2) * cs;

            for (unsigned ox = 0; ox < ow; ox += 2)
            {
                const unsigned sx0 = srcCol(ox);
                const unsigned sx1 = srcCol(ox + 1);

                float rgb[4][3];
                quadRgb(raw, sx0, sy0, rgb[0]);
                quadRgb(raw, sx1, sy0, rgb[1]);
                quadRgb(raw, sx0, sy1, rgb[2]);
                quadRgb(raw, sx1, sy1, rgb[3]);

                y0[ox] = luma(rgb[0]);
                y0[ox + 1] = luma(rgb[1]);
                y1[ox] = luma(rgb[2]);
                y1[ox + 1] = luma(rgb[3]);

                float mean[3];
                for (int c = 0; c < 3; ++c)
                    mean[c] = 0.25f * (rgb[0][c] + rgb[1][c] + rgb[2][c] + rgb[3][c]);

                u[ox / 2] = chroma(mean, cb_[0], cb_[1], cb_[2]);
                v[ox / 2] = chroma(mean, cr_[0], cr_[1], cr_[2]);
            }
        }
    }

private:
    static constexpr size_t kGammaSize = 4096;

    /* Nearest source quad, snapped to the even origin of a Bayer cell. */
    unsigned srcRow(unsigned oy) const
    {
        const unsigned r = static_cast<unsigned>(static_cast<uint64_t>(oy) * geom_.raw_height / geom_.out_height);
        return std::min(r & ~1u, geom_.raw_height - 2);
    }
    unsigned srcCol(unsigned ox) const
    {
        const unsigned c = static_cast<unsigned>(static_cast<uint64_t>(ox) * geom_.raw_width / geom_.out_width);
        return std::min(c & ~1u, geom_.raw_width - 2);
    }

    /* One Bayer quad -> gamma-encoded R'G'B' in 0..1. */
    void quadRgb(const uint8_t *raw, unsigned sx, unsigned sy, float out[3]) const
    {
        const uint16_t *r0 = reinterpret_cast<const uint16_t *>(raw + static_cast<size_t>(sy) * geom_.raw_stride);
        const uint16_t *r1 = reinterpret_cast<const uint16_t *>(raw + static_cast<size_t>(sy + 1) * geom_.raw_stride);

        const unsigned c0 = r0[sx] >> geom_.raw_shift;
        const unsigned c1 = r0[sx + 1] >> geom_.raw_shift;
        const unsigned c2 = r1[sx] >> geom_.raw_shift;
        const unsigned c3 = r1[sx + 1] >> geom_.raw_shift;

        /* The highest code this render actually saw. Cheap (four compares
         * against a register on a path that already touched these samples) and
         * it is the only way to check sensor_clip_code against the hardware
         * instead of trusting it — see that field's comment. */
        const unsigned hi = std::max(std::max(c0, c1), std::max(c2, c3));
        if (hi > max_code_)
            max_code_ = hi;

        const float s[4] = { lin_[c0], lin_[c1], lin_[c2], lin_[c3] };

        float cam[3];
        if (geom_.mono)
        {
            /* No CFA: the quad is four luminance samples, not one colour per
             * position. Averaging them (rather than picking a fake Bayer
             * channel) is what keeps the signal in all three output channels
             * — see the struct comment on CcmpPreviewGeometry::mono. */
            const float mean = 0.25f * (s[0] + s[1] + s[2] + s[3]);
            cam[CCMP_PREVIEW_R] = cam[CCMP_PREVIEW_G] = cam[CCMP_PREVIEW_B] = mean;
        }
        else
        {
            /* Two greens per quad; the other two positions are one sample each. */
            cam[0] = cam[1] = cam[2] = 0.f;
            int green = 0;
            for (int i = 0; i < 4; ++i)
            {
                if (geom_.cfa[i] == CCMP_PREVIEW_G)
                {
                    cam[CCMP_PREVIEW_G] += s[i];
                    ++green;
                }
                else
                    cam[geom_.cfa[i]] = s[i];
            }
            if (green > 1)
                cam[CCMP_PREVIEW_G] /= static_cast<float>(green);
        }

        /* How close the most-exposed channel is to the sensor's clip, measured
         * BEFORE the gains and the matrix — see desaturateHighlight() for why
         * it cannot be measured after them. */
        const float peak = std::max(std::max(s[0], s[1]), std::max(s[2], s[3]));

        float lin[3];
        for (int row = 0; row < 3; ++row)
            lin[row] = m_[row * 3 + 0] * cam[0] + m_[row * 3 + 1] * cam[1] + m_[row * 3 + 2] * cam[2];

        desaturateHighlight(lin, peak);

        for (int row = 0; row < 3; ++row)
            out[row] = gammaEncode(lin[row]);
    }

    /* ── the clipped-channel cast ─────────────────────────────────────────────
     *
     * The decompand fixes the transfer, but it cannot un-clip. A NEUTRAL
     * highlight that reaches the sensor's ceiling arrives here with all three
     * channels pinned on the SAME code — the imx585's ClearHDR HG/LG merge
     * clamps digitally, so unlike a photodiode saturation there is no channel
     * that pins first. The gains then scale that equal-code plateau by r_gain
     * and b_gain, R and B overshoot a G that cannot move, and the blown area
     * goes magenta — the same hue as the original defect, from a different
     * cause, and it survives the decompand untouched.
     *
     * ** THE REFERENCE IS THE SENSOR'S CLIP, NOT THE TABLE'S TOP CODE. ** This
     * used to compare `peak` against 1.0, i.e. the top of the compander's
     * output domain. The sensor never gets there (see
     * CcmpPreviewColour::sensor_clip_code), so the correction was unreachable
     * on real footage and every blown highlight stayed magenta. `hl_lo_` is now
     * derived from sensor_clip_code, and the ramp still ends exactly AT the
     * clip, so a pixel that reaches it is fully desaturated.
     *
     * ** THE TRIGGER IS RAW SATURATION, AND IT CANNOT BE ANYTHING ELSE. ** The
     * tempting test is "did any channel come out of the matrix above 1", since
     * that is true throughout the zone. It is also true of a SATURATED COLOUR
     * THAT NEVER CLIPPED: a pure red at 79% of raw full scale leaves the 2.5x
     * gain and the CCM's 1.9 diagonal at 1.5, and testing the output would
     * render a bright red practical as white. Clipping is a property of the
     * sensor, so it has to be read where the sensor wrote it — `peak` is the
     * largest of the quad's four samples in the table's normalised domain,
     * taken before the gains and the matrix touch anything.
     *
     * Blending toward the maximum drives a blown pixel to neutral, which then
     * clamps to white. That is what a monitoring image should show: blown reads
     * as white, not as a colour. A highlight that is genuinely one colour also
     * whitens once it CLIPS, which is what every raw converter does and is the
     * accepted trade — past clipping the hue is not measured data. The
     * distinction this keeps is between clipped and merely bright.
     *
     * This is a PREVIEW-only correction. It has no counterpart in the DNG and
     * must not acquire one: the file's job is to carry what was recorded,
     * clipped channels included, so a grade can reconstruct them. */
    void desaturateHighlight(float lin[3], float peak) const
    {
        if (peak <= hl_lo_)
            return;

        const float s = std::min(1.f, (peak - hl_lo_) * hl_scale_);
        /* Observability: how much of the frame the correction actually
         * COMPLETED on. The peak code alone cannot say — a frame can peak well
         * above the anchor while the body of the blown area sits below it and
         * desaturates by nothing, which is exactly how the first anchor looked
         * correct and did nothing. */
        if (s >= 0.99f)
            ++full_desat_;
        const float mx = std::max(lin[0], std::max(lin[1], lin[2]));
        for (int i = 0; i < 3; ++i)
            lin[i] += s * (mx - lin[i]);
    }

    float gammaEncode(float lin) const
    {
        if (!(lin > 0.f))
            return gamma_[0];
        if (lin >= 1.f)
            return gamma_[kGammaSize - 1];
        return gamma_[static_cast<size_t>(lin * static_cast<float>(kGammaSize - 1))];
    }

    void setYuvCoeffs(double kr, double kg, double kb)
    {
        /* Limited ("studio") range, which is what the compositor in
         * dualHdmiPreviewStage assumes when it clears a pane to Y=16 / UV=128. */
        y_[0] = static_cast<float>(219.0 * kr);
        y_[1] = static_cast<float>(219.0 * kg);
        y_[2] = static_cast<float>(219.0 * kb);

        const double sb = 0.5 / (1.0 - kb);
        cb_[0] = static_cast<float>(224.0 * -kr * sb);
        cb_[1] = static_cast<float>(224.0 * -kg * sb);
        cb_[2] = static_cast<float>(224.0 * 0.5);

        const double sr = 0.5 / (1.0 - kr);
        cr_[0] = static_cast<float>(224.0 * 0.5);
        cr_[1] = static_cast<float>(224.0 * -kg * sr);
        cr_[2] = static_cast<float>(224.0 * -kb * sr);
    }

    uint8_t luma(const float rgb[3]) const
    {
        const float y = 16.f + y_[0] * rgb[0] + y_[1] * rgb[1] + y_[2] * rgb[2];
        return clamp8(y);
    }
    static uint8_t chroma(const float rgb[3], float c0, float c1, float c2)
    {
        return clamp8(128.f + c0 * rgb[0] + c1 * rgb[1] + c2 * rgb[2]);
    }
    static uint8_t clamp8(float v)
    {
        const int i = static_cast<int>(v + 0.5f);
        return static_cast<uint8_t>(i < 0 ? 0 : (i > 255 ? 255 : i));
    }

    bool ready_ = false;
    CcmpPreviewGeometry geom_;
    std::vector<float> lin_;
    float m_[9] = { 1.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 1.f };
    float gamma_[kGammaSize] = {};
    double gamma_built_ = 0.0;   /* 0 = never built, and no valid gamma is 0 */
    float hl_ref_ = 1.f;         /* normalised level of sensor_clip_code      */
    float hl_lo_ = 0.98f;        /* raw level where desaturation starts       */
    float hl_scale_ = 50.f;      /* 1/highlight_rolloff; 0 = correction off   */
    /* Observability only, never read by the render itself. mutable so render()
     * stays const: it describes the data that went through, not the renderer's
     * configuration. The stage serialises render() under its own mutex, so the
     * unsynchronised update is safe there. */
    mutable unsigned max_code_ = 0;
    mutable unsigned long full_desat_ = 0;
    float y_[3] = {};
    float cb_[3] = {};
    float cr_[3] = {};
};

#endif /* CINEPI_CCMP_PREVIEW_HPP */
