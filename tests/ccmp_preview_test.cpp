/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * ccmp_preview_test.cpp - the CCMP12 preview renderer.
 *
 * Includes only cinepi/ccmp_preview.hpp (which pulls in ccmp_lut.hpp), so it
 * builds with nothing but the standard library and exercises the same code that
 * ships.
 *
 * THE POINT OF THIS TEST. The renderer exists because the previews and the DNG
 * thumbnail render magenta in 12-bit ClearHDR while the recorded DNG does not:
 * the DNG carries a LinearizationTable and the ISP was never told about the
 * compander. §3.1 of the chart analysis measured the defect as a ratio that
 * MOVES WITH LEVEL — R/G swings 51% (binned) / 31% (full res) across the
 * neutral ramp against a 1.3% floor on the linear modes.
 *
 * So the test is built the way §3.1 was: put a neutral ramp in, and require the
 * ratio out to be flat. A grey in has to be a grey out at every level, which in
 * YUV means U = V = 128. `defect_is_real` first re-derives the untreated case
 * from the same model, so the instrument is shown to have the range to see the
 * fault before it is used to certify the fix.
 *
 * The tolerances are the mode's own 12-bit quantisation and are computed here,
 * not chosen: on the binned mode's middle segment one code is 64 L, so two
 * channels of one neutral patch land on grid points that are not in the same
 * ratio. That residual is the sensor's, not the renderer's.
 */

#include "cinepi/ccmp_preview.hpp"

#include <cmath>
#include <cstdint>
#include <iostream>
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

/* Cinemate's default cg_rb, i.e. what --awbgains carries in the shipping
 * configuration and therefore what metadata ColourGains reports. */
constexpr double kRGain = 2.5;
constexpr double kBGain = 2.2;

/* The neutral ramp, in L (16-bit ClearHDR LSB above black). Spans the identity
 * segment, both knees and the top, so no segment of the curve is untested. */
const double kRamp[] = { 40.0, 100.0, 400.0, 2000.0, 9000.0, 30000.0, 58000.0 };

/* One patch, three channels, at the levels a NEUTRAL subject actually presents
 * to the sensor: the raw channels sit BELOW green by the gains that later
 * neutralise them. */
struct Patch
{
    double L[3];
};

Patch neutral_patch(double L_green)
{
    Patch p;
    p.L[CCMP_PREVIEW_R] = L_green / kRGain;
    p.L[CCMP_PREVIEW_G] = L_green;
    p.L[CCMP_PREVIEW_B] = L_green / kBGain;
    return p;
}

/* L -> the 12-bit code the sensor stores. This is the FORWARD model from
 * ccmp_lut.hpp, so the test drives the renderer with codes built the way the
 * hardware builds them rather than with the inverse of the table under test. */
int store(double L, const CcmpParams &p)
{
    const double c = std::floor(ccmp_encode_level(L, p) + 0.5);
    if (c < 0.0)
        return 0;
    const double top = static_cast<double>(p.code_count() - 1);
    return static_cast<int>(c > top ? top : c);
}

/* A synthetic raw frame: every Bayer quad carries the same patch. 12 bits
 * MSB-aligned in a 16-bit word, which is how PiSP delivers the mode. */
std::vector<uint8_t> make_frame(const CcmpPreviewGeometry &g, const Patch &patch, const CcmpParams &p)
{
    std::vector<uint8_t> buf(g.raw_stride * g.raw_height, 0);
    uint16_t code[4];
    for (int i = 0; i < 4; ++i)
        code[i] = static_cast<uint16_t>(store(patch.L[g.cfa[i]], p) << g.raw_shift);

    for (unsigned y = 0; y < g.raw_height; ++y)
    {
        uint16_t *row = reinterpret_cast<uint16_t *>(buf.data() + static_cast<size_t>(y) * g.raw_stride);
        for (unsigned x = 0; x < g.raw_width; ++x)
            row[x] = code[(y & 1u) * 2 + (x & 1u)];
    }
    return buf;
}

CcmpPreviewGeometry geometry_for(unsigned raw_w, unsigned raw_h, unsigned out_w, unsigned out_h)
{
    CcmpPreviewGeometry g;
    g.raw_width = raw_w;
    g.raw_height = raw_h;
    g.raw_stride = static_cast<size_t>(raw_w) * 2;
    g.raw_shift = 4;
    g.out_width = out_w;
    g.out_height = out_h;
    g.out_stride = out_w;
    return g;
}

struct Yuv
{
    int y, u, v;
};

/* Every pixel of a flat frame is the same; read the middle one and confirm the
 * frame really is flat while we are there. */
Yuv render_flat(const CcmpPreviewRenderer &r, const std::vector<uint8_t> &raw, bool &flat)
{
    const CcmpPreviewGeometry &g = r.geometry();
    std::vector<uint8_t> out(g.out_stride * g.out_height * 3 / 2, 0);
    r.render(raw.data(), out.data());

    const uint8_t *yp = out.data();
    const uint8_t *up = yp + g.out_stride * g.out_height;
    const uint8_t *vp = up + (g.out_stride / 2) * (g.out_height / 2);

    Yuv mid { yp[(g.out_height / 2) * g.out_stride + g.out_width / 2],
              up[(g.out_height / 4) * (g.out_stride / 2) + g.out_width / 4],
              vp[(g.out_height / 4) * (g.out_stride / 2) + g.out_width / 4] };

    flat = true;
    for (unsigned y = 0; y < g.out_height && flat; ++y)
        for (unsigned x = 0; x < g.out_width; ++x)
            if (yp[y * g.out_stride + x] != mid.y)
            {
                flat = false;
                break;
            }
    return mid;
}

std::string fmt(double v)
{
    char b[64];
    std::snprintf(b, sizeof(b), "%.2f", v);
    return b;
}

/* ── the defect, re-derived ──────────────────────────────────────────────────
 *
 * What the ISP does: take the stored code as if it were linear, subtract the
 * black tag, apply the gains. Returns the worst R/G departure from neutral
 * across the ramp, as a fraction. §3.1 measured 51% binned / 31% full res; this
 * is the same quantity from the model, and it is what the renderer has to
 * remove. */
double untreated_spread(const CcmpParams &p)
{
    double lo = 1e9, hi = -1e9;
    for (double Lg : kRamp)
    {
        const Patch patch = neutral_patch(Lg);
        const double r = (store(patch.L[CCMP_PREVIEW_R], p) - p.pedestal) * kRGain;
        const double g = store(patch.L[CCMP_PREVIEW_G], p) - p.pedestal;
        if (g <= 0.0)
            continue;
        const double ratio = r / g;
        lo = std::min(lo, ratio);
        hi = std::max(hi, ratio);
    }
    return (hi - lo);
}

void run_mode(double binning, const char *label, unsigned raw_w, unsigned raw_h)
{
    std::cout << "\n" << label << " (b=" << static_cast<long long>(binning) << ")\n";

    CcmpParams params;
    if (!ccmp_params_for_binning(binning, params))
    {
        check(false, "params for binning");
        return;
    }
    CcmpLut lut;
    std::string err;
    if (!lut.build(params, &err))
    {
        check(false, "lut.build", err);
        return;
    }

    /* The instrument has to be able to see the fault before it can certify the
     * fix. A neutral ramp read as linear is not neutral, by a lot. */
    const double spread = untreated_spread(params);
    check(spread > 0.20, "defect_is_real: untreated R/G spread over the ramp",
          fmt(spread * 100.0) + "% (>20% expected; §3.1 measured 51/31%)");

    CcmpPreviewGeometry geom = geometry_for(raw_w, raw_h, 640, 360);
    CcmpPreviewRenderer r;
    if (!r.configure(geom, lut, &err))
    {
        check(false, "configure", err);
        return;
    }

    CcmpPreviewColour colour;
    colour.r_gain = kRGain;
    colour.b_gain = kBGain;
    r.setColour(colour);

    /* THE ACCEPTANCE TEST. A grey in is a grey out, at every level. The
     * tolerance is the mode's own quantisation: on the binned middle segment one
     * code is 64 L, so a neutral patch's three channels land on grid points that
     * are not exactly in the gains' ratio. */
    /* Measured worst departure is 1 code binned and 0 full res, so 4 is real
     * headroom over the quantisation while still being an order of magnitude
     * tighter than any ordering or black-level mistake would produce. */
    const int kChromaTol = 4;
    int worst = 0;
    std::string worst_at = "none";
    for (double Lg : kRamp)
    {
        const std::vector<uint8_t> raw = make_frame(geom, neutral_patch(Lg), params);
        bool flat = false;
        const Yuv px = render_flat(r, raw, flat);
        check(flat, "flat frame renders flat at L=" + fmt(Lg));

        const int du = std::abs(px.u - 128), dv = std::abs(px.v - 128);
        if (std::max(du, dv) > worst)
        {
            worst = std::max(du, dv);
            worst_at = fmt(Lg);
        }
        check(du <= kChromaTol && dv <= kChromaTol,
              "neutral stays neutral at L=" + fmt(Lg),
              "U=" + std::to_string(px.u) + " V=" + std::to_string(px.v) + " Y=" + std::to_string(px.y));
    }
    std::cout << "     worst chroma departure " << worst << "/128 at L=" << worst_at << "\n";

    /* ── the clipped-channel cast ────────────────────────────────────────────
     *
     * Observed on hardware after the decompand landed: mid-tones neutral,
     * overexposed highlights still magenta. Green clips at code 4095 first
     * because it carries the most light, red and blue keep rising, and the
     * gains then push them past a green that cannot move.
     *
     * Drive a NEUTRAL subject well past green's clip and require the result to
     * stay neutral. Without desaturateHighlight() this is strongly magenta —
     * asserted below by turning the correction off, so the test carries its own
     * proof that it is testing something. */
    {
        /* Green clips at L = white; 4x past it puts red under its own clip and
         * blue near it, which is the middle of the magenta zone. */
        const double L_clip = static_cast<double>(lut.white_level()) - lut.black_level();
        const std::vector<uint8_t> raw = make_frame(geom, neutral_patch(L_clip * 4.0), params);

        bool f = false;
        const Yuv on = render_flat(r, raw, f);
        check(std::abs(on.u - 128) <= kChromaTol && std::abs(on.v - 128) <= kChromaTol,
              "a blown neutral highlight stays neutral",
              "U=" + std::to_string(on.u) + " V=" + std::to_string(on.v) + " Y=" + std::to_string(on.y));
        check(on.y >= 235, "and reads as white", "Y=" + std::to_string(on.y));

        CcmpPreviewColour off = colour;
        off.highlight_rolloff = 0.0;
        r.setColour(off);
        const Yuv bad = render_flat(r, raw, f);
        check(std::abs(bad.v - 128) > 3 * kChromaTol, "and is magenta without the correction",
              "U=" + std::to_string(bad.u) + " V=" + std::to_string(bad.v));
        r.setColour(colour);
    }

    /* Black in, black out. The table's output carries the +200 pedestal and the
     * renderer has to take it off; if it does not, the blacks lift AND the gains
     * scale a constant, which is a cast in the shadows. */
    Patch black;
    black.L[0] = black.L[1] = black.L[2] = 0.0;
    const std::vector<uint8_t> raw_black = make_frame(geom, black, params);
    bool flat = false;
    const Yuv px = render_flat(r, raw_black, flat);
    check(px.y == 16 && px.u == 128 && px.v == 128, "black is limited-range black",
          "Y=" + std::to_string(px.y) + " U=" + std::to_string(px.u) + " V=" + std::to_string(px.v));

    /* Monotonic in level: the ramp has to come out as a ramp. A table applied
     * with the wrong binning still produces a plausible picture, but it does not
     * keep this ordering against the forward model that built the codes. */
    int prev = -1;
    bool monotonic = true;
    for (double Lg : kRamp)
    {
        const std::vector<uint8_t> raw = make_frame(geom, neutral_patch(Lg), params);
        bool f = false;
        const int y = render_flat(r, raw, f).y;
        if (y < prev)
            monotonic = false;
        prev = y;
    }
    check(monotonic, "the neutral ramp renders monotonically");
}

void run_common()
{
    std::cout << "\ncommon\n";

    CcmpParams params;
    ccmp_params_for_binning(4.0, params);
    CcmpLut lut;
    std::string err;
    lut.build(params, &err);

    /* Detail 2: the container is 12-in-16 MSB-aligned. A shift of 0 means codes
     * up to 65535 against a 4096-entry table, which is a read past the end —
     * configure() has to refuse it rather than render from adjacent memory. */
    CcmpPreviewGeometry bad = geometry_for(1928, 1090, 640, 360);
    bad.raw_shift = 0;
    CcmpPreviewRenderer r;
    check(!r.configure(bad, lut, &err), "refuses a container that overruns the table", err);
    check(!r.ready(), "and stays unconfigured");

    /* Odd geometry has no whole Bayer quad and no whole chroma pair. */
    CcmpPreviewGeometry odd = geometry_for(1928, 1090, 641, 360);
    check(!r.configure(odd, lut, &err), "refuses an odd output width");

    /* An empty table is not a curve. */
    CcmpLut empty;
    check(!r.configure(geometry_for(1928, 1090, 640, 360), empty, &err), "refuses an unbuilt table");

    /* CFA: a red-only patch has to come out red. Cr above 128 and Cb below is
     * the cheapest assertion that the quad order was not transposed — a swapped
     * CFA renders a red subject blue and nothing else in this file would say so. */
    CcmpPreviewGeometry geom = geometry_for(1928, 1090, 640, 360);
    if (r.configure(geom, lut, &err))
    {
        CcmpPreviewColour colour;
        colour.r_gain = kRGain;
        colour.b_gain = kBGain;
        r.setColour(colour);

        Patch red;
        red.L[CCMP_PREVIEW_R] = 20000.0;
        red.L[CCMP_PREVIEW_G] = 0.0;
        red.L[CCMP_PREVIEW_B] = 0.0;
        bool f = false;
        const Yuv px = render_flat(r, make_frame(geom, red, params), f);
        check(px.v > 150 && px.u < 128, "a red patch renders red",
              "U=" + std::to_string(px.u) + " V=" + std::to_string(px.v));

        Patch blue;
        blue.L[CCMP_PREVIEW_R] = 0.0;
        blue.L[CCMP_PREVIEW_G] = 0.0;
        blue.L[CCMP_PREVIEW_B] = 20000.0;
        const Yuv pb = render_flat(r, make_frame(geom, blue, params), f);
        check(pb.u > 150 && pb.v < 128, "a blue patch renders blue",
              "U=" + std::to_string(pb.u) + " V=" + std::to_string(pb.v));
    }
    else
        check(false, "configure for the CFA check", err);
}

/* ── mono: no CFA ────────────────────────────────────────────────────────────
 *
 * The imx585 mono sensor presents R16/R12 to ccmpPreviewStage — same 12-in-16
 * container as the colour sensor's SBGGR16/12, but with no CFA. Faking a
 * Bayer pattern here would put the whole signal in one channel; the fix is a
 * quad-average into luminance instead, with white balance and the CCM forced
 * to identity because a mono tuning has neither measured. */
void run_mono()
{
    std::cout << "\nmono (no CFA)\n";

    CcmpParams params;
    if (!ccmp_params_for_binning(1.0, params))
    {
        check(false, "mono params for binning");
        return;
    }
    CcmpLut lut;
    std::string err;
    if (!lut.build(params, &err))
    {
        check(false, "mono lut.build", err);
        return;
    }

    CcmpPreviewGeometry geom = geometry_for(1928, 1090, 640, 360);
    geom.mono = true;
    CcmpPreviewRenderer r;
    if (!r.configure(geom, lut, &err))
    {
        check(false, "mono configure", err);
        return;
    }

    /* Skewed gains and a wildly non-identity CCM — exactly what a mono
     * tuning does NOT have. Gating setColour on geom.mono rather than on
     * these values is the point of the test: if it were gated on the gains
     * instead, this frame would still come out with a colour cast. */
    CcmpPreviewColour colour;
    colour.r_gain = 3.0;
    colour.b_gain = 0.4;
    colour.ccm[0] = 2.5;
    colour.ccm[4] = 0.3;
    colour.ccm[8] = 4.0;
    r.setColour(colour);

    /* Four DIFFERENT codes in one quad — a genuine per-photosite luminance
     * spread, which is what a mono sensor actually delivers (no two adjacent
     * photosites read identically). Averaging is the only correct treatment;
     * splitting them across three channels the way the CFA path does would
     * leave a cast. */
    const double levels[4] = { 200.0, 2000.0, 5000.0, 9000.0 };
    std::vector<uint8_t> raw(geom.raw_stride * geom.raw_height, 0);
    uint16_t code[4];
    for (int i = 0; i < 4; ++i)
        code[i] = static_cast<uint16_t>(store(levels[i], params) << geom.raw_shift);
    for (unsigned y = 0; y < geom.raw_height; ++y)
    {
        uint16_t *row = reinterpret_cast<uint16_t *>(raw.data() + static_cast<size_t>(y) * geom.raw_stride);
        for (unsigned x = 0; x < geom.raw_width; ++x)
            row[x] = code[(y & 1u) * 2 + (x & 1u)];
    }

    bool flat = false;
    const Yuv px = render_flat(r, raw, flat);
    check(flat, "mono quad renders flat");
    check(std::abs(px.u - 128) <= 1 && std::abs(px.v - 128) <= 1,
          "no CFA means no chroma, regardless of skewed gains/CCM",
          "U=" + std::to_string(px.u) + " V=" + std::to_string(px.v));

    /* Exposure is the one colour.* field mono still honours — it is not part
     * of the identity guard, only the gains and the CCM are. */
    CcmpPreviewColour bright = colour;
    bright.exposure = 2.0;
    r.setColour(bright);
    const Yuv px2 = render_flat(r, raw, flat);
    check(px2.y > px.y, "exposure still moves a mono render",
          "Y1=" + std::to_string(px.y) + " Y2=" + std::to_string(px2.y));
}

} // namespace

int main()
{
    std::cout << "ccmp_preview_test\n";

    run_mode(4.0, "2x2 binned 1928x1090", 1928, 1090);
    run_mode(1.0, "full res 3856x2180", 3856, 2180);
    run_common();
    run_mono();

    std::cout << "\n" << (g_failures ? "FAILED " : "PASSED ") << g_failures << " failure(s)\n";
    return g_failures ? 1 : 0;
}
