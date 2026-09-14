/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * clip_convergence_test.cpp - the level-free clamp trigger.
 *
 * Build and run standalone, no meson needed:
 *   c++ -std=c++17 -Wall -Wextra -O1 -I. tests/clip_convergence_test.cpp \
 *       -o /tmp/clip_convergence_test && /tmp/clip_convergence_test
 *
 * THE POINT OF THIS TEST. Three shipped builds anchored this correction on a
 * raw code and all three were wrong, because the clamp code moves with gain,
 * blend and scene. The vectors below are the measurement that replaced it —
 * taken from three real takes, developed and probed per region — and the one
 * that matters is `genuine_white_at_the_clamp_level`: a real white subject
 * sitting at the SAME level as a clamped one. No anchor can separate those two.
 * If that check ever starts passing for the wrong reason, this rule has lost
 * the only thing it has over the anchor it replaced.
 */

#include "cinepi/clip_convergence.hpp"

#include <cmath>
#include <iostream>
#include <string>

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

std::string f2(float v)
{
    char b[32];
    std::snprintf(b, sizeof(b), "%.3f", v);
    return b;
}

/* r, g, b as the renderer sees them: linear above black, normalised. */
float blend(float r, float g, float b)
{
    ClipConvergence c;
    return clip_convergence_blend(r, g, b, c);
}

} // namespace

int main()
{
    std::cout << "clip_convergence_test\n\nmeasured on hardware\n";

    /* 12-bit HD, the lamp that forced this rule: R and G pinned on one code,
     * B never near the ceiling. 2nd/max = 1.000 at level 0.56. */
    {
        const float s = blend(0.564f, 0.564f, 0.446f);
        check(s >= 0.99f, "12-bit HD lamp: R and G pinned, B low -> fully corrected", "s=" + f2(s));
    }

    /* The orange surround of that same lamp, unclipped: 2nd/max 0.912 at 0.35.
     * This is the false positive that a min/max-over-four-samples rule and a
     * level threshold both let through. */
    {
        const float s = blend(0.352f, 0.321f, 0.122f);
        check(s == 0.f, "the orange surround of it is left alone", "s=" + f2(s));
    }

    /* 16-bit 4K sky, all three converged at 0.83. */
    {
        const float s = blend(0.827f, 0.829f, 0.827f);
        check(s >= 0.99f, "16-bit sky: all three converged -> fully corrected", "s=" + f2(s));
    }

    /* THE ONE AN ANCHOR CANNOT DO. A genuinely white subject measured at 0.74,
     * i.e. ABOVE the level the 16-bit clamp sat at in the same frame (0.83 is
     * the clamp, this white is real data below it and a level rule catches
     * both). 2nd/max 0.659 says it is real. */
    {
        const float s = blend(0.740f, 0.488f, 0.462f);
        check(s == 0.f, "genuine_white_at_the_clamp_level is NOT touched", "s=" + f2(s));
    }

    std::cout << "\nthe two ends the rule must not fire on\n";

    /* Black: every channel agrees with every other, which is why the level gate
     * exists at all. */
    check(blend(0.010f, 0.011f, 0.010f) == 0.f, "a dark neutral quad is not converged-bright");
    check(blend(0.0f, 0.0f, 0.0f) == 0.f, "pure black returns 0 rather than dividing by it");

    /* A neutral subject under the shipping AWB gains arrives with its channels
     * far apart -- R = G/1.8, B = G/1.7 -- which is the whole reason
     * convergence means something. */
    {
        const float g = 0.80f;
        const float s = blend(g / 1.8f, g, g / 1.7f);
        check(s == 0.f, "a bright NEUTRAL subject under the AWB gains is left alone", "s=" + f2(s));
    }

    /* A saturated colour that never clipped: the trap desaturateHighlight()'s
     * own comment warns about, from the other direction. */
    check(blend(0.79f, 0.05f, 0.03f) == 0.f, "a bright saturated red is left alone");

    std::cout << "\nthe ramp\n";

    /* Half way up the ratio ramp, well above the level gate. */
    {
        const float mx = 0.60f;
        const float s = blend(mx, mx * 0.9885f, 0.1f);
        check(std::fabs(s - 0.5f) < 0.02f, "mid-ramp ratio gives a half blend", "s=" + f2(s));
    }
    /* Just under the ratio floor: nothing, however bright. */
    check(blend(1.0f, 0.969f, 0.2f) == 0.f, "a ratio just under the floor gives exactly 0");

    /* The level ramp, at a ratio that would otherwise fire fully. */
    {
        const float s = blend(0.45f, 0.45f, 0.05f);
        check(s > 0.4f && s < 0.6f, "mid-level ramp halves the blend too", "s=" + f2(s));
    }

    /* The mid-tones are where a per-pixel rule goes wrong: an ordinary colour
     * can have its top two channels close, and one noisy pixel then fires on
     * its own. The dimmest clamp measured is 0.56, so the gate sits far above
     * anything here. */
    check(blend(0.30f, 0.30f, 0.10f) == 0.f, "a converged MID-TONE is below the gate and ignored");
    check(blend(0.39f, 0.39f, 0.10f) == 0.f, "and so is one just under it");

    /* The A/B against every build before this rule existed. */
    {
        ClipConvergence off;
        off.enabled = false;
        check(clip_convergence_blend(0.564f, 0.564f, 0.446f, off) == 0.f,
              "disabled leaves the anchor alone as the only trigger");
    }

    /* Order must not matter: the rule is about the top two, whichever they are. */
    {
        const float a = blend(0.564f, 0.564f, 0.446f);
        const float b = blend(0.446f, 0.564f, 0.564f);
        const float c = blend(0.564f, 0.446f, 0.564f);
        check(a == b && b == c, "any two channels may be the pinned pair",
              "R+G " + f2(a) + "  G+B " + f2(b) + "  R+B " + f2(c));
    }

    std::cout << "\none pixel is never a clamp\n";

    /* The 2x2 block rule, which is what takes the isolated count to zero. A
     * lone fired quad among unfired neighbours must come out at 0; a block
     * where all four fire keeps its blend. */
    check(clip_convergence_block(1.f, 0.f, 0.f, 0.f) == 0.f, "a lone fired quad in its block is suppressed");
    check(clip_convergence_block(1.f, 1.f, 1.f, 1.f) == 1.f, "a block that agrees keeps the full blend");
    check(clip_convergence_block(1.f, 0.8f, 0.9f, 1.f) == 0.8f, "a partly-agreeing block takes the weakest");

    std::cout << "\n" << (g_failures ? "FAILED " : "PASSED ") << g_failures << " failure(s)\n";
    return g_failures ? 1 : 0;
}
