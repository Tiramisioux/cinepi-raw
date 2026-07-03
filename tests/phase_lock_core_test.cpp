// SPDX-License-Identifier: BSD-2-Clause
//
// Unit tests for the frame-rate phase-lock core (cinepi/phase_lock_core.hpp).
//
// Pure / self-contained: no libcamera, no Redis. Build & run:
//   c++ -std=c++17 -O2 -I.. tests/phase_lock_core_test.cpp -o /tmp/pll_test && /tmp/pll_test
// (or via meson: `meson test phase_lock_core`).
//
// Covers the two behaviours this branch changed — the FrameWallClock reference
// (verified via epoch-magnitude invariance) and the --sync role gate — plus the
// servo properties the loop has always promised: convergence, sub-line dither,
// deadband hold, and drop/reconfigure gap handling.

#include "cinepi/phase_lock_core.hpp"

#include <cstdio>
#include <cmath>
#include <vector>
#include <string>
#include <cstdint>

using cinepi::PhaseLockState;
using cinepi::PhaseLockParams;
using cinepi::PhaseLockResult;
using cinepi::phaseLockStep;

// ── tiny test harness ───────────────────────────────────────────────────────
static int g_failures = 0;
static int g_checks   = 0;
#define CHECK(cond, msg)                                                        \
    do {                                                                        \
        ++g_checks;                                                             \
        if (!(cond)) {                                                          \
            ++g_failures;                                                       \
            std::printf("  FAIL: %s  (%s:%d)\n", (msg), __FILE__, __LINE__);    \
        }                                                                       \
    } while (0)

static bool approx(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

// ── sensor + VBLANK quantiser model ─────────────────────────────────────────
// Emulates the hardware the loop drives: the requested integer-us duration is
// snapped to an integer number of sensor lines (VBLANK quantisation), optionally
// plus a fixed crystal offset. This is what makes a single line count unable to
// hit the exact target, so the loop must dither to get the average right.
struct SensorModel {
    double lineUs;       // one VBLANK line in microseconds
    double biasPpm;      // fixed crystal offset (ppm) applied to the realised period
    // Realised frame period (ns) for a requested integer-us duration.
    int64_t realisedPeriodNs(long reqDurUs) const {
        double lines = std::lround(static_cast<double>(reqDurUs) / lineUs);
        double periodUs = lines * lineUs;
        periodUs *= (1.0 + biasPpm * 1e-6);
        return static_cast<int64_t>(std::llround(periodUs * 1000.0));
    }
};

// Drive the loop for `frames` frames against a sensor model, starting the
// reference clock at `baseNs`. Returns the realised-period and durUs/phaseErr
// trace. Models preview (isRecording=false) by default.
struct RunTrace {
    std::vector<long>    durUs;
    std::vector<double>  phaseErrUs;
    std::vector<int64_t> periodNs;
    int setControlsCount = 0;
};
static RunTrace runLoop(SensorModel sensor, double fps, int frames, int64_t baseNs,
                        bool isRecording = false,
                        PhaseLockParams params = PhaseLockParams{})
{
    PhaseLockState s;
    RunTrace tr;
    int64_t ref = baseNs;
    long curDur = static_cast<long>(std::lround(1.0e6 / fps)); // nominal to start
    for (int i = 0; i < frames; ++i) {
        PhaseLockResult r = phaseLockStep(s, params, /*enabled=*/true,
                                          /*roleClient=*/false, isRecording, fps, ref);
        if (r.setControls) ++tr.setControlsCount;
        if (r.servoRan) {
            curDur = r.durUs;
            tr.durUs.push_back(r.durUs);
            tr.phaseErrUs.push_back(r.phaseErrUs);
        }
        // advance the sensor by the realised period for the current request
        int64_t per = sensor.realisedPeriodNs(curDur);
        tr.periodNs.push_back(per);
        ref += per;
    }
    return tr;
}

// ── tests ───────────────────────────────────────────────────────────────────

static void test_role_gate_client_suppresses() {
    std::printf("test_role_gate_client_suppresses\n");
    PhaseLockState s; PhaseLockParams p;
    int64_t ref = 1778972127858905000LL; // epoch-scale
    bool everSet = false, everActive = false;
    for (int i = 0; i < 500; ++i) {
        PhaseLockResult r = phaseLockStep(s, p, /*enabled=*/true, /*roleClient=*/true,
                                          /*isRecording=*/(i % 2), 25.0, ref);
        everSet    |= r.setControls;
        everActive |= r.active;
        CHECK(r.releasedLock, "client role must always release the lock");
        ref += 40'000'000;
    }
    CHECK(!everSet,    "client role must never push FrameDurationLimits");
    CHECK(!everActive, "client role must never become active");
    CHECK(!s.active,   "state stays inactive under client role");
}

static void test_disabled_suppresses() {
    std::printf("test_disabled_suppresses\n");
    PhaseLockState s; PhaseLockParams p;
    int64_t ref = 0;
    for (int i = 0; i < 100; ++i) {
        PhaseLockResult r = phaseLockStep(s, p, /*enabled=*/false, false, false, 25.0, ref);
        CHECK(r.releasedLock && !r.setControls && !r.active, "disabled releases, no output");
        ref += 40'000'000;
    }
}

static void test_invalid_target_releases() {
    std::printf("test_invalid_target_releases\n");
    PhaseLockState s; PhaseLockParams p;
    PhaseLockResult r = phaseLockStep(s, p, true, false, false, /*target=*/1.0, 1000);
    CHECK(r.releasedLock && !r.active, "target<=1 releases");
    r = phaseLockStep(s, p, true, false, false, /*target=*/0.0, 2000);
    CHECK(r.releasedLock && !r.active, "target 0 releases");
}

static void test_first_frame_arms_no_output() {
    std::printf("test_first_frame_arms_no_output\n");
    PhaseLockState s; PhaseLockParams p;
    PhaseLockResult r = phaseLockStep(s, p, true, false, false, 24.0, 5000);
    CHECK(r.rearmed && r.active && !r.servoRan && !r.setControls,
          "first enabled frame arms the datum, no servo, no control push");
    CHECK(approx(s.baseDurUs, 1.0e6 / 24.0, 1e-6), "baseDur set to ideal period");
    CHECK(s.frameCount == 0, "frame count reset at arm");
}

// THE branch-critical test: switching the reference from the boot-relative
// SensorTimestamp to the epoch-scale FrameWallClock must not change any decision
// (the loop only uses int64 *differences*, then casts to double). If epoch-scale
// magnitudes lost precision or overflowed, the traces would diverge.
static void test_reference_magnitude_invariance() {
    std::printf("test_reference_magnitude_invariance\n");
    SensorModel sensor{ /*lineUs=*/11.0, /*biasPpm=*/350.0 };
    const double fps = 25.0;
    const int frames = 4000;
    RunTrace boot  = runLoop(sensor, fps, frames, /*baseNs=*/0LL);
    RunTrace epoch = runLoop(sensor, fps, frames, /*baseNs=*/1778972127858905000LL);
    CHECK(boot.durUs.size() == epoch.durUs.size(), "same number of servo frames");
    bool identical = boot.durUs.size() == epoch.durUs.size();
    if (identical) {
        for (size_t i = 0; i < boot.durUs.size(); ++i) {
            if (boot.durUs[i] != epoch.durUs[i] ||
                !approx(boot.phaseErrUs[i], epoch.phaseErrUs[i], 1e-6)) {
                identical = false;
                std::printf("  diverge at %zu: boot dur=%ld err=%.3f | epoch dur=%ld err=%.3f\n",
                            i, boot.durUs[i], boot.phaseErrUs[i],
                            epoch.durUs[i], epoch.phaseErrUs[i]);
                break;
            }
        }
    }
    CHECK(identical, "FrameWallClock (epoch) trace identical to boot-relative trace");
}

// Convergence + sub-line dither + average-exact: a sensor whose single-line
// quantisation cannot represent the exact target must still average to it.
static void test_convergence_and_dither_average_exact() {
    std::printf("test_convergence_and_dither_average_exact\n");
    // 24fps, line=11us -> ideal 41666.67us is NOT a multiple of 11 -> must dither.
    SensorModel sensor{ /*lineUs=*/11.0, /*biasPpm=*/473.0 };
    const double fps = 24.0;
    const int frames = 6000;
    RunTrace tr = runLoop(sensor, fps, frames, /*baseNs=*/1778972127858905000LL);

    // (a) bounded phase error after warmup
    double maxAbsAfter = 0.0;
    for (size_t i = 1000; i < tr.phaseErrUs.size(); ++i)
        maxAbsAfter = std::max(maxAbsAfter, std::fabs(tr.phaseErrUs[i]));
    CHECK(maxAbsAfter < 300.0, "phase error stays bounded (<300us) after convergence");

    // (b) average realised period equals the ideal to sub-line accuracy
    const double idealUs = 1.0e6 / fps;
    double sumUs = 0.0; int n = 0;
    for (size_t i = 2000; i < tr.periodNs.size(); ++i) { sumUs += tr.periodNs[i] / 1000.0; ++n; }
    double avgUs = sumUs / n;
    std::printf("  avg realised period = %.4f us (ideal %.4f, line 11.0)\n", avgUs, idealUs);
    CHECK(approx(avgUs, idealUs, 1.0), "dithered average within 1us of ideal (<< one 11us line)");

    // (c) dither actually present (>=2 distinct durations requested)
    long lo = tr.durUs[2000], hi = tr.durUs[2000];
    for (size_t i = 2000; i < tr.durUs.size(); ++i) { lo = std::min(lo, tr.durUs[i]); hi = std::max(hi, tr.durUs[i]); }
    CHECK(hi > lo, "loop dithers the requested duration (>=2 distinct values)");
}

// When already locked inside the deadband, the loop must hold (no integral walk,
// no control churn).
static void test_deadband_holds() {
    std::printf("test_deadband_holds\n");
    // A perfectly-representable target: line divides the ideal period exactly,
    // zero bias -> once locked, phase error stays ~0 and nothing should change.
    SensorModel sensor{ /*lineUs=*/10.0, /*biasPpm=*/0.0 }; // 40000us = 4000*10 exact
    const double fps = 25.0;
    RunTrace tr = runLoop(sensor, fps, 3000, /*baseNs=*/1778972127858905000LL);
    // count control pushes in the tail (should be ~0 once parked in the deadband)
    int tailPushes = 0;
    PhaseLockState s; PhaseLockParams p; int64_t ref = 1778972127858905000LL;
    long cur = 40000;
    for (int i = 0; i < 3000; ++i) {
        PhaseLockResult r = phaseLockStep(s, p, true, false, false, fps, ref);
        if (r.servoRan) cur = r.durUs;
        if (i > 1500 && r.setControls) ++tailPushes;
        ref += sensor.realisedPeriodNs(cur);
    }
    std::printf("  tail control pushes = %d\n", tailPushes);
    CHECK(tailPushes == 0, "locked-in-deadband loop does not churn FrameDurationLimits");
    (void)tr;
}

// Recording-side drop burst: a >1.5-frame gap must be absorbed (frame count
// jumps, NO re-arm), so the phase error doesn't spike.
static void test_recording_gap_absorbed_not_rearmed() {
    std::printf("test_recording_gap_absorbed_not_rearmed\n");
    PhaseLockState s; PhaseLockParams p;
    const double fps = 25.0; const int64_t per = 40'000'000;
    int64_t ref = 1778972127858905000LL;
    // arm + a few normal recording frames
    for (int i = 0; i < 50; ++i) { phaseLockStep(s, p, true, false, /*rec=*/true, fps, ref); ref += per; }
    uint64_t fcBefore = s.frameCount;
    // inject a 4-frame gap (3 dropped) while recording
    ref += 4 * per;
    PhaseLockResult r = phaseLockStep(s, p, true, false, /*rec=*/true, fps, ref);
    CHECK(!r.rearmed, "recording gap must NOT re-arm (it absorbs)");
    CHECK(r.servoRan, "recording gap still runs the servo");
    CHECK(s.frameCount >= fcBefore + 3, "absorbed the ~3 missed frames into the count");
    CHECK(std::fabs(r.phaseErrUs) < 200.0, "absorbed gap does not spike the phase error");
}

// Preview discontinuity (reconfigure/stall): a >1.5-frame gap must re-arm.
static void test_preview_gap_rearms() {
    std::printf("test_preview_gap_rearms\n");
    PhaseLockState s; PhaseLockParams p;
    const double fps = 25.0; const int64_t per = 40'000'000;
    int64_t ref = 1778972127858905000LL;
    for (int i = 0; i < 50; ++i) { phaseLockStep(s, p, true, false, /*rec=*/false, fps, ref); ref += per; }
    ref += 5 * per; // big preview gap
    PhaseLockResult r = phaseLockStep(s, p, true, false, /*rec=*/false, fps, ref);
    CHECK(r.rearmed && !r.servoRan, "preview gap re-arms the datum");
    CHECK(s.frameCount == 0, "datum reset on preview re-arm");
}

static void test_fps_change_rearms() {
    std::printf("test_fps_change_rearms\n");
    PhaseLockState s; PhaseLockParams p;
    int64_t ref = 1778972127858905000LL; const int64_t per = 40'000'000;
    for (int i = 0; i < 50; ++i) { phaseLockStep(s, p, true, false, false, 25.0, ref); ref += per; }
    PhaseLockResult r = phaseLockStep(s, p, true, false, false, /*new fps=*/24.0, ref);
    CHECK(r.rearmed, "an fps change re-arms");
    CHECK(approx(s.targetFps, 24.0, 1e-9), "target updated to new fps");
    CHECK(approx(s.baseDurUs, 1.0e6 / 24.0, 1e-6), "baseDur updated to new ideal period");
}

// Recording drift larger than half a frame must NOT be reset (it is tracked),
// and the requested duration stays clamped to base +/- clampUs.
static void test_recording_large_drift_tracked_and_clamped() {
    std::printf("test_recording_large_drift_tracked_and_clamped\n");
    PhaseLockParams p;
    PhaseLockState s;
    const double fps = 25.0; const double base = 1.0e6 / fps;
    int64_t ref = 1778972127858905000LL;
    // arm
    phaseLockStep(s, p, true, false, true, fps, ref);
    // Feed a sensor that runs slow by ~1000ppm so a real drift accumulates while
    // recording; ensure reqDur never leaves the clamp band and it never re-arms.
    long cur = static_cast<long>(std::lround(base));
    bool everRearmedAfterStart = false;
    for (int i = 0; i < 4000; ++i) {
        int64_t per = static_cast<int64_t>(std::llround(cur * (1.0 + 1000e-6) * 1000.0));
        ref += per;
        PhaseLockResult r = phaseLockStep(s, p, true, false, /*rec=*/true, fps, ref);
        if (r.rearmed) everRearmedAfterStart = true;
        if (r.servoRan) {
            cur = r.durUs;
            CHECK(r.durUs >= std::lround(base - p.clampUs) - 1 &&
                  r.durUs <= std::lround(base + p.clampUs) + 1,
                  "requested duration stays within the clamp band while recording");
        }
    }
    CHECK(!everRearmedAfterStart, "recording never re-arms on genuine drift");
}

int main() {
    std::printf("=== phase_lock_core unit tests ===\n");
    test_role_gate_client_suppresses();
    test_disabled_suppresses();
    test_invalid_target_releases();
    test_first_frame_arms_no_output();
    test_reference_magnitude_invariance();
    test_convergence_and_dither_average_exact();
    test_deadband_holds();
    test_recording_gap_absorbed_not_rearmed();
    test_preview_gap_rearms();
    test_fps_change_rearms();
    test_recording_large_drift_tracked_and_clamped();

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    if (g_failures == 0) std::printf("ALL TESTS PASSED\n");
    return g_failures == 0 ? 0 : 1;
}
