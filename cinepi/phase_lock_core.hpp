/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * phase_lock_core.hpp - pure, dependency-free core of the frame-rate phase lock.
 *
 * This is the control law only: no Redis, no libcamera, no threads. CinePIController
 * owns the I/O (reads fps_user / gains, pushes FrameDurationLimits, writes telemetry)
 * and delegates the per-frame decision to phaseLockStep() below, so the unit tests
 * exercise the *same* code that ships — the math cannot drift away from the binary.
 *
 * Behaviour is a line-for-line port of the original updatePhaseLock():
 *   - PI servo on accumulated phase error vs the operator's nominal fps, measured
 *     against the supplied reference clock (the Pi wall clock / FrameWallClock).
 *   - integer-us rounding of the requested duration is where the downstream
 *     integer-VBLANK quantisation becomes a first-order sigma-delta dither, so the
 *     *average* cadence is exact.
 *   - off when disabled, and suppressed on the --sync client (roleClient) so the
 *     dual-sensor genlock's rpi.sync owns that sensor's VBLANK.
 */
#pragma once

#include <cstdint>
#include <cmath>
#include <algorithm>

namespace cinepi {

struct PhaseLockParams {
    double kp        = 0.06;   // proportional gain (damping)
    double ki        = 0.0015; // integral gain (removes steady offset)
    double deadbandUs = 6.0;   // hold below this |phase error| (anti-jitter)
    double clampUs   = 150.0;  // never wander further than this from nominal
};

struct PhaseLockState {
    bool     active     = false; // lock currently running
    double   integral   = 0.0;   // integral accumulator (us)
    double   targetFps  = 0.0;   // nominal target (fps_user)
    double   baseDurUs  = 0.0;   // ideal period 1e6/target (us)
    double   reqDurUs   = 0.0;   // current requested duration (us, float)
    int64_t  t0Ns       = 0;     // reference ts at lock start
    int64_t  lastTsNs   = 0;     // last reference ts (gap detect)
    uint64_t frameCount = 0;     // frames since lock start
    long     lastDurUs  = -1;    // last duration pushed downstream
};

struct PhaseLockResult {
    bool   active       = false; // state.active after this step
    bool   releasedLock = false; // disabled / client / invalid target -> released
    bool   rearmed      = false; // this frame (re)armed the datum (no servo)
    bool   servoRan     = false; // reached the servo path (telemetry valid)
    bool   setControls  = false; // push FrameDurationLimits {durUs, durUs}
    long   durUs        = 0;     // requested duration (valid iff servoRan)
    double phaseErrUs   = 0.0;   // accumulated phase error (valid iff servoRan)
};

/*
 * One frame of the phase lock. Mutates `s`; returns the decision. Pure (no I/O),
 * so it is fully unit-testable and deterministic.
 *
 *   enabled      fps_phase_lock runtime flag
 *   roleClient   this instance is the --sync client (options_->sync == 2)
 *   isRecording  recording vs preview (changes gap handling / safety re-arm)
 *   targetFps    operator nominal fps (fps_user)
 *   refTsNs      per-frame reference timestamp in ns (FrameWallClock)
 */
inline PhaseLockResult phaseLockStep(PhaseLockState &s, const PhaseLockParams &p,
                                     bool enabled, bool roleClient, bool isRecording,
                                     double targetFps, int64_t refTsNs)
{
    PhaseLockResult r{};

    /* Disabled, or this instance is the --sync client: release the lock. The
     * client's VBLANK belongs to libcamera rpi.sync (relative A->B genlock); the
     * absolute Pi-clock discipline runs only on the master (--sync off or server). */
    if (!enabled || roleClient) {
        s.active = false;
        s.lastTsNs = 0;
        r.releasedLock = true;
        return r;
    }

    const double target = targetFps;
    if (target <= 1.0) {
        s.active = false;
        s.lastTsNs = 0;
        r.releasedLock = true;
        return r;
    }

    const int64_t dt = (s.active && s.lastTsNs != 0) ? (refTsNs - s.lastTsNs) : 0;
    s.lastTsNs = refTsNs;
    const double periodNs = 1.0e9 / target;

    /* >1.5 frame gap: PREVIEW -> reconfigure/stall (re-arm); RECORDING -> drop
     * burst (absorb the missed frames so phase doesn't spike). */
    const bool bigGap     = (s.active && dt > static_cast<int64_t>(1.5 * periodNs));
    const bool forceRearm = (bigGap && !isRecording);

    if (!s.active || std::abs(target - s.targetFps) > 1e-6 || forceRearm) {
        s.targetFps  = target;
        s.baseDurUs  = 1.0e6 / target;
        s.reqDurUs   = s.baseDurUs;
        s.integral   = 0.0;
        s.t0Ns       = refTsNs;
        s.frameCount = 0;
        s.lastDurUs  = -1;
        s.active     = true;
        r.active = true;
        r.rearmed = true;
        return r;
    }

    if (bigGap) {
        const uint64_t missed =
            static_cast<uint64_t>((static_cast<double>(dt) + 0.5 * periodNs) / periodNs);
        if (missed > 1)
            s.frameCount += (missed - 1);
    }

    s.frameCount++;
    const double targetPeriodNs = 1.0e9 / s.targetFps;
    const double idealNs   = static_cast<double>(s.frameCount) * targetPeriodNs;
    const double elapsedNs = static_cast<double>(refTsNs - s.t0Ns);
    /* phaseErr > 0 -> running slow/behind (shorten); < 0 -> fast/ahead (lengthen). */
    const double phaseErrUs = (elapsedNs - idealNs) * 1e-3;

    /* Preview-only safety re-arm: a >half-frame leak from a reconfigure burst that
     * slipped past the gap detector would take tens of seconds to bleed off — reset
     * the datum now. During RECORDING a large error is genuine drift; track it. */
    if (!isRecording && std::abs(phaseErrUs) > 0.5e6 / s.targetFps) {
        s.reqDurUs   = s.baseDurUs;
        s.integral   = 0.0;
        s.t0Ns       = refTsNs;
        s.frameCount = 0;
        s.lastDurUs  = -1;
        r.active = true;
        r.rearmed = true;
        return r;
    }

    /* PI servo: proportional term damps (pure integral is an undamped oscillator),
     * small integral removes the VBLANK-quantisation steady offset. Held inside the
     * deadband so a locked loop doesn't chatter. */
    if (std::abs(phaseErrUs) > p.deadbandUs) {
        s.integral += phaseErrUs;
        const double iClamp = p.clampUs / std::max(p.ki, 1e-9); /* anti-windup */
        s.integral = std::clamp(s.integral, -iClamp, iClamp);
        s.reqDurUs = s.baseDurUs - (p.kp * phaseErrUs + p.ki * s.integral);
        s.reqDurUs = std::clamp(s.reqDurUs, s.baseDurUs - p.clampUs, s.baseDurUs + p.clampUs);
    }

    /* Round to integer us: this is where the integer-VBLANK quantisation becomes the
     * sigma-delta dither, and it caps control traffic to actual line flips. */
    const long durUs = std::lround(s.reqDurUs);
    r.servoRan   = true;
    r.active     = true;
    r.phaseErrUs = phaseErrUs;
    r.durUs      = durUs;
    if (durUs != s.lastDurUs) {
        s.lastDurUs = durUs;
        r.setControls = true;
    }
    return r;
}

} // namespace cinepi
