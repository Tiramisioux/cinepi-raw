#include "startup_gate.hpp"

#include <cassert>
int main()
{
    StartupGate gate;
    StartupGate::Config cfg;
    cfg.preroll_ms = 0;
    cfg.start_queue_frames = 2;
    cfg.ignore_start_frames = 2;
    gate.configure(cfg);

    gate.markEncoderReady();
    gate.markWriterReady();
    gate.markTakeReady(true);

    gate.arm();
    assert(gate.phase() == StartupGate::Phase::Armed);

    // Not enough frames yet
    assert(!gate.tryTransitionToRecording(1, StartupGate::Clock::now()));
    // Threshold reached
    assert(gate.tryTransitionToRecording(2, StartupGate::Clock::now()));
    assert(gate.phase() == StartupGate::Phase::Recording);

    // Cadence not active until first DNG + ignore window elapsed
    assert(!gate.cadenceActive());
    gate.noteFirstDngWritten();
    gate.noteFrameWritten();
    assert(!gate.cadenceActive());
    gate.noteFrameWritten();
    gate.noteFrameWritten();
    assert(gate.cadenceActive());

    gate.disarm();
    assert(gate.phase() == StartupGate::Phase::Idle);

    return 0;
}
