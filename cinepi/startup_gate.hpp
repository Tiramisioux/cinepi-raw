#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>

#include <spdlog/spdlog.h>

class StartupGate
{
public:
    using Clock = std::chrono::steady_clock;

    enum class Phase
    {
        Idle,
        Armed,
        Recording
    };

    struct Config
    {
        uint32_t preroll_ms { 300 };
        uint32_t start_queue_frames { 6 };
        uint32_t ignore_start_frames { 12 };
    };

    StartupGate();

    void setLogger(const std::shared_ptr<spdlog::logger> &logger);
    void configure(const Config &config);

    void markEncoderReady();
    void markWriterReady();
    void markTakeReady(bool ready);

    void arm();
    void disarm();

    bool tryTransitionToRecording(size_t queued_frames, Clock::time_point now);
    void noteFirstDngWritten();
    void noteFrameWritten();

    bool cadenceActive() const;
    bool shouldIgnoreFrame(uint64_t frame_index) const;

    Phase phase() const;

private:
    void logOnce(const char *marker);
    void maybeActivateCadenceLocked();

    mutable std::mutex mutex_;
    std::shared_ptr<spdlog::logger> logger_;

    Config config_{};
    Phase phase_ { Phase::Idle };

    bool encoder_ready_ { false };
    bool writer_ready_  { false };
    bool take_ready_    { false };
    bool first_dng_written_ { false };
    bool cadence_active_    { false };

    Clock::time_point arm_time_ {};
    uint64_t frames_written_ { 0 };

    bool logged_encoder_ready_ { false };
    bool logged_writer_ready_  { false };
};

