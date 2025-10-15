#include "startup_gate.hpp"

#include <utility>

StartupGate::StartupGate() = default;

void StartupGate::setLogger(const std::shared_ptr<spdlog::logger> &logger)
{
    std::lock_guard<std::mutex> lock(mutex_);
    logger_ = logger;
}

void StartupGate::configure(const Config &config)
{
    std::lock_guard<std::mutex> lock(mutex_);
    config_ = config;
}

void StartupGate::logOnce(const char *marker)
{
    if (!logger_)
        return;
    logger_->info(marker);
}

void StartupGate::markEncoderReady()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!logged_encoder_ready_)
    {
        logOnce("ENCODER_READY");
        logged_encoder_ready_ = true;
    }
    encoder_ready_ = true;
    maybeActivateCadenceLocked();
}

void StartupGate::markWriterReady()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!logged_writer_ready_)
    {
        logOnce("WRITER_READY");
        logged_writer_ready_ = true;
    }
    writer_ready_ = true;
    maybeActivateCadenceLocked();
}

void StartupGate::markTakeReady(bool ready)
{
    std::lock_guard<std::mutex> lock(mutex_);
    take_ready_ = ready;
    if (!ready)
    {
        first_dng_written_ = false;
        cadence_active_ = false;
        frames_written_ = 0;
    }
    maybeActivateCadenceLocked();
}

void StartupGate::arm()
{
    std::lock_guard<std::mutex> lock(mutex_);
    phase_ = Phase::Armed;
    arm_time_ = Clock::now();
    cadence_active_ = false;
    frames_written_ = 0;
    first_dng_written_ = false;
    logOnce("CADENCE_ARMED");
}

void StartupGate::disarm()
{
    std::lock_guard<std::mutex> lock(mutex_);
    phase_ = Phase::Idle;
    cadence_active_ = false;
    first_dng_written_ = false;
    frames_written_ = 0;
    take_ready_ = false;
}

bool StartupGate::tryTransitionToRecording(size_t queued_frames, Clock::time_point now)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (phase_ != Phase::Armed)
        return phase_ == Phase::Recording;

    const bool queue_ready = queued_frames >= config_.start_queue_frames;
    const bool time_ready = config_.preroll_ms == 0 ||
                            std::chrono::duration_cast<std::chrono::milliseconds>(now - arm_time_).count() >=
                                static_cast<long long>(config_.preroll_ms);

    if (take_ready_ && (queue_ready || time_ready))
    {
        phase_ = Phase::Recording;
        cadence_active_ = false;
        frames_written_ = 0;
        first_dng_written_ = false;
        return true;
    }

    return false;
}

void StartupGate::noteFirstDngWritten()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!first_dng_written_)
    {
        first_dng_written_ = true;
        logOnce("FIRST_DNG_WRITTEN");
    }
    maybeActivateCadenceLocked();
}

void StartupGate::noteFrameWritten()
{
    std::lock_guard<std::mutex> lock(mutex_);
    ++frames_written_;
    maybeActivateCadenceLocked();
}

bool StartupGate::cadenceActive() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return cadence_active_;
}

bool StartupGate::shouldIgnoreFrame(uint64_t frame_index) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return frame_index < config_.ignore_start_frames;
}

StartupGate::Phase StartupGate::phase() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return phase_;
}

void StartupGate::maybeActivateCadenceLocked()
{
    if (cadence_active_)
        return;

    if (phase_ != Phase::Recording)
        return;

    if (!encoder_ready_ || !writer_ready_ || !take_ready_ || !first_dng_written_)
        return;

    if (frames_written_ < config_.ignore_start_frames)
        return;

    cadence_active_ = true;
    logOnce("CADENCE_ACTIVE");
}
