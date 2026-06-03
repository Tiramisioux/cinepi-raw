#include <alsa/asoundlib.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cerrno>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <optional>
#include <queue>
#include <sched.h>
#include <sys/prctl.h>
#include <unistd.h>
#include <string>
#include <thread>
#include <vector>

namespace {

std::atomic<bool> stopRequested{false};

void handleSignal(int)
{
    stopRequested.store(true);
}

struct Options
{
    std::string device;
    std::string format;
    std::string monitorOutput;
    std::string output;
    unsigned int channels = 0;
    unsigned int rate = 0;
    bool discardOutput = false;
};

struct FormatInfo
{
    snd_pcm_format_t alsaFormat = SND_PCM_FORMAT_UNKNOWN;
    unsigned int bitsPerSample = 0;
    unsigned int bytesPerSample = 0;
};

std::optional<FormatInfo> parseFormat(const std::string &format)
{
    if (format == "S16_LE")
        return FormatInfo{SND_PCM_FORMAT_S16_LE, 16, 2};
    if (format == "S24_3LE")
        return FormatInfo{SND_PCM_FORMAT_S24_3LE, 24, 3};
    return std::nullopt;
}

bool parseArgs(int argc, char **argv, Options &options)
{
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        auto requireValue = [&](const std::string &name) -> const char * {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for " << name << '\n';
                std::exit(2);
            }
            return argv[++i];
        };

        if (arg == "--device")
            options.device = requireValue(arg);
        else if (arg == "--format")
            options.format = requireValue(arg);
        else if (arg == "--channels")
            options.channels = static_cast<unsigned int>(std::stoul(requireValue(arg)));
        else if (arg == "--rate")
            options.rate = static_cast<unsigned int>(std::stoul(requireValue(arg)));
        else if (arg == "--monitor-output")
            options.monitorOutput = requireValue(arg);
        else if (arg == "--output")
            options.output = requireValue(arg);
        else if (arg == "--discard-output")
            options.discardOutput = true;
        else {
            std::cerr << "Unknown argument: " << arg << '\n';
            return false;
        }
    }

    return !options.device.empty() &&
           !options.format.empty() &&
           (!options.output.empty() || options.discardOutput) &&
           options.channels > 0 &&
           options.rate > 0;
}

timespec currentClock(clockid_t clockId)
{
    timespec ts{};
    clock_gettime(clockId, &ts);
    return ts;
}

void emitTimestamp(const char *tag, const timespec &ts)
{
    std::cout << '<' << tag << ':' << ts.tv_sec << '.'
              << std::setw(9) << std::setfill('0') << ts.tv_nsec << ">\n";
}

void writeLe16(std::ostream &stream, uint16_t value)
{
    const char bytes[2] = {
        static_cast<char>(value & 0xff),
        static_cast<char>((value >> 8) & 0xff),
    };
    stream.write(bytes, sizeof(bytes));
}

void writeLe32(std::ostream &stream, uint32_t value)
{
    const char bytes[4] = {
        static_cast<char>(value & 0xff),
        static_cast<char>((value >> 8) & 0xff),
        static_cast<char>((value >> 16) & 0xff),
        static_cast<char>((value >> 24) & 0xff),
    };
    stream.write(bytes, sizeof(bytes));
}

void writeWaveHeader(std::fstream &stream,
                     unsigned int channels,
                     unsigned int rate,
                     unsigned int bitsPerSample,
                     uint32_t dataBytes)
{
    const uint16_t blockAlign =
        static_cast<uint16_t>(channels * ((bitsPerSample + 7) / 8));
    const uint32_t byteRate = rate * static_cast<uint32_t>(blockAlign);
    const uint32_t riffSize = 36 + dataBytes;

    stream.seekp(0, std::ios::beg);
    stream.write("RIFF", 4);
    writeLe32(stream, riffSize);
    stream.write("WAVE", 4);
    stream.write("fmt ", 4);
    writeLe32(stream, 16);
    writeLe16(stream, 1);
    writeLe16(stream, static_cast<uint16_t>(channels));
    writeLe32(stream, rate);
    writeLe32(stream, byteRate);
    writeLe16(stream, blockAlign);
    writeLe16(stream, static_cast<uint16_t>(bitsPerSample));
    stream.write("data", 4);
    writeLe32(stream, dataBytes);
}

int32_t readSample24(const uint8_t *ptr)
{
    int32_t value = static_cast<int32_t>(ptr[0]) |
                    (static_cast<int32_t>(ptr[1]) << 8) |
                    (static_cast<int32_t>(ptr[2]) << 16);
    if (value & 0x00800000)
        value |= ~0x00ffffff;
    return value;
}

int vuPercentFromPeak(uint64_t peak, uint64_t maxPeak)
{
    if (maxPeak == 0)
        return 0;
    const double normalized = static_cast<double>(peak) / static_cast<double>(maxPeak);
    return std::clamp(static_cast<int>(std::lround(normalized * 100.0)), 0, 100);
}

void emitVu(const uint8_t *buffer,
            snd_pcm_sframes_t framesRead,
            unsigned int channels,
            const FormatInfo &formatInfo)
{
    if (framesRead <= 0 || channels == 0)
        return;

    std::vector<uint64_t> peaks(channels, 0);
    const unsigned int frameBytes = channels * formatInfo.bytesPerSample;

    for (snd_pcm_sframes_t frame = 0; frame < framesRead; ++frame) {
        const uint8_t *framePtr = buffer + (static_cast<size_t>(frame) * frameBytes);
        for (unsigned int ch = 0; ch < channels; ++ch) {
            const uint8_t *samplePtr = framePtr + (ch * formatInfo.bytesPerSample);
            int32_t sample = 0;
            if (formatInfo.alsaFormat == SND_PCM_FORMAT_S16_LE) {
                int16_t s16 = 0;
                std::memcpy(&s16, samplePtr, sizeof(s16));
                sample = s16;
            } else if (formatInfo.alsaFormat == SND_PCM_FORMAT_S24_3LE) {
                sample = readSample24(samplePtr);
            }

            const uint64_t magnitude =
                static_cast<uint64_t>(std::abs(static_cast<long long>(sample)));
            peaks[ch] = std::max(peaks[ch], magnitude);
        }
    }

    const uint64_t maxPeak =
        (formatInfo.alsaFormat == SND_PCM_FORMAT_S24_3LE) ? 0x7fffffULL : 0x7fffULL;

    const int left = vuPercentFromPeak(peaks[0], maxPeak);
    const int right = vuPercentFromPeak(peaks[std::min<unsigned int>(1, channels - 1)], maxPeak);
    std::cout << "<VU:" << left << '|' << right << '|' << left << '|' << right << ">\n";
}

int recoverCaptureError(snd_pcm_t *handle, int err)
{
    if (err == -EPIPE)
        return snd_pcm_prepare(handle);
    if (err == -ESTRPIPE) {
        while ((err = snd_pcm_resume(handle)) == -EAGAIN)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        if (err < 0)
            return snd_pcm_prepare(handle);
        return 0;
    }
    return err;
}

// Raise the capture thread to SCHED_FIFO and pin it to the last CPU core so
// DNG-writer threads never share that core. On Pi 4/5 the NVMe SSD and USB
// mic share the same xHCI USB controller; a 4K DNG burst causes USB interrupt
// latency that stalls ALSA reads even with SCHED_FIFO if the capture thread
// is competing for CPU time with DNG workers. Pinning audio to an exclusive
// core (the last one) eliminates that competition: the DNG-writer affinity
// (set from cinemate via --disk-affinity / --encode-affinity) avoids that same
// core, so the audio interrupt is always serviced immediately.
//
// Priority defaults to 80 — well above DNG workers (SCHED_NORMAL) and below
// kernel RT threads (~99). Overridable via CINEPI_AUDIO_RT_PRIORITY env var.
void tryElevateRealtimePriority()
{
    // ── SCHED_FIFO priority ───────────────────────────────────────────────
    int priority = 80;
    if (const char *rtPriorityEnv = std::getenv("CINEPI_AUDIO_RT_PRIORITY")) {
        try {
            priority = std::stoi(rtPriorityEnv);
        } catch (...) {
            std::cerr << "Ignoring invalid CINEPI_AUDIO_RT_PRIORITY value\n";
        }
    }

    const int minPriority = sched_get_priority_min(SCHED_FIFO);
    const int maxPriority = sched_get_priority_max(SCHED_FIFO);
    if (minPriority < 0 || maxPriority < 0) {
        std::cerr << "SCHED_FIFO priority range unavailable; leaving capture at default scheduling\n";
        return;
    }
    priority = std::clamp(priority, minPriority, maxPriority);

    sched_param param{};
    param.sched_priority = priority;
    if (sched_setscheduler(0, SCHED_FIFO, &param) == 0) {
        std::cerr << "Capture thread elevated to SCHED_FIFO priority " << priority << "\n";
    } else {
        const int savedErrno = errno;
        std::cerr << "Could not set SCHED_FIFO capture priority (" << std::strerror(savedErrno)
                  << "); continuing at default scheduling. Grant CAP_SYS_NICE or raise the"
                     " rtprio ulimit for xrun-resistant capture.\n";
    }

    // ── CPU isolation: pin to last core ──────────────────────────────────
    // Keeps the capture thread on a core the DNG writers are told to avoid
    // (via --disk-affinity / --encode-affinity passed from cinemate), so
    // USB audio interrupts are always dispatched to an uncontested CPU.
    const long nCpus = sysconf(_SC_NPROCESSORS_ONLN);
    if (nCpus > 1) {
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        const int audioCpu = static_cast<int>(nCpus) - 1;
        CPU_SET(audioCpu, &cpuset);
        if (sched_setaffinity(0, sizeof(cpuset), &cpuset) == 0) {
            std::cerr << "Capture thread pinned to CPU " << audioCpu
                      << " (of " << nCpus << " available)\n";
        } else {
            std::cerr << "Could not pin capture thread to CPU " << audioCpu
                      << "; running on any core\n";
        }
    }
}

} // namespace

bool configurePlaybackPcm(snd_pcm_t *pcm,
                          snd_pcm_format_t format,
                          unsigned int channels,
                          unsigned int rate)
{
    snd_pcm_hw_params_t *hw = nullptr;
    snd_pcm_hw_params_alloca(&hw);

    int err = 0;
    if ((err = snd_pcm_hw_params_any(pcm, hw)) < 0 ||
        (err = snd_pcm_hw_params_set_access(pcm, hw, SND_PCM_ACCESS_RW_INTERLEAVED)) < 0 ||
        (err = snd_pcm_hw_params_set_format(pcm, hw, format)) < 0 ||
        (err = snd_pcm_hw_params_set_channels(pcm, hw, channels)) < 0) {
        std::cerr << "Failed to configure monitor playback parameters: "
                  << snd_strerror(err) << '\n';
        return false;
    }

    int dir = 0;
    unsigned int requestedRate = rate;
    if ((err = snd_pcm_hw_params_set_rate_near(pcm, hw, &requestedRate, &dir)) < 0) {
        std::cerr << "Failed to set monitor playback rate: " << snd_strerror(err) << '\n';
        return false;
    }

    unsigned int periodTimeUs = 10000;
    unsigned int bufferTimeUs = 40000;
    snd_pcm_hw_params_set_period_time_near(pcm, hw, &periodTimeUs, &dir);
    snd_pcm_hw_params_set_buffer_time_near(pcm, hw, &bufferTimeUs, &dir);

    if ((err = snd_pcm_hw_params(pcm, hw)) < 0) {
        std::cerr << "Failed to apply monitor playback parameters: "
                  << snd_strerror(err) << '\n';
        return false;
    }

    if ((err = snd_pcm_prepare(pcm)) < 0) {
        std::cerr << "snd_pcm_prepare for monitor output failed: "
                  << snd_strerror(err) << '\n';
        return false;
    }

    return true;
}

void upmixMonoToStereo(const uint8_t *input,
                       snd_pcm_sframes_t framesRead,
                       unsigned int bytesPerSample,
                       std::vector<uint8_t> &output)
{
    output.resize(static_cast<size_t>(framesRead) * bytesPerSample * 2);
    for (snd_pcm_sframes_t frame = 0; frame < framesRead; ++frame) {
        const uint8_t *src = input + (static_cast<size_t>(frame) * bytesPerSample);
        uint8_t *dst = output.data() + (static_cast<size_t>(frame) * bytesPerSample * 2);
        std::memcpy(dst, src, bytesPerSample);
        std::memcpy(dst + bytesPerSample, src, bytesPerSample);
    }
}

bool writeMonitorFrames(snd_pcm_t *pcm,
                        const uint8_t *buffer,
                        snd_pcm_sframes_t framesRead,
                        unsigned int captureChannels,
                        const FormatInfo &formatInfo,
                        std::vector<uint8_t> &scratch)
{
    const uint8_t *playbackBytes = buffer;
    unsigned int playbackChannels = captureChannels;
    if (captureChannels == 1) {
        upmixMonoToStereo(buffer, framesRead, formatInfo.bytesPerSample, scratch);
        playbackBytes = scratch.data();
        playbackChannels = 2;
    }

    snd_pcm_sframes_t written = 0;
    while (written < framesRead) {
        const uint8_t *chunk = playbackBytes +
            (static_cast<size_t>(written) * playbackChannels * formatInfo.bytesPerSample);
        snd_pcm_sframes_t rc = snd_pcm_writei(pcm, chunk, framesRead - written);
        if (rc < 0) {
            int err = recoverCaptureError(pcm, static_cast<int>(rc));
            if (err < 0) {
                std::cerr << "Monitor playback write failed: " << snd_strerror(err) << '\n';
                return false;
            }
            continue;
        }
        written += rc;
    }

    return true;
}

int main(int argc, char **argv)
{
    std::setvbuf(stdout, nullptr, _IOLBF, 0);

#ifdef PR_SET_PDEATHSIG
    prctl(PR_SET_PDEATHSIG, SIGHUP);
    if (getppid() == 1)
        return 1;
#endif

    Options options;
    if (!parseArgs(argc, argv, options)) {
        std::cerr << "Usage: cinepi-audio-capture --device <name> --format <S16_LE|S24_3LE>"
                  << " --channels <n> --rate <hz> [--monitor-output <alsa>] "
                  << "[--output <wav> | --discard-output]\n";
        return 2;
    }

    const auto formatInfoOpt = parseFormat(options.format);
    if (!formatInfoOpt) {
        std::cerr << "Unsupported audio format: " << options.format << '\n';
        return 2;
    }
    const FormatInfo formatInfo = *formatInfoOpt;

    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    std::fstream output;
    if (!options.discardOutput) {
        std::filesystem::path outputPath(options.output);
        output.open(outputPath, std::ios::binary | std::ios::out | std::ios::trunc);
        if (!output.is_open()) {
            std::cerr << "Failed to open output WAV: " << options.output << '\n';
            return 1;
        }
    }

    snd_pcm_t *pcm = nullptr;
    int err = snd_pcm_open(&pcm, options.device.c_str(), SND_PCM_STREAM_CAPTURE, 0);
    if (err < 0) {
        std::cerr << "snd_pcm_open failed: " << snd_strerror(err) << '\n';
        return 1;
    }

    snd_pcm_hw_params_t *hw = nullptr;
    snd_pcm_hw_params_alloca(&hw);
    if ((err = snd_pcm_hw_params_any(pcm, hw)) < 0 ||
        (err = snd_pcm_hw_params_set_access(pcm, hw, SND_PCM_ACCESS_RW_INTERLEAVED)) < 0 ||
        (err = snd_pcm_hw_params_set_format(pcm, hw, formatInfo.alsaFormat)) < 0 ||
        (err = snd_pcm_hw_params_set_channels(pcm, hw, options.channels)) < 0) {
        std::cerr << "Failed to configure ALSA capture parameters: " << snd_strerror(err) << '\n';
        snd_pcm_close(pcm);
        return 1;
    }

    unsigned int rate = options.rate;
    int dir = 0;
    if ((err = snd_pcm_hw_params_set_rate_near(pcm, hw, &rate, &dir)) < 0) {
        std::cerr << "Failed to set capture rate: " << snd_strerror(err) << '\n';
        snd_pcm_close(pcm);
        return 1;
    }

    if (!options.discardOutput)
        writeWaveHeader(output, options.channels, rate, formatInfo.bitsPerSample, 0);

    // The idle monitor needs low latency for live VU/HDMI, but the record path has
    // no live monitor — give it a large ring buffer so the capture thread can ride
    // out DNG-writer storage stalls without overrunning. An overrun discards the
    // ALSA ring (dropped samples = silent holes in the WAV). 1 s of headroom covers
    // the multi-hundred-ms stalls seen under 4K NVMe write load while keeping the
    // end-of-take drain grace at its 1 s floor (a larger buffer would lengthen the
    // post-stop drain). The wall-clock reconciliation backstops any rarer >1 s stall.
    unsigned int periodTimeUs = 10000;
    unsigned int bufferTimeUs = options.discardOutput ? 40000u : 1000000u;
    snd_pcm_hw_params_set_period_time_near(pcm, hw, &periodTimeUs, &dir);
    snd_pcm_hw_params_set_buffer_time_near(pcm, hw, &bufferTimeUs, &dir);

    if ((err = snd_pcm_hw_params(pcm, hw)) < 0) {
        std::cerr << "Failed to apply ALSA capture parameters: " << snd_strerror(err) << '\n';
        snd_pcm_close(pcm);
        return 1;
    }

    {
        snd_pcm_uframes_t negotiatedBuffer = 0;
        if (snd_pcm_hw_params_get_buffer_size(hw, &negotiatedBuffer) == 0) {
            const unsigned long long bufMs =
                static_cast<unsigned long long>(negotiatedBuffer) * 1000ULL /
                (rate > 0 ? rate : 48000);
            std::cerr << "Capture ring buffer: " << negotiatedBuffer << " frames (~"
                      << bufMs << " ms)"
                      << (options.discardOutput ? " [idle monitor]" : " [record]")
                      << '\n';
        }
    }

    snd_pcm_t *monitorPcm = nullptr;
    std::vector<uint8_t> playbackScratch;
    if (!options.monitorOutput.empty()) {
        err = snd_pcm_open(&monitorPcm, options.monitorOutput.c_str(), SND_PCM_STREAM_PLAYBACK, 0);
        if (err < 0) {
            std::cerr << "snd_pcm_open monitor output failed: " << snd_strerror(err)
                      << " (continuing without live monitor output)\n";
            monitorPcm = nullptr;
        }

        if (monitorPcm) {
            const unsigned int playbackChannels = (options.channels == 1) ? 2 : options.channels;
            if (!configurePlaybackPcm(monitorPcm, formatInfo.alsaFormat, playbackChannels, rate)) {
                snd_pcm_close(monitorPcm);
                monitorPcm = nullptr;
                std::cerr << "Monitor output setup failed; continuing without live monitor output\n";
            }
        }
    }

    snd_pcm_uframes_t periodFrames = 0;
    snd_pcm_hw_params_get_period_size(hw, &periodFrames, &dir);
    if (periodFrames == 0)
        periodFrames = 256;

    if ((err = snd_pcm_prepare(pcm)) < 0) {
        std::cerr << "snd_pcm_prepare failed: " << snd_strerror(err) << '\n';
        if (monitorPcm)
            snd_pcm_close(monitorPcm);
        snd_pcm_close(pcm);
        return 1;
    }

    if ((err = snd_pcm_start(pcm)) < 0) {
        std::cerr << "snd_pcm_start failed: " << snd_strerror(err) << '\n';
        if (monitorPcm)
            snd_pcm_close(monitorPcm);
        snd_pcm_close(pcm);
        return 1;
    }

    // Keep the ALSA read loop scheduled ahead of DNG-writer storage I/O so xruns
    // (and the drift they cause) do not accumulate under heavy 4K / exFAT load.
    if (!options.discardOutput)
        tryElevateRealtimePriority();

    const timespec triggerMono = currentClock(CLOCK_MONOTONIC);
    const timespec triggerReal = currentClock(CLOCK_REALTIME);

    std::cout << "<NEGOTIATED_SAMPLE_RATE: " << rate << ">\n";
    emitTimestamp("TS_START", triggerMono);
    emitTimestamp("TS_FIRST_BUFFER_B", triggerMono);
    emitTimestamp("TS_AUDIO_START_REALTIME", triggerReal);

    const unsigned int frameBytes = options.channels * formatInfo.bytesPerSample;
    std::vector<uint8_t> buffer(static_cast<size_t>(periodFrames) * frameBytes);

    uint64_t framesCaptured = 0;
    bool emittedFirstBufferAfter = false;
    uint64_t dataBytes = 0;

    // Decouple capture from disk I/O so exFAT write stalls (which block write()
    // calls for hundreds of ms under 4K DNG-writer pressure) never stall the ALSA
    // read loop and cause sample loss. The capture thread pushes audio into a RAM
    // queue; the writer thread drains it to disk independently.
    struct WriteQueue {
        std::queue<std::vector<uint8_t>> items;
        std::mutex mtx;
        std::condition_variable cv;
        std::atomic<bool> done{false};
        std::atomic<bool> error{false};
    };
    WriteQueue wq;

    std::thread writerThread;
    if (!options.discardOutput) {
        writerThread = std::thread([&]() {
            while (true) {
                std::unique_lock<std::mutex> lock(wq.mtx);
                wq.cv.wait(lock, [&] { return !wq.items.empty() || wq.done.load(); });
                while (!wq.items.empty()) {
                    auto buf = std::move(wq.items.front());
                    wq.items.pop();
                    lock.unlock();
                    output.write(reinterpret_cast<const char *>(buf.data()),
                                 static_cast<std::streamsize>(buf.size()));
                    if (!output.good()) {
                        wq.error.store(true);
                        std::cerr << "WAV writer: disk write failed\n";
                    }
                    lock.lock();
                }
                if (wq.done.load())
                    break;
            }
        });
    }

    // Push an audio chunk to the writer thread. Caller updates framesCaptured /
    // dataBytes so the wall-clock reconciliation stays accurate regardless of how
    // much data the writer thread has actually flushed to disk.
    auto enqueueAudio = [&](const uint8_t *data, size_t bytes) {
        {
            std::lock_guard<std::mutex> lock(wq.mtx);
            wq.items.emplace(data, data + bytes);
        }
        wq.cv.notify_one();
    };

    // Running wall-clock anchor. Each successful read reconciles total frames
    // written against elapsed real time and pads any shortfall with silence, so the
    // WAV stays locked to wall-clock length whether samples were lost to an ALSA
    // xrun (-EPIPE) OR silently under-delivered by the USB device (no error raised
    // at all — invisible to a -EPIPE-gated fill). Anchor is set on the first read.
    std::optional<std::chrono::steady_clock::time_point> captureAnchorMono;
    const long long maxGapFrames = static_cast<long long>(rate) * 5;
    // Pad only past one period of shortfall so normal scheduling jitter never
    // injects silence; sub-period losses accumulate against the fixed anchor and
    // get padded once they cross the threshold, so nothing leaks permanently.
    const long long reconcileToleranceFrames = static_cast<long long>(periodFrames);

    while (true) {
        const snd_pcm_sframes_t framesToRead = static_cast<snd_pcm_sframes_t>(periodFrames);

        snd_pcm_sframes_t framesRead =
            snd_pcm_readi(pcm, buffer.data(), framesToRead);
        if (framesRead == -EINTR && stopRequested.load()) {
            // Stop immediately on both paths. The writer thread flushes any queued
            // audio, so the ALSA-ring drain is not needed. Draining kept reading
            // post-stop ambient mic audio for up to ~1 s (the drain grace), which
            // added a visible tail to the WAV and made the final clap appear late
            // in the NLE.
            break;
        }
        if (framesRead < 0) {
            err = recoverCaptureError(pcm, static_cast<int>(framesRead));
            if (err < 0) {
                std::cerr << "Capture read failed: " << snd_strerror(err) << '\n';
                break;
            }
            // Dropped samples (xrun). The running wall-clock reconciliation on the
            // next good read pads the gap, keeping the WAV aligned to real time.
            continue;
        }

        if (!emittedFirstBufferAfter) {
            emitTimestamp("TS_FIRST_BUFFER_A", currentClock(CLOCK_MONOTONIC));
            emittedFirstBufferAfter = true;
        }

        // Reconcile against the running wall-clock anchor BEFORE writing this read's
        // audio, so inserted silence lands in the gap that opened *before* these
        // samples. In steady state expected ≈ written and nothing is padded; when
        // frames went missing (xrun OR silent under-delivery) expected outruns
        // written and we insert exactly the shortfall.
        const auto nowMono = std::chrono::steady_clock::now();
        if (!captureAnchorMono)
            captureAnchorMono = nowMono;
        if (!options.discardOutput) {
            const double elapsedSeconds =
                std::chrono::duration<double>(nowMono - *captureAnchorMono).count();
            const long long shortfallFrames =
                std::llround(elapsedSeconds * static_cast<double>(rate)) -
                static_cast<long long>(framesCaptured);
            if (shortfallFrames > reconcileToleranceFrames) {
                const long long gapFrames = std::min(shortfallFrames, maxGapFrames);
                const size_t silenceBytes = static_cast<size_t>(gapFrames) * frameBytes;
                const std::vector<uint8_t> silence(silenceBytes, 0);
                enqueueAudio(silence.data(), silenceBytes);
                dataBytes += static_cast<uint64_t>(silenceBytes);
                framesCaptured += static_cast<uint64_t>(gapFrames);
                std::cerr << "Inserted " << gapFrames
                          << " silent frame(s) to cover a capture shortfall of "
                          << std::fixed << std::setprecision(3)
                          << (static_cast<double>(gapFrames) / static_cast<double>(rate))
                          << "s; WAV stays aligned to wall clock\n";
            }
        }

        const size_t bytesRead = static_cast<size_t>(framesRead) * frameBytes;
        if (!options.discardOutput) {
            if (wq.error.load()) {
                std::cerr << "Stopping capture after WAV writer disk error\n";
                break;
            }
            enqueueAudio(buffer.data(), bytesRead);
        }

        dataBytes += static_cast<uint64_t>(bytesRead);
        framesCaptured += static_cast<uint64_t>(framesRead);
        emitVu(buffer.data(), framesRead, options.channels, formatInfo);

        if (monitorPcm &&
            !writeMonitorFrames(monitorPcm,
                                buffer.data(),
                                framesRead,
                                options.channels,
                                formatInfo,
                                playbackScratch)) {
            snd_pcm_drop(monitorPcm);
            snd_pcm_close(monitorPcm);
            monitorPcm = nullptr;
            std::cerr << "Disabling live monitor output after playback failure; VU capture continues\n";
        }

        if (stopRequested.load()) {
            break; // Writer thread handles flush; no ALSA drain needed.
        }
    }

    // Drain the writer queue before touching the output stream again.
    if (!options.discardOutput && writerThread.joinable()) {
        {
            std::lock_guard<std::mutex> lock(wq.mtx);
            wq.done.store(true);
        }
        wq.cv.notify_one();
        writerThread.join();
    }

    std::cout << "<SAMPLES_CAPTURED: " << framesCaptured << ">\n";
    emitTimestamp("TS_CLOSE_FILE", currentClock(CLOCK_MONOTONIC));

    if (!options.discardOutput) {
        writeWaveHeader(output,
                        options.channels,
                        rate,
                        formatInfo.bitsPerSample,
                        static_cast<uint32_t>(std::min<uint64_t>(dataBytes, std::numeric_limits<uint32_t>::max())));
        output.close();
    }

    emitTimestamp("TS_END", currentClock(CLOCK_MONOTONIC));

    if (monitorPcm) {
        snd_pcm_drain(monitorPcm);
        snd_pcm_close(monitorPcm);
    }

    snd_pcm_drop(pcm);
    snd_pcm_close(pcm);

    // Signal to the parent (cinepi_sound) that the dsnoop capture PCM has been
    // fully released.  The parent waits for this marker (or a short settle period)
    // before launching the recorder so the recorder always opens a cold dsnoop
    // connection with zero pre-start backlog.
    if (options.discardOutput) {
        std::cout << "<AUDIO_MONITOR_RELEASED>\n";
        std::cout.flush();
    }

    return 0;
}
