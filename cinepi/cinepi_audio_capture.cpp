#include <alsa/asoundlib.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
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
    std::string output;
    unsigned int channels = 0;
    unsigned int rate = 0;
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
        else if (arg == "--output")
            options.output = requireValue(arg);
        else {
            std::cerr << "Unknown argument: " << arg << '\n';
            return false;
        }
    }

    return !options.device.empty() &&
           !options.format.empty() &&
           !options.output.empty() &&
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

} // namespace

int main(int argc, char **argv)
{
    std::setvbuf(stdout, nullptr, _IOLBF, 0);

    Options options;
    if (!parseArgs(argc, argv, options)) {
        std::cerr << "Usage: cinepi-audio-capture --device <name> --format <S16_LE|S24_3LE>"
                  << " --channels <n> --rate <hz> --output <wav>\n";
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

    std::filesystem::path outputPath(options.output);
    std::fstream output(outputPath, std::ios::binary | std::ios::out | std::ios::trunc);
    if (!output.is_open()) {
        std::cerr << "Failed to open output WAV: " << options.output << '\n';
        return 1;
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

    writeWaveHeader(output, options.channels, rate, formatInfo.bitsPerSample, 0);

    unsigned int periodTimeUs = 10000;
    unsigned int bufferTimeUs = 40000;
    snd_pcm_hw_params_set_period_time_near(pcm, hw, &periodTimeUs, &dir);
    snd_pcm_hw_params_set_buffer_time_near(pcm, hw, &bufferTimeUs, &dir);

    if ((err = snd_pcm_hw_params(pcm, hw)) < 0) {
        std::cerr << "Failed to apply ALSA capture parameters: " << snd_strerror(err) << '\n';
        snd_pcm_close(pcm);
        return 1;
    }

    snd_pcm_uframes_t periodFrames = 0;
    snd_pcm_hw_params_get_period_size(hw, &periodFrames, &dir);
    if (periodFrames == 0)
        periodFrames = 256;

    if ((err = snd_pcm_prepare(pcm)) < 0) {
        std::cerr << "snd_pcm_prepare failed: " << snd_strerror(err) << '\n';
        snd_pcm_close(pcm);
        return 1;
    }

    if ((err = snd_pcm_start(pcm)) < 0) {
        std::cerr << "snd_pcm_start failed: " << snd_strerror(err) << '\n';
        snd_pcm_close(pcm);
        return 1;
    }

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
    bool draining = false;
    std::optional<std::chrono::steady_clock::time_point> drainDeadline;
    const auto drainGrace =
        std::chrono::milliseconds(std::max<unsigned int>(1000, bufferTimeUs / 1000));

    while (true) {
        snd_pcm_sframes_t framesToRead = static_cast<snd_pcm_sframes_t>(periodFrames);
        if (draining) {
            snd_pcm_sframes_t available = snd_pcm_avail_update(pcm);
            if (available < 0) {
                err = recoverCaptureError(pcm, static_cast<int>(available));
                if (err < 0) {
                    std::cerr << "Capture drain failed: " << snd_strerror(err) << '\n';
                    break;
                }
                continue;
            }

            if (available == 0) {
                if (drainDeadline &&
                    std::chrono::steady_clock::now() < *drainDeadline) {
                    snd_pcm_wait(pcm, 1);
                    continue;
                }
                break;
            }

            framesToRead = std::min<snd_pcm_sframes_t>(framesToRead, available);
        }

        snd_pcm_sframes_t framesRead =
            snd_pcm_readi(pcm, buffer.data(), framesToRead);
        if (framesRead == -EINTR && stopRequested.load()) {
            if (!draining) {
                draining = true;
                drainDeadline = std::chrono::steady_clock::now() + drainGrace;
            }
            continue;
        }
        if (framesRead < 0) {
            err = recoverCaptureError(pcm, static_cast<int>(framesRead));
            if (err < 0) {
                std::cerr << "Capture read failed: " << snd_strerror(err) << '\n';
                break;
            }
            continue;
        }

        if (!emittedFirstBufferAfter) {
            emitTimestamp("TS_FIRST_BUFFER_A", currentClock(CLOCK_MONOTONIC));
            emittedFirstBufferAfter = true;
        }

        const size_t bytesRead = static_cast<size_t>(framesRead) * frameBytes;
        output.write(reinterpret_cast<const char *>(buffer.data()),
                     static_cast<std::streamsize>(bytesRead));
        if (!output.good()) {
            std::cerr << "Failed to write WAV payload\n";
            break;
        }

        dataBytes += static_cast<uint64_t>(bytesRead);
        framesCaptured += static_cast<uint64_t>(framesRead);
        emitVu(buffer.data(), framesRead, options.channels, formatInfo);

        if (stopRequested.load() && !draining) {
            draining = true;
            drainDeadline = std::chrono::steady_clock::now() + drainGrace;
        }
    }

    std::cout << "<SAMPLES_CAPTURED: " << framesCaptured << ">\n";
    emitTimestamp("TS_CLOSE_FILE", currentClock(CLOCK_MONOTONIC));

    writeWaveHeader(output,
                    options.channels,
                    rate,
                    formatInfo.bitsPerSample,
                    static_cast<uint32_t>(std::min<uint64_t>(dataBytes, std::numeric_limits<uint32_t>::max())));
    output.close();

    emitTimestamp("TS_END", currentClock(CLOCK_MONOTONIC));

    snd_pcm_drop(pcm);
    snd_pcm_close(pcm);
    return 0;
}
