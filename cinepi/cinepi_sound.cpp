#include "cinepi_sound.hpp"
#include <algorithm>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/rational.hpp>
#include <boost/numeric/conversion/cast.hpp>
#include <climits>
#include <cmath>
#include <cstring>
#include <ctime>
#include <fstream>
#include <future>
#include <limits>
#include <optional>
#include <regex>
#include <unordered_set>
#include <sys/prctl.h>
#include <sys/wait.h>

constexpr int FIXED_AUDIO_SAMPLE_RATE = 48000;
constexpr int FALLBACK_AUDIO_SAMPLE_RATE = 44100;
constexpr char RECORDER_VU_REDIS_KEY[] = "audio_vu";
constexpr char REDIS_DEFAULT_URL[] = "redis://127.0.0.1:6379/0";
constexpr auto RECORDER_VU_PUBLISH_INTERVAL = std::chrono::milliseconds(33);
constexpr auto AUDIO_MONITOR_SHUTDOWN_TIMEOUT = std::chrono::milliseconds(250);

// The first audio buffer marker lands after the hardware has already started
// filling the capture pipeline. Subtract this latency when estimating the
// point where the recorded content actually begins.
constexpr double AUDIO_CAPTURE_LATENCY_MS = 120.0; // milliseconds

namespace {

std::string shellQuote(const std::string &value)
{
    std::string quoted = "'";
    for (char ch : value) {
        if (ch == '\'')
            quoted += "'\\''";
        else
            quoted += ch;
    }
    quoted += "'";
    return quoted;
}

std::string locateAudioCaptureHelper()
{
    std::vector<std::filesystem::path> candidates;

    char exePath[PATH_MAX] = {};
    const ssize_t exeLen = readlink("/proc/self/exe", exePath, sizeof(exePath) - 1);
    if (exeLen > 0) {
        exePath[exeLen] = '\0';
        candidates.emplace_back(std::filesystem::path(exePath).parent_path() / "cinepi-audio-capture");
    }

    candidates.emplace_back("/usr/local/bin/cinepi-audio-capture");
    candidates.emplace_back("/usr/bin/cinepi-audio-capture");

    for (const auto &candidate : candidates) {
        std::error_code ec;
        if (std::filesystem::exists(candidate, ec))
            return candidate.string();
    }

    return {};
}

std::string formatSeconds(double value)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6) << std::max(0.0, value);
    return oss.str();
}

std::optional<double> probeDurationSeconds(const std::string &filename)
{
    std::ostringstream cmd;
    cmd << "ffprobe -v error -show_entries format=duration "
        << "-of default=noprint_wrappers=1:nokey=1 "
        << shellQuote(filename);

    FILE *fp = popen(cmd.str().c_str(), "r");
    if (!fp)
        return std::nullopt;

    char buffer[256];
    std::string output;
    while (fgets(buffer, sizeof(buffer), fp) != nullptr)
        output += buffer;

    int rc = pclose(fp);
    if (rc != 0)
        return std::nullopt;

    try {
        size_t parsed = 0;
        double seconds = std::stod(output, &parsed);
        (void)parsed;
        if (std::isfinite(seconds) && seconds >= 0.0)
            return seconds;
    } catch (...) {
    }

    return std::nullopt;
}

uint64_t chooseAudioStartTimestamp(uint64_t first_buffer_before,
                                   uint64_t first_buffer_after,
                                   uint64_t process_start)
{
    uint64_t chosen = 0;
    for (uint64_t candidate : { first_buffer_before, first_buffer_after }) {
        if (candidate == 0)
            continue;
        if (chosen == 0 || candidate < chosen)
            chosen = candidate;
    }

    if (chosen == 0)
        chosen = process_start;

    return chosen;
}

double fallbackDurationFromSamples(int samplesCaptured, int sampleRate)
{
    if (samplesCaptured <= 0 || sampleRate <= 0)
        return 0.0;

    return static_cast<double>(samplesCaptured) / static_cast<double>(sampleRate);
}

int bcdToInt(uint8_t value)
{
    return ((value >> 4) & 0x0f) * 10 + (value & 0x0f);
}

uint8_t intToBcd(int value)
{
    value = std::clamp(value, 0, 99);
    return static_cast<uint8_t>(((value / 10) << 4) | (value % 10));
}

double nominalTimecodeFramerate(double framerate)
{
    if (!std::isfinite(framerate) || framerate <= 0.0)
        return 0.0;

    return std::max(1.0, std::round(framerate));
}

std::string buildTimecodeString(const std::array<uint8_t, 8> &timecode)
{
    std::ostringstream oss;
    oss << std::setw(2) << std::setfill('0') << bcdToInt(timecode[3]) << ':'
        << std::setw(2) << std::setfill('0') << bcdToInt(timecode[2]) << ':'
        << std::setw(2) << std::setfill('0') << bcdToInt(timecode[1]) << ':'
        << std::setw(2) << std::setfill('0') << bcdToInt(timecode[0]);
    return oss.str();
}

std::string formatOriginationDate(const std::array<uint16_t, 3> &originationDate)
{
    std::ostringstream oss;
    oss << std::setw(4) << std::setfill('0') << static_cast<int>(originationDate[0]) << '-'
        << std::setw(2) << std::setfill('0') << static_cast<int>(originationDate[1]) << '-'
        << std::setw(2) << std::setfill('0') << static_cast<int>(originationDate[2]);
    return oss.str();
}

std::string formatOriginationTime(const std::array<uint8_t, 8> &timecode)
{
    std::ostringstream oss;
    oss << std::setw(2) << std::setfill('0') << bcdToInt(timecode[3]) << ':'
        << std::setw(2) << std::setfill('0') << bcdToInt(timecode[2]) << ':'
        << std::setw(2) << std::setfill('0') << bcdToInt(timecode[1]);
    return oss.str();
}

uint64_t computeTimeReferenceSamples(const std::array<uint8_t, 8> &timecode,
                                     int sampleRate,
                                     double framerate)
{
    if (sampleRate <= 0)
        return 0;

    const double timecodeFramerate = nominalTimecodeFramerate(framerate);

    const uint64_t hours = static_cast<uint64_t>(bcdToInt(timecode[3]));
    const uint64_t minutes = static_cast<uint64_t>(bcdToInt(timecode[2]));
    const uint64_t seconds = static_cast<uint64_t>(bcdToInt(timecode[1]));
    const uint64_t frames = static_cast<uint64_t>(bcdToInt(timecode[0]));

    uint64_t timeReference = ((hours * 3600ULL) + (minutes * 60ULL) + seconds) *
                             static_cast<uint64_t>(sampleRate);

    if (timecodeFramerate > 0.0 && frames > 0) {
        const double frameSamples =
            static_cast<double>(frames) * static_cast<double>(sampleRate) / timecodeFramerate;
        timeReference += static_cast<uint64_t>(std::llround(frameSamples));
    }

    return timeReference;
}

bool parseVuLine(const std::string &line, std::array<int, 4> &vuMeter)
{
    int left = 0, right = 0, aux1 = 0, aux2 = 0;
    if (std::sscanf(line.c_str(), "<VU:%d|%d|%d|%d>", &left, &right, &aux1, &aux2) != 4)
        return false;

    vuMeter[0] = left;
    vuMeter[1] = right;
    vuMeter[2] = aux1;
    vuMeter[3] = aux2;
    return true;
}

std::optional<uintmax_t> waitForStableFile(const std::string &filename,
                                           int maxAttempts = 50,
                                           std::chrono::milliseconds interval =
                                               std::chrono::milliseconds(100))
{
    uintmax_t previousSize = 0;
    int stableReads = 0;

    for (int attempt = 0; attempt < maxAttempts; ++attempt) {
        std::error_code ec;
        if (std::filesystem::exists(filename, ec)) {
            const auto currentSize = std::filesystem::file_size(filename, ec);
            if (!ec && currentSize > 0) {
                if (currentSize == previousSize)
                    ++stableReads;
                else
                    stableReads = 0;

                previousSize = currentSize;
                if (stableReads >= 2)
                    return currentSize;
            }
        }

        std::this_thread::sleep_for(interval);
    }

    return std::nullopt;
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

int shellExitCode(int status)
{
    if (status < 0)
        return status;
    if (WIFEXITED(status))
        return WEXITSTATUS(status);
    if (WIFSIGNALED(status))
        return 128 + WTERMSIG(status);
    return status;
}

double configuredFramerate(const RawOptions *options)
{
    return (options && options->framerate && *options->framerate > 0.0)
               ? *options->framerate
               : 0.0;
}

struct ParsedWavMetadata
{
    std::array<uint8_t, 8> timecode{};
    std::array<uint16_t, 3> originationDate{};
    double framerate = 0.0;
};

std::optional<ParsedWavMetadata> offsetMetadataFrames(const ParsedWavMetadata &metadata,
                                                      int frameOffset)
{
    if (frameOffset == 0)
        return metadata;

    const double timecodeFramerate = nominalTimecodeFramerate(metadata.framerate);
    if (timecodeFramerate <= 0.0)
        return std::nullopt;

    const int fps = static_cast<int>(std::llround(timecodeFramerate));
    if (fps <= 0)
        return std::nullopt;

    std::tm localTime{};
    localTime.tm_year = static_cast<int>(metadata.originationDate[0]) - 1900;
    localTime.tm_mon = static_cast<int>(metadata.originationDate[1]) - 1;
    localTime.tm_mday = static_cast<int>(metadata.originationDate[2]);
    localTime.tm_hour = bcdToInt(metadata.timecode[3]);
    localTime.tm_min = bcdToInt(metadata.timecode[2]);
    localTime.tm_sec = bcdToInt(metadata.timecode[1]);
    localTime.tm_isdst = -1;

    time_t baseSeconds = std::mktime(&localTime);
    if (baseSeconds == static_cast<time_t>(-1))
        return std::nullopt;

    int frame = bcdToInt(metadata.timecode[0]) + frameOffset;
    int secondsOffset = frame / fps;
    int normalizedFrame = frame % fps;
    if (normalizedFrame < 0) {
        normalizedFrame += fps;
        --secondsOffset;
    }

    baseSeconds += secondsOffset;

    std::tm *adjustedLocalTime = std::localtime(&baseSeconds);
    if (!adjustedLocalTime)
        return std::nullopt;

    ParsedWavMetadata adjusted = metadata;
    adjusted.timecode = {
        intToBcd(normalizedFrame),
        intToBcd(adjustedLocalTime->tm_sec),
        intToBcd(adjustedLocalTime->tm_min),
        intToBcd(adjustedLocalTime->tm_hour),
        0, 0, 0, 0
    };
    adjusted.originationDate = {
        static_cast<uint16_t>(adjustedLocalTime->tm_year + 1900),
        static_cast<uint16_t>(adjustedLocalTime->tm_mon + 1),
        static_cast<uint16_t>(adjustedLocalTime->tm_mday)
    };
    adjusted.framerate = timecodeFramerate;
    return adjusted;
}

std::optional<ParsedWavMetadata> offsetMetadataSeconds(const ParsedWavMetadata &metadata,
                                                       double secondsOffset)
{
    const double timecodeFramerate = nominalTimecodeFramerate(metadata.framerate);
    if (timecodeFramerate <= 0.0)
        return std::nullopt;

    const int frameOffset = static_cast<int>(std::llround(secondsOffset * timecodeFramerate));
    return offsetMetadataFrames(metadata, frameOffset);
}

std::optional<ParsedWavMetadata> buildMetadataFromWallclockNs(int64_t timestampNs,
                                                              double framerate)
{
    if (timestampNs < 0)
        return std::nullopt;

    const double timecodeFramerate = nominalTimecodeFramerate(framerate);

    const time_t seconds = static_cast<time_t>(timestampNs / 1000000000LL);
    const int64_t subsecondNs = timestampNs % 1000000000LL;
    std::tm *localTime = localtime(&seconds);
    if (!localTime)
        return std::nullopt;

    int frame = 0;
    if (timecodeFramerate > 0.0) {
        const double fraction = static_cast<double>(subsecondNs) / 1e9;
        frame = static_cast<int>(std::floor((fraction * timecodeFramerate) + 1.0e-9));
        const int maxFrame = std::max(0, static_cast<int>(std::ceil(timecodeFramerate)) - 1);
        frame = std::clamp(frame, 0, maxFrame);
    }

    ParsedWavMetadata metadata;
    metadata.timecode = {
        intToBcd(frame),
        intToBcd(localTime->tm_sec),
        intToBcd(localTime->tm_min),
        intToBcd(localTime->tm_hour),
        0, 0, 0, 0
    };
    metadata.originationDate = {
        static_cast<uint16_t>(localTime->tm_year + 1900),
        static_cast<uint16_t>(localTime->tm_mon + 1),
        static_cast<uint16_t>(localTime->tm_mday)
    };
    metadata.framerate = timecodeFramerate;
    return metadata;
}

std::optional<ParsedWavMetadata> parseTakeMetadataFromFolder(const std::string &folder,
                                                             double fallbackFramerate)
{
    static const std::regex takeRegex(
        R"(^CINEPI_(\d{2})-(\d{2})-(\d{2})_(\d{2})(\d{2})(\d{2})_F(\d{2})_C\d+_(cam[01X])$)");

    std::smatch match;
    if (!std::regex_match(folder, match, takeRegex))
        return std::nullopt;

    const double timecodeFramerate = nominalTimecodeFramerate(fallbackFramerate);

    const int year = 2000 + std::stoi(match[1].str());
    const int month = std::stoi(match[2].str());
    const int day = std::stoi(match[3].str());
    const int hour = std::stoi(match[4].str());
    const int minute = std::stoi(match[5].str());
    const int second = std::stoi(match[6].str());
    int frame = std::stoi(match[7].str());

    if (month < 1 || month > 12 || day < 1 || day > 31 ||
        hour < 0 || hour > 23 || minute < 0 || minute > 59 ||
        second < 0 || second > 59) {
        return std::nullopt;
    }

    if (timecodeFramerate > 0.0) {
        const int maxFrame = std::max(0, static_cast<int>(std::ceil(timecodeFramerate)) - 1);
        frame = std::clamp(frame, 0, maxFrame);
    } else {
        frame = std::clamp(frame, 0, 99);
    }

    ParsedWavMetadata metadata;
    metadata.timecode = {
        intToBcd(frame),
        intToBcd(second),
        intToBcd(minute),
        intToBcd(hour),
        0, 0, 0, 0
    };
    metadata.originationDate = {
        static_cast<uint16_t>(year),
        static_cast<uint16_t>(month),
        static_cast<uint16_t>(day)
    };
    metadata.framerate = timecodeFramerate;
    return metadata;
}

} // namespace

// A helper function to convert a double to a rational number
boost::rational<int> doubleToRational(double value, double tolerance = 1.0e-6) {
    int sign = (value < 0) ? -1 : 1;
    value = std::abs(value);

    int lower_n = 0;
    int lower_d = 1;
    int upper_n = 1;
    int upper_d = 0;

    int middle_n;
    int middle_d;

    while (true) {
        middle_n = lower_n + upper_n;
        middle_d = lower_d + upper_d;

        if (static_cast<double>(middle_n) > value * middle_d) {
            upper_n = middle_n;
            upper_d = middle_d;
        } else {
            lower_n = middle_n;
            lower_d = middle_d;
        }

        if (std::abs(static_cast<double>(middle_n) / middle_d - value) <= tolerance || middle_d > 1000000) {
            break;
        }
    }

    return boost::rational<int>(middle_n * sign, middle_d);
}

FILE * popen2(std::string command, std::string type, int & pid)
{
    pid_t child_pid;
    int fd[2];
    pipe(fd);

    if((child_pid = fork()) == -1)
    {
        perror("fork");
        exit(1);
    }

    if (child_pid == 0)
    {
        const std::string exec_command = "exec " + command;

        if (type == "r")
        {
            close(fd[READ]);
            dup2(fd[WRITE], 1);
        }
        else
        {
            close(fd[WRITE]);
            dup2(fd[READ], 0);
        }

#ifdef PR_SET_PDEATHSIG
        prctl(PR_SET_PDEATHSIG, SIGHUP);
        if (getppid() == 1)
            _exit(1);
#endif
        setpgid(child_pid, child_pid);
        execl("/bin/sh", "/bin/sh", "-c", exec_command.c_str(), NULL);
        exit(0);
    }
    else
    {
        if (type == "r") {
            close(fd[WRITE]);
        } else {
            close(fd[READ]);
        }
    }

    pid = child_pid;

    if (type == "r") {
        return fdopen(fd[READ], "r");
    }

    return fdopen(fd[WRITE], "w");
}

int pclose2(FILE * fp, pid_t pid)
{
    int stat;

    fclose(fp);
    while (waitpid(pid, &stat, 0) == -1)
    {
        if (errno != EINTR)
        {
            stat = -1;
            break;
        }
    }

    return stat;
}

static int run_with_stderr_capture(const std::string& cmd, std::string& first_line) {
    FILE* fp = popen((cmd + " 2>&1").c_str(), "r");
    if (!fp) {
        return -1;
    }

    std::string line_buf;
    char buf[1024];
    bool have_line = false;

    while (true) {
        size_t n = fread(buf, 1, sizeof(buf), fp);
        if (n == 0) {
            if (feof(fp) || ferror(fp)) {
                break;
            }
        } else if (!have_line) {
            line_buf.append(buf, n);
            auto pos = line_buf.find('\n');
            if (pos != std::string::npos) {
                first_line = line_buf.substr(0, pos);
                have_line = true;
            } else if (line_buf.size() > 1024) {
                first_line = line_buf;
                have_line = true;
            }
        }
    }

    if (have_line) {
        while (!first_line.empty() && (first_line.back() == '\r' || first_line.back() == '\n')) {
            first_line.pop_back();
        }
    } else {
        first_line.clear();
    }

    return pclose(fp);
}

void cleanupStaleIdleMonitorProcesses(const std::shared_ptr<spdlog::logger> &console)
{
    const std::array<std::pair<const char *, const char *>, 2> cleanupCommands = {{
        {
            "pkill -f \"cinepi-audio-capture.*--discard-output\"",
            "stale idle audio monitor helpers",
        },
        {
            "pkill -f \"alsaloop -C .* -P .* -t 10000 -A 1 -d\"",
            "stale legacy HDMI monitor loops",
        },
    }};

    for (const auto &[command, description] : cleanupCommands) {
        const int rc = std::system(command);
        const int exitCode = shellExitCode(rc);
        if (exitCode == 0) {
            console->info("Cleaned up {}", description);
        } else if (exitCode != 1) {
            console->warn("Cleanup command failed for {} (rc={})", description, exitCode);
        }
    }
}

uint64_t extractTime(const std::string& line) {
    size_t colon_pos = line.find(':');
    size_t dot_pos = line.find('.');
    size_t end_pos = line.find('>');

    uint64_t seconds = std::stoull(line.substr(colon_pos + 1, dot_pos - colon_pos - 1));
    uint64_t nanoseconds = std::stoull(line.substr(dot_pos + 1, end_pos - dot_pos - 1));

    return (seconds * 1e+9) + nanoseconds;
}

CinePISound::CinePISound(CinePIRecorder *app) :
    vu_meter({0, 0, 0, 0}),
    samples_captured(0),
    capturedAudioSampleRate(0),
    ts_start(0),
    ts_first_buffer_b(0),
    ts_first_buffer_a(0),
    ts_close_file(0),
    ts_end(0),
    ts_audio_start_realtime(0),
    audioFormat("S16_LE"),
    audioChannels(1),
    audioSampleRate(FIXED_AUDIO_SAMPLE_RATE),
    arec_pipe(nullptr),
    canRecordAudio(false),
    defaultDevice(""),
    console(nullptr),
    pid(-1),
    recording_(false),
    record_(false),
    audio_capture_started_(false),
    app_(app),
    options_(app->GetOptions()),
    abortThread_(false),
    monitor_playback_pipe_(nullptr),
    monitoring_playback_(false),
    monitor_vu_pipe_(nullptr),
    monitoring_vu_(false),
    udev(nullptr),
    udev_dev(nullptr),
    udev_mon(nullptr),
    udev_fd(-1)
{
    console = spdlog::stdout_color_mt("cinepi_sound");
    console->set_level(spdlog::level::debug);  // or trace if you want even more
    initRedis();
}

CinePISound::~CinePISound() {
    record_ = false;
    {
        std::lock_guard<std::mutex> lock(pending_audio_capture_mutex_);
        pending_audio_capture_.reset();
    }
    if (pid > 0)
        kill(-pid, SIGTERM);
    recording_ = false;
    audio_capture_started_ = false;
    abortThread_ = true;
    stopMonitoring();
    if (sound_thread_.joinable())
        sound_thread_.join();
    clearRecorderVuMeter();
    if (udev_mon)
        udev_monitor_unref(udev_mon);
    udev_unref(udev);
}

void CinePISound::initRedis()
{
    try {
        const std::string redisUrl =
            options_ && options_->redis ? *options_->redis : std::string(REDIS_DEFAULT_URL);
        redis_ = std::make_unique<sw::redis::Redis>(redisUrl);
        console->debug("Connected CinePISound Redis client to {}", redisUrl);
    } catch (const std::exception &exc) {
        redis_.reset();
        console->warn("CinePISound could not connect to Redis for recorder VU publishing: {}",
                      exc.what());
    }
}

void CinePISound::publishRecorderVuMeter(bool force)
{
    if (!redis_)
        return;

    const auto now = std::chrono::steady_clock::now();
    if (!force && (now - last_vu_publish_ts_) < RECORDER_VU_PUBLISH_INTERVAL)
        return;

    try {
        std::ostringstream value;
        value << vu_meter[0] << '|'
              << vu_meter[1] << '|'
              << vu_meter[2] << '|'
              << vu_meter[3];
        redis_->set(RECORDER_VU_REDIS_KEY, value.str());
        last_published_vu_ = vu_meter;
        last_vu_publish_ts_ = now;
    } catch (const std::exception &exc) {
        console->debug("Failed to publish recorder VU to Redis: {}", exc.what());
    }
}

void CinePISound::clearRecorderVuMeter()
{
    if (!redis_)
        return;

    try {
        redis_->del(RECORDER_VU_REDIS_KEY);
    } catch (const std::exception &exc) {
        console->debug("Failed to clear recorder VU from Redis: {}", exc.what());
    }

    last_published_vu_.fill(0);
    last_vu_publish_ts_ = std::chrono::steady_clock::time_point{};
}

void CinePISound::start() {
    cleanupStaleIdleMonitorProcesses(console);
    detectRecordingDevices();
    parseHardwareParams();  // ensure audio config is ready before recording

    if (!sound_thread_.joinable()) {
        sound_thread_ = std::thread(std::bind(&CinePISound::soundThread, this));
    }

    if (!canRecordAudio) {
        console->warn("start(): Audio not ready — recording disabled");
    }
}


bool CinePISound::tryAudioConfig(const std::string& device, const std::string& format,
                                 int channels, int rate)
{
    std::ostringstream cmd;
    cmd << "arecord -D " << device
        << " -f " << format
        << " -c " << channels
        << " -r " << rate
        << " -d 1 -t raw";

    std::string stderr_one;
    int rc = run_with_stderr_capture(cmd.str(), stderr_one);
    int exit_code = -1;
    if (rc >= 0) {
        if (WIFEXITED(rc)) {
            exit_code = WEXITSTATUS(rc);
        } else if (WIFSIGNALED(rc)) {
            exit_code = 128 + WTERMSIG(rc);
        }
    }

    if (exit_code == 0) {
        console->info("Probe OK: {} (fmt {}, ch {}, {} Hz)", device, format, channels, rate);
        return true;
    } else {
        console->debug("Probe FAILED rc={} : {} | {}", exit_code, cmd.str(), stderr_one);
        console->warn("Audio probe failed for {} (fmt {}, ch {}, {} Hz): {}", device, format, channels, rate,
                     stderr_one.empty() ? "no error output" : stderr_one);
        return false;
    }
}

void CinePISound::record_start() {
    console->info("record_start() called");

    if (!canRecordAudio) {
        console->warn("Audio recording not allowed (canRecordAudio = false)");
        return;
    }

    resetTakeMetadata();

    if (auto fallbackMetadata =
            parseTakeMetadataFromFolder(options_->folder, configuredFramerate(options_))) {
        takeStartTimeCode_ = fallbackMetadata->timecode;
        takeStartOriginationDate_ = fallbackMetadata->originationDate;
        takeStartFramerate_ = fallbackMetadata->framerate;
        takeStartMetadataValid_ = true;

        const std::string fallbackRate =
            takeStartFramerate_ > 0.0 ? formatSeconds(takeStartFramerate_) : std::string("unknown");
        console->info("Cached fallback WAV metadata from take name: {} ({} fps)",
                      buildTimecodeString(takeStartTimeCode_),
                      fallbackRate);
    } else {
        console->warn("Unable to parse deterministic fallback WAV metadata from take name: {}",
                      options_->folder);
    }

    std::ostringstream oss;
    oss << options_->mediaDest << '/' << options_->folder << '/' << options_->folder << ".wav";
    std::string filename = oss.str();
    std::filesystem::create_directories(options_->mediaDest + "/" + options_->folder);

    if (defaultDevice.empty() || audioFormat.empty() || audioChannels == 0) {
        console->error("record_start(): Missing audio configuration — cannot start recording");
        return;
    }

    const std::string helperBinary = locateAudioCaptureHelper();
    const bool use_plain_arecord_for_16bit = (defaultDevice == "mic_16bit");
    const std::string capturePath =
        use_plain_arecord_for_16bit
            ? "plain-arecord-mic16"
            : (!helperBinary.empty() ? "cinepi-audio-capture" : "plain-arecord-helper-missing");
    const bool captureEmitsMarkers = !use_plain_arecord_for_16bit && !helperBinary.empty();
    const bool stopMonitorBeforeLaunch = !use_plain_arecord_for_16bit;

    cmdStream.str("");
    cmdStream.clear();
    if (captureEmitsMarkers) {
        cmdStream << shellQuote(helperBinary)
                  << " --device " << shellQuote(defaultDevice)
                  << " --format " << shellQuote(audioFormat)
                  << " --channels " << audioChannels
                  << " --rate " << audioSampleRate
                  << " --output " << shellQuote(filename)
                  << " 2>&1";
    } else {
        if (use_plain_arecord_for_16bit) {
            console->info("Using plain arecord for mic_16bit take capture while leaving idle monitoring active");
        } else {
            console->warn("cinepi-audio-capture helper not found; falling back to arecord without precise audio-start markers");
        }
        cmdStream << "arecord"
                  << " -q"
                  << " -D " << defaultDevice
                  << " -f " << audioFormat
                  << " -c " << audioChannels
                  << " -r " << audioSampleRate
                  << " -t wav"
                  << " --disable-softvol"
                  << " " << shellQuote(filename)
                  << " 2>&1";
    }

    console->info("Audio capture path selected: {} (device {}, format {}, channels {}, rate {}, emits_start_markers {}, stop_idle_monitor_before_launch {})",
                  capturePath,
                  defaultDevice,
                  audioFormat,
                  audioChannels,
                  audioSampleRate,
                  captureEmitsMarkers ? "yes" : "no",
                  stopMonitorBeforeLaunch ? "yes" : "no");
    console->info("Queueing audio capture command: {}", cmdStream.str());
    console->info("Video recording will continue immediately while audio starts asynchronously");

    samples_captured = 0;
    capturedAudioSampleRate = 0;
    vu_meter.fill(0);
    ts_start = 0;
    ts_first_buffer_b = 0;
    ts_first_buffer_a = 0;
    ts_close_file = 0;
    ts_end = 0;
    ts_audio_start_realtime = 0;
    audio_capture_started_ = false;
    audio_capture_emits_markers_ = captureEmitsMarkers;
    audio_capture_path_ = capturePath;
    publishRecorderVuMeter(true);

    record_ = true;
    std::lock_guard<std::mutex> lock(pending_audio_capture_mutex_);
    pending_audio_capture_ = PendingAudioCapture{
        cmdStream.str(),
        capturePath,
        stopMonitorBeforeLaunch,
        captureEmitsMarkers
    };
}

void CinePISound::launchPendingRecordingStart()
{
    std::optional<PendingAudioCapture> pending;
    {
        std::lock_guard<std::mutex> lock(pending_audio_capture_mutex_);
        if (!pending_audio_capture_)
            return;

        pending = pending_audio_capture_;
        pending_audio_capture_.reset();
    }

    if (!record_) {
        console->warn("Audio capture start was canceled before helper launch");
        return;
    }

    if (pending->stop_monitoring_before_launch) {
        stopMonitoring();

        if (!record_) {
            console->warn("Audio capture start was canceled after monitor shutdown");
            return;
        }
    } else {
        console->info("Leaving idle audio monitor running during take capture");
    }

    console->info("Launching audio capture command asynchronously: {}", pending->command);
    arec_pipe = popen2(pending->command, "r", pid);

    if (!arec_pipe) {
        pid = -1;
        console->warn("Failed to launch audio capture helper; continuing take without audio");
        return;
    }

    audio_capture_emits_markers_ = pending->emits_helper_markers;
    audio_capture_path_ = pending->capture_path;
    audio_capture_started_ = !audio_capture_emits_markers_;
    if (!audio_capture_emits_markers_ && capturedAudioSampleRate <= 0)
        capturedAudioSampleRate = audioSampleRate;

    console->info("Audio capture path active: {} (emits_start_markers {}, stop_idle_monitor_before_launch {})",
                  audio_capture_path_,
                  audio_capture_emits_markers_ ? "yes" : "no",
                  pending->stop_monitoring_before_launch ? "yes" : "no");
    console->info("Audio capture helper started successfully (pid={})", pid);
    recording_ = true;
}

void CinePISound::record_stop() {
    if(!canRecordAudio)
        return;

    record_ = false;
    {
        std::lock_guard<std::mutex> lock(pending_audio_capture_mutex_);
        pending_audio_capture_.reset();
    }
    if(pid > 0){
        kill(-pid, SIGTERM); // Send to full process group
    }
    console->info("Sound recording stopped.");

    if (canRecordAudio) {
        startMonitoring();
    }
}

bool CinePISound::recording_ended() {
    bool state = false;
    if((pid < 0) && recording_){
        recording_ = false;
        state = true;
    }
    return state;
}

bool CinePISound::isRecording() {
    return (recording_ && (ts_first_buffer_b > 0 || !audio_capture_emits_markers_)) || !canRecordAudio;
}

void CinePISound::detectRecordingDevices() {
    std::string list;
    {
        FILE* fp = popen("arecord -l 2>/dev/null", "r");
        if (fp) {
            char buf[512];
            while (fgets(buf, sizeof(buf), fp)) {
                list += buf;
            }
            pclose(fp);
        }
    }
    if (list.find("card ") == std::string::npos) {
        console->error("No recording devices detected!");
        canRecordAudio = false;
    } else {
        canRecordAudio = true;
    }
    console->debug("Audio device present: {}", canRecordAudio ? "yes" : "no");
}

void CinePISound::stopMonitoring() {
    stopPlaybackMonitoring();
    stopIdleVuMonitoring();
    vu_meter.fill(0);
    clearRecorderVuMeter();
}

void CinePISound::startMonitoring() {
    if (!canRecordAudio || recording_ || record_) {
        return;
    }

    startIdleVuMonitoring();
    if (!monitoring_vu_)
        startPlaybackMonitoring();
}

void CinePISound::stopPlaybackMonitoring() {
    if (monitor_playback_pid_ > 0) {
        kill(-monitor_playback_pid_, SIGTERM);
        if (monitor_playback_pipe_) {
            pclose2(monitor_playback_pipe_, monitor_playback_pid_);
            monitor_playback_pipe_ = nullptr;
        }
        monitor_playback_pid_ = -1;
        monitoring_playback_ = false;
        console->info("Stopped audio monitoring playback");
    }
}

void CinePISound::startPlaybackMonitoring() {
    if (monitoring_playback_) {
        return;
    }

    std::string outputDevice = getPreferredMonitorOutput();
    std::ostringstream mon_cmd;
    mon_cmd << "alsaloop -C " << defaultDevice
            << " -P " << outputDevice
            << " -t 10000 -A 1 -d"
            << " 2>/dev/null";
    console->info("Starting audio monitoring: {}", mon_cmd.str());
    monitor_playback_pipe_ = popen2(mon_cmd.str(), "r", monitor_playback_pid_);
    if (monitor_playback_pid_ > 0 && monitor_playback_pipe_) {
        monitoring_playback_ = true;
    }
}

void CinePISound::stopIdleVuMonitoring()
{
    if (monitor_vu_pid_ > 0)
        kill(-monitor_vu_pid_, SIGTERM);

    if (idle_vu_thread_.joinable()) {
        auto joinFuture = std::async(std::launch::async, [this]() {
            idle_vu_thread_.join();
        });

        if (joinFuture.wait_for(AUDIO_MONITOR_SHUTDOWN_TIMEOUT) != std::future_status::ready) {
            console->warn("Idle audio monitor did not exit after SIGTERM; forcing shutdown");
            if (monitor_vu_pid_ > 0)
                kill(-monitor_vu_pid_, SIGKILL);
            joinFuture.wait();
        }
    }

    if (monitor_vu_pipe_) {
        pclose2(monitor_vu_pipe_, monitor_vu_pid_);
        monitor_vu_pipe_ = nullptr;
        monitor_vu_pid_ = -1;
    }
    monitoring_vu_ = false;
}

void CinePISound::startIdleVuMonitoring()
{
    if (monitoring_vu_)
        return;

    const std::string helperBinary = locateAudioCaptureHelper();
    if (helperBinary.empty()) {
        console->warn("cinepi-audio-capture helper not found; idle VU monitoring disabled");
        return;
    }

    std::ostringstream mon_cmd;
    const std::string outputDevice = getPreferredMonitorOutput();
    mon_cmd << shellQuote(helperBinary)
            << " --device " << shellQuote(defaultDevice)
            << " --format " << shellQuote(audioFormat)
            << " --channels " << audioChannels
            << " --rate " << audioSampleRate
            << " --monitor-output " << shellQuote(outputDevice)
            << " --discard-output"
            << " 2>&1";
    console->info("Starting idle audio monitor via helper: {}", mon_cmd.str());
    monitor_vu_pipe_ = popen2(mon_cmd.str(), "r", monitor_vu_pid_);
    if (monitor_vu_pid_ > 0 && monitor_vu_pipe_) {
        monitoring_vu_ = true;
        idle_vu_thread_ = std::thread(std::bind(&CinePISound::idleVuThread, this));
    }
}

void CinePISound::idleVuThread()
{
    if (!monitor_vu_pipe_)
        return;

    char buffer[256];
    while (!abortThread_ && monitor_vu_pipe_ && fgets(buffer, sizeof(buffer), monitor_vu_pipe_) != NULL) {
        std::string line(buffer);
        if (line.find("<VU:") == std::string::npos) {
            line.erase(std::remove(line.begin(), line.end(), '\n'), line.end());
            if (!line.empty())
                console->warn("Idle audio monitor: {}", line);
            continue;
        }
        if (parseVuLine(line, vu_meter))
            publishRecorderVuMeter();
    }

    if (monitor_vu_pipe_) {
        pclose2(monitor_vu_pipe_, monitor_vu_pid_);
        monitor_vu_pipe_ = nullptr;
    }
    monitor_vu_pid_ = -1;
    monitoring_vu_ = false;

    if (!recording_ && !record_) {
        vu_meter.fill(0);
        clearRecorderVuMeter();
    }
}

std::vector<std::string> CinePISound::parseArecordAliases() {
    std::vector<std::string> aliases;
    std::unordered_set<std::string> seen;
    FILE* fp = popen("arecord -l 2>/dev/null", "r");
    if (!fp) {
        console->warn("parseArecordAliases(): failed to run arecord -l");
        return aliases;
    }

    std::regex re(R"(card\s+(\d+):.*device\s+(\d+):)");
    char buf[512];
    while (fgets(buf, sizeof(buf), fp)) {
        std::cmatch match;
        if (std::regex_search(buf, match, re)) {
            std::string card = match[1];
            std::string device = match[2];
            for (const auto& prefix : {"plughw:", "hw:"}) {
                std::string alias = std::string(prefix) + card + "," + device;
                if (seen.insert(alias).second) {
                    aliases.push_back(alias);
                }
            }
        }
    }
    pclose(fp);

    std::sort(aliases.begin(), aliases.end());
    aliases.erase(std::unique(aliases.begin(), aliases.end()), aliases.end());

    console->debug("parseArecordAliases(): discovered {} aliases", aliases.size());
    return aliases;
}

void CinePISound::soundThread() {
    init_udev();

    while (!abortThread_) {
        if (pid <= 0 && !recording_) {
            launchPendingRecordingStart();
        }

        // Always drain the arecord pipe until the child exits, even after record_stop()
        // clears the record_ flag. Otherwise the pipe would never be closed and the WAV
        // file would remain incomplete/unwritten, resulting in 0 WAV clips.
        if (pid > 0) {
            auto handleAudioCaptureLine = [this](std::string line) {
                if (!line.empty() && line.back() == '\r')
                    line.pop_back();

                if (line.empty())
                    return;

                if (line.find("<VU:") != std::string::npos) {
                    if (parseVuLine(line, vu_meter))
                        publishRecorderVuMeter();
                } else if (line.find("<TS_START:") != std::string::npos) {
                    audio_capture_started_ = true;
                    ts_start = extractTime(line);
                } else if (line.find("<TS_FIRST_BUFFER_B:") != std::string::npos) {
                    audio_capture_started_ = true;
                    ts_first_buffer_b = extractTime(line);
                } else if (line.find("<TS_FIRST_BUFFER_A:") != std::string::npos) {
                    audio_capture_started_ = true;
                    ts_first_buffer_a = extractTime(line);
                } else if (line.find("<TS_AUDIO_START_REALTIME:") != std::string::npos) {
                    audio_capture_started_ = true;
                    ts_audio_start_realtime = extractTime(line);
                } else if (line.find("<NEGOTIATED_SAMPLE_RATE:") != std::string::npos) {
                    audio_capture_started_ = true;
                    sscanf(line.c_str(), "<NEGOTIATED_SAMPLE_RATE: %d>", &capturedAudioSampleRate);
                } else if (line.find("<SAMPLES_CAPTURED:") != std::string::npos) {
                    sscanf(line.c_str(), "<SAMPLES_CAPTURED: %d>", &samples_captured);
                } else if (line.find("<TS_CLOSE_FILE:") != std::string::npos) {
                    ts_close_file = extractTime(line);
                } else if (line.find("<TS_END:") != std::string::npos) {
                    ts_end = extractTime(line);
                } else {
                    console->warn("Audio capture helper: {}", line);
                }
            };

            char buffer[256];
            std::string pendingOutput;
            while (fgets(buffer, sizeof(buffer), arec_pipe) != NULL) {
                pendingOutput += buffer;
                size_t newlinePos = std::string::npos;
                while ((newlinePos = pendingOutput.find('\n')) != std::string::npos) {
                    std::string line = pendingOutput.substr(0, newlinePos);
                    pendingOutput.erase(0, newlinePos + 1);
                    handleAudioCaptureLine(line);
                }
            }
            if (!pendingOutput.empty())
                handleAudioCaptureLine(pendingOutput);
            pclose2(arec_pipe, pid);
            pid = -1;
        }

        if (recording_ended()) {
            auto finishRecordingAttempt = [this]() {
                audio_capture_started_ = false;
                audio_capture_emits_markers_ = true;
                audio_capture_path_ = "unknown";
                vu_meter.fill(0);
                clearRecorderVuMeter();
                startMonitoring();
            };

            if (!audio_capture_started_ && audio_capture_emits_markers_) {
                console->warn("Audio capture helper exited before capture actually started; continuing take without WAV");
                finishRecordingAttempt();
                continue;
            }

            std::ostringstream fn_oss;
            fn_oss << options_->mediaDest << '/' << options_->folder << '/' << options_->folder << ".wav";
            std::string filename = fn_oss.str();

            const auto wavSize = waitForStableFile(filename);
            if (!wavSize) {
                console->critical("Cannot attach WAV metadata: {} did not become ready after recording stopped",
                                  filename);
                finishRecordingAttempt();
                continue;
            }
            console->debug("WAV ready for metadata update: {} bytes", *wavSize);

            int64_t vts_start = 0, vts_end = 0;
            int64_t frames = app_->GetEncoder()->timestamps.size();
            if (frames > 0) {
                vts_start = app_->GetEncoder()->timestamps.front();
                vts_end = app_->GetEncoder()->timestamps.back();
            }

            if (frames <= 0 || vts_end < vts_start) {
                console->critical("Cannot retime WAV: invalid video timestamp range");
                finishRecordingAttempt();
                continue;
            }

            double video_span_seconds = (vts_end - vts_start) / 1e9;
            double average_frame_duration_seconds = 0.0;
            if (frames > 1)
                average_frame_duration_seconds = video_span_seconds / static_cast<double>(frames - 1);
            else if (options_->framerate && *options_->framerate > 0.0)
                average_frame_duration_seconds = 1.0 / static_cast<double>(*options_->framerate);

            double video_duration_seconds = video_span_seconds + average_frame_duration_seconds;
            if (video_duration_seconds <= 0.0)
                video_duration_seconds = video_span_seconds;

            const uint64_t audio_marker_ns = chooseAudioStartTimestamp(
                ts_first_buffer_b,
                ts_first_buffer_a,
                ts_start);
            const bool have_audio_start_marker = audio_marker_ns != 0;
            const bool have_precise_audio_start_marker = ts_audio_start_realtime != 0;
            const std::string audioCapturePath =
                audio_capture_path_.empty() ? "unknown" : audio_capture_path_;
            const std::string audioStartMarkerStatus =
                have_precise_audio_start_marker
                    ? "precise-realtime"
                    : (have_audio_start_marker
                           ? "estimated-buffer"
                           : (audio_capture_emits_markers_ ? "missing-expected" : "not-emitted"));
            const int inputSampleRate =
                capturedAudioSampleRate > 0 ? capturedAudioSampleRate : audioSampleRate;

            double audio_content_start_seconds = 0.0;
            double start_delta_seconds = 0.0;
            double input_duration_seconds = 0.0;

            if (have_audio_start_marker) {
                const double latency_bias_seconds =
                    have_precise_audio_start_marker ? 0.0 : (AUDIO_CAPTURE_LATENCY_MS / 1000.0);
                audio_content_start_seconds =
                    static_cast<double>(audio_marker_ns) / 1e9 - latency_bias_seconds;
                const double video_start_seconds =
                    static_cast<double>(vts_start) / 1e9;
                start_delta_seconds = audio_content_start_seconds - video_start_seconds;

                auto probed_input_duration = probeDurationSeconds(filename);
                input_duration_seconds = probed_input_duration.value_or(
                    fallbackDurationFromSamples(samples_captured, inputSampleRate));

                if (have_precise_audio_start_marker) {
                    console->info(
                        "Using precise audio-start marker for honest WAV timecode: video {:.6f}s, input {:.6f}s, start delta {:+.6f}s, capture rate {} Hz; leaving PCM untouched",
                        video_duration_seconds,
                        input_duration_seconds,
                        start_delta_seconds,
                        inputSampleRate);
                } else {
                    console->info(
                        "Using estimated audio-start metadata for honest WAV timecode: video {:.6f}s, input {:.6f}s, start delta {:+.6f}s, capture rate {} Hz; leaving PCM untouched",
                        video_duration_seconds,
                        input_duration_seconds,
                        start_delta_seconds,
                        inputSampleRate);
                }
            }

            std::ostringstream tmp_oss;
            tmp_oss << options_->mediaDest << '/' << options_->folder << "/temp.wav";

            std::ostringstream ffmpeg_oss;
            const auto &encoderTimecode = app_->GetEncoder()->originationTimeCode;
            const auto &encoderDate = app_->GetEncoder()->originationDate;
            double output_framerate =
                takeStartFramerate_ > 0.0
                    ? takeStartFramerate_
                    : nominalTimecodeFramerate(configuredFramerate(options_));

            std::array<uint8_t, 8> metadataTimecode = encoderTimecode;
            std::array<uint16_t, 3> metadataDate = encoderDate;
            std::string metadataSource = "encoder";

            if (have_precise_audio_start_marker) {
                if (auto audioStartMetadata =
                        buildMetadataFromWallclockNs(static_cast<int64_t>(ts_audio_start_realtime),
                                                     output_framerate)) {
                    metadataTimecode = audioStartMetadata->timecode;
                    metadataDate = audioStartMetadata->originationDate;
                    output_framerate = audioStartMetadata->framerate;
                    metadataSource = "audio-start";
                } else if (takeStartMetadataValid_) {
                    ParsedWavMetadata takeStartMetadata;
                    takeStartMetadata.timecode = takeStartTimeCode_;
                    takeStartMetadata.originationDate = takeStartOriginationDate_;
                    takeStartMetadata.framerate =
                        takeStartFramerate_ > 0.0 ? takeStartFramerate_ : output_framerate;
                    if (auto estimatedAudioStartMetadata =
                            offsetMetadataSeconds(takeStartMetadata, start_delta_seconds)) {
                        metadataTimecode = estimatedAudioStartMetadata->timecode;
                        metadataDate = estimatedAudioStartMetadata->originationDate;
                        output_framerate = estimatedAudioStartMetadata->framerate;
                        metadataSource = "audio-start-estimate";
                        console->warn(
                            "Failed to derive realtime audio-start metadata; estimated honest WAV timecode from take start instead");
                    } else {
                        metadataTimecode = takeStartMetadata.timecode;
                        metadataDate = takeStartMetadata.originationDate;
                        output_framerate = takeStartMetadata.framerate;
                        metadataSource = "video-start-fallback";
                        console->warn(
                            "Failed to derive realtime audio-start metadata and could not offset take-start metadata; falling back to take start");
                    }
                }
            } else if (have_audio_start_marker && takeStartMetadataValid_) {
                ParsedWavMetadata takeStartMetadata;
                takeStartMetadata.timecode = takeStartTimeCode_;
                takeStartMetadata.originationDate = takeStartOriginationDate_;
                takeStartMetadata.framerate =
                    takeStartFramerate_ > 0.0 ? takeStartFramerate_ : output_framerate;
                if (auto estimatedAudioStartMetadata =
                        offsetMetadataSeconds(takeStartMetadata, start_delta_seconds)) {
                    metadataTimecode = estimatedAudioStartMetadata->timecode;
                    metadataDate = estimatedAudioStartMetadata->originationDate;
                    output_framerate = estimatedAudioStartMetadata->framerate;
                    metadataSource = "audio-start-estimate";
                } else {
                    metadataTimecode = takeStartMetadata.timecode;
                    metadataDate = takeStartMetadata.originationDate;
                    output_framerate = takeStartMetadata.framerate;
                    metadataSource = "video-start-fallback";
                    console->warn(
                        "Failed to offset take-start metadata to the measured audio start; falling back to take start");
                }
            } else if (takeStartMetadataValid_) {
                metadataTimecode = takeStartTimeCode_;
                metadataDate = takeStartOriginationDate_;
                if (takeStartFramerate_ > 0.0)
                    output_framerate = takeStartFramerate_;
                metadataSource =
                    audio_capture_emits_markers_ ? "take-start-fallback" : "plain-arecord-fallback";
            }

            if (!have_audio_start_marker) {
                console->warn("No audio start marker received on capture path {} (marker status {}); writing WAV metadata from {} without touching PCM",
                              audioCapturePath,
                              audioStartMarkerStatus,
                              metadataSource);
            }

            const double nominalOutputFramerate = nominalTimecodeFramerate(output_framerate);
            const int audioStartOffsetFrames =
                (have_audio_start_marker && nominalOutputFramerate > 0.0)
                    ? static_cast<int>(std::llround(start_delta_seconds * nominalOutputFramerate))
                    : 0;
            const long long audioStartOffsetSamples =
                have_audio_start_marker
                    ? std::llround(start_delta_seconds * static_cast<double>(inputSampleRate))
                    : 0;
            const std::string timecode_tag = buildTimecodeString(metadataTimecode);
            const std::string origination_date = formatOriginationDate(metadataDate);
            const std::string origination_time = formatOriginationTime(metadataTimecode);
            const int outputSampleRate =
                inputSampleRate > 0 ? inputSampleRate : audioSampleRate;
            const uint64_t time_reference =
                computeTimeReferenceSamples(metadataTimecode, outputSampleRate, output_framerate);

            ffmpeg_oss << "ffmpeg -hide_banner -loglevel error -y -i " << shellQuote(filename);
            ffmpeg_oss
                       << " -map 0:a:0"
                       << " -c:a copy"
                       << " -write_bext 1"
                       << " -metadata " << shellQuote("description=CinePI Description")
                       << " -metadata " << shellQuote("originator=" + options_->ucm.value_or("CinePI"))
                       << " -metadata " << shellQuote("originator_reference=" + options_->serial)
                       << " -metadata " << shellQuote("origination_date=" + origination_date)
                       << " -metadata " << shellQuote("origination_time=" + origination_time)
                       << " -metadata " << shellQuote("time_reference=" + std::to_string(time_reference))
                       << " -metadata " << shellQuote("timecode=" + timecode_tag)
                       << ' ' << shellQuote(tmp_oss.str());

            std::string ffmpeg_error;
            const int ffmpeg_status = run_with_stderr_capture(ffmpeg_oss.str(), ffmpeg_error);
            if (shellExitCode(ffmpeg_status) != 0) {
                console->critical("ffmpeg WAV metadata write failed (rc={}): {}",
                                  shellExitCode(ffmpeg_status),
                                  ffmpeg_error.empty() ? "no stderr output" : ffmpeg_error);
                finishRecordingAttempt();
                continue;
            }

            std::error_code rename_ec;
            std::filesystem::rename(tmp_oss.str(), filename, rename_ec);
            if (rename_ec) {
                console->critical("Failed to replace WAV with metadata-updated version: {}", rename_ec.message());
                finishRecordingAttempt();
                continue;
            }

            const std::string ixml =
                generateIXML(metadataTimecode,
                             output_framerate,
                             metadataSource,
                             audioCapturePath,
                             audioStartMarkerStatus,
                             have_audio_start_marker,
                             start_delta_seconds,
                             audioStartOffsetFrames,
                             audioStartOffsetSamples);
            if (!appendIXMLChunk(filename, ixml)) {
                console->critical("Failed to append iXML chunk to WAV");
            } else {
                console->info("Attached WAV metadata without altering PCM: timecode {}, rate {}, source {}, capture path {}, marker status {}, audio start offset {:+.6f}s ({} frames, {} samples), BEXT + iXML",
                              timecode_tag,
                              output_framerate,
                              metadataSource,
                              audioCapturePath,
                              audioStartMarkerStatus,
                              start_delta_seconds,
                              audioStartOffsetFrames,
                              audioStartOffsetSamples);
            }

            finishRecordingAttempt();
            continue;
        }

        fd_set fds;
        struct timeval tv {0, 0};
        FD_ZERO(&fds);
        FD_SET(udev_fd, &fds);

        int ret = select(udev_fd + 1, &fds, NULL, NULL, &tv);
        if (ret > 0 && FD_ISSET(udev_fd, &fds)) {
            udev_dev = udev_monitor_receive_device(udev_mon);
            if (udev_dev) {
                std::string device = udev_device_get_sysname(udev_dev);
                std::string action = udev_device_get_action(udev_dev);

                if ((action == "add" || action == "change") &&
                    device.find("card") != std::string::npos) {
                    console->critical("Action:{} | Device:{}", action, device);
                    detectRecordingDevices();
                    parseHardwareParams();
                } else if (action == "remove" && device.find("card") != std::string::npos) {
                    stopMonitoring();
                    recording_ = false;
                    canRecordAudio = false;
                    audioFormat = "";
                    defaultDevice.clear();
                    audioSampleRate = 0;
                    audioChannels = 0;
                    clearRecorderVuMeter();
                    console->critical("Sound card removed!");
                }
                udev_device_unref(udev_dev);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void CinePISound::resetTakeMetadata()
{
    takeStartTimeCode_.fill(0);
    takeStartOriginationDate_.fill(0);
    takeStartFramerate_ = 0.0;
    takeStartMetadataValid_ = false;
}

std::string CinePISound::generateIXML(const std::array<uint8_t, 8> &timecode,
                                      double framerate,
                                      const std::string &timecodeSource,
                                      const std::string &audioCapturePath,
                                      const std::string &audioStartMarkerStatus,
                                      bool haveAudioStartOffset,
                                      double audioStartOffsetSeconds,
                                      int audioStartOffsetFrames,
                                      long long audioStartOffsetSamples) const {
    boost::property_tree::ptree tree;

    tree.put("BWFXML.IXML_VERSION", "1.5");
    tree.put("BWFXML.PROJECT", "CinePI V2");
    tree.put("BWFXML.NOTE", "CinePI Note");
    tree.put("BWFXML.CIRCLED", "true");
    tree.put("BWFXML.TAPE", "CINEPI");
    tree.put("BWFXML.TIMECODE", buildTimecodeString(timecode));
    const double timecodeFramerate = nominalTimecodeFramerate(framerate);
    boost::rational<int> r = doubleToRational(timecodeFramerate > 0.0 ? timecodeFramerate : 25.0);
    std::string tfps = std::to_string(r.numerator()) + "/" + std::to_string(r.denominator());

    tree.put("BWFXML.SPEED.MASTER_SPEED", tfps);
    tree.put("BWFXML.SPEED.CURRENT_SPEED", tfps);
    tree.put("BWFXML.SPEED.TIMECODE_RATE", tfps);
    tree.put("BWFXML.SPEED.TIMECODE_FLAG", "NDF");
    tree.put("BWFXML.CINEPI_TIMECODE_SOURCE", timecodeSource);
    tree.put("BWFXML.CINEPI_AUDIO_CAPTURE_PATH", audioCapturePath);
    tree.put("BWFXML.CINEPI_AUDIO_START_MARKER_STATUS", audioStartMarkerStatus);
    if (haveAudioStartOffset) {
        std::ostringstream offsetSeconds;
        offsetSeconds << std::showpos << std::fixed << std::setprecision(6) << audioStartOffsetSeconds;
        tree.put("BWFXML.CINEPI_AUDIO_START_OFFSET_SECONDS", offsetSeconds.str());
        tree.put("BWFXML.CINEPI_AUDIO_START_OFFSET_FRAMES", std::to_string(audioStartOffsetFrames));
        tree.put("BWFXML.CINEPI_AUDIO_START_OFFSET_SAMPLES", std::to_string(audioStartOffsetSamples));
    }

    boost::property_tree::xml_writer_settings<std::string> settings('\t', 1);
    std::ostringstream oss;
    boost::property_tree::write_xml(oss, tree, settings);
    return oss.str();
}

bool CinePISound::appendIXMLChunk(const std::string& wav_path, const std::string& xml_payload) {
    std::fstream stream(wav_path, std::ios::in | std::ios::out | std::ios::binary);
    if (!stream) {
        console->error("appendIXMLChunk(): failed to open {}", wav_path);
        return false;
    }

    char riff_header[12];
    stream.read(riff_header, sizeof(riff_header));
    if (stream.gcount() != static_cast<std::streamsize>(sizeof(riff_header)) ||
        std::memcmp(riff_header, "RIFF", 4) != 0 ||
        std::memcmp(riff_header + 8, "WAVE", 4) != 0) {
        console->error("appendIXMLChunk(): {} is not a RIFF/WAVE file", wav_path);
        return false;
    }

    stream.seekp(0, std::ios::end);
    stream.write("iXML", 4);
    writeLe32(stream, static_cast<uint32_t>(xml_payload.size()));
    stream.write(xml_payload.data(), static_cast<std::streamsize>(xml_payload.size()));
    if (xml_payload.size() % 2 != 0)
        stream.put('\0');

    const std::streamoff file_size = stream.tellp();
    if (file_size < 8) {
        console->error("appendIXMLChunk(): invalid final WAV size for {}", wav_path);
        return false;
    }

    stream.seekp(4, std::ios::beg);
    writeLe32(stream, static_cast<uint32_t>(file_size - 8));
    return stream.good();
}

std::string CinePISound::getPreferredMonitorOutput() {
    std::string jackDetectCmd = "amixer get Headphone | grep '\\[on\\]'";
    int jackPresent = std::system(jackDetectCmd.c_str());
    if (WIFEXITED(jackPresent) && WEXITSTATUS(jackPresent) == 0) {
        console->info("Headphone jack detected — using analog output");
        return "plughw:CARD=Headphones,DEV=0"; // This should match your actual card name for jack
    } else {
        console->info("No headphones detected — using HDMI output (vc4hdmi0)");
        // Route through ALSA's default HDMI device so mono monitoring can be
        // converted to the sink's supported channel layout.
        return "default:CARD=vc4hdmi0";
    }
}

void CinePISound::parseHardwareParams() {
    audioFormat.clear();
    defaultDevice.clear();
    audioChannels  = 0;
    canRecordAudio = false;

    for (int rate : {FIXED_AUDIO_SAMPLE_RATE, FALLBACK_AUDIO_SAMPLE_RATE}) {
        if (tryAudioConfig("mic_24bit", "S24_3LE", 2, rate)) {
            audioSampleRate = rate;
            audioFormat    = "S24_3LE";
            audioChannels  = 2;
            defaultDevice  = "mic_24bit";
            canRecordAudio = true;
            console->info("parseHardwareParams(): using mic_24bit @ {} Hz", rate);
        } else if (tryAudioConfig("mic_16bit", "S16_LE", 1, rate)) {
            audioSampleRate = rate;
            audioFormat    = "S16_LE";
            audioChannels  = 1;
            defaultDevice  = "mic_16bit";
            canRecordAudio = true;
            console->info("parseHardwareParams(): using mic_16bit @ {} Hz", rate);
        } else {
            auto aliases = parseArecordAliases();
            for (const auto& alias : aliases) {
                for (int channels : {1, 2}) {
                    if (tryAudioConfig(alias, "S16_LE", channels, rate)) {
                        audioSampleRate = rate;
                        audioFormat    = "S16_LE";
                        audioChannels  = channels;
                        defaultDevice  = alias;
                        canRecordAudio = true;
                        console->info("parseHardwareParams(): using fallback alias {} ({} ch) @ {} Hz",
                                      alias, channels, rate);
                        break;
                    }
                }
                if (canRecordAudio) break;
            }
            if (!canRecordAudio) {
                console->warn("parseHardwareParams(): no usable audio devices at {} Hz; trying next rate", rate);
            }
        }

        if (canRecordAudio) {
            break;
        }
    }

    if (!canRecordAudio) {
        console->error("parseHardwareParams(): no usable audio devices after probing aliases");
    }

    if (canRecordAudio) {
        publishMicSelection();
    }

    stopMonitoring();

    if (canRecordAudio) {
        startMonitoring();
    }
}

void CinePISound::publishMicSelection() {
    if (!canRecordAudio) {
        return;
    }

    std::ostringstream rc;
    rc << "redis-cli MSET "
       << "MIC_PCM_ALIAS " << defaultDevice << ' '
       << "MIC_FORMAT " << audioFormat << ' '
       << "MIC_CHANNELS " << audioChannels << ' '
       << "MIC_RATE " << audioSampleRate;
    int r = std::system(rc.str().c_str());
    int code = (r >= 0 && WIFEXITED(r)) ? WEXITSTATUS(r) : -1;
    console->debug("Published MIC_* to Redis (rc={})", code);
}


void CinePISound::init_udev() {
    udev = udev_new();
    if (!udev) {
        console->error("Can't create udev");
        return;
    }

    udev_mon = udev_monitor_new_from_netlink(udev, "udev");
    udev_monitor_filter_add_match_subsystem_devtype(udev_mon, "sound", NULL);
    udev_monitor_enable_receiving(udev_mon);
    udev_fd = udev_monitor_get_fd(udev_mon);
}
