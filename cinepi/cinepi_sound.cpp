#include "cinepi_sound.hpp"
#include <algorithm>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/rational.hpp>
#include <boost/numeric/conversion/cast.hpp>
#include <cmath>
#include <cstring>
#include <ctime>
#include <fstream>
#include <limits>
#include <optional>
#include <regex>
#include <unordered_set>
#include <sys/wait.h>

constexpr int FIXED_AUDIO_SAMPLE_RATE = 48000;
constexpr int FALLBACK_AUDIO_SAMPLE_RATE = 44100;

// The first audio buffer marker lands after the hardware has already started
// filling the capture pipeline. Subtract this latency when estimating the
// point where the recorded content actually begins.
constexpr double AUDIO_CAPTURE_LATENCY_MS = 120.0; // milliseconds
constexpr double FILTER_EPSILON_SECONDS = 1.0e-6;

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

std::string buildAtempoFilter(double tempo)
{
    if (!std::isfinite(tempo) || tempo <= 0.0)
        return {};

    std::vector<double> stages;
    while (tempo < 0.5) {
        stages.push_back(0.5);
        tempo /= 0.5;
    }
    while (tempo > 2.0) {
        stages.push_back(2.0);
        tempo /= 2.0;
    }
    stages.push_back(tempo);

    std::ostringstream oss;
    bool first = true;
    for (double stage : stages) {
        if (!first)
            oss << ',';
        oss << "atempo=" << formatSeconds(stage);
        first = false;
    }
    return oss.str();
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

    const uint64_t hours = static_cast<uint64_t>(bcdToInt(timecode[3]));
    const uint64_t minutes = static_cast<uint64_t>(bcdToInt(timecode[2]));
    const uint64_t seconds = static_cast<uint64_t>(bcdToInt(timecode[1]));
    const uint64_t frames = static_cast<uint64_t>(bcdToInt(timecode[0]));

    uint64_t timeReference = ((hours * 3600ULL) + (minutes * 60ULL) + seconds) *
                             static_cast<uint64_t>(sampleRate);

    if (std::isfinite(framerate) && framerate > 0.0 && frames > 0) {
        const double frameSamples =
            static_cast<double>(frames) * static_cast<double>(sampleRate) / framerate;
        timeReference += static_cast<uint64_t>(std::llround(frameSamples));
    }

    return timeReference;
}

std::string ffmpegCodecForAudioFormat(const std::string &audioFormat)
{
    if (audioFormat == "S24_3LE")
        return "pcm_s24le";
    if (audioFormat == "S16_LE")
        return "pcm_s16le";
    return "pcm_s16le";
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

std::optional<ParsedWavMetadata> buildMetadataFromWallclockNs(int64_t timestampNs,
                                                              double framerate)
{
    if (timestampNs < 0)
        return std::nullopt;

    const time_t seconds = static_cast<time_t>(timestampNs / 1000000000LL);
    const int64_t subsecondNs = timestampNs % 1000000000LL;
    std::tm *localTime = localtime(&seconds);
    if (!localTime)
        return std::nullopt;

    int frame = 0;
    if (std::isfinite(framerate) && framerate > 0.0) {
        const double fraction = static_cast<double>(subsecondNs) / 1e9;
        frame = static_cast<int>(std::floor((fraction * framerate) + 1.0e-9));
        const int maxFrame = std::max(0, static_cast<int>(std::ceil(framerate)) - 1);
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
    metadata.framerate = framerate;
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

    const int year = 2000 + std::stoi(match[1].str());
    const int month = std::stoi(match[2].str());
    const int day = std::stoi(match[3].str());
    const int hour = std::stoi(match[4].str());
    const int minute = std::stoi(match[5].str());
    const int second = std::stoi(match[6].str());
    const double folderFramerate = static_cast<double>(std::stoi(match[7].str()));

    if (month < 1 || month > 12 || day < 1 || day > 31 ||
        hour < 0 || hour > 23 || minute < 0 || minute > 59 ||
        second < 0 || second > 59) {
        return std::nullopt;
    }

    ParsedWavMetadata metadata;
    // Take names are second-precision only, so frame 00 is the deterministic fallback.
    metadata.timecode = {
        intToBcd(0),
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
    metadata.framerate = folderFramerate > 0.0 ? folderFramerate : fallbackFramerate;
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

        setpgid(child_pid, child_pid);
        execl("/bin/sh", "/bin/sh", "-c", command.c_str(), NULL);
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
    ts_start(0),
    ts_first_buffer_b(0),
    ts_first_buffer_a(0),
    ts_close_file(0),
    ts_end(0),
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
    app_(app),
    options_(app->GetOptions()),
    abortThread_(false),
    monitor_pipe(nullptr),
    monitoring_(false),
    udev(nullptr),
    udev_dev(nullptr),
    udev_mon(nullptr),
    udev_fd(-1)
{
    console = spdlog::stdout_color_mt("cinepi_sound");
    console->set_level(spdlog::level::debug);  // or trace if you want even more

}

CinePISound::~CinePISound() {
    abortThread_ = true;
    if (sound_thread_.joinable())
        sound_thread_.join();
    udev_unref(udev);
}

void CinePISound::start() {
    detectRecordingDevices();
    parseHardwareParams();  // ensure audio config is ready before recording

    if (canRecordAudio) {
        sound_thread_ = std::thread(std::bind(&CinePISound::soundThread, this));
    } else {
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

    stopMonitoring();
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

    std::string vu_mode = (audioChannels == 2) ? "stereo" : "mono";

    cmdStream.str("");
    cmdStream.clear();
    cmdStream << "arecord"
              << " -D " << defaultDevice
              << " -f " << audioFormat
              << " -c " << audioChannels
              << " -r " << audioSampleRate
              << " -t wav"
              << " --disable-softvol"
              << " -V " << vu_mode
              << " " << filename << " 2>&1";

    console->info("Executing arecord: {}", cmdStream.str());

    samples_captured = 0;
    vu_meter.fill(0);
    ts_start = 0;
    ts_first_buffer_b = 0;
    ts_first_buffer_a = 0;
    ts_close_file = 0;
    ts_end = 0;

    arec_pipe = popen2(cmdStream.str(), "r", pid);

    if (!arec_pipe) {
        console->error("Failed to open pipe to arecord");
    } else {
        console->info("arecord process started successfully (pid={})", pid);
        recording_ = true;
        record_ = true;
    }
}

void CinePISound::record_stop() {
    if(!canRecordAudio)
        return;

    record_ = false;
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
    return (recording_ && ts_first_buffer_b > 0) || !canRecordAudio;
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
    if (monitor_pid > 0) {
        kill(-monitor_pid, SIGTERM);
        if (monitor_pipe) {
            pclose2(monitor_pipe, monitor_pid);
            monitor_pipe = nullptr;
        }
        monitor_pid = -1;
        monitoring_ = false;
        console->info("Stopped audio monitoring");
    }
}

void CinePISound::startMonitoring() {
    if (!canRecordAudio || recording_ || record_ || monitoring_) {
        return;
    }

    std::string outputDevice = getPreferredMonitorOutput();
    std::ostringstream mon_cmd;
    mon_cmd << "alsaloop -C " << defaultDevice
            << " -P " << outputDevice
            << " -t 10000 -A 1 -d"
            << " 2>/dev/null";
    console->info("Starting audio monitoring: {}", mon_cmd.str());
    monitor_pipe = popen2(mon_cmd.str(), "r", monitor_pid);
    if (monitor_pid > 0 && monitor_pipe) {
        monitoring_ = true;
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
        // Always drain the arecord pipe until the child exits, even after record_stop()
        // clears the record_ flag. Otherwise the pipe would never be closed and the WAV
        // file would remain incomplete/unwritten, resulting in 0 WAV clips.
        if (pid > 0) {
            char buffer[256];
            std::string result = "";
            while (fgets(buffer, sizeof(buffer), arec_pipe) != NULL) {
                result += buffer;
                std::istringstream ss(result);
                std::string line;
                while (std::getline(ss, line)) {
                    if (line.empty()) continue;
                    if (line.find("<VU:") != std::string::npos) {
                        sscanf(line.c_str(), "<VU:%d|%d|%d|%d>", &vu_meter[0], &vu_meter[1], &vu_meter[2], &vu_meter[3]);
                    } else if (line.find("<TS_START:") != std::string::npos) {
                        ts_start = extractTime(line);
                    } else if (line.find("<TS_FIRST_BUFFER_B:") != std::string::npos) {
                        ts_first_buffer_b = extractTime(line);
                    } else if (line.find("<TS_FIRST_BUFFER_A:") != std::string::npos) {
                        ts_first_buffer_a = extractTime(line);
                    } else if (line.find("<SAMPLES_CAPTURED:") != std::string::npos) {
                        sscanf(line.c_str(), "<SAMPLES_CAPTURED: %d>", &samples_captured);
                    } else if (line.find("<TS_CLOSE_FILE:") != std::string::npos) {
                        ts_close_file = extractTime(line);
                    } else if (line.find("<TS_END:") != std::string::npos) {
                        ts_end = extractTime(line);
                    }
                }
            }
            pclose2(arec_pipe, pid);
            pid = -1;
        }

        if (recording_ended()) {
            std::ostringstream fn_oss;
            fn_oss << options_->mediaDest << '/' << options_->folder << '/' << options_->folder << ".wav";
            std::string filename = fn_oss.str();

            if (!std::filesystem::exists(filename)) break;

            int64_t vts_start = 0, vts_end = 0;
            int64_t frames = app_->GetEncoder()->timestamps.size();
            if (frames > 0) {
                vts_start = app_->GetEncoder()->timestamps.front();
                vts_end = app_->GetEncoder()->timestamps.back();
            }

            if (frames <= 0 || vts_end < vts_start) {
                console->critical("Cannot retime WAV: invalid video timestamp range");
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
            if (audio_marker_ns == 0) {
                console->critical("Cannot retime WAV: no audio start marker received");
                continue;
            }

            const double latency_bias_seconds = AUDIO_CAPTURE_LATENCY_MS / 1000.0;
            const double audio_content_start_seconds =
                static_cast<double>(audio_marker_ns) / 1e9 - latency_bias_seconds;
            const double video_start_seconds =
                static_cast<double>(vts_start) / 1e9;
            const double start_delta_seconds =
                audio_content_start_seconds - video_start_seconds;

            double trim_start_seconds = 0.0;
            double pad_start_seconds = 0.0;
            if (start_delta_seconds < 0.0)
                trim_start_seconds = -start_delta_seconds;
            else
                pad_start_seconds = start_delta_seconds;

            auto probed_input_duration = probeDurationSeconds(filename);
            double input_duration_seconds = probed_input_duration.value_or(
                fallbackDurationFromSamples(samples_captured, audioSampleRate));

            if (input_duration_seconds <= 0.0) {
                console->critical("Cannot retime WAV: failed to determine audio duration");
                continue;
            }

            trim_start_seconds = std::clamp(trim_start_seconds, 0.0, input_duration_seconds);
            double content_input_duration_seconds =
                std::max(0.0, input_duration_seconds - trim_start_seconds);
            double content_target_duration_seconds =
                std::max(0.0, video_duration_seconds - pad_start_seconds);

            double tempo = 1.0;
            if (content_input_duration_seconds > FILTER_EPSILON_SECONDS &&
                content_target_duration_seconds > FILTER_EPSILON_SECONDS) {
                tempo = content_input_duration_seconds / content_target_duration_seconds;
            }

            console->info(
                "Retiming WAV: video {:.6f}s, input {:.6f}s, start delta {:+.6f}s, trim {:.6f}s, pad {:.6f}s, tempo {:.6f}",
                video_duration_seconds,
                input_duration_seconds,
                start_delta_seconds,
                trim_start_seconds,
                pad_start_seconds,
                tempo);

            std::ostringstream tmp_oss;
            tmp_oss << options_->mediaDest << '/' << options_->folder << "/temp.wav";

            std::vector<std::string> filters;
            if (trim_start_seconds > FILTER_EPSILON_SECONDS)
                filters.push_back("atrim=start=" + formatSeconds(trim_start_seconds));
            filters.push_back("asetpts=PTS-STARTPTS");

            if (std::abs(tempo - 1.0) > 1.0e-4) {
                auto atempo = buildAtempoFilter(tempo);
                if (!atempo.empty())
                    filters.push_back(atempo);
            }

            if (pad_start_seconds > FILTER_EPSILON_SECONDS) {
                long long pad_start_ms = llround(pad_start_seconds * 1000.0);
                filters.push_back("adelay=" + std::to_string(std::max<long long>(0, pad_start_ms)) + ":all=true");
            }

            filters.push_back("apad");

            std::ostringstream filter_oss;
            for (size_t i = 0; i < filters.size(); ++i) {
                if (i)
                    filter_oss << ',';
                filter_oss << filters[i];
            }

            std::ostringstream ffmpeg_oss;
            const auto &encoderTimecode = app_->GetEncoder()->originationTimeCode;
            const auto &encoderDate = app_->GetEncoder()->originationDate;
            double output_framerate =
                takeStartFramerate_ > 0.0 ? takeStartFramerate_ : configuredFramerate(options_);

            std::array<uint8_t, 8> metadataTimecode = encoderTimecode;
            std::array<uint16_t, 3> metadataDate = encoderDate;
            std::string metadataSource = "encoder";

            const int64_t audioContentStartNs =
                static_cast<int64_t>(std::llround(audio_content_start_seconds * 1e9));
            if (auto audioStartMetadata =
                    buildMetadataFromWallclockNs(audioContentStartNs, output_framerate)) {
                metadataTimecode = audioStartMetadata->timecode;
                metadataDate = audioStartMetadata->originationDate;
                output_framerate = audioStartMetadata->framerate;
                metadataSource = "audio-start";
            } else if (takeStartMetadataValid_) {
                metadataTimecode = takeStartTimeCode_;
                metadataDate = takeStartOriginationDate_;
                if (takeStartFramerate_ > 0.0)
                    output_framerate = takeStartFramerate_;
                metadataSource = "take-name";
            }

            const std::string timecode_tag = buildTimecodeString(metadataTimecode);
            const std::string origination_date = formatOriginationDate(metadataDate);
            const std::string origination_time = formatOriginationTime(metadataTimecode);
            const uint64_t time_reference =
                computeTimeReferenceSamples(metadataTimecode, audioSampleRate, output_framerate);

            const std::string ffmpeg_codec = ffmpegCodecForAudioFormat(audioFormat);

            ffmpeg_oss << "ffmpeg -hide_banner -loglevel error -y -i " << shellQuote(filename)
                       << " -filter:a " << shellQuote(filter_oss.str())
                       << " -t " << formatSeconds(video_duration_seconds)
                       << " -ac " << audioChannels
                       << " -ar " << audioSampleRate
                       << " -c:a " << ffmpeg_codec
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
                console->critical("ffmpeg WAV retime failed (rc={}): {}",
                                  shellExitCode(ffmpeg_status),
                                  ffmpeg_error.empty() ? "no stderr output" : ffmpeg_error);
                continue;
            }

            std::error_code rename_ec;
            std::filesystem::rename(tmp_oss.str(), filename, rename_ec);
            if (rename_ec) {
                console->critical("Failed to replace WAV with retimed version: {}", rename_ec.message());
                continue;
            }

            const std::string ixml = generateIXML(metadataTimecode, output_framerate);
            if (!appendIXMLChunk(filename, ixml)) {
                console->critical("Failed to append iXML chunk to WAV");
            } else {
                console->info("Attached WAV metadata: timecode {}, rate {}, source {}, BEXT + iXML",
                              timecode_tag,
                              output_framerate,
                              metadataSource);
            }

            vu_meter.fill(0);
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

                if (action == "change" && device.find("card") != std::string::npos) {
                    console->critical("Action:{} | Device:{}", action, device);
                    detectRecordingDevices();
                    if (canRecordAudio) parseHardwareParams();
                } else if (action == "remove" && device.find("card") != std::string::npos) {
                    recording_ = false;
                    canRecordAudio = false;
                    audioFormat = "";
                    audioSampleRate = 0;
                    audioChannels = 0;
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
                                      double framerate) const {
    boost::property_tree::ptree tree;

    tree.put("BWFXML.IXML_VERSION", "1.5");
    tree.put("BWFXML.PROJECT", "CinePI V2");
    tree.put("BWFXML.NOTE", "CinePI Note");
    tree.put("BWFXML.CIRCLED", "true");
    tree.put("BWFXML.TAPE", "CINEPI");
    tree.put("BWFXML.TIMECODE", buildTimecodeString(timecode));
    boost::rational<int> r = doubleToRational(framerate > 0.0 ? framerate : 25.0);
    std::string tfps = std::to_string(r.numerator()) + "/" + std::to_string(r.denominator());

    tree.put("BWFXML.SPEED.MASTER_SPEED", tfps);
    tree.put("BWFXML.SPEED.CURRENT_SPEED", tfps);
    tree.put("BWFXML.SPEED.TIMECODE_RATE", tfps);
    tree.put("BWFXML.SPEED.TIMECODE_FLAG", "NDF");

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
        return "hdmi:CARD=vc4hdmi0,DEV=0";
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
