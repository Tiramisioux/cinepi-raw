#include "cinepi_sound.hpp"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/rational.hpp>
#include <boost/numeric/conversion/cast.hpp>
#include <fstream>
#include <sys/wait.h>

constexpr int FIXED_AUDIO_SAMPLE_RATE = 48000;

constexpr double AUDIO_TRIM_OFFSET_MS = 120.0; // milliseconds

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

bool file_exists(const std::string& path) {
    return std::filesystem::exists(path);
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
    if (!fp) return -1;
    char buf[256] = {0};
    if (fgets(buf, sizeof(buf), fp)) first_line = buf;
    int rc = pclose(fp);
    return rc;
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
    int exit_code = (rc >= 0 && WIFEXITED(rc)) ? WEXITSTATUS(rc) : -1;

    if (exit_code == 0) {
        console->info("Probe OK: {} (fmt {}, ch {}, {} Hz)", device, format, channels, rate);
        return true;
    } else {
        console->debug("Probe FAILED rc={} : {} | {}", exit_code, cmd.str(), stderr_one);
        return false;
    }
}

void CinePISound::record_start() {
    console->info("record_start() called");

    if (!canRecordAudio) {
        console->warn("Audio recording not allowed (canRecordAudio = false)");
        return;
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
              << " --disable-resample"
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

void CinePISound::soundThread() {
    init_udev();

    while (!abortThread_) {
        if (record_ && (pid > 0)) {
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

            double vts_delta = (vts_end - vts_start) / 1e9;
            if (vts_start < static_cast<int64_t>(ts_first_buffer_b)) {
                console->critical("Frame start before audio!!!!");
                return;
            }

            double trim_offset_seconds = AUDIO_TRIM_OFFSET_MS / 1000.0;
            double start_time = (vts_start - ts_first_buffer_b) / 1e9 - trim_offset_seconds;

            std::ostringstream tmp_oss;
            tmp_oss << options_->mediaDest << '/' << options_->folder << "/temp.wav";
            std::ostringstream xml_oss;
            xml_oss << options_->mediaDest << '/' << options_->folder << "/" << options_->folder << ".xml";
            std::ostringstream ffmpeg_oss;
            ffmpeg_oss << "ffmpeg -y -i " << filename
                       << " -ss " << start_time
                       << " -t " << vts_delta
                       << " -ar " << audioSampleRate
                       << " -acodec pcm_s16le " << tmp_oss.str() << " > /dev/null 2>&1";

            system(ffmpeg_oss.str().c_str());
            system(("mv " + tmp_oss.str() + " " + filename).c_str());

            generateXML(xml_oss.str());

            if (file_exists(xml_oss.str())) {
                std::ostringstream bwfedit;
                bwfedit << "bwfmetaedit " << filename << " --in-iXML=" << xml_oss.str();
                std::ostringstream bwfedit_core;
                auto& oTC = app_->GetEncoder()->originationTimeCode;
                auto& oDt = app_->GetEncoder()->originationDate;

                uint64_t timeReference = (static_cast<uint64_t>(oTC[0]) * 3600 * audioSampleRate)
                                       + (static_cast<uint64_t>(oTC[1]) * 60 * audioSampleRate)
                                       + (static_cast<uint64_t>(oTC[2]) * audioSampleRate);

                std::ostringstream timeStr;
                timeStr << std::setw(2) << std::setfill('0') << static_cast<int>(oTC[0]) << ":"
                        << std::setw(2) << std::setfill('0') << static_cast<int>(oTC[1]) << ":"
                        << std::setw(2) << std::setfill('0') << static_cast<int>(oTC[2]);
                std::ostringstream dateStr;
                dateStr << std::setw(4) << std::setfill('0') << static_cast<int>(oDt[0]) << "-"
                        << std::setw(2) << static_cast<int>(oDt[1]) << "-"
                        << std::setw(2) << static_cast<int>(oDt[2]);

                bwfedit_core << "bwfmetaedit " << filename
                             << " --BextVersion=1"
                             << " --Description='CinePI Description'"
                             << " --Originator='" << options_->ucm.value_or("CinePI") << "'"
                             << " --OriginatorReference='" << options_->serial << "'"
                             << " --OriginationDate=" << dateStr.str()
                             << " --OriginationTime=" << timeStr.str()
                             << " --TimeReference=" << timeReference;

                system(bwfedit.str().c_str());
                system(("rm " + xml_oss.str()).c_str());
                system(bwfedit_core.str().c_str());
            } else {
                console->critical("XML does not exist!");
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

void CinePISound::generateXML(std::string fn) {
    boost::property_tree::ptree tree;

    tree.put("BWFXML.IXML_VERSION", "1.5");
    tree.put("BWFXML.PROJECT", "CinePI V2");
    tree.put("BWFXML.NOTE", "CinePI Note");
    tree.put("BWFXML.CIRCLED", "true");
    tree.put("BWFXML.TAPE", "CINEPI");

    boost::rational<int> r = doubleToRational(*options_->framerate);
    std::string tfps = std::to_string(r.numerator()) + "/" + std::to_string(r.denominator());

    tree.put("BWFXML.SPEED.MASTER_SPEED", tfps);
    tree.put("BWFXML.SPEED.CURRENT_SPEED", tfps);
    tree.put("BWFXML.SPEED.TIMECODE_RATE", tfps);
    tree.put("BWFXML.SPEED.TIMECODE_FLAG", "NDF");

    boost::property_tree::xml_writer_settings<std::string> settings('\t', 1);
    boost::property_tree::write_xml(fn, tree, std::locale(), settings);
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
    audioSampleRate = FIXED_AUDIO_SAMPLE_RATE;

    if (tryAudioConfig("mic_24bit", "S24_3LE", 2, audioSampleRate)) {
        audioFormat    = "S24_3LE";
        audioChannels  = 2;
        defaultDevice  = "mic_24bit";
        canRecordAudio = true;
        console->info("parseHardwareParams(): using mic_24bit");
    } else if (tryAudioConfig("mic_16bit", "S16_LE", 1, audioSampleRate)) {
        audioFormat    = "S16_LE";
        audioChannels  = 1;
        defaultDevice  = "mic_16bit";
        canRecordAudio = true;
        console->info("parseHardwareParams(): using mic_16bit");
    } else {
        audioFormat.clear();
        defaultDevice.clear();
        audioChannels   = 0;
        canRecordAudio  = false;
        console->error("parseHardwareParams(): no usable mic_* alias found");
    }

    if (canRecordAudio) {
        publishMicSelection();
    }

    if (monitor_pid > 0) {
        kill(-monitor_pid, SIGTERM);
        if (monitor_pipe) {
            pclose2(monitor_pipe, monitor_pid);
            monitor_pipe = nullptr;
        }
        monitor_pid = -1;
    }

    if (canRecordAudio) {
        std::string outputDevice = getPreferredMonitorOutput();
        std::ostringstream mon_cmd;
        mon_cmd << "alsaloop -C " << defaultDevice
                << " -P " << outputDevice
                << " -t 10000 -A 1 -d";
        console->info("Starting audio monitoring: {}", mon_cmd.str());
        monitor_pipe = popen2(mon_cmd.str(), "r", monitor_pid);
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