#include "cinepi_sound.hpp"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/rational.hpp>
#include <boost/numeric/conversion/cast.hpp>

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

    /* child process */
    if (child_pid == 0)
    {
        if (type == "r")
        {
            close(fd[READ]);    //Close the READ end of the pipe since the child's fd is write-only
            dup2(fd[WRITE], 1); //Redirect stdout to pipe
        }
        else
        {
            close(fd[WRITE]);    //Close the WRITE end of the pipe since the child's fd is read-only
            dup2(fd[READ], 0);   //Redirect stdin to pipe
        }

        setpgid(child_pid, child_pid); //Needed so negative PIDs can kill children of /bin/sh
        execl("/bin/sh", "/bin/sh", "-c", command.c_str(), NULL);
        exit(0);
    }
    else
    {
        if (type == "r")
        {
            close(fd[WRITE]); //Close the WRITE end of the pipe since parent's fd is read-only
        }
        else
        {
            close(fd[READ]); //Close the READ end of the pipe since parent's fd is write-only
        }
    }

    pid = child_pid;

    if (type == "r")
    {
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
uint64_t extractTime(const std::string& line) {
    size_t colon_pos = line.find(':');
    size_t dot_pos = line.find('.');
    size_t end_pos = line.find('>');

    uint64_t seconds = std::stoull(line.substr(colon_pos + 1, dot_pos - colon_pos - 1));
    uint64_t nanoseconds = std::stoull(line.substr(dot_pos + 1, end_pos - dot_pos - 1));

    return (seconds * 1e+9) + nanoseconds;
}

CinePISound::CinePISound(CinePIRecorder *app) : 
    app_(app),
    pid(-1),
    abortThread_(false),
    arec_pipe(nullptr),
    udev(nullptr),
    udev_dev(nullptr),
    udev_mon(nullptr),
    udev_fd(-1),
    vu_meter({0,0,0,0}),
    samples_captured(0),
    ts_first_buffer_b(0),
    record_(false),
    recording_(false),
    canRecordAudio(false),
    defaultDevice("hw:0,0"),
    options_(app->GetOptions()) 
{
    console = spdlog::stdout_color_mt("cinepi_sound");
}

CinePISound::~CinePISound() {
    abortThread_ = true;
    sound_thread_.join();
    udev_unref(udev);
}

void CinePISound::start() {
    detectRecordingDevices();
    if (canRecordAudio) {
        parseHardwareParams();
        sound_thread_ = std::thread(std::bind(&CinePISound::soundThread, this));
    }
}

void CinePISound::soundThread() {
    init_udev();

    while(!abortThread_){
        if(record_ && (pid > 0)){

            char buffer[256];
            std::string result = "";
            while(fgets(buffer, sizeof(buffer), arec_pipe) != NULL) {
                result += buffer;
                std::istringstream ss(result);
                std::string line;
                while (std::getline(ss, line)) {
                    if(line.empty()){
                        continue;
                    }
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

        if(recording_ended()){
            //check if the .wav file exists
            std::ostringstream fn_oss;
            fn_oss << options_->mediaDest << '/' 
                << options_->folder << '/' 
                << options_->folder << ".wav";
            std::string filename = fn_oss.str();

            if (!std::filesystem::exists(filename))
                break;

            /// after the .wav file is saved, use timestamps to match video length and conform to BWF standard by adding iXML. 
            double start_time, duration, audio_duration, vts_delta;
            audio_duration = (((double)samples_captured/(double)audioSampleRate));
        
            console->debug("rec start: {} samples: {} audio dur: {}", ts_first_buffer_b, samples_captured, audio_duration);
            int64_t vts_start, vts_end, frames;
            frames = app_->GetEncoder()->timestamps.size();
            if(frames > 0){
                vts_start = app_->GetEncoder()->timestamps.front();
                vts_end = app_->GetEncoder()->timestamps.back();
                vts_delta = (vts_end-vts_start)/1000000000.0;
                console->debug("rec start: {} rec_end: {} rec_dur: {} rec_frames: {}", vts_start, vts_end, vts_delta, frames);
            }

            if(vts_start < ts_first_buffer_b){
                console->critical("Frame start before audio!!!!");
                return;
            }

            start_time = (vts_start-ts_first_buffer_b)/1000000000.0;

            std::ostringstream xml_oss;
            xml_oss << options_->mediaDest << '/' 
                << options_->folder << '/' 
                << options_->folder << ".xml";
            std::string xml_filename = xml_oss.str();

            std::ostringstream tmp_oss;
            tmp_oss << options_->mediaDest << '/' 
                << options_->folder << '/' 
                << "temp.wav";
            std::string tmp_filename = tmp_oss.str();

            std::ostringstream ffmpeg_oss;
            ffmpeg_oss << "ffmpeg -i " << filename << " -ss " << start_time << " -c copy " << tmp_filename << " > /dev/null 2>&1";

            std::ostringstream mv_oss;
            mv_oss << "mv " << tmp_filename << " " << filename;

            std::ostringstream bwfedit;
            bwfedit << "bwfmetaedit " << filename << " --in-iXML=" << xml_filename;

            std::ostringstream bwfedit_rm_xml;
            bwfedit_rm_xml << "rm " << xml_filename;

            std::array<uint8_t, 8>& oTC = app_->GetEncoder()->originationTimeCode;
            std::array<uint16_t,3>& oDt = app_->GetEncoder()->originationDate;

            std::ostringstream originationTimeStream;
            originationTimeStream << std::setw(2) << std::setfill('0') << static_cast<int>(oTC[0]) << ":"
                                << std::setw(2) << std::setfill('0') << static_cast<int>(oTC[1]) << ":"
                                << std::setw(2) << std::setfill('0') << static_cast<int>(oTC[2]);
            std::string originationTime = originationTimeStream.str();

            std::stringstream originationDateStream;
            originationDateStream << std::setfill('0') << std::setw(4) << static_cast<int>(oDt[0]) << "-"
            << std::setw(2) << static_cast<int>(oDt[1]) << "-"
            << std::setw(2) << static_cast<int>(oDt[2]);
            std::string originationDate = originationDateStream.str(); 

            uint64_t timeReference = (static_cast<uint64_t>(oTC[0]) * 3600 * audioSampleRate) + (static_cast<uint64_t>(oTC[1]) * 60 * audioSampleRate) + (static_cast<uint64_t>(oTC[2]) * 1 * audioSampleRate);

            std::ostringstream bwfedit_core;
            bwfedit_core << "bwfmetaedit " << filename 
                << " --BextVersion="            << "1"
                << " --Description="            << "'CinePI Description'"
                << " --Originator='"            << options_->ucm.value_or("CinePI") << "'"
                << " --OriginatorReference='"    << options_->serial << "'"
                << " --OriginationDate="        << originationDate
                << " --OriginationTime="        << originationTime
                << " --Timereference="          << timeReference;

            system(ffmpeg_oss.str().c_str());
            system(mv_oss.str().c_str());
            generateXML(xml_filename);
            if(file_exists(xml_filename)){
                system(bwfedit.str().c_str());
                system(bwfedit_rm_xml.str().c_str());
                system(bwfedit_core.str().c_str());
            } else {
                console->critical("XML does not exist!");
            }

            vu_meter.fill(0);
        }

        {
            fd_set fds;
            struct timeval tv;
            int ret;

            FD_ZERO(&fds);
            FD_SET(udev_fd, &fds);
            tv.tv_sec = 0;
            tv.tv_usec = 0;

            ret = select(udev_fd+1, &fds, NULL, NULL, &tv);
            if (ret > 0 && FD_ISSET(udev_fd, &fds)) {
                udev_dev = udev_monitor_receive_device(udev_mon);
                if (udev_dev) {
                    
                    std::string device = std::string(udev_device_get_sysname(udev_dev));
                    std::string action = std::string(udev_device_get_action(udev_dev));

                    if(action == "change" && (strstr(device.c_str(), std::string("card").c_str()) != nullptr)){
                        console->critical("Action:{} | Device:{}", action, device);
                        detectRecordingDevices();
                        if (canRecordAudio) {
                            parseHardwareParams();
                        }
                    } else if(action == "remove" && (strstr(device.c_str(), std::string("card").c_str()) != nullptr)){
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
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10)); 
    }
}

void CinePISound::record_start() {
    if(!canRecordAudio)
        return;
    std::ostringstream oss;
    oss << options_->mediaDest << '/' 
        << options_->folder << '/' 
        << options_->folder << ".wav";

    std::string filename = oss.str();

    cmdStream.str("");
    cmdStream.clear();
    cmdStream << "arecord -D " << defaultDevice 
                << " -f " << audioFormat
                << " -c" << audioChannels
                << " -r " << audioSampleRate 
                << " -V mono"
                << " " << filename
                << " 2>&1";

    record_ = true;

    samples_captured = 0;
    ts_start = 0;
    ts_first_buffer_b = 0;
    ts_first_buffer_a = 0;
    ts_close_file = 0;
    ts_end = 0;

    arec_pipe = popen2(cmdStream.str(), "r", pid);
    if(pid > 0){
        recording_ = true;
    }
        
    console->debug("arecord started pid:{}", pid);
    console->info("Sound recording started.");
}

void CinePISound::record_stop() {
    if(!canRecordAudio)
        return;
    record_ = false;

    if(pid > 0){
        kill(pid+1, SIGTERM);
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
    return (recording_ && (ts_first_buffer_b > 0) || !canRecordAudio);
}


void CinePISound::detectRecordingDevices() {
    FILE* pipe = popen("arecord -l", "r");
    if(!pipe) {
        console->error("Failed to run arecord -l");
        canRecordAudio = false;
        return;
    }

    char buffer[128];
    std::string result = "";
    while(fgets(buffer, sizeof(buffer), pipe) != NULL) {
        result += buffer;
    }
    pclose(pipe);

    console->debug("{}", result);

    if(result.find("card ") == std::string::npos) {
        console->error("No recording devices detected!");
        canRecordAudio = false;
    } else {
        // Extract the default device or set your logic
        // For simplicity, here we're just setting the first detected device as the default
        size_t start = result.find("card ");
        size_t end = result.find(":", start);
        defaultDevice = "hw:" + result.substr(start + 5, 1) + ",0"; // assumes single-digit card numbers
    }

    canRecordAudio = true;
    console->debug("{}", defaultDevice);
}

void CinePISound::init_udev(){
    /* create udev object */
	udev = udev_new();
	if (!udev) {
        console->error("Can't create udev\n");
	}

    udev_mon = udev_monitor_new_from_netlink(udev, "udev");
	udev_monitor_filter_add_match_subsystem_devtype(udev_mon, "sound", NULL);
	udev_monitor_enable_receiving(udev_mon);
	udev_fd = udev_monitor_get_fd(udev_mon);
}

void CinePISound::parseHardwareParams() {

    audioFormat = "S16_LE";
    audioSampleRate = 48000;
    audioChannels = 1;
    return;

    std::string cmd = "arecord -D " + defaultDevice + " --dump-hw-params 2>&1";
    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe) {
        console->error("Failed to run arecord --dump-hw-params");
        return;
    }

    char buffer[256];
    std::string result = "";
    while (fgets(buffer, sizeof(buffer), pipe) != NULL) {
        result += buffer;
    }
    pclose(pipe);

    // Create a stringstream from the result
    std::stringstream ss(result);
    std::string line;

    // SAMPLE_BITS:\s*\[(\d+)\s+(\d+)\]
    // CHANNELS:\s*\[(\d+)\s+(\d+)\]
    // RATE:\s*\[(\d+)\s+(\d+)\]
    // \bFORMAT:\s+(.+)\s*$

    // Regular expressions for channels and rates
    std::regex channelsRegex("CHANNELS:\\s*\\[*([0-9]+)\\s*([0-9]*)\\]*");
    std::regex ratesRegex("RATE:\\s*\\[*([0-9]+)\\s*([0-9]*)\\]*");
    std::regex formatRegex("\\bFORMAT:\\s+(.+)\\s*$");

    std::smatch matches;
    while (std::getline(ss, line)) {
        // Match channels
        if (std::regex_search(line, matches, channelsRegex) && matches.size() > 0) {
            int audioChannelsMin = std::stoi(matches[1]);
            int audioChannelsMax = std::stoi(matches[2]);
            audioChannels = audioChannelsMax;
        }

        // Match rates
        if (std::regex_search(line, matches, ratesRegex) && matches.size() > 0) {
            int audioSampleRateMin = std::stoi(matches[1]);
            int audioSampleRateMax = std::stoi(matches[2]);
            audioSampleRate = audioSampleRateMax;
        }

        // Match format
        if (std::regex_search(line, matches, formatRegex) && matches.size() > 0) {
            std::vector<std::string> frmts;
            std::stringstream ss(matches[1]);
            std::string s;
            while (getline(ss, s, ' ')) {
                frmts.push_back(s);
            }

            if(frmts.size() > 0){
                audioFormat = frmts.back();
            } else {
                audioFormat = "S16_LE";
            }

        }
    }
    console->critical("HW CAP: {} {} {}", audioFormat, audioChannels, audioSampleRate);
}


void CinePISound::generateXML(std::string fn){
    boost::property_tree::ptree tree;

    tree.put("BWFXML.IXML_VERSION", "1.5");
    tree.put("BWFXML.PROJECT", "CinePI V2");
    tree.put("BWFXML.NOTE", "CinePI Note");
    tree.put("BWFXML.CIRCLED", "true");
    tree.put("BWFXML.TAPE", "CINEPI");

    boost::rational<int> rationalNumber = doubleToRational(*options_->framerate);

    std::string tfps = std::to_string(rationalNumber.numerator()) + "/" + std::to_string(rationalNumber.denominator());

    tree.put("BWFXML.SPEED.MASTER_SPEED", tfps);
    tree.put("BWFXML.SPEED.CURRENT_SPEED", tfps);
    tree.put("BWFXML.SPEED.TIMECODE_RATE", tfps);
    tree.put("BWFXML.SPEED.TIMECODE_FLAG", "NDF");

    const std::string filename = fn;
    boost::property_tree::xml_writer_settings<std::string> settings('\t', 1);
    boost::property_tree::write_xml(filename, tree, std::locale(), settings);
}