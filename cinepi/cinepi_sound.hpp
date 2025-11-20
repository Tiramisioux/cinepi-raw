#pragma once

#include <array>
#include <chrono>
#include <cstdio>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <regex>
#include <sstream>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>

#include <thread>
#include <pthread.h>
#include <sched.h>

#include <libudev.h>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include "cinepi_recorder.hpp"
#include "raw_options.hpp"

#define READ   0
#define WRITE  1

FILE * popen2(std::string command, std::string type, int & pid);
int pclose2(FILE * fp, pid_t pid);
uint64_t extractTime(const std::string& line);

class CinePISound {
public:
    CinePISound(CinePIRecorder *app);
    ~CinePISound();

    void start();
    void soundThread();
    void record_start();
    void record_stop();
    bool isRecording();

    std::array<int, 4> vu_meter;

private:
    void init_udev();
    void detectRecordingDevices();
    void parseHardwareParams();
    bool tryAudioConfig(const std::string& device, const std::string& format, int channels, int rate);
    bool recording_ended();
    void generateXML(std::string fn);
    void publishMicSelection();
    std::vector<std::string> parseArecordAliases();
    void stopMonitoring();
    void startMonitoring();




    int samples_captured;
    uint64_t ts_start, ts_first_buffer_b, ts_first_buffer_a, ts_close_file, ts_end;
    std::string audioFormat;
    int audioChannels;
    int audioSampleRate;

    FILE* arec_pipe;
    bool canRecordAudio;
    std::string defaultDevice;
    std::shared_ptr<spdlog::logger> console;
    int pid;
    bool recording_;
    bool record_;
    std::stringstream cmdStream;
    CinePIRecorder *app_;
    RawOptions *options_;
    bool abortThread_;
    std::thread sound_thread_;

    std::string getPreferredMonitorOutput();
    int monitor_pid = -1;
    FILE* monitor_pipe = nullptr;
    bool monitoring_ = false;

    struct udev *udev;
    struct udev_device *udev_dev;
    struct udev_monitor *udev_mon;
    int udev_fd;
    
};
