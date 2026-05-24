#pragma once

#include <array>
#include <chrono>
#include <cstdio>
#include <csignal>
#include <cstdlib>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <memory>
#include <regex>
#include <sstream>
#include <sys/wait.h>
#include <unistd.h>
#include <mutex>
#include <optional>
#include <vector>

#include <thread>
#include <pthread.h>
#include <sched.h>

#include <libudev.h>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <sw/redis++/redis++.h>

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
    std::string generateIXML(const std::array<uint8_t, 8>& timecode,
                             double framerate,
                             const std::string& timecodeSource,
                             const std::string& audioCapturePath,
                             const std::string& audioStartMarkerStatus,
                             bool haveAudioStartOffset,
                             double audioStartOffsetSeconds,
                             int audioStartOffsetFrames,
                             long long audioStartOffsetSamples,
                             bool haveAudioAvailableDiagnostic,
                             const std::string& audioAvailableTimecode,
                             long long audioAvailableDeltaFrames,
                             double audioAvailableDeltaSeconds) const;
    bool appendIXMLChunk(const std::string& wav_path, const std::string& xml_payload);
    void resetTakeMetadata();
    void publishMicSelection();
    void initRedis();
    void publishRecorderVuMeter(bool force = false);
    void clearRecorderVuMeter();
    std::vector<std::string> parseArecordAliases();
    void stopMonitoring();
    void launchPendingRecordingStart();
    void startMonitoring();
    void startPlaybackMonitoring();
    void stopPlaybackMonitoring();
    void startIdleVuMonitoring();
    void stopIdleVuMonitoring();
    void idleVuThread();

    struct PendingAudioCapture
    {
        std::string command;
        std::string capture_path;
        bool stop_monitoring_before_launch = true;
        bool emits_helper_markers = true;
    };

    int samples_captured;
    int capturedAudioSampleRate;
    uint64_t ts_start, ts_first_buffer_b, ts_first_buffer_a, ts_close_file, ts_end;
    uint64_t ts_audio_start_realtime;
    uint64_t ts_audio_available_realtime;
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
    bool audio_capture_started_;
    bool audio_capture_emits_markers_ = true;
    std::string audio_capture_path_ = "unknown";
    std::mutex pending_audio_capture_mutex_;
    std::stringstream cmdStream;
    CinePIRecorder *app_;
    RawOptions *options_;
    std::unique_ptr<sw::redis::Redis> redis_;
    bool abortThread_;
    std::thread sound_thread_;
    std::array<int, 4> last_published_vu_{};
    std::chrono::steady_clock::time_point last_vu_publish_ts_{};
    std::array<uint8_t, 8> takeStartTimeCode_{};
    std::array<uint16_t, 3> takeStartOriginationDate_{};
    double takeStartFramerate_ = 0.0;
    std::optional<PendingAudioCapture> pending_audio_capture_;
    bool takeStartMetadataValid_ = false;

    std::string getPreferredMonitorOutput();
    int monitor_playback_pid_ = -1;
    FILE* monitor_playback_pipe_ = nullptr;
    bool monitoring_playback_ = false;
    int monitor_vu_pid_ = -1;
    FILE* monitor_vu_pipe_ = nullptr;
    bool monitoring_vu_ = false;
    std::thread idle_vu_thread_;

    struct udev *udev;
    struct udev_device *udev_dev;
    struct udev_monitor *udev_mon;
    int udev_fd;
    
};
