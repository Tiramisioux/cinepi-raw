/* SPDX-License-Identifier: BSD-2-Clause */
/*

 * cinepi_hw_sync.cpp - Helper application to send hardware sync pulses.
 * Based on libcamera-hw-sync example.
 */

#include <arpa/inet.h>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <unistd.h>
#include <errno.h>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <poll.h>
#ifdef HAVE_LGPIO
#include <lgpio.h>
#include <atomic>
#endif

using namespace std;
using namespace std::chrono;

static std::shared_ptr<spdlog::logger> console = [] {
    auto lg = spdlog::stdout_color_mt("cinepi_hw_sync");
    lg->set_level(spdlog::level::debug);      // keep full verbosity
    lg->set_pattern("%v");                    // <<< NEW – message only
    return lg;
}();



struct SyncPayload {
    uint32_t frameDuration;
    uint64_t systemFrameTimestamp;
    uint64_t wallClockFrameTimestamp;
    uint64_t systemReadyTime;
    uint64_t wallClockReadyTime;
};

#ifdef HAVE_LGPIO
struct GpioContext {
    std::mutex mtx;
    std::condition_variable cv;
    std::atomic<uint64_t> ts{0};
    std::atomic<bool> ready{false};
};

static void gpio_alert_cb(int num, lgGpioAlert_p alert, void *userdata)
{
    if (num > 0 && userdata) {
        auto *ctx = static_cast<GpioContext *>(userdata);
        {
            std::lock_guard<std::mutex> lk(ctx->mtx);
            ctx->ts = alert[num - 1].report.timestamp;
            ctx->ready = true;
        }
        ctx->cv.notify_one();
    }
}
#endif // HAVE_LGPIO

static void usage(const char *argv0)
{
    cout << "Usage: " << argv0
         << " [--source timer|stdin|gpio] [--fps N]\n"
         << "            [--group ADDRESS] [--port PORT]\n"
         << "            [--chip NAME] [--line PIN]\n"

         << "            [--out-pin PIN] [--out-fps N]";
}

int main(int argc, char **argv)
{
    string source = "timer";
    double fps = 30.0;
    double outFps = 30.0;
    string group = "239.255.255.250";
    uint16_t port = 10000;
    string chipName = "gpiochip4";
    int line = -1;
    int outLine = -1;

    for (int i = 1; i < argc; ++i) {
        string arg = argv[i];
        if (arg == "--source" && i + 1 < argc) {
            source = argv[++i];
        } else if (arg == "--fps" && i + 1 < argc) {
            fps = stod(argv[++i]);
            outFps = fps;
        } else if (arg == "--group" && i + 1 < argc) {
            group = argv[++i];
        } else if (arg == "--port" && i + 1 < argc) {
            port = static_cast<uint16_t>(stoi(argv[++i]));
        } else if (arg == "--chip" && i + 1 < argc) {
            chipName = argv[++i];
        } else if (arg == "--line" && i + 1 < argc) {
            line = stoi(argv[++i]);
        } else if (arg == "--out-pin" && i + 1 < argc) {
            outLine = stoi(argv[++i]);

        } else if (arg == "--out-fps" && i + 1 < argc) {
            outFps = stod(argv[++i]);
        } else {
            usage(argv[0]);
            return 0;
        }
    }

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        console->error("Failed to create socket: {}", strerror(errno));
        return 1;
    }
    console->info("UDP socket created");
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr(group.c_str());
    addr.sin_port = htons(port);

    microseconds frameDuration(static_cast<int>(1e6 / fps));
    microseconds outFrameDuration(static_cast<int>(1e6 / outFps));
    if (outFps != fps)
        console->info("Output pulse fps {} (main fps {})", outFps, fps);
    uint64_t frame = 0;
    console->info("libcamera-hw-sync started with source={} fps={}", source, fps);
    if (source == "gpio")
        console->info("GPIO chip={} line={}", chipName, line);
    if (outLine >= 0)
        console->info("Output pin {} enabled at {} fps", outLine, outFps);

#ifdef HAVE_LGPIO
    int chip = -1;
    int rc = 0;
    GpioContext gpioctx;
#endif

#ifdef HAVE_LGPIO
    if (source == "gpio" || outLine >= 0) {
        int num = 0;
        if (chipName.rfind("gpiochip", 0) == 0)
            num = stoi(chipName.substr(8));
        else
            num = stoi(chipName);

        chip = lgGpiochipOpen(num);
        if (chip < 0 && num == 4)
            chip = lgGpiochipOpen(0);
        if (chip < 0) {
            console->error("Failed to open {} or gpiochip0", chipName);
            return 1;
        }
        console->info("GPIO chip {} opened", chipName);

        if (source == "gpio") {
            rc = lgGpioClaimAlert(chip, 0, LG_RISING_EDGE, line, -1);
            if (rc < 0) {
                console->error("Failed to claim alert on line {}", line);
                return 1;
            }
            console->info("Alert claimed on GPIO line {}", line);

            rc = lgGpioSetAlertsFunc(chip, line, gpio_alert_cb, &gpioctx);
            if (rc < 0) {
                console->error("Failed to set alert callback");
                return 1;
            }
            console->info("GPIO alert callback installed");
        }

        if (outLine >= 0) {
            rc = lgGpioClaimOutput(chip, 0, outLine, 0);
            if (rc < 0) {
                console->error("Failed to claim output on line {}", outLine);
                return 1;
            }
            console->info("Output GPIO line {} claimed", outLine);
        }
    }
#else  /* ---------- sysfs fallback when HAVE_LGPIO is not defined ---------- */
    int  gpio_fd       = -1;
    bool gpio_exported = false;
    int  out_fd        = -1;
    bool out_exported  = false;
    int  out_global_line = -1;

    if (source == "gpio" || outLine >= 0) {
        /* Translate (chipName,line)  → global GPIO number used by sysfs */
        int global_line = line;
        if (chipName.rfind("gpiochip", 0) == 0) {
            int chip_idx = std::stoi(chipName.substr(8));
            std::string base_file =
                "/sys/class/gpio/gpiochip" + std::to_string(chip_idx) + "/base";
            FILE *f = fopen(base_file.c_str(), "r");
            if (!f) {
                console->error("Cannot read {} ({})", base_file, strerror(errno));
                return 1;
            }
            int base = 0;
            fscanf(f, "%d", &base);
            fclose(f);
            global_line = base + line;
        }

        std::string base = "/sys/class/gpio/gpio" + std::to_string(global_line);
        int out_global = outLine;
        if (outLine >= 0 && chipName.rfind("gpiochip", 0) == 0) {
            int chip_idx = std::stoi(chipName.substr(8));
            std::string base_file_out =
                "/sys/class/gpio/gpiochip" + std::to_string(chip_idx) + "/base";
            FILE *f = fopen(base_file_out.c_str(), "r");
            if (f) { int b = 0; fscanf(f, "%d", &b); fclose(f); out_global = b + outLine; }
        }
        out_global_line = out_global;
        std::string base_out;
        if (outLine >= 0)
            base_out = "/sys/class/gpio/gpio" + std::to_string(out_global);

        /* ------------------------------------------------------------------
         * 1. export the line (needs root or udev rule that gives you write
         *    access to /sys/class/gpio/export)
         * ------------------------------------------------------------------ */
        struct stat st{};
        if (stat(base.c_str(), &st) < 0) {
            int fd = open("/sys/class/gpio/export", O_WRONLY);
            if (fd < 0) {
                console->error("Cannot write /sys/class/gpio/export ({}). "
                               "Try running with sudo or add the user to the "
                               "'gpio' group.", strerror(errno));
                return 1;
            }
            std::string s = std::to_string(global_line);
            write(fd, s.c_str(), s.size());
            close(fd);
            gpio_exported = true;
        }

        if (outLine >= 0 && stat(base_out.c_str(), &st) < 0) {
            int fd = open("/sys/class/gpio/export", O_WRONLY);
            if (fd >= 0) {
                std::string s = std::to_string(out_global);
                write(fd, s.c_str(), s.size());
                close(fd);
                out_exported = true;
            }
        }

        /* wait until the kernel has created the directory */
        for (int i = 0; i < 100; ++i) {        // up to ~1 s total
            if (stat(base.c_str(), &st) == 0)
                break;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        if (stat(base.c_str(), &st) < 0) {
            console->error("GPIO {} did not appear under /sys/class/gpio", global_line);
            return 1;
        }
        if (outLine >= 0 && stat(base_out.c_str(), &st) < 0) {
            console->error("GPIO {} did not appear under /sys/class/gpio", out_global);
            return 1;
        }

        auto write_str = [](const std::string &path, const char *str) {
            int fd = open(path.c_str(), O_WRONLY);
            if (fd >= 0) { write(fd, str, strlen(str)); close(fd); }
            else         console->warn("Cannot open {} ({})", path, strerror(errno));
        };

        write_str(base + "/direction", "in");
        write_str(base + "/edge",      "rising");
        if (outLine >= 0) {
            write_str(base_out + "/direction", "out");
            write_str(base_out + "/value", "0");
        }

        /* open the value file non-blocking so we can poll() on it */
        std::string val = base + "/value";
        gpio_fd = open(val.c_str(), O_RDONLY | O_NONBLOCK);
        if (gpio_fd < 0) {
            console->error("Failed to open GPIO value file {} ({})", val, strerror(errno));
            return 1;
        }
        if (outLine >= 0) {
            out_fd = open((base_out + "/value").c_str(), O_WRONLY);
            if (out_fd < 0)
                console->warn("Failed to open output GPIO value file {}", base_out + "/value");
        }
        console->info("GPIO sysfs monitoring global line {}", global_line);
    }
#endif /* --------- end of sysfs fallback block --------- */

    uint64_t prevUs = 0;
    bool first = true;
    uint64_t lastOutUs = 0;

    while (true) {
        uint64_t nowUs;
        if (source == "timer") {
            this_thread::sleep_for(frameDuration);
            nowUs = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
        } else if (source == "stdin") {
            getchar();
            nowUs = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
        }
#ifdef HAVE_LGPIO
        else if (source == "gpio") {
            std::unique_lock<std::mutex> lk(gpioctx.mtx);
            gpioctx.cv.wait(lk, [&] { return gpioctx.ready.load(); });
            nowUs = gpioctx.ts.load() / 1000; // lgpio timestamp is in ns
            gpioctx.ready = false;
        }
#else
        else if (source == "gpio") {
            struct pollfd pfd{ gpio_fd, POLLPRI, 0 };
            poll(&pfd, 1, -1);
            lseek(gpio_fd, 0, SEEK_SET);
            char buf[8];
            read(gpio_fd, buf, sizeof(buf));
            nowUs = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
        }
#endif
        else {
            console->error("Unknown source type: {}", source);
            return 1;
        }

        console->info("Pulse {} received at {} us", frame, nowUs);

        if (!first) {
            uint64_t diff = nowUs - prevUs;
            uint64_t exp = frameDuration.count();
            if (diff < exp * 9 / 10 || diff > exp * 11 / 10)
                console->warn("Pulse interval {} us (expected ~{} us)", diff, exp);
            else
                console->debug("Pulse interval {} us", diff);
        }
        first = false;
        prevUs = nowUs;

        if (outLine >= 0 && (lastOutUs == 0 || nowUs - lastOutUs >= outFrameDuration.count())) {
#ifdef HAVE_LGPIO
            lgGpioWrite(chip, outLine, 1);
            this_thread::sleep_for(std::chrono::milliseconds(5));
            lgGpioWrite(chip, outLine, 0);
#else
            if (out_fd >= 0) {
                write(out_fd, "1", 1);
                fsync(out_fd);
                this_thread::sleep_for(std::chrono::milliseconds(5));
                lseek(out_fd, 0, SEEK_SET);
                write(out_fd, "0", 1);
                fsync(out_fd);
                lseek(out_fd, 0, SEEK_SET);
            }
#endif
            lastOutUs = nowUs;
        }

        SyncPayload payload{};
        payload.frameDuration = frameDuration.count();
        payload.wallClockFrameTimestamp = nowUs;
        payload.wallClockReadyTime = nowUs + 100 * payload.frameDuration;

        ssize_t ret = sendto(sock, &payload, sizeof(payload), 0,
                             reinterpret_cast<sockaddr *>(&addr), sizeof(addr));
        if (ret < 0)
            console->error("sendto failed: {}", strerror(errno));

        console->info("Frame {} sent", frame++);
    }

#ifdef HAVE_LGPIO
    if (chip >= 0) {
        lgGpiochipClose(chip);
        console->info("GPIO chip closed");
    }
#else
    if (gpio_fd >= 0)
        close(gpio_fd);
    if (out_fd >= 0)
        close(out_fd);
    if (gpio_exported || out_exported) {
        int fd = open("/sys/class/gpio/unexport", O_WRONLY);
        if (fd >= 0) {
            if (gpio_exported) {
                std::string s = std::to_string(line);
                write(fd, s.c_str(), s.size());
            }
            if (out_exported) {
                std::string s2 = std::to_string(out_global_line);
                write(fd, s2.c_str(), s2.size());
            }
            close(fd);
        }
    }
#endif

    return 0;
}

