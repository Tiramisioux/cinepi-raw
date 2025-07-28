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
#ifdef HAVE_LGPIO
#include <lgpio.h>
#include <atomic>
#endif

using namespace std;
using namespace std::chrono;

// Keep the logging style consistent with the other CinePI modules
static std::shared_ptr<spdlog::logger> console = [] {
    auto lg = spdlog::stdout_color_mt("cinepi_hw_sync");
    lg->set_level(spdlog::level::debug); // match verbosity used elsewhere
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
         << "            [--chip NAME] [--line PIN]\n";
}

int main(int argc, char **argv)
{
    string source = "timer";
    double fps = 30.0;
    string group = "239.255.255.250";
    uint16_t port = 10000;
    string chipName = "gpiochip4";
    int line = -1;

    for (int i = 1; i < argc; ++i) {
        string arg = argv[i];
        if (arg == "--source" && i + 1 < argc) {
            source = argv[++i];
        } else if (arg == "--fps" && i + 1 < argc) {
            fps = stod(argv[++i]);
        } else if (arg == "--group" && i + 1 < argc) {
            group = argv[++i];
        } else if (arg == "--port" && i + 1 < argc) {
            port = static_cast<uint16_t>(stoi(argv[++i]));
        } else if (arg == "--chip" && i + 1 < argc) {
            chipName = argv[++i];
        } else if (arg == "--line" && i + 1 < argc) {
            line = stoi(argv[++i]);
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
    uint64_t frame = 0;
    console->info("libcamera-hw-sync started with source={} fps={}", source, fps);
    if (source == "gpio")
        console->info("GPIO chip={} line={}", chipName, line);

#ifdef HAVE_LGPIO
    int chip = -1;
    int rc = 0;
    GpioContext gpioctx;
#endif

#ifdef HAVE_LGPIO
    if (source == "gpio") {
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
#else
    if (source == "gpio") {
        console->error("GPIO source requested but lgpio not available");
        return 1;
    }
#endif

    uint64_t prevUs = 0;
    bool first = true;

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
#endif
        else {
            console->error("Unknown source type: {}", source);
            return 1;
        }

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
#endif

    return 0;
}

