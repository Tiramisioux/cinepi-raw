/* SPDX-License-Identifier: BSD-2-Clause */

#include "cinepi_options.hpp"

#include <boost/program_options.hpp>
#include <spdlog/spdlog.h>
#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <limits>
#include <optional>
#include <stdexcept>
#include <sstream>

using namespace boost::program_options;

CinePiOptions::CinePiOptions()
        : RawOptions()
        , same_hdmi(false)
        , keep16(false)
        , hdmi_port(-1)
{
        /* --------------------------------------------------------------
         * Register CinePi-only flags so that -h prints them.
         * Boost parses them too, but we still extract them manually
         * so they never reach RawOptions.
         * -------------------------------------------------------------- */
        options_description cinepi_group("CinePi-raw specific options");
        cinepi_group.add_options()
                ("cam-port",
                        value<std::string>()->implicit_value(""),
                        "Physical camera port to use (e.g. cam0 or cam1)")
                ("hdmi-port",
                        value<int>()->default_value(-1),
                        "For DRM preview choose HDMI socket (0 = HDMI-0, 1 = HDMI-1, -1 = auto)")
                ("same-hdmi",
                        value<bool>()->default_value(false)->implicit_value(true),
                        "Force preview and GUI to share the same HDMI output")
                ("keep16",
                        value<bool>()->default_value(false)->implicit_value(true),
                        "Write full 16-bit DNGs (disable 12-bit packing)")
                ("zoom",
                    value<float>()
                        ->implicit_value(1.0f, "1.0")   // 1st arg = float, 2nd = help text
                        ->default_value(1.0f),
                    "Digital zoom factor for streams 0 & 2 "
                    "(1.0 = full frame, 2.0 = 200 % centre-crop)")
                // ("zoom-raw",
                //         value<bool>()->default_value(false)->implicit_value(true),
                //         "Apply --zoom crop to the RAW stream as well")
                //         ;
                ("scaler-crops",
                    value<std::string>()->implicit_value(""),
                    "Per-stream crop rectangles as fractions:\n"
                    "x,y,w,h[:x,y,w,h ...]   (0-1, up to 3 streams)")
                ("encode-workers",
                    value<unsigned int>()->default_value(4),
                    "Number of DNG encode worker threads")
                ("disk-workers",
                    value<unsigned int>()->default_value(2),
                    "Number of DNG disk writer threads")
                ("encode-affinity",
                    value<std::string>()->implicit_value(""),
                    "CPU list (e.g. 4,5 or 2-3) to pin encode workers")
                ("disk-affinity",
                    value<std::string>()->implicit_value(""),
                    "CPU list (e.g. 0-1) to pin disk workers")
                ("encode-nice",
                    value<int>(),
                    "Nice level (-20..19) for encode workers")
                ("disk-nice",
                    value<int>(),
                    "Nice level (-20..19) for disk workers")
                ("preroll-ms",
                        value<unsigned int>()->default_value(300),
                        "Warm-up duration before writing DNG files (0 disables)")
                ("start-queue-frames",
                        value<unsigned int>()->default_value(6),
                        "Minimum queued frames before starting disk writes")
                ("ignore-start-frames",
                        value<unsigned int>()->default_value(12),
                        "Frames to ignore for drop detection after recording begins")
                ("sync-policy",
                        value<std::string>()->default_value("never"),
                        "Disk sync policy: never, take, or interval[=N]")
                ("sync-interval",
                        value<unsigned int>()->default_value(0),
                        "When --sync-policy=interval, fdatasync every N frames")
                ("drop-cache-after-close",
                        value<bool>()->default_value(false)->implicit_value(true),
                        "Call posix_fadvise(..., DONTNEED) after each frame (off by default)")
                ("selftest",
                        value<bool>()->default_value(false)->implicit_value(true),
                        "Run synthetic disk self-test and exit")
                ("selftest-seconds",
                        value<unsigned int>()->default_value(1),
                        "Duration in seconds for --selftest runs");
        options_.add(cinepi_group);
}

/* helper: convert "x,y,w,h" string to four floats */
static std::array<float,4> parseCrop(const std::string &tok)
{
    if (tok.empty())                  // “::”  → empty rectangle
        return { 0.f, 0.f, 0.f, 0.f };

    float x,y,w,h;
    if (sscanf(tok.c_str(), "%f,%f,%f,%f", &x,&y,&w,&h) != 4)
        throw std::runtime_error("Invalid --scaler-crops token: " + tok);
    return { x,y,w,h };
}


static std::string trimToken(const std::string &token)
{
        const auto start = token.find_first_not_of(" \t");
        if (start == std::string::npos)
                return "";
        const auto end = token.find_last_not_of(" \t");
        return token.substr(start, end - start + 1);
}

static std::string toLower(std::string value)
{
        std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
                return static_cast<char>(std::tolower(c));
        });
        return value;
}

static unsigned int parseWorkerCount(const std::string &flag, const std::string &value)
{
        try
        {
                size_t pos = 0;
                unsigned long parsed = std::stoul(value, &pos, 10);
                if (pos != value.size())
                        throw std::runtime_error(flag + " contains trailing characters: " + value.substr(pos));
                if (parsed == 0)
                        throw std::runtime_error(flag + " must be greater than zero");
                if (parsed > std::numeric_limits<uint32_t>::max())
                        throw std::runtime_error(flag + " exceeds supported range");
                return static_cast<unsigned int>(parsed);
        }
        catch (const std::invalid_argument &)
        {
                throw std::runtime_error(flag + " requires a positive integer value");
        }
        catch (const std::out_of_range &)
        {
                throw std::runtime_error(flag + " is out of range");
        }
}

static unsigned int parseUnsignedOption(const std::string &flag,
                                        const std::string &value,
                                        unsigned int min_value,
                                        unsigned int max_value = std::numeric_limits<unsigned int>::max())
{
        try
        {
                size_t pos = 0;
                unsigned long parsed = std::stoul(value, &pos, 10);
                if (pos != value.size())
                        throw std::runtime_error(flag + " contains trailing characters: " + value.substr(pos));
                if (parsed < min_value)
                        throw std::runtime_error(flag + " must be >= " + std::to_string(min_value));
                if (parsed > max_value)
                        throw std::runtime_error(flag + " exceeds supported range");
                return static_cast<unsigned int>(parsed);
        }
        catch (const std::invalid_argument &)
        {
                throw std::runtime_error(flag + " requires an unsigned integer value");
        }
        catch (const std::out_of_range &)
        {
                throw std::runtime_error(flag + " is out of range");
        }
}

static RawSyncPolicy parseSyncPolicy(const std::string &flag,
                                              const std::string &value,
                                              RawOptions &options)
{
        std::string lower = toLower(value);

        auto setInterval = [&](const std::string &interval_str) {
                if (interval_str.empty())
                        return;
                options.sync_interval = parseUnsignedOption(flag, interval_str, 1);
        };

        if (lower == "never")
        {
                options.sync_interval = 0;
                return RawSyncPolicy::Never;
        }
        if (lower == "take")
        {
                options.sync_interval = 0;
                return RawSyncPolicy::Take;
        }
        if (lower.rfind("interval", 0) == 0)
        {
                size_t pos = lower.find_first_of("=:");
                if (pos != std::string::npos)
                        setInterval(lower.substr(pos + 1));
                return RawSyncPolicy::Interval;
        }

        throw std::runtime_error(flag + " must be one of never, take, interval[=N]");
}

static const char *syncPolicyName(RawSyncPolicy policy)
{
        switch (policy)
        {
        case RawSyncPolicy::Never:
                return "never";
        case RawSyncPolicy::Take:
                return "take";
        case RawSyncPolicy::Interval:
                return "interval";
        }
        return "unknown";
}

static bool parseBoolOption(const std::string &flag, const std::string &value)
{
        std::string lower = toLower(value);
        if (lower == "1" || lower == "true" || lower == "yes" || lower == "on")
                return true;
        if (lower == "0" || lower == "false" || lower == "no" || lower == "off")
                return false;
        throw std::runtime_error(flag + " must be true/false (or 1/0)");
}

static int parseNiceValue(const std::string &flag, const std::string &value)
{
        try
        {
                size_t pos = 0;
                int parsed = std::stoi(value, &pos, 10);
                if (pos != value.size())
                        throw std::runtime_error(flag + " contains trailing characters: " + value.substr(pos));
                if (parsed < -20 || parsed > 19)
                        throw std::runtime_error(flag + " must be between -20 and 19");
                return parsed;
        }
        catch (const std::invalid_argument &)
        {
                throw std::runtime_error(flag + " requires an integer value");
        }
        catch (const std::out_of_range &)
        {
                throw std::runtime_error(flag + " is out of range");
        }
}

static std::vector<int> parseCpuList(const std::string &flag, const std::string &value)
{
        if (value.empty())
                throw std::runtime_error(flag + " requires a CPU list");

        std::vector<int> cpus;
        std::stringstream ss(value);
        std::string token;

        while (std::getline(ss, token, ','))
        {
                token = trimToken(token);
                if (token.empty())
                        throw std::runtime_error(flag + " contains an empty entry");

                auto dash = token.find('-');
                if (dash == std::string::npos)
                {
                        size_t pos = 0;
                        int cpu = std::stoi(token, &pos, 10);
                        if (pos != token.size() || cpu < 0)
                                throw std::runtime_error(flag + " has invalid CPU index: " + token);
                        cpus.push_back(cpu);
                        continue;
                }

                std::string start_str = trimToken(token.substr(0, dash));
                std::string end_str   = trimToken(token.substr(dash + 1));
                if (start_str.empty() || end_str.empty())
                        throw std::runtime_error(flag + " has invalid range: " + token);

                size_t pos1 = 0;
                size_t pos2 = 0;
                int start_cpu = std::stoi(start_str, &pos1, 10);
                int end_cpu   = std::stoi(end_str, &pos2, 10);
                if (pos1 != start_str.size() || pos2 != end_str.size() || start_cpu < 0 || end_cpu < 0)
                        throw std::runtime_error(flag + " has invalid range: " + token);
                if (end_cpu < start_cpu)
                        throw std::runtime_error(flag + " has descending range: " + token);

                for (int cpu = start_cpu; cpu <= end_cpu; ++cpu)
                        cpus.push_back(cpu);
        }

        if (cpus.empty())
                throw std::runtime_error(flag + " resolved to an empty CPU set");

        std::sort(cpus.begin(), cpus.end());
        cpus.erase(std::unique(cpus.begin(), cpus.end()), cpus.end());
        return cpus;
}

static std::string cpuListToString(const std::optional<std::vector<int>> &cpus)
{
        if (!cpus || cpus->empty())
                return std::string("auto");

        std::ostringstream oss;
        for (size_t i = 0; i < cpus->size(); ++i)
        {
                if (i)
                        oss << ',';
                oss << (*cpus)[i];
        }
        return oss.str();
}


bool CinePiOptions::Parse(int argc, char *argv[])
{
        std::vector<char *> forward;            // args for RawOptions
        forward.reserve(argc);
        forward.push_back(argv[0]);              // program name

        std::string scaler_crops_str;            // raw string for help / logging

        /* ------------------------------------------------------------------
         * Manually pull out CinePi flags so RawOptions never sees them.
         * ------------------------------------------------------------------ */
        for (int i = 1; i < argc; ++i) {
                std::string arg = argv[i];

                /* cam-port ------------------------------------------------- */
                if (arg.rfind("--cam-port=", 0) == 0) {
                        camPort = arg.substr(sizeof("--cam-port=") - 1);
                        continue;
                }
                if (arg == "--cam-port") {
                        if (i + 1 >= argc)
                                throw std::runtime_error("--cam-port requires a value");
                        camPort = argv[++i];
                        continue;
                }

                /* hdmi-port ----------------------------------------------- */
                if (arg.rfind("--hdmi-port=", 0) == 0) {
                        hdmi_port = std::stoi(arg.substr(sizeof("--hdmi-port=") - 1));
                        continue;
                }
                if (arg == "--hdmi-port") {
                        if (i + 1 >= argc)
                                throw std::runtime_error("--hdmi-port requires a value");
                        hdmi_port = std::stoi(argv[++i]);
                        continue;
                }

                /* same-hdmi / keep16 -------------------------------------- */
                if (arg == "--same-hdmi") { same_hdmi = true; continue; }
                if (arg == "--keep16")    { keep16    = true; continue; }

                if (arg.rfind("--zoom=", 0) == 0) {
                        SetZoom(std::stof(arg.substr(sizeof("--zoom=") - 1)));
                        continue;
                }

                if (arg == "--zoom") {
                        if (i + 1 >= argc)
                                throw std::runtime_error("--zoom requires a value");
                         SetZoom(std::stof(argv[++i]));
                        continue;
                }


                if (arg.rfind("--scaler-crops=", 0) == 0) {
                        scaler_crops_str = arg.substr(sizeof("--scaler-crops=") - 1);
                        continue;
                }

                if (arg == "--scaler-crops") {
                        if (i + 1 >= argc)
                                throw std::runtime_error("--scaler-crops requires a value");
                        scaler_crops_str = argv[++i];
                        continue;
                }

                if (arg.rfind("--encode-workers=", 0) == 0) {
                        RawOptions::encode_workers = parseWorkerCount("--encode-workers", arg.substr(sizeof("--encode-workers=") - 1));
                        continue;
                }
                if (arg == "--encode-workers") {
                        if (i + 1 >= argc)
                                throw std::runtime_error("--encode-workers requires a value");
                        RawOptions::encode_workers = parseWorkerCount("--encode-workers", argv[++i]);
                        continue;
                }

                if (arg.rfind("--disk-workers=", 0) == 0) {
                        RawOptions::disk_workers = parseWorkerCount("--disk-workers", arg.substr(sizeof("--disk-workers=") - 1));
                        continue;
                }
                if (arg == "--disk-workers") {
                        if (i + 1 >= argc)
                                throw std::runtime_error("--disk-workers requires a value");
                        RawOptions::disk_workers = parseWorkerCount("--disk-workers", argv[++i]);
                        continue;
                }

                if (arg.rfind("--encode-affinity=", 0) == 0) {
                        RawOptions::encode_affinity = parseCpuList("--encode-affinity", arg.substr(sizeof("--encode-affinity=") - 1));
                        continue;
                }
                if (arg == "--encode-affinity") {
                        if (i + 1 >= argc)
                                throw std::runtime_error("--encode-affinity requires a value");
                        RawOptions::encode_affinity = parseCpuList("--encode-affinity", argv[++i]);
                        continue;
                }

                if (arg.rfind("--disk-affinity=", 0) == 0) {
                        RawOptions::disk_affinity = parseCpuList("--disk-affinity", arg.substr(sizeof("--disk-affinity=") - 1));
                        continue;
                }
                if (arg == "--disk-affinity") {
                        if (i + 1 >= argc)
                                throw std::runtime_error("--disk-affinity requires a value");
                        RawOptions::disk_affinity = parseCpuList("--disk-affinity", argv[++i]);
                        continue;
                }

                if (arg.rfind("--encode-nice=", 0) == 0) {
                        RawOptions::encode_nice = parseNiceValue("--encode-nice", arg.substr(sizeof("--encode-nice=") - 1));
                        continue;
                }
                if (arg == "--encode-nice") {
                        if (i + 1 >= argc)
                                throw std::runtime_error("--encode-nice requires a value");
                        RawOptions::encode_nice = parseNiceValue("--encode-nice", argv[++i]);
                        continue;
                }

                if (arg.rfind("--disk-nice=", 0) == 0) {
                        RawOptions::disk_nice = parseNiceValue("--disk-nice", arg.substr(sizeof("--disk-nice=") - 1));
                        continue;
                }
                if (arg == "--disk-nice") {
                        if (i + 1 >= argc)
                                throw std::runtime_error("--disk-nice requires a value");
                        RawOptions::disk_nice = parseNiceValue("--disk-nice", argv[++i]);
                        continue;
                }

                if (arg.rfind("--preroll-ms=", 0) == 0) {
                        RawOptions::preroll_ms = parseUnsignedOption("--preroll-ms", arg.substr(sizeof("--preroll-ms=") - 1), 0);
                        continue;
                }
                if (arg == "--preroll-ms") {
                        if (i + 1 >= argc)
                                throw std::runtime_error("--preroll-ms requires a value");
                        RawOptions::preroll_ms = parseUnsignedOption("--preroll-ms", argv[++i], 0);
                        continue;
                }

                if (arg.rfind("--start-queue-frames=", 0) == 0) {
                        RawOptions::start_queue_frames = parseUnsignedOption("--start-queue-frames", arg.substr(sizeof("--start-queue-frames=") - 1), 0);
                        continue;
                }
                if (arg == "--start-queue-frames") {
                        if (i + 1 >= argc)
                                throw std::runtime_error("--start-queue-frames requires a value");
                        RawOptions::start_queue_frames = parseUnsignedOption("--start-queue-frames", argv[++i], 0);
                        continue;
                }

                if (arg.rfind("--ignore-start-frames=", 0) == 0) {
                        RawOptions::ignore_start_frames = parseUnsignedOption("--ignore-start-frames", arg.substr(sizeof("--ignore-start-frames=") - 1), 0);
                        continue;
                }
                if (arg == "--ignore-start-frames") {
                        if (i + 1 >= argc)
                                throw std::runtime_error("--ignore-start-frames requires a value");
                        RawOptions::ignore_start_frames = parseUnsignedOption("--ignore-start-frames", argv[++i], 0);
                        continue;
                }

                if (arg.rfind("--sync-policy=", 0) == 0) {
                        RawOptions::sync_policy = parseSyncPolicy("--sync-policy", arg.substr(sizeof("--sync-policy=") - 1), *this);
                        continue;
                }
                if (arg == "--sync-policy") {
                        if (i + 1 >= argc)
                                throw std::runtime_error("--sync-policy requires a value");
                        RawOptions::sync_policy = parseSyncPolicy("--sync-policy", argv[++i], *this);
                        continue;
                }

                if (arg.rfind("--sync-interval=", 0) == 0) {
                        RawOptions::sync_interval = parseUnsignedOption("--sync-interval", arg.substr(sizeof("--sync-interval=") - 1), 1);
                        RawOptions::sync_policy = RawSyncPolicy::Interval;
                        continue;
                }
                if (arg == "--sync-interval") {
                        if (i + 1 >= argc)
                                throw std::runtime_error("--sync-interval requires a value");
                        RawOptions::sync_interval = parseUnsignedOption("--sync-interval", argv[++i], 1);
                        RawOptions::sync_policy = RawSyncPolicy::Interval;
                        continue;
                }

                if (arg.rfind("--drop-cache-after-close=", 0) == 0) {
                        RawOptions::drop_cache_after_close = parseBoolOption("--drop-cache-after-close", arg.substr(sizeof("--drop-cache-after-close=") - 1));
                        continue;
                }
                if (arg == "--drop-cache-after-close") {
                        RawOptions::drop_cache_after_close = true;
                        continue;
                }

                if (arg.rfind("--selftest=", 0) == 0) {
                        RawOptions::selftest = parseBoolOption("--selftest", arg.substr(sizeof("--selftest=") - 1));
                        continue;
                }
                if (arg == "--selftest") {
                        RawOptions::selftest = true;
                        continue;
                }

                if (arg.rfind("--selftest-seconds=", 0) == 0) {
                        RawOptions::selftest_seconds = parseUnsignedOption("--selftest-seconds", arg.substr(sizeof("--selftest-seconds=") - 1), 1);
                        continue;
                }
                if (arg == "--selftest-seconds") {
                        if (i + 1 >= argc)
                                throw std::runtime_error("--selftest-seconds requires a value");
                        RawOptions::selftest_seconds = parseUnsignedOption("--selftest-seconds", argv[++i], 1);
                        continue;
                }

                /* not a CinePi flag – forward it */
                forward.push_back(argv[i]);
        }

        /* Base class parses all remaining (rpicam-apps) options              */
        int new_argc = static_cast<int>(forward.size());
        bool ok = RawOptions::Parse(new_argc, forward.data());

        /* --------------------------------------------------------------
         * Post-process scaler-crops string into vector<array<float,4>>
         * -------------------------------------------------------------- */
        if (!scaler_crops_str.empty()) {
                std::stringstream ss(scaler_crops_str);
                std::string tok;
                while (std::getline(ss, tok, ':'))
                        scaler_crops_rects.emplace_back(parseCrop(tok));
        }

        if (scaler_crops_rects.empty() && Zoom() != 1.0f)
        {
        float w = 1.0f / Zoom();
        float h = w;
        float x = (1.0f - w) / 2.0f;
        float y = x;
        std::array<float,4> r{ x, y, w, h };

        scaler_crops_rects = {
                r,                                 // stream-0 preview
                ZoomRaw() ? r : std::array<float,4>{0,0,1,1},  // stream-1 RAW
                r                                  // stream-2 lo-res
        };
        }

        /* Derive default camPort if user left it blank */
        if (camPort.empty())
                camPort = "cam" + std::to_string(camera);

        /* Update base class member so utils see the correct value */
        RawOptions::camPort = camPort;

        /* Log summary ------------------------------------------------- */
        spdlog::info("cinepi-cli: camPort='{}'  hdmi_port={}  same_hdmi={}  zoom={}  crops={}"
                     "crops={} rectangles",
                     camPort,
                     hdmi_port,
                     same_hdmi ? "true" : "false",
                     Zoom(),
                     scaler_crops_rects.size());

        spdlog::info("cinepi-cli: encode_workers={} disk_workers={} encode_affinity={} disk_affinity={} encode_nice={} disk_nice={}",
                      RawOptions::encode_workers,
                      RawOptions::disk_workers,
                      cpuListToString(RawOptions::encode_affinity),
                      cpuListToString(RawOptions::disk_affinity),
                      RawOptions::encode_nice ? std::to_string(*RawOptions::encode_nice) : std::string("auto"),
                      RawOptions::disk_nice ? std::to_string(*RawOptions::disk_nice) : std::string("auto"));

        spdlog::info("cinepi-cli: preroll={}ms start_queue={} ignore_start={} sync_policy={} sync_interval={} drop_cache={} selftest={} duration={}s",
                     RawOptions::preroll_ms,
                     RawOptions::start_queue_frames,
                     RawOptions::ignore_start_frames,
                     syncPolicyName(RawOptions::sync_policy),
                     RawOptions::sync_interval,
                     RawOptions::drop_cache_after_close ? "on" : "off",
                     RawOptions::selftest ? "on" : "off",
                     RawOptions::selftest_seconds);

        return ok;
}
