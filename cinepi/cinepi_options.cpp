/* SPDX-License-Identifier: BSD-2-Clause */

#include "cinepi_options.hpp"

#include <boost/program_options.hpp>
#include <spdlog/spdlog.h>
#include <cstdlib>
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

                ("sync-source", value<std::string>()->default_value("timer"),
                    "Source for sync pulses: timer|stdin|gpio")
                ("sync-fps", value<double>()->default_value(30.0),
                    "Frame rate for generated sync pulses")
                ("sync-group", value<std::string>()->default_value("239.255.255.250"),
                    "Multicast group address for sync")
                ("sync-port", value<int>()->default_value(10000),
                    "UDP port for sync")
                ("sync-chip", value<std::string>()->default_value("gpiochip4"),
                    "GPIO chip name for sync source")
                ("sync-line", value<int>()->default_value(-1),
                    "GPIO line number for sync source");
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

                /* sync helper options -------------------------------------- */
                if (arg.rfind("--sync-source=", 0) == 0) {
                        sync_source = arg.substr(sizeof("--sync-source=") - 1);
                        continue;
                }
                if (arg == "--sync-source") {
                        if (i + 1 >= argc)
                                throw std::runtime_error("--sync-source requires a value");
                        sync_source = argv[++i];
                        continue;
                }

                if (arg.rfind("--sync-fps=", 0) == 0) {
                        sync_fps = std::stod(arg.substr(sizeof("--sync-fps=") - 1));
                        continue;
                }
                if (arg == "--sync-fps") {
                        if (i + 1 >= argc)
                                throw std::runtime_error("--sync-fps requires a value");
                        sync_fps = std::stod(argv[++i]);
                        continue;
                }

                if (arg.rfind("--sync-group=", 0) == 0) {
                        sync_group = arg.substr(sizeof("--sync-group=") - 1);
                        continue;
                }
                if (arg == "--sync-group") {
                        if (i + 1 >= argc)
                                throw std::runtime_error("--sync-group requires a value");
                        sync_group = argv[++i];
                        continue;
                }

                if (arg.rfind("--sync-port=", 0) == 0) {
                        sync_port = static_cast<uint16_t>(std::stoi(arg.substr(sizeof("--sync-port=") - 1)));
                        continue;
                }
                if (arg == "--sync-port") {
                        if (i + 1 >= argc)
                                throw std::runtime_error("--sync-port requires a value");
                        sync_port = static_cast<uint16_t>(std::stoi(argv[++i]));
                        continue;
                }

                if (arg.rfind("--sync-chip=", 0) == 0) {
                        sync_chip = arg.substr(sizeof("--sync-chip=") - 1);
                        continue;
                }
                if (arg == "--sync-chip") {
                        if (i + 1 >= argc)
                                throw std::runtime_error("--sync-chip requires a value");
                        sync_chip = argv[++i];
                        continue;
                }

                if (arg.rfind("--sync-line=", 0) == 0) {
                        sync_line = std::stoi(arg.substr(sizeof("--sync-line=") - 1));
                        continue;
                }
                if (arg == "--sync-line") {
                        if (i + 1 >= argc)
                                throw std::runtime_error("--sync-line requires a value");
                        sync_line = std::stoi(argv[++i]);
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
        std::string sync_mode;
        switch (sync)
        {
            case 1:  sync_mode = "server"; break;
            case 2:  sync_mode = "client"; break;
            default: sync_mode = "off";    break;
        }

        spdlog::info(
            "cinepi-cli: camPort='{}' hdmi_port={} same_hdmi={} zoom={} crops={}"
            " rectangles sync_mode={} src={} fps={} grp={} port={} chip={} line={}",
            camPort,
            hdmi_port,
            same_hdmi ? "true" : "false",
            Zoom(),
            scaler_crops_rects.size(),
            sync_mode,
            sync_source,
            sync_fps,
            sync_group,
            sync_port,
            sync_chip,
            sync_line);

        if (sync != 0 && !framerate)
            spdlog::info("--framerate not specified; using --sync-fps ({}) as camera framerate", sync_fps);

        return ok;
}
