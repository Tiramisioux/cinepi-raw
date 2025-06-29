#include "cinepi_options.hpp"

#include <cstdlib>
#include <stdexcept>
#include <string>
#include <vector>
#include <spdlog/spdlog.h>
#include <boost/program_options.hpp>

using namespace boost::program_options;

CinePiOptions::CinePiOptions()
    : RawOptions()
    , same_hdmi(false)
    , keep16(false)
    , hdmi_port(-1)
{
    // CinePi-only flags grouped separately
    options_description cinepi_group("CinePi-raw specific options");
    cinepi_group.add_options()
        ("cam-port",
            value<std::string>(&camPort)->implicit_value(""),
            "Physical camera port to use (e.g. cam0 or cam1)")
        ("hdmi-port",
            value<int>(&hdmi_port)->default_value(-1),
            "For DRM preview choose HDMI socket (0 = HDMI-0, 1 = HDMI-1, -1 = automatic)")
        ("same-hdmi",
            value<bool>(&same_hdmi)->default_value(false)->implicit_value(true),
            "Force both apps to use the same HDMI output")
        ("keep16",
            value<bool>(&keep16)->default_value(false)->implicit_value(true),
            "Write full 16-bit DNG files (disable 12-bit packing)");

    // Inject into the main options_description
    options_.add(cinepi_group);
}

bool CinePiOptions::Parse(int argc, char *argv[])
{
    std::vector<char *> forward;
    forward.reserve(argc);
    forward.push_back(argv[0]);  // program name

    // Manually extract CinePi flags, support both --flag value and --flag=value
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];

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

        if (arg == "--same-hdmi") {
            same_hdmi = true;
            continue;
        }
        if (arg == "--keep16") {
            keep16 = true;
            continue;
        }

        // Not a CinePi flag – forward to RawOptions parser
        forward.push_back(argv[i]);
    }

    // Base parse handles all other flags (and help/version)
    int new_argc = static_cast<int>(forward.size());
    bool ok = RawOptions::Parse(new_argc, forward.data());

    // Derive default camPort if not explicitly set
    if (camPort.empty()) {
        camPort = "cam" + std::to_string(camera);
    }

    // Log configuration
    spdlog::info(
        "cinepi-cli: camPort='{}'  hdmi_port={}  same_hdmi={}",
        camPort,
        hdmi_port,
        same_hdmi ? "true" : "false"
    );

    return ok;
    // return ok && !redis_channel.empty();
}
