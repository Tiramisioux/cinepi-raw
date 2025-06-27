// cinepi_options.cpp
#include "cinepi_options.hpp"

#include <cstdlib>
#include <stdexcept>
#include <string>
#include <vector>
#include <spdlog/spdlog.h>

CinePiOptions::CinePiOptions()
        : RawOptions()
        , same_hdmi(false)
        , redis_channel("cp_controls")
        , keep16(false)
{}

/* ──────────────────────────────────────────────────────────────── *
 *  Parse CinePi-specific CLI flags, then forward the remainder     *
 *  to RawOptions::Parse.                                           *
 * ──────────────────────────────────────────────────────────────── */
bool CinePiOptions::Parse(int argc, char *argv[])
{
    std::vector<char *> forward;
    forward.reserve(argc);
    forward.push_back(argv[0]);                       // argv[0] == binary name

    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];

        if (arg == "--same-hdmi")        { same_hdmi = true;           continue; }
        if (arg == "--keep16")           { keep16    = true;           continue; }

        if (arg == "--redis-channel")
        {
            if (i + 1 >= argc)
                throw std::runtime_error("--redis-channel requires a value");
            redis_channel = argv[++i];
            continue;
        }

        if (arg == "--cam-port")                           // NEW  ★
        {
            if (i + 1 >= argc)
                throw std::runtime_error("--cam-port requires a value");
            camPort = argv[++i];                           // RawOptions member
            continue;
        }

        /* Not one of ours – forward to RawOptions parser. */
        forward.push_back(argv[i]);
    }

    /* Call base-class parser with stripped argv. */
    int  new_argc = static_cast<int>(forward.size());
    bool ok       = RawOptions::Parse(new_argc, forward.data());

    /* Quick sanity log. */
    spdlog::info(
        "cinepi-cli: camPort='{}'  hdmi_port={}  same_hdmi={}  redis_channel='{}'",
        camPort.empty() ? "<unset>" : camPort.c_str(),
        hdmi_port,
        same_hdmi ? "true" : "false",
        redis_channel
    );

    return ok && !redis_channel.empty();
}
