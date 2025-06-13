// cinepi_options.cpp
#include "cinepi_options.hpp"

#include <cstdlib>
#include <stdexcept>
#include <string>
#include <vector>          // ← NEW
#include <spdlog/spdlog.h> // ← NEW

CinePiOptions::CinePiOptions()
        : RawOptions()
        , same_hdmi(false)
        , redis_channel("cp_controls")
        , keep16(false)
{}

// -----------------------------------------------------------------------------
//  Parse:  first consume CinePi‑specific flags, then forward the rest to the
//          standard RawOptions parser.  Finally log what we captured.
// -----------------------------------------------------------------------------
bool CinePiOptions::Parse(int argc, char *argv[])
{
    std::vector<char *> forward;
    forward.reserve(argc);
    forward.push_back(argv[0]);                 // keep programme name

    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];

        if (arg == "--same-hdmi")
        {
            same_hdmi = true;
        }
        else if (arg == "--redis-channel")
        {
            if (i + 1 >= argc)
                throw std::runtime_error("--redis-channel needs a value");
            redis_channel = argv[++i];
        }
        else if (arg == "--keep16")
        {
            keep16 = true;
        }

        else
        {
            // Not one of ours – forward it to RawOptions.
            forward.push_back(argv[i]);
        }
    }

    // Call the base‑class parser with the stripped argument list.
    int new_argc = static_cast<int>(forward.size());
    bool ok = RawOptions::Parse(new_argc, forward.data());

    // -------- sanity log ------------------------------------------------------
    spdlog::info("CLI parsed: hdmi_port={}  same_hdmi={}  redis_channel='{}'",
                this->hdmi_port,
                 same_hdmi ? "true" : "false",
                 redis_channel);
    // -------------------------------------------------------------------------

    return ok;
}
