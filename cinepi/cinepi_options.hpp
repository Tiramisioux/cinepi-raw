// cinepi_options.hpp
#pragma once
#include "raw_options.hpp"
#include <string>

class CinePiOptions : public RawOptions {
public:
    CinePiOptions();
    bool Parse(int argc, char *argv[]) override;

    /* --- extra flags ---------------------------------------------- */
    bool        same_hdmi;        // force both apps to one output
//     std::string redis_channel;    // channel to publish preview id
    bool        keep16;           // do not down-pack 16-bit streams
    int         hdmi_port;        // ← add this line
};
