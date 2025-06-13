// cinepi_options.hpp
#pragma once

#include "raw_options.hpp"      // The class you already use

/*  Extend RawOptions with the switches we’ll need later.
 *  All the built‑in libcamera/rpicam flags still work unchanged.           */
class CinePiOptions : public RawOptions
{
public:
        CinePiOptions();              // sets sane defaults
        /** Parse CLI.  Returns false only when --help/--version consumed. */
        bool Parse(int argc, char *argv[]) override;

        /* --- extra flags ---------------------------------------------- */
        bool        same_hdmi;        // force both apps to one output
        std::string redis_channel;    // channel to publish preview id
        bool        keep16;           // NEW : do not down-pack 16-bit streams
};
