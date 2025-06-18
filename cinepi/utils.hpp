// utils.hpp – central declarations & thin wrappers
// SPDX-License-Identifier: BSD-2-Clause
#pragma once

#include <string>
#include <filesystem>
#include <libcamera/control_ids.h>

#include "raw_options.hpp"       // full RawOptions definition
#include "cinepi_options.hpp"    // pulls in CinePiOptions (derives from RawOptions)

namespace fs = std::filesystem;

// -----------------------------------------------------------------------------
// Misc helpers
// -----------------------------------------------------------------------------
enum CompressionType { NONE = 1, LOSSLESS = 7 };

bool is_mounted(const char *mount_point);

// -----------------------------------------------------------------------------
// RawOptions‑based API  (implemented in utils.cpp)
// -----------------------------------------------------------------------------
bool  disk_mounted(const RawOptions *opt);
void  generate_filename(RawOptions *opt, unsigned int clip,
                        const libcamera::ControlList &metadata = libcamera::ControlList());
bool  create_clip_folder (RawOptions *opt, unsigned int clip);
bool  create_stills_folder(RawOptions *opt, unsigned int stills);

// -----------------------------------------------------------------------------
// CinePiOptions convenience wrappers – inline, zero‑cost
// -----------------------------------------------------------------------------
inline bool disk_mounted(const CinePiOptions *opt)
{
    return disk_mounted(static_cast<const RawOptions *>(opt));
}

inline void generate_filename(CinePiOptions *opt, unsigned int clip,
                              const libcamera::ControlList &metadata)
{
    generate_filename(static_cast<RawOptions *>(opt), clip, metadata);
}

inline bool create_clip_folder(CinePiOptions *opt, unsigned int clip)
{
    return create_clip_folder(static_cast<RawOptions *>(opt), clip);
}

inline bool create_stills_folder(CinePiOptions *opt, unsigned int stills)
{
    return create_stills_folder(static_cast<RawOptions *>(opt), stills);
}

// -----------------------------------------------------------------------------
// Hardware introspection
// -----------------------------------------------------------------------------
std::string getHwId();