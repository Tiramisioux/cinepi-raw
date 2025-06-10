
#pragma once
#include <iostream>
// #include "raw_options.hpp"
#include "cinepi_options.hpp"
#include <filesystem>
#include <fstream>
#include <string>
#include <sstream>
#include <iterator>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

namespace fs = std::filesystem;

enum CompressionType { NONE = 1, LOSSLESS = 7 };

bool is_mounted(const char *mount_point);
// bool disk_mounted(RawOptions const *options);
// void generate_filename(RawOptions *options, unsigned int clip_number,
//                        const libcamera::ControlList &metadata = libcamera::ControlList());

bool disk_mounted(const CinePiOptions *options);
void generate_filename(CinePiOptions *options, unsigned int clip_number,
    const libcamera::ControlList &metadata);

// bool create_clip_folder(RawOptions *options, unsigned int clip_number = 0);
// bool create_stills_folder(RawOptions *options, unsigned int still_number = 0);

bool create_clip_folder(CinePiOptions *options, unsigned int clip_number);
bool create_stills_folder(CinePiOptions *options, unsigned int still_number);

std::string getHwId();

// -----------------------------------------------------------------
// Legacy wrappers for RawOptions – simply cast and forward
// -----------------------------------------------------------------
struct RawOptions;                       // forward-declare

inline bool disk_mounted(const RawOptions *opt) {
    return disk_mounted(reinterpret_cast<const CinePiOptions*>(opt));
}
inline void generate_filename(RawOptions *opt, unsigned int clip,
                              const libcamera::ControlList &md = libcamera::ControlList())
{
    generate_filename(reinterpret_cast<CinePiOptions*>(opt), clip, md);
}
inline bool create_clip_folder(RawOptions *opt, unsigned int clip) {
    return create_clip_folder(reinterpret_cast<CinePiOptions*>(opt), clip);
}
inline bool create_stills_folder(RawOptions *opt, unsigned int stills) {
    return create_stills_folder(reinterpret_cast<CinePiOptions*>(opt), stills);
}