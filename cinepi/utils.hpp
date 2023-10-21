#include <iostream>
#include "raw_options.hpp"
#include <filesystem>

namespace fs = std::filesystem;

enum CompressionType { NONE = 1, LOSSLESS = 7 };

bool disk_mounted(RawOptions const *options);
void generate_filename(RawOptions *options, unsigned int clip_number = 0);
bool create_clip_folder(RawOptions *options, unsigned int clip_number = 0);
bool create_stills_folder(RawOptions *options, unsigned int still_number = 0);
