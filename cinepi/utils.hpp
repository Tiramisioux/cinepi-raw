#include <iostream>
#include <chrono>  // Required for std::chrono::system_clock and std::chrono::duration_cast
#include "raw_options.hpp"
#include <filesystem>
namespace fs = std::filesystem;

enum CompressionType { NONE = 1, LOSSLESS = 7 };

static bool disk_mounted(RawOptions const *options){
	return fs::exists(fs::path(options->mediaDest));
}

static void generate_filename(RawOptions *options, unsigned int clip_number = 0)
{
	char filename[128];
	std::time_t raw_time;
	std::time(&raw_time);
	char time_string[32];
	std::tm *time_info = std::localtime(&raw_time);
	std::strftime(time_string, sizeof(time_string), "%y-%m-%d_%H%M", time_info);
	snprintf(filename, sizeof(filename), "%s_%s_C%05d", "CINEPI", time_string, clip_number);
	options->folder = std::string(filename);
}

// New function to generate the clip folder name
static std::string generate_clip_foldername(RawOptions *options, unsigned int clip_number = 0)
{
    char foldername[128];
    std::time_t raw_time;
    std::time(&raw_time);
    char time_string[32];
    std::tm *time_info = std::localtime(&raw_time);
    
    // Get the current time's milliseconds
    auto now = std::chrono::system_clock::now();
    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    
    // Calculate the frame number based on 24 fps
    int frame_number = (milliseconds.count() * 24) / 1000;

    // Generate the folder name with the frame number
    std::strftime(time_string, sizeof(time_string), "%y-%m-%d_%H%M%S", time_info);
    snprintf(foldername, sizeof(foldername), "CINEPI_%s%02d_C%05d", time_string, frame_number, clip_number);
    
    return std::string(foldername);
}

// Modified create_clip_folder function with error handling
static bool create_clip_folder(RawOptions *options, unsigned int clip_number = 0)
{
    if(!disk_mounted(options))
        return false;
    
    // Use the new function to get the folder name
    options->folder = generate_clip_foldername(options, clip_number);
    
    try {
        return fs::create_directories(options->mediaDest + std::string("/") + options->folder);
    } catch(const std::exception& e) {
        std::cerr << "Error creating clip folder: " << e.what() << std::endl;
        return false;
    }
}

// Modified create_stills_folder function with error handling
static bool create_stills_folder(RawOptions *options, unsigned int still_number = 0)
{
	if(!disk_mounted(options))
		return false;
	std::string stillsPath = options->mediaDest + std::string("/stills");
	bool exists = fs::exists(fs::path(stillsPath));
	generate_filename(options, still_number);
	if(!exists){
        try {
		    return fs::create_directories(options->mediaDest + std::string("/stills"));
        } catch(const std::exception& e) {
            std::cerr << "Error creating stills folder: " << e.what() << std::endl;
            return false;
        }
	}
	return exists;
}
