#include "utils.hpp"
#include <chrono>
#include <libcamera/control_ids.h>
#include <libcamera/formats.h>
#include <sys/time.h> // Required for gettimeofday

#include "cinepi_options.hpp"


bool is_mounted(const char *mount_point) {
    FILE *fp = fopen("/proc/mounts", "r");
    if (fp == NULL) {
        perror("Error opening /proc/mounts");
        return false;
    }

    char *line = NULL;
    size_t len = 0;
    ssize_t read;
    bool found = false;

    while ((read = getline(&line, &len, fp)) != -1) {
        if (strstr(line, mount_point) != NULL) {
            found = true;
            break;
        }
    }

    free(line);
    fclose(fp);

    return found;
}

// bool disk_mounted(RawOptions const *options){

bool disk_mounted(const CinePiOptions *options){

	return fs::exists(fs::path(options->mediaDest)) && is_mounted(options->mediaDest.c_str());
}

// void generate_filename(RawOptions *options, unsigned int clip_number,

void generate_filename(CinePiOptions *options, unsigned int clip_number,
                       const libcamera::ControlList &metadata)
{
    char filename[128];

    // Capture current time with microsecond precision
    struct timeval tv;
    gettimeofday(&tv, nullptr);

    // Extract seconds and microseconds
    std::time_t raw_time = tv.tv_sec;        // Seconds since epoch
    long microseconds = tv.tv_usec;          // Microseconds within the current second

    // Format time into a string
    char time_string[32];
    std::tm *time_info = std::localtime(&raw_time);
    std::strftime(time_string, sizeof(time_string), "%y-%m-%d_%H%M%S", time_info);

    // Default frame rate
    double frameRate = 24.0;

    // Retrieve frame rate from metadata
    if (!metadata.empty()) {
        auto frameDuration = metadata.get(libcamera::controls::FrameDuration);
        if (frameDuration && *frameDuration > 0) {
            frameRate = 1e6 / static_cast<double>(*frameDuration); // Convert nanoseconds to microseconds
        } else {
            std::cerr << "WARNING: FrameDuration not available. Using default frame rate: "
                      << frameRate << std::endl;
        }
    }

    // Calculate frame number within the current second (0 to frameRate - 1)
    int frameNumber = static_cast<int>((microseconds) / (1'000'000 / frameRate)) % static_cast<int>(frameRate);

    // Add HHMMSS and frame number (FF) to filename
    snprintf(filename, sizeof(filename), "CINEPI_%s_F%02d_C%05d", time_string, frameNumber, clip_number);

    options->folder = std::string(filename);
}


// bool create_clip_folder(RawOptions *options, unsigned int clip_number)

bool create_clip_folder(CinePiOptions *options, unsigned int clip_number)
{
	if(!disk_mounted(options))
		return false;
	
    // generate_filename(options, clip_number);
	
    libcamera::ControlList dummy;                 // no metadata available here
    generate_filename(options, clip_number, dummy);
    
    return fs::create_directories(options->mediaDest + std::string("/") + options->folder);
}


// bool create_stills_folder(RawOptions *options, unsigned int still_number)

bool create_stills_folder(CinePiOptions *options, unsigned int still_number)
{
	if(!disk_mounted(options))
		return false;
	std::string stillsPath = options->mediaDest + std::string("/stills");
	bool exists = fs::exists(fs::path(stillsPath));

    // generate_filename(options, still_number);

    libcamera::ControlList dummy;
    generate_filename(options, still_number, dummy);

    if(!exists){
		return fs::create_directories(options->mediaDest + std::string("/stills"));
	}
	return exists;
}


std::string getHwId() {
    std::ifstream cpuinfo("/proc/cpuinfo");
    std::string line;
    std::string serialTag = "Serial";
    
    while (std::getline(cpuinfo, line)) {
        if (line.find(serialTag) != std::string::npos) {
            std::string serial = line.substr(line.find(":") + 1);
            // Remove leading and trailing whitespace
            size_t start = serial.find_first_not_of(" \t");
            size_t end = serial.find_last_not_of(" \t");
            if (start != std::string::npos) {
                return serial.substr(start, end - start + 1);
            }
        }
    }
    cpuinfo.close();

    // Fallback to MAC address
    std::ifstream macFile("/sys/class/net/eth0/address");
    if (macFile.is_open()) {
        std::getline(macFile, line);
        macFile.close();
        return line;
    }

    return "UNKNOWN"; // As a final fallback
} 