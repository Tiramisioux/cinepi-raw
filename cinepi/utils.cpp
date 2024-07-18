#include "utils.hpp"

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

bool disk_mounted(RawOptions const *options){
	return fs::exists(fs::path(options->mediaDest)) && is_mounted(options->mediaDest.c_str());
}

void generate_filename(RawOptions *options, unsigned int clip_number)
{
	char filename[128];
	std::time_t raw_time;
	std::time(&raw_time);
	char time_string[32];
	std::tm *time_info = std::localtime(&raw_time);
	std::strftime(time_string, sizeof(time_string), "%y-%m-%d_%H%M%S", time_info);
	snprintf(filename, sizeof(filename), "%s_%s_C%05d", "CINEPI", time_string, clip_number);
	options->folder = std::string(filename);
}

bool create_clip_folder(RawOptions *options, unsigned int clip_number)
{
	if(!disk_mounted(options))
		return false;
	generate_filename(options, clip_number);
	return fs::create_directories(options->mediaDest + std::string("/") + options->folder);
}


bool create_stills_folder(RawOptions *options, unsigned int still_number)
{
	if(!disk_mounted(options))
		return false;
	std::string stillsPath = options->mediaDest + std::string("/stills");
	bool exists = fs::exists(fs::path(stillsPath));
	generate_filename(options, still_number);
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