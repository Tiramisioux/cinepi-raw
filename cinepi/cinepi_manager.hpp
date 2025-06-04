#pragma once
#include <sys/inotify.h>
#include <sys/select.h>
#include <unistd.h>
#include <iostream>
#include <limits.h>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

namespace fs = std::filesystem;

class CinePIManager{
    public:
        CinePIManager(CinePIRecorder *app);
        ~CinePIManager()
    private:
        // Function to initialize inotify and add a watch on the mount point
        int watch_mount_point(const char* path) {
            int inotifyFd = inotify_init1(IN_NONBLOCK); // Initialize inotify in non-blocking mode
            if (inotifyFd < 0) {
                std::cerr << "inotify_init1 failed" << std::endl;
                return -1;
            }

            int wd = inotify_add_watch(inotifyFd, path, IN_CREATE | IN_DELETE);
            if (wd < 0) {
                std::cerr << "inotify_add_watch failed" << std::endl;
                close(inotifyFd);
                return -1;
            }

            return inotifyFd; // Return the file descriptor for inotify
        }


};