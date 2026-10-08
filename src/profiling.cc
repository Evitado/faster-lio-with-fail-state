#include "profiling.h"

#include <ros/console.h>
#include <spawn.h>
#include <fcntl.h>
#include <ctime>
#include <filesystem>

extern char **environ;

namespace faster_lio::profiling {

bool Start(const std::string &save_dir, int port) {
    std::error_code ec;
    std::filesystem::create_directories(save_dir, ec);
    if (ec) {
        ROS_ERROR_STREAM("profiling: cannot create " << save_dir << ": " << ec.message() << ", profiling stays off");
        return false;
    }

    char stamp[32];
    const std::time_t now = std::time(nullptr);
    std::strftime(stamp, sizeof(stamp), "%Y-%m-%d_%H-%M-%S", std::localtime(&now));
    const std::filesystem::path trace = std::filesystem::path(save_dir) / ("faster_lio_" + std::string(stamp) + ".tracy");
    const std::string log = std::filesystem::path(trace).replace_extension(".capture.log").string();
    const std::string port_str = std::to_string(port);

    // the client reads TRACY_PORT when it starts (StartupProfiler below)
    setenv("TRACY_PORT", port_str.c_str(), 1);

    // tracy-capture records until this process disconnects, i.e. exits; its own session keeps a Ctrl-C in the
    // terminal from stopping it before the node has sent its last data
    posix_spawn_file_actions_t files;
    posix_spawn_file_actions_init(&files);
    posix_spawn_file_actions_addopen(&files, STDIN_FILENO, "/dev/null", O_RDONLY, 0);
    posix_spawn_file_actions_addopen(&files, STDOUT_FILENO, log.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
    posix_spawn_file_actions_adddup2(&files, STDOUT_FILENO, STDERR_FILENO);
    posix_spawn_file_actions_addclosefrom_np(&files, 3);  // not the node's ROS sockets
    posix_spawnattr_t attr;
    posix_spawnattr_init(&attr);
    posix_spawnattr_setflags(&attr, POSIX_SPAWN_SETSID);

    const std::string capture = std::filesystem::exists(FASTER_LIO_TRACY_CAPTURE) ? FASTER_LIO_TRACY_CAPTURE
                                                                                   : "tracy-capture";
    const std::string trace_str = trace.string();
    const char *argv[] = {capture.c_str(), "-o", trace_str.c_str(), "-f", "-a", "127.0.0.1", "-p", port_str.c_str(),
                          nullptr};
    pid_t pid = 0;
    const int err = posix_spawnp(&pid, capture.c_str(), &files, &attr, const_cast<char *const *>(argv), environ);
    posix_spawn_file_actions_destroy(&files);
    posix_spawnattr_destroy(&attr);
    if (err != 0) {
        ROS_ERROR_STREAM("profiling: cannot start " << capture << ": " << std::strerror(err) << ", profiling stays off");
        return false;
    }

    tracy::StartupProfiler();
    enabled = true;
    ROS_WARN_STREAM("profiling: recording to " << trace_str << " (tracy-capture pid " << pid << ", port " << port
                                               << ")");
    return true;
}

void Stop() {
    if (!enabled) {
        return;
    }
    enabled = false;
    tracy::ShutdownProfiler();  // sends what is still queued; the recorder then writes the file and exits
}

}  // namespace faster_lio::profiling
