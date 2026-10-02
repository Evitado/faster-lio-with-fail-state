#ifndef FASTER_LIO_PROFILING_H
#define FASTER_LIO_PROFILING_H

/// Tracy instrumentation. Everything here compiles to nothing unless the package is configured with
/// -DFASTER_LIO_TRACY=ON (see profiling/README.md), so the release build carries no profiling cost.
///
///   PROFILE_SCOPE("name")        zone covering the rest of the enclosing scope
///   PROFILE_FRAME()              marks the end of one processed scan (a "frame" in the viewer)
///   PROFILE_PLOT("name", value)  one sample of a numeric plot
///   PROFILE_THREAD_NAME("name")  names the calling thread in the viewer
///   profiling::PlotProcessStats() samples cpu / memory usage of this process and of the whole machine

#ifdef FASTER_LIO_TRACY

#include <tracy/Tracy.hpp>

#include <sys/resource.h>
#include <unistd.h>
#include <chrono>
#include <cstdio>
#include <cstring>

#define PROFILE_SCOPE(name) ZoneScopedN(name)
#define PROFILE_FRAME() FrameMark
#define PROFILE_PLOT(name, value) TracyPlot(name, value)
#define PROFILE_PLOT_CONFIG(name, type) TracyPlotConfig(name, type, false, true, 0)
#define PROFILE_THREAD_NAME(name) tracy::SetThreadName(name)

namespace faster_lio::profiling {

/// Plots, per call:
///  - "process cpu [cores]": cpu time used by this process (all threads) since the last call / wall time, so 1.0
///    means one fully busy core. Tracy's own "CPU usage" plot shows the whole machine for comparison.
///  - "process rss": resident memory of this process
///  - "process threads": number of threads
///  - "system mem available": MemAvailable from /proc/meminfo, i.e. what is left for other processes
inline void PlotProcessStats() {
    using Clock = std::chrono::steady_clock;
    static bool configured = false;
    static Clock::time_point last_wall;
    static double last_cpu = 0;
    static const long page_size = sysconf(_SC_PAGESIZE);
    if (!configured) {
        PROFILE_PLOT_CONFIG("process rss", tracy::PlotFormatType::Memory);
        PROFILE_PLOT_CONFIG("system mem available", tracy::PlotFormatType::Memory);
        configured = true;
    }

    rusage usage{};
    getrusage(RUSAGE_SELF, &usage);
    const double cpu = usage.ru_utime.tv_sec + usage.ru_utime.tv_usec * 1e-6 + usage.ru_stime.tv_sec +
                       usage.ru_stime.tv_usec * 1e-6;
    const Clock::time_point now = Clock::now();
    if (last_wall != Clock::time_point()) {
        const double wall = std::chrono::duration<double>(now - last_wall).count();
        if (wall > 0) {
            PROFILE_PLOT("process cpu [cores]", (cpu - last_cpu) / wall);
        }
    }
    last_cpu = cpu;
    last_wall = now;

    // /proc/self/stat: field 20 is num_threads, field 24 is rss in pages
    if (FILE *f = std::fopen("/proc/self/stat", "r")) {
        char buf[1024];
        const size_t n = std::fread(buf, 1, sizeof(buf) - 1, f);
        std::fclose(f);
        buf[n] = '\0';
        // skip "pid (comm)", comm may contain spaces; the next token is field 3 (state)
        if (const char *p = std::strrchr(buf, ')')) {
            long num_threads = 0, rss_pages = 0;
            int field = 3;
            for (const char *tok = p + 1; *tok && field <= 24; ++field) {
                while (*tok == ' ') ++tok;
                if (field == 20) num_threads = std::strtol(tok, nullptr, 10);
                if (field == 24) rss_pages = std::strtol(tok, nullptr, 10);
                while (*tok && *tok != ' ') ++tok;
            }
            PROFILE_PLOT("process threads", static_cast<int64_t>(num_threads));
            PROFILE_PLOT("process rss", static_cast<int64_t>(rss_pages * page_size));
        }
    }

    if (FILE *f = std::fopen("/proc/meminfo", "r")) {
        char line[256];
        while (std::fgets(line, sizeof(line), f)) {
            long kb = 0;
            if (std::sscanf(line, "MemAvailable: %ld kB", &kb) == 1) {
                PROFILE_PLOT("system mem available", static_cast<int64_t>(kb) * 1024);
                break;
            }
        }
        std::fclose(f);
    }
}

}  // namespace faster_lio::profiling

#else

#define PROFILE_SCOPE(name)
#define PROFILE_FRAME()
#define PROFILE_PLOT(name, value)
#define PROFILE_PLOT_CONFIG(name, type)
#define PROFILE_THREAD_NAME(name)

namespace faster_lio::profiling {
inline void PlotProcessStats() {}
}  // namespace faster_lio::profiling

#endif

#endif  // FASTER_LIO_PROFILING_H
