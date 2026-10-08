#include <ros/ros.h>
#include <csignal>

#include "laser_mapping.h"
#include "profiling.h"

#ifdef FASTER_LIO_TRACY_MEMORY
#include <atomic>
#include <cstdint>
#include <cstdlib>
#include <new>

// Route operator new/delete through Tracy's memory view (allocations, live memory over time). Buffers allocated with
// malloc (e.g. Eigen's dynamic matrices) are not seen here; the "process rss" plot covers them.
// Reporting starts in main(): allocations made while Tracy initializes during static init would break its startup
// calibration. A small header remembers whether a block was reported, so blocks allocated before main are never
// freed into Tracy (the viewer rejects frees of unknown pointers).
namespace {
std::atomic<bool> g_report_allocations{false};
constexpr uint64_t kReported = 0x7472616379a110cULL;
struct alignas(16) AllocHeader {
    uint64_t tag;
    uint64_t size;
};

void *TrackedAlloc(std::size_t size) noexcept {
    auto *header = static_cast<AllocHeader *>(std::malloc(sizeof(AllocHeader) + size));
    if (!header) return nullptr;
    void *ptr = header + 1;
    header->size = size;
    header->tag = g_report_allocations.load(std::memory_order_relaxed) ? kReported : 0;
    if (header->tag == kReported) TracyAlloc(ptr, size);
    return ptr;
}

void TrackedFree(void *ptr) noexcept {
    if (!ptr) return;
    auto *header = static_cast<AllocHeader *>(ptr) - 1;
    if (header->tag == kReported) TracyFree(ptr);
    std::free(header);
}
}  // namespace

void *operator new(std::size_t size) {
    void *ptr = TrackedAlloc(size);
    if (!ptr) throw std::bad_alloc();
    return ptr;
}
void *operator new[](std::size_t size) { return operator new(size); }
void *operator new(std::size_t size, const std::nothrow_t &) noexcept { return TrackedAlloc(size); }
void *operator new[](std::size_t size, const std::nothrow_t &) noexcept { return TrackedAlloc(size); }
void operator delete(void *ptr) noexcept { TrackedFree(ptr); }
void operator delete[](void *ptr) noexcept { TrackedFree(ptr); }
void operator delete(void *ptr, std::size_t) noexcept { TrackedFree(ptr); }
void operator delete[](void *ptr, std::size_t) noexcept { TrackedFree(ptr); }
void operator delete(void *ptr, const std::nothrow_t &) noexcept { TrackedFree(ptr); }
void operator delete[](void *ptr, const std::nothrow_t &) noexcept { TrackedFree(ptr); }
#endif

void SigHandle(int sig) {
    faster_lio::options::FLAG_EXIT = true;
    ROS_WARN("catch sig %d", sig);
}

int main(int argc, char **argv) {
#ifdef FASTER_LIO_TRACY_MEMORY
    g_report_allocations = true;
#endif
    PROFILE_THREAD_NAME("faster_lio main");
#ifdef FASTER_LIO_TRACY
    ROS_WARN_STREAM("PROFILING BUILD: Tracy collects data in memory until tracy-capture connects (profiling/README.md)");
#endif

    ros::init(argc, argv, "faster_lio");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    auto laser_mapping = std::make_shared<faster_lio::LaserMapping>();
    laser_mapping->InitROS(nh, pnh);

    signal(SIGINT, SigHandle);
    ros::Rate rate(100);

    while (ros::ok()) {
        if (faster_lio::options::FLAG_EXIT) {
            break;
        }
        {
            PROFILE_SCOPE("ros::spinOnce");
            ros::spinOnce();
        }
        laser_mapping->Run();
        {
            PROFILE_SCOPE("idle (rate.sleep)");
            rate.sleep();
        }
    }

    return 0;
}
