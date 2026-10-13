#include <ros/ros.h>

#include "laser_mapping.h"
#include "profiling.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "faster_lio");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // before anything is instrumented: profiling::Start sets up Tracy and the recorder
    if (pnh.param<bool>("profiling_enable", false)) {
        faster_lio::profiling::Start(pnh.param<std::string>("profiling_save_path", "/tmp/faster_lio_profiling"),
                                     pnh.param<int>("profiling_port", 8186));
    }
    PROFILE_THREAD_NAME("faster_lio main");

    auto laser_mapping = std::make_shared<faster_lio::LaserMapping>();
    laser_mapping->InitROS(nh, pnh);

    // scans are processed from the lidar and IMU callbacks; returns on shutdown (SIGINT, rosnode kill)
    ros::spin();

    faster_lio::profiling::Stop();
    return 0;
}
