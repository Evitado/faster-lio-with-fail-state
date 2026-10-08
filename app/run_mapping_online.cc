#include <ros/ros.h>
#include <csignal>

#include "laser_mapping.h"
#include "profiling.h"

void SigHandle(int sig) {
    faster_lio::options::FLAG_EXIT = true;
    ROS_WARN("catch sig %d", sig);
}

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

    faster_lio::profiling::Stop();
    return 0;
}
