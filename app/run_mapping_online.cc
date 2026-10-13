//
// Created by xiang on 2021/10/8.
//
#include <gflags/gflags.h>
#include <unistd.h>
#include <csignal>

#include "laser_mapping.h"
#include "profiling.h"
#include "ros/init.h"
#include "ros/node_handle.h"

/// run the lidar mapping in online mode

// DEFINE_string(traj_log_file, "./Log/traj.txt", "path to traj log file");
void SigHandle(int sig) {
    faster_lio::options::FLAG_EXIT = true;
    ROS_WARN("catch sig %d", sig);
}

int main(int argc, char **argv) {
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);

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

    // ros::spin();

    // online, almost same with offline, just receive the messages from ros
    while (ros::ok()) {
        if (faster_lio::options::FLAG_EXIT) {
            break;
        }
        ros::spinOnce();
        laser_mapping->Run();
        rate.sleep();
    }

    LOG(INFO) << "finishing mapping";
    laser_mapping->Finish();
    faster_lio::profiling::Stop();

    // faster_lio::Timer::PrintAll();
    // LOG(INFO) << "save trajectory to: " << FLAGS_traj_log_file;
    // laser_mapping->Savetrajectory(FLAGS_traj_log_file);

    return 0;
}
