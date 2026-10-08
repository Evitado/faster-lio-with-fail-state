#ifndef FASTER_LIO_LASER_MAPPING_H
#define FASTER_LIO_LASER_MAPPING_H

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <tbb/global_control.h>
#include <tf/transform_listener.h>
#include <atomic>
#include <mutex>

#include "common_lib.h"
#include "faster_lio/SaveTrajectory.h"
#include "imu_processing.hpp"
#include "ivox3d/ivox3d.h"
#include "options.h"
#include "pointcloud_preprocess.h"

namespace faster_lio {

class LaserMapping {
   public:
    using IVoxType = IVox<3, MapPointType>;

    LaserMapping();

    bool InitROS(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);

    void Run();
    // services
    bool startLIO(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool stopLIO(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool saveTrajectory(faster_lio::SaveTrajectory::Request &req, faster_lio::SaveTrajectory::Response &res);

    // callbacks of lidar and imu
    void StandardPCLCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void IMUCallBack(const sensor_msgs::Imu::ConstPtr &msg_in);

    // sync lidar with imu
    bool SyncPackages();

    /// interface of mtk, customized obseravtion model
    void ObsModel(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data);

    /// Obtained from https://arxiv.org/abs/2411.06766
    /// publishes the condition number of the translation part of JtJ from the last observation model call
    void PublishConditionNumber();

    void PublishPath(const ros::Publisher pub_path);
    void PublishOdometry(const ros::Publisher &pub_odom_aft_mapped);
    void PublishKeypoints(const ros::Publisher &pub_odom_aft_mapped);
    void PublishFrameWorld();
    void PublishFrameBody(const ros::Publisher &pub_laser_cloud_body);

   private:
    template <typename T>
    void SetPosestamp(T &out);

    PointType PointBodyToWorld(PointType const &pi);
    void PointBodyLidarToIMU(PointType const *const pi, PointType *const po);

    void MapIncremental();

    /// per-scan profiler plots and frame mark (no-op unless built with FASTER_LIO_TRACY)
    void ProfileScan();

    void SubAndPubToROS();

    bool LoadParams();

   private:
    /// modules
    IVoxType::Options ivox_options_;
    std::shared_ptr<IVoxType> ivox_ = nullptr;                    // localmap in ivox
    std::shared_ptr<PointCloudPreprocess> preprocess_ = nullptr;  // point cloud preprocess
    std::shared_ptr<ImuProcess> p_imu_ = nullptr;                 // imu process

    /// local map
    double filter_size_map_min_ = 0;

    /// params
    std::vector<double> extrinT_{3, 0.0};  // lidar-imu translation
    std::vector<double> extrinR_{9, 0.0};  // lidar-imu rotation

    /// point clouds data
    CloudPtr scan_undistort_{new PointCloudType()};   // scan after undistortion
    CloudPtr scan_down_body_{new PointCloudType()};   // downsampled scan in body
    CloudPtr scan_down_world_{new PointCloudType()};  // downsampled scan in world
    std::vector<MapPointVector> nearest_points_;      // nearest map points of current scan
    common::VV4F corr_pts_;                           // inlier pts
    common::VV4F corr_norm_;                          // inlier plane norms
    float filter_size_surf_ = 0.5f;                   // voxel size the current scan is downsampled with
    std::vector<float> residuals_;                    // point-to-plane residuals
    std::vector<uint8_t> point_selected_surf_;        // selected points (not vector<bool>: written from TBB threads)
    common::VV4F plane_coef_;                         // plane coeffs
    std::vector<common::V3F> search_pos_;             // world position of each point at its last nn search
    std::vector<uint8_t> plane_ok_;                   // whether that search gave a valid plane
    bool searched_this_scan_ = false;                 // an nn search already ran for the current scan
    float nn_reuse_distance_ = 0.01f;                 // re-search a point only if it moved more than this [m]
    std::atomic<int> nn_searches_{0};                 // points searched in the current scan (profiling)

    /// ros pub and sub stuffs
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber sub_pcl_;
    ros::Subscriber sub_imu_;
    ros::Publisher pub_laser_cloud_world_;
    ros::Publisher keypoints_pub_;
    ros::Publisher pub_laser_cloud_body_;
    ros::Publisher pub_odom_aft_mapped_;
    ros::Publisher pub_path_;
    ros::Publisher pub_cond_number;
    ros::ServiceServer start_lio_service_;
    ros::ServiceServer stop_lio_service_;
    ros::ServiceServer save_trajectory_service_;
    tf::TransformListener tf_listener_;
    tf::StampedTransform lidar_to_base_;  // last good lidar->base_link transform
    bool has_lidar_to_base_ = false;

    std::mutex mtx_buffer_;
    std::deque<double> time_buffer_;
    std::deque<CloudPtr> lidar_buffer_;
    std::deque<sensor_msgs::Imu::ConstPtr> imu_buffer_;
    nav_msgs::Odometry odom_aft_mapped_;

    /// sync state
    double last_timestamp_lidar_ = 0;
    double lidar_end_time_ = 0;
    double last_timestamp_imu_ = -1.0;
    double first_lidar_time_ = 0.0;
    bool lidar_pushed_ = false;

    /// statistics and flags ///
    bool flg_first_scan_ = true;
    bool flg_EKF_inited_ = false;
    double lidar_mean_scantime_ = 0.0;
    int scan_num_ = 0;
    int effect_feat_num_ = 0;
    int obs_model_calls_ = 0;  // IEKF iterations of the current scan

    ///////////////////////// EKF inputs and output ///////////////////////////////////////////////////////
    common::MeasureGroup measures_;                    // sync IMU and lidar scan
    esekfom::esekf<state_ikfom, 12, input_ikfom> kf_;  // esekf
    state_ikfom state_point_;                          // ekf current state
    bool extrinsic_est_en_ = true;
    Eigen::Matrix<double, 6, 6> cond_jtj_ = Eigen::Matrix<double, 6, 6>::Zero();  // JtJ of rot/pos part of H
    bool cond_jtj_valid_ = false;

    /////////////////////////  publishing //////////////////////////////////////////////////////////////////
    bool scan_pub_en_ = false;
    bool dense_pub_en_ = false;
    bool scan_body_pub_en_ = false;
    std::unique_ptr<tbb::global_control> tbb_control_;  // caps TBB worker threads when set

    nav_msgs::Path path_;

    /// odometry pose of every scan since the last start_lidar_odom, written out by the save_trajectory service
    struct TrajectoryPose {
        double stamp;
        common::V3D pos;
        Eigen::Quaterniond rot;
    };
    std::vector<TrajectoryPose> trajectory_;
    std::string trajectory_file_;  // default output of save_trajectory
    geometry_msgs::PoseStamped msg_body_pose_;

    // toggled by the start/stop services
    bool lidar_odom_ = false;
    std::string base_link_frame_;
    std::string lidar_frame_;
    std::string global_frame_;
};

}  // namespace faster_lio

#endif  // FASTER_LIO_LASER_MAPPING_H
