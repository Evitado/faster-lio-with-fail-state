#include <tbb/blocked_range.h>
#include <tf/transform_broadcaster.h>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <fstream>
#include <numeric>

#include <tbb/parallel_for.h>
#include "common_lib.h"
#include "laser_mapping.h"
#include "ros/node_handle.h"
#include "tf/transform_listener.h"
#include "utils.h"

#include <std_msgs/Float64.h>
#include <pcl/console/print.h>

namespace faster_lio {

bool LaserMapping::InitROS(const ros::NodeHandle &nh, const ros::NodeHandle &pnh) {
    nh_ = nh;
    pnh_ = pnh;
    LoadParams();
    SubAndPubToROS();
    // localmap init (after LoadParams)
    ivox_ = std::make_shared<IVoxType>(ivox_options_);
    // esekf init
    std::vector<double> epsi(23, 0.001);
    kf_.init_dyn_share(
        get_f, df_dx, df_dw,
        [this](state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) { ObsModel(s, ekfom_data); },
        options::NUM_MAX_ITERATIONS, epsi.data());

    return true;
}

bool LaserMapping::InitWithoutROS(const std::string &config_yaml) {
    LOG(INFO) << "init laser mapping from " << config_yaml;
    if (!LoadParamsFromYAML(config_yaml)) {
        return false;
    }

    // localmap init (after LoadParams)
    ivox_ = std::make_shared<IVoxType>(ivox_options_);

    // esekf init
    std::vector<double> epsi(23, 0.001);
    kf_.init_dyn_share(
        get_f, df_dx, df_dw,
        [this](state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) { ObsModel(s, ekfom_data); },
        options::NUM_MAX_ITERATIONS, epsi.data());

    if (std::is_same<IVoxType, IVox<3, IVoxNodeType::PHC, pcl::PointXYZI>>::value == true) {
        LOG(INFO) << "using phc ivox";
    } else if (std::is_same<IVoxType, IVox<3, IVoxNodeType::DEFAULT, pcl::PointXYZI>>::value == true) {
        LOG(INFO) << "using default ivox";
    }

    return true;
}

bool LaserMapping::LoadParams() {
    // get params from param server
    int lidar_type, ivox_nearby_type;
    double gyr_cov, acc_cov, b_gyr_cov, b_acc_cov;
    double filter_size_surf_min;
    common::V3D lidar_T_wrt_IMU;
    common::M3D lidar_R_wrt_IMU;

    pnh_.param<std::string>("base_link_frame", base_link_frame_, "base_footprint_tug");
    pnh_.param<std::string>("lidar_frame", lidar_frame_, "main_sensor_lidar");
    pnh_.param<std::string>("global_frame", global_frame_, "world");
    nh_.param<bool>("path_save_en", path_save_en_, true);
    nh_.param<bool>("publish/path_publish_en", path_pub_en_, true);
    nh_.param<bool>("publish/scan_publish_en", scan_pub_en_, true);
    nh_.param<bool>("publish/dense_publish_en", dense_pub_en_, false);
    nh_.param<bool>("publish/scan_bodyframe_pub_en", scan_body_pub_en_, true);
    nh_.param<bool>("publish/scan_effect_pub_en", scan_effect_pub_en_, false);
    // nh_.param<std::string>("publish/tf_imu_frame", tf_imu_frame_, "body");
    // nh_.param<std::string>("publish/tf_world_frame", tf_world_frame_, "camera_init");

    nh_.param<int>("max_iteration", options::NUM_MAX_ITERATIONS, 4);
    nh_.param<float>("esti_plane_threshold", options::ESTI_PLANE_THRESHOLD, 0.1);
    nh_.param<std::string>("map_file_path", map_file_path_, "");
    nh_.param<bool>("common/time_sync_en", time_sync_en_, false);
    nh_.param<double>("filter_size_surf", filter_size_surf_min, 0.5);
    nh_.param<double>("filter_size_map", filter_size_map_min_, 0.0);
    nh_.param<double>("cube_side_length", cube_len_, 200);
    nh_.param<float>("mapping/det_range", det_range_, 300.f);
    nh_.param<double>("mapping/gyr_cov", gyr_cov, 0.1);
    nh_.param<double>("mapping/acc_cov", acc_cov, 0.1);
    nh_.param<double>("mapping/b_gyr_cov", b_gyr_cov, 0.0001);
    nh_.param<double>("mapping/b_acc_cov", b_acc_cov, 0.0001);
    nh_.param<double>("preprocess/blind", preprocess_->Blind(), 0.01);
    nh_.param<float>("preprocess/time_scale", preprocess_->TimeScale(), 1e-3);
    nh_.param<int>("preprocess/lidar_type", lidar_type, 1);
    nh_.param<int>("preprocess/scan_line", preprocess_->NumScans(), 16);
    nh_.param<int>("point_filter_num", preprocess_->PointFilterNum(), 2);
    nh_.param<bool>("feature_extract_enable", preprocess_->FeatureEnabled(), false);
    nh_.param<bool>("runtime_pos_log_enable", runtime_pos_log_, true);
    nh_.param<bool>("mapping/extrinsic_est_en", extrinsic_est_en_, true);
    nh_.param<bool>("pcd_save/pcd_save_en", pcd_save_en_, false);
    nh_.param<int>("pcd_save/interval", pcd_save_interval_, -1);
    nh_.param<std::vector<double>>("mapping/extrinsic_T", extrinT_, std::vector<double>());
    nh_.param<std::vector<double>>("mapping/extrinsic_R", extrinR_, std::vector<double>());

    nh_.param<float>("ivox_grid_resolution", ivox_options_.resolution_, 0.2);
    nh_.param<int>("ivox_nearby_type", ivox_nearby_type, 18);

    LOG(INFO) << "lidar_type " << lidar_type;
    if (lidar_type == 1) {
        preprocess_->SetLidarType(LidarType::AVIA);
        LOG(INFO) << "Using AVIA Lidar";
    } else if (lidar_type == 2) {
        preprocess_->SetLidarType(LidarType::VELO32);
        LOG(INFO) << "Using Velodyne 32 Lidar";
    } else if (lidar_type == 3) {
        preprocess_->SetLidarType(LidarType::OUST64);
        LOG(INFO) << "Using OUST 64 Lidar";
    } else {
        LOG(WARNING) << "unknown lidar_type";
        return false;
    }

    if (ivox_nearby_type == 0) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::CENTER;
    } else if (ivox_nearby_type == 6) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY6;
    } else if (ivox_nearby_type == 18) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    } else if (ivox_nearby_type == 26) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY26;
    } else {
        LOG(WARNING) << "unknown ivox_nearby_type, use NEARBY18";
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    }

    path_.header.stamp = ros::Time::now();
    path_.header.frame_id = global_frame_;

    voxel_scan_.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);

    lidar_T_wrt_IMU = common::VecFromArray<double>(extrinT_);
    lidar_R_wrt_IMU = common::MatFromArray<double>(extrinR_);

    p_imu_->SetExtrinsic(lidar_T_wrt_IMU, lidar_R_wrt_IMU);
    p_imu_->SetGyrCov(common::V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu_->SetAccCov(common::V3D(acc_cov, acc_cov, acc_cov));
    p_imu_->SetGyrBiasCov(common::V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu_->SetAccBiasCov(common::V3D(b_acc_cov, b_acc_cov, b_acc_cov));
    return true;
}

bool LaserMapping::LoadParamsFromYAML(const std::string &yaml_file) {
    // get params from yaml
    int lidar_type, ivox_nearby_type;
    double gyr_cov, acc_cov, b_gyr_cov, b_acc_cov;
    double filter_size_surf_min;
    common::V3D lidar_T_wrt_IMU;
    common::M3D lidar_R_wrt_IMU;

    auto yaml = YAML::LoadFile(yaml_file);
    try {
        path_pub_en_ = yaml["publish"]["path_publish_en"].as<bool>();
        scan_pub_en_ = yaml["publish"]["scan_publish_en"].as<bool>();
        dense_pub_en_ = yaml["publish"]["dense_publish_en"].as<bool>();
        scan_body_pub_en_ = yaml["publish"]["scan_bodyframe_pub_en"].as<bool>();
        scan_effect_pub_en_ = yaml["publish"]["scan_effect_pub_en"].as<bool>();
        // TODO: think about this
        //  tf_imu_frame_ = yaml["publish"]["tf_imu_frame"].as<std::string>("body");
        //  tf_world_frame_ = yaml["publish"]["tf_world_frame"].as<std::string>(global_frame_);
        path_save_en_ = yaml["path_save_en"].as<bool>();

        options::NUM_MAX_ITERATIONS = yaml["max_iteration"].as<int>();
        options::ESTI_PLANE_THRESHOLD = yaml["esti_plane_threshold"].as<float>();
        time_sync_en_ = yaml["common"]["time_sync_en"].as<bool>();

        filter_size_surf_min = yaml["filter_size_surf"].as<float>();
        filter_size_map_min_ = yaml["filter_size_map"].as<float>();
        cube_len_ = yaml["cube_side_length"].as<int>();
        det_range_ = yaml["mapping"]["det_range"].as<float>();
        gyr_cov = yaml["mapping"]["gyr_cov"].as<float>();
        acc_cov = yaml["mapping"]["acc_cov"].as<float>();
        b_gyr_cov = yaml["mapping"]["b_gyr_cov"].as<float>();
        b_acc_cov = yaml["mapping"]["b_acc_cov"].as<float>();
        preprocess_->Blind() = yaml["preprocess"]["blind"].as<double>();
        preprocess_->TimeScale() = yaml["preprocess"]["time_scale"].as<double>();
        lidar_type = yaml["preprocess"]["lidar_type"].as<int>();
        preprocess_->NumScans() = yaml["preprocess"]["scan_line"].as<int>();
        preprocess_->PointFilterNum() = yaml["point_filter_num"].as<int>();
        preprocess_->FeatureEnabled() = yaml["feature_extract_enable"].as<bool>();
        extrinsic_est_en_ = yaml["mapping"]["extrinsic_est_en"].as<bool>();
        pcd_save_en_ = yaml["pcd_save"]["pcd_save_en"].as<bool>();
        pcd_save_interval_ = yaml["pcd_save"]["interval"].as<int>();
        extrinT_ = yaml["mapping"]["extrinsic_T"].as<std::vector<double>>();
        extrinR_ = yaml["mapping"]["extrinsic_R"].as<std::vector<double>>();

        ivox_options_.resolution_ = yaml["ivox_grid_resolution"].as<float>();
        ivox_nearby_type = yaml["ivox_nearby_type"].as<int>();
    } catch (...) {
        LOG(ERROR) << "bad conversion";
        return false;
    }

    LOG(INFO) << "lidar_type " << lidar_type;
    if (lidar_type == 1) {
        preprocess_->SetLidarType(LidarType::AVIA);
        LOG(INFO) << "Using AVIA Lidar";
    } else if (lidar_type == 2) {
        preprocess_->SetLidarType(LidarType::VELO32);
        LOG(INFO) << "Using Velodyne 32 Lidar";
    } else if (lidar_type == 3) {
        preprocess_->SetLidarType(LidarType::OUST64);
        LOG(INFO) << "Using OUST 64 Lidar";
    } else {
        LOG(WARNING) << "unknown lidar_type";
        return false;
    }

    if (ivox_nearby_type == 0) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::CENTER;
    } else if (ivox_nearby_type == 6) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY6;
    } else if (ivox_nearby_type == 18) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    } else if (ivox_nearby_type == 26) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY26;
    } else {
        LOG(WARNING) << "unknown ivox_nearby_type, use NEARBY18";
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    }

    voxel_scan_.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);

    lidar_T_wrt_IMU = common::VecFromArray<double>(extrinT_);
    lidar_R_wrt_IMU = common::MatFromArray<double>(extrinR_);

    p_imu_->SetExtrinsic(lidar_T_wrt_IMU, lidar_R_wrt_IMU);
    p_imu_->SetGyrCov(common::V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu_->SetAccCov(common::V3D(acc_cov, acc_cov, acc_cov));
    p_imu_->SetGyrBiasCov(common::V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu_->SetAccBiasCov(common::V3D(b_acc_cov, b_acc_cov, b_acc_cov));

    run_in_offline_ = true;
    return true;
}

void LaserMapping::SubAndPubToROS() {
    // ROS subscribe initialization
    std::string lidar_topic, imu_topic;
    nh_.param<std::string>("common/lid_topic", lidar_topic, "/livox/lidar");
    nh_.param<std::string>("common/imu_topic", imu_topic, "/livox/imu");

    sub_pcl_ = nh_.subscribe<sensor_msgs::PointCloud2>(
        lidar_topic, 200000, [this](const sensor_msgs::PointCloud2::ConstPtr &msg) { StandardPCLCallBack(msg); });

    sub_imu_ = nh_.subscribe<sensor_msgs::Imu>(imu_topic, 200000,
                                               [this](const sensor_msgs::Imu::ConstPtr &msg) { IMUCallBack(msg); });

    // ROS publisher init
    path_.header.stamp = ros::Time::now();
    path_.header.frame_id = global_frame_;

    pub_laser_cloud_world_ = pnh_.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100000);
    keypoints_pub_ = pnh_.advertise<sensor_msgs::PointCloud2>("keypoints", 100);
    pub_laser_cloud_body_ = pnh_.advertise<sensor_msgs::PointCloud2>("/cloud_registered_body", 100000);
    pub_laser_cloud_effect_world_ = pnh_.advertise<sensor_msgs::PointCloud2>("/cloud_registered_effect_world", 100000);
    pub_odom_aft_mapped_ = pnh_.advertise<nav_msgs::Odometry>("odometry", 100);
    pub_path_ = pnh_.advertise<nav_msgs::Path>("trajectory", 100);
    pub_cond_number = pnh_.advertise<std_msgs::Float64>("condition_number", 100);

    start_lio_service_ = pnh_.advertiseService("start_lidar_odom", &LaserMapping::startLIO, this);
    stop_lio_service_ = pnh_.advertiseService("stop_lidar_odom", &LaserMapping::stopLIO, this);
}

LaserMapping::LaserMapping() {
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    preprocess_.reset(new PointCloudPreprocess());
    p_imu_.reset(new ImuProcess());
}

bool LaserMapping::startLIO(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    path_.poses.clear();
    lidar_odom_ = true;
    ROS_INFO("Starting Lidar Odometry ..............!");
    return true;
}

bool LaserMapping::stopLIO(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    lidar_odom_ = false;
    return true;
}
void LaserMapping::Reset() {
    // cleared map
    ivox_->Reset();
    flg_first_scan_ = true;
    path_.poses.clear();
    p_imu_->Reset();
    pcl_wait_save_->clear();
    lidar_buffer_.clear();
    time_buffer_.clear();
    time_buffer_.clear();
    lidar_pushed_ = false;
}

void LaserMapping::Run() {
    if (!SyncPackages()) {
        return;
    }

    /// IMU process, kf prediction, undistortion
    p_imu_->Process(measures_, kf_, scan_undistort_);
    if (scan_undistort_->empty() || (scan_undistort_ == nullptr)) {
        LOG(WARNING) << "No point, skip this scan!";
        return;
    }

    if (!lidar_odom_) {
        voxel_scan_.setInputCloud(scan_undistort_);
        scan_down_body_->clear();
        scan_down_world_->clear();
        voxel_scan_.filter(*scan_down_body_);
        scan_down_world_->reserve(scan_down_body_->size());
        std::for_each(scan_down_body_->begin(), scan_down_body_->end(),
                      [&](const auto &point) { scan_down_world_->push_back(PointBodyToWorld(point)); });

        PublishOdometry(pub_odom_aft_mapped_);
        PublishKeypoints(keypoints_pub_);
        path_.poses.clear();
        PublishPath(pub_path_);
        flg_first_scan_ = true;
        return;
    }

    /// the first scan
    if (flg_first_scan_) {
        ivox_->AddPoints(scan_undistort_->points);
        first_lidar_time_ = measures_.lidar_bag_time_;
        flg_first_scan_ = false;
        return;
    }
    flg_EKF_inited_ = (measures_.lidar_bag_time_ - first_lidar_time_) >= options::INIT_TIME;

    /// downsample
    Timer::Evaluate(
        [&, this]() {
            voxel_scan_.setInputCloud(scan_undistort_);
            voxel_scan_.filter(*scan_down_body_);
        },
        "Downsample PointCloud");

    int cur_pts = scan_down_body_->size();
    if (cur_pts < 5) {
        lidar_odom_ = false;
        LOG(WARNING) << "Too few points, skip this scan!" << scan_undistort_->size() << ", " << scan_down_body_->size();
        return;
    }
    scan_down_world_->resize(cur_pts);
    nearest_points_.resize(cur_pts);
    residuals_.resize(cur_pts, 0);
    point_selected_surf_.resize(cur_pts, true);
    plane_coef_.resize(cur_pts, common::V4F::Zero());

    // ICP and iterated Kalman filter update
    Timer::Evaluate(
        [&, this]() {
            // iterated state estimation
            double solve_H_time = 0;
            // update the observation model, will call nn and point-to-plane residual computation
            kf_.update_iterated_dyn_share_modified(options::LASER_POINT_COV, solve_H_time);
            // save the state
            state_point_ = kf_.get_x();
            euler_cur_ = SO3ToEuler(state_point_.rot);
            pos_lidar_ = state_point_.pos + state_point_.rot * state_point_.offset_T_L_I;
        },
        "IEKF Solve and Update");

    // update local map
    Timer::Evaluate([&, this]() { MapIncremental(); }, "    Incremental Mapping");

    // publish or save map pcd
    PublishKeypoints(keypoints_pub_);
    PublishPath(pub_path_);
    if (run_in_offline_) {
        if (pcd_save_en_) {
            PublishFrameWorld();
        }
        if (path_save_en_) {
            PublishPath(pub_path_);
        }
    } else {
        if (pub_odom_aft_mapped_) {
            PublishOdometry(pub_odom_aft_mapped_);
        }
        if (path_pub_en_ || path_save_en_) {
            PublishPath(pub_path_);
        }
        if (scan_pub_en_ || pcd_save_en_) {
            PublishFrameWorld();
        }
        if (scan_pub_en_ && scan_body_pub_en_) {
            PublishFrameBody(pub_laser_cloud_body_);
        }
    }
    // Debug variables
    frame_num_++;
}

void LaserMapping::StandardPCLCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    mtx_buffer_.lock();
    Timer::Evaluate(
        [&, this]() {
            scan_count_++;
            if (msg->header.stamp.toSec() < last_timestamp_lidar_) {
                LOG(ERROR) << "lidar loop back, clear buffer";
                lidar_buffer_.clear();
            }

            PointCloudType::Ptr ptr(new PointCloudType());
            preprocess_->Process(msg, ptr);
            lidar_buffer_.push_back(ptr);
            time_buffer_.push_back(msg->header.stamp.toSec());
            last_timestamp_lidar_ = msg->header.stamp.toSec();
        },
        "Preprocess (Standard)");

    mtx_buffer_.unlock();
}

void LaserMapping::IMUCallBack(const sensor_msgs::Imu::ConstPtr &msg_in) {
    publish_count_++;
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    if (abs(timediff_lidar_wrt_imu_) > 0.1 && time_sync_en_) {
        msg->header.stamp = ros::Time().fromSec(timediff_lidar_wrt_imu_ + msg_in->header.stamp.toSec());
    }

    double timestamp = msg->header.stamp.toSec();

    mtx_buffer_.lock();
    if (timestamp < last_timestamp_imu_) {
        LOG(WARNING) << "imu loop back, clear buffer";
        imu_buffer_.clear();
    }

    last_timestamp_imu_ = timestamp;
    imu_buffer_.emplace_back(msg);
    mtx_buffer_.unlock();
}

bool LaserMapping::SyncPackages() {
    if (lidar_buffer_.empty() || imu_buffer_.empty()) {
        return false;
    }

    /*** push a lidar scan ***/
    if (!lidar_pushed_) {
        measures_.lidar_ = lidar_buffer_.front();
        measures_.lidar_bag_time_ = time_buffer_.front();

        if (measures_.lidar_->points.size() <= 1) {
            LOG(WARNING) << "Too few input point cloud!";
            lidar_end_time_ = measures_.lidar_bag_time_ + lidar_mean_scantime_;
        } else if (measures_.lidar_->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime_) {
            lidar_end_time_ = measures_.lidar_bag_time_ + lidar_mean_scantime_;
        } else {
            scan_num_++;
            lidar_end_time_ = measures_.lidar_bag_time_ + measures_.lidar_->points.back().curvature / double(1000);
            lidar_mean_scantime_ +=
                (measures_.lidar_->points.back().curvature / double(1000) - lidar_mean_scantime_) / scan_num_;
        }

        measures_.lidar_end_time_ = lidar_end_time_;
        lidar_pushed_ = true;
    }

    if (last_timestamp_imu_ < lidar_end_time_) {
        return false;
    }

    /*** push imu_ data, and pop from imu_ buffer ***/
    double imu_time = imu_buffer_.front()->header.stamp.toSec();
    measures_.imu_.clear();
    while ((!imu_buffer_.empty()) && (imu_time < lidar_end_time_)) {
        imu_time = imu_buffer_.front()->header.stamp.toSec();
        if (imu_time > lidar_end_time_) break;
        measures_.imu_.push_back(imu_buffer_.front());
        imu_buffer_.pop_front();
    }

    lidar_buffer_.pop_front();
    time_buffer_.pop_front();
    lidar_pushed_ = false;
    return true;
}

void LaserMapping::PrintState(const state_ikfom &s) {
    LOG(INFO) << "state r: " << s.rot.coeffs().transpose() << ", t: " << s.pos.transpose()
              << ", off r: " << s.offset_R_L_I.coeffs().transpose() << ", t: " << s.offset_T_L_I.transpose();
}

void LaserMapping::MapIncremental() {
    PointVector points_to_add;
    PointVector point_no_need_downsample;

    int cur_pts = scan_down_body_->size();
    points_to_add.reserve(cur_pts);
    point_no_need_downsample.reserve(cur_pts);

    std::vector<size_t> index(cur_pts);
    std::iota(index.begin(), index.end(), 0.0);

    std::for_each(index.begin(), index.end(), [&](const size_t &i) {
        /* transform to world frame */
        scan_down_world_->points[i] = PointBodyToWorld(scan_down_body_->points[i]);

        /* decide if need add to map */
        PointType &point_world = scan_down_world_->points[i];
        if (!nearest_points_[i].empty() && flg_EKF_inited_) {
            const PointVector &points_near = nearest_points_[i];

            Eigen::Vector3f center =
                ((point_world.getVector3fMap() / filter_size_map_min_).array().floor() + 0.5) * filter_size_map_min_;

            Eigen::Vector3f dis_2_center = points_near[0].getVector3fMap() - center;

            if (fabs(dis_2_center.x()) > 0.5 * filter_size_map_min_ &&
                fabs(dis_2_center.y()) > 0.5 * filter_size_map_min_ &&
                fabs(dis_2_center.z()) > 0.5 * filter_size_map_min_) {
                point_no_need_downsample.emplace_back(point_world);
                return;
            }

            // TODO delete this and tbbfy the loop based on num points
            bool need_add = true;
            float dist = common::calc_dist(point_world.getVector3fMap(), center);
            if (points_near.size() >= options::NUM_MATCH_POINTS) {
                for (int readd_i = 0; readd_i < options::NUM_MATCH_POINTS; readd_i++) {
                    if (common::calc_dist(points_near[readd_i].getVector3fMap(), center) < dist + 1e-6) {
                        need_add = false;
                        break;
                    }
                }
            }
            if (need_add) {
                points_to_add.emplace_back(point_world);
            }
        } else {
            points_to_add.emplace_back(point_world);
        }
    });

    Timer::Evaluate(
        [&, this]() {
            ivox_->AddPoints(points_to_add);
            ivox_->AddPoints(point_no_need_downsample);
        },
        "    IVox Add Points");
}

void LaserMapping::computeConditionNumber(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& h_x)
{
    Eigen::Matrix<double, 6, 6> A = Eigen::Matrix<double, 6, 6>::Zero();
    for (int i = 0; i < h_x.rows(); ++i)
    {
        /// Input is J[1x12], we get [1x6]
        const Eigen::Matrix<double, 1, 6>& J = h_x.row(i).head<6>();
        /// for each jacobian do JtJ and sum all of them
        const Eigen::Matrix<double, 6, 6> JTJ = J.transpose() * J;
        A += JTJ;
    }
    /// Extract only the translation part becoming 3x3 = C
    const Eigen::Matrix<double, 3, 3> C = A.topLeftCorner<3, 3>();
    /// CTC
    const Eigen::Matrix<double, 3, 3> CTC = C.transpose() * C;

    /// Compute eigenvalues
    Eigen::EigenSolver<Eigen::Matrix3d> solver(CTC);
    Eigen::Vector3d eigenvalues = solver.eigenvalues().real();

    /// Get max and min eigenvalues
    double min_eigenvalue = eigenvalues.minCoeff();
    double max_eigenvalue = eigenvalues.maxCoeff();

    /// Compute condition number
    /// Adding a small constant to the denominator to avoid dividing by a very small number
    const auto condition_number = sqrt(max_eigenvalue/(min_eigenvalue + 1e-7));

    std_msgs::Float64 msg;
    msg.data = condition_number;
    pub_cond_number.publish(msg);
    return;
}
/**
 * Lidar point cloud registration
 * will be called by the eskf custom observation model
 * compute point-to-plane residual here
 * @param s kf state
 * @param ekfom_data H matrix
 */
void LaserMapping::ObsModel(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) {
    int cnt_pts = scan_down_body_->size();

    std::vector<size_t> index(cnt_pts);
    std::iota(index.begin(), index.end(), 0);
    // for (size_t i = 0; i < index.size(); ++i) {
    // index[i] = i;
    // }

    /// Computes point to plane distances
    Timer::Evaluate(
        [&, this]() {
            auto R_wl = (s.rot * s.offset_R_L_I).cast<float>();
            auto t_wl = (s.rot * s.offset_T_L_I + s.pos).cast<float>();

            tbb::parallel_for(tbb::blocked_range<int>(0, cnt_pts), [&](tbb::blocked_range<int> r) {
                for (auto i = r.begin(); i < r.end(); ++i) {
                    // TODO: these non const should die
                    const PointType &point_body = scan_down_body_->points[i];

                    /* transform to world frame */
                    common::V3F p_body = point_body.getVector3fMap();
                    PointType point_world = PointType();
                    point_world.getVector3fMap() = R_wl * p_body + t_wl;
                    point_world.intensity = point_body.intensity;
                    scan_down_world_->points[i] = point_world;

                    if (ekfom_data.converge) {
                        /** Find the closest surfaces in the map **/
                        PointVector points_near;
                        ivox_->GetClosestPoint(point_world, points_near, options::NUM_MATCH_POINTS);
                        nearest_points_[i] = points_near;
                        point_selected_surf_[i] = points_near.size() >= options::MIN_NUM_MATCH_POINTS;
                        if (point_selected_surf_[i]) {
                            point_selected_surf_[i] =
                                common::esti_plane(plane_coef_[i], points_near, options::ESTI_PLANE_THRESHOLD);
                        }
                    }

                    if (point_selected_surf_[i]) {
                        auto temp = point_world.getVector4fMap();
                        temp[3] = 1.0;
                        float pd2 = plane_coef_[i].dot(temp);

                        bool valid_corr = p_body.norm() > 81 * pd2 * pd2;
                        if (valid_corr) {
                            point_selected_surf_[i] = true;
                            residuals_[i] = pd2;
                        }
                    }
                }
            });
        },
        "    ObsModel (Lidar Match)");

    effect_feat_num_ = 0;

    corr_pts_.resize(cnt_pts);
    corr_norm_.resize(cnt_pts);
    for (int i = 0; i < cnt_pts; i++) {
        if (point_selected_surf_[i]) {
            corr_norm_[effect_feat_num_] = plane_coef_[i];
            corr_pts_[effect_feat_num_] = scan_down_body_->points[i].getVector4fMap();
            corr_pts_[effect_feat_num_][3] = residuals_[i];

            effect_feat_num_++;
        }
    }
    corr_pts_.resize(effect_feat_num_);
    corr_norm_.resize(effect_feat_num_);

    if (effect_feat_num_ < 1) {
        ekfom_data.valid = false;
        LOG(WARNING) << "No Effective Points!";
        return;
    }

    Timer::Evaluate(
        [&, this]() {
            /*** Computation of Measurement Jacobian matrix H and measurements vector ***/
            ekfom_data.h_x = Eigen::MatrixXd::Zero(effect_feat_num_, 12);  // 23
            ekfom_data.h.resize(effect_feat_num_);

            index.resize(effect_feat_num_);
            /// Rotation lidar to IMU
            const common::M3F off_R = s.offset_R_L_I.toRotationMatrix().cast<float>();
            /// Translation lidar to IMU
            const common::V3F off_t = s.offset_T_L_I.cast<float>();
            const common::M3F Rt = s.rot.toRotationMatrix().transpose().cast<float>();

            tbb::parallel_for(tbb::blocked_range<int>(0, index.size()), [&](tbb::blocked_range<int> r) {
                for (auto i = r.begin(); i < r.end(); ++i) {
                    common::V3F point_this_be = corr_pts_[i].head<3>();
                    common::M3F point_be_crossmat = SKEW_SYM_MATRIX(point_this_be);
                    common::V3F point_this = off_R * point_this_be + off_t;
                    common::M3F point_crossmat = SKEW_SYM_MATRIX(point_this);

                    /*** get the normal vector of closest surface/corner ***/
                    common::V3F norm_vec = corr_norm_[i].head<3>();

                    /*** calculate the Measurement Jacobian matrix H ***/
                    common::V3F C(Rt * norm_vec);
                    common::V3F A(point_crossmat * C);

                    if (extrinsic_est_en_) {
                        common::V3F B(point_be_crossmat * off_R.transpose() * C);
                        ekfom_data.h_x.block<1, 12>(i, 0) << norm_vec[0], norm_vec[1], norm_vec[2], A[0], A[1], A[2],
                            B[0], B[1], B[2], C[0], C[1], C[2];
                    } else {
                        ekfom_data.h_x.block<1, 12>(i, 0) << norm_vec[0], norm_vec[1], norm_vec[2], A[0], A[1], A[2],
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
                    }

                    /*** Measurement: distance to the closest surface/corner ***/
                    ekfom_data.h(i) = -corr_pts_[i][3];
                }
            });
        },
        "    ObsModel (IEKF Build Jacobian)");
    computeConditionNumber(ekfom_data.h_x);
}

/////////////////////////////////////  debug save / show /////////////////////////////////////////////////////

void LaserMapping::PublishPath(const ros::Publisher pub_path) {
    SetPosestamp(msg_body_pose_);
    msg_body_pose_.header.stamp = ros::Time().fromSec(lidar_end_time_);
    msg_body_pose_.header.frame_id = global_frame_;

    /*** if path is too large, the rvis will crash ***/
    path_.poses.push_back(msg_body_pose_);
    if (run_in_offline_ == false) {
        pub_path.publish(path_);
    }
}

void LaserMapping::PublishKeypoints(const ros::Publisher &pubLaserCloudFull) {
    // ROS_INFO("Internally the keypoints size is %zu", feats_down_body->size());
    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*scan_down_world_, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time_);
    laserCloudmsg.header.frame_id = global_frame_;
    pubLaserCloudFull.publish(laserCloudmsg);
}
void LaserMapping::PublishOdometry(const ros::Publisher &pub_odom_aft_mapped) {
    // TODO: change this

    static tf::TransformBroadcaster br;
    if (!lidar_odom_) {
        // Broadcast the tf
        geometry_msgs::TransformStamped transform_msg;
        transform_msg.header.stamp = ros::Time().fromSec(lidar_end_time_);
        transform_msg.header.frame_id = global_frame_;
        transform_msg.child_frame_id = base_link_frame_;
        transform_msg.transform.rotation.x = 0;
        transform_msg.transform.rotation.y = 0;
        transform_msg.transform.rotation.z = 0;
        transform_msg.transform.rotation.w = 1;
        transform_msg.transform.translation.x = 0;
        transform_msg.transform.translation.y = 0;
        transform_msg.transform.translation.z = 0;
        br.sendTransform(transform_msg);

        // publish odometry msg as Identity
        odom_aft_mapped_.header.stamp = ros::Time().fromSec(lidar_end_time_);
        odom_aft_mapped_.header.frame_id = global_frame_;
        odom_aft_mapped_.child_frame_id = base_link_frame_;
        odom_aft_mapped_.pose.pose.orientation.x = 0;
        odom_aft_mapped_.pose.pose.orientation.y = 0;
        odom_aft_mapped_.pose.pose.orientation.z = 0;
        odom_aft_mapped_.pose.pose.orientation.w = 1;
        odom_aft_mapped_.pose.pose.position.x = 0;
        odom_aft_mapped_.pose.pose.position.y = 0;
        odom_aft_mapped_.pose.pose.position.z = 0;
        pub_odom_aft_mapped.publish(odom_aft_mapped_);
        return;
    }
    odom_aft_mapped_.header.frame_id = global_frame_;
    // TODO: think about this
    odom_aft_mapped_.child_frame_id = base_link_frame_;
    odom_aft_mapped_.header.stamp = ros::Time().fromSec(lidar_end_time_);
    SetPosestamp(odom_aft_mapped_.pose);
    pub_odom_aft_mapped.publish(odom_aft_mapped_);
    auto P = kf_.get_P();
    for (int i = 0; i < 6; i++) {
        int k = i < 3 ? i + 3 : i - 3;
        odom_aft_mapped_.pose.covariance[i * 6 + 0] = P(k, 3);
        odom_aft_mapped_.pose.covariance[i * 6 + 1] = P(k, 4);
        odom_aft_mapped_.pose.covariance[i * 6 + 2] = P(k, 5);
        odom_aft_mapped_.pose.covariance[i * 6 + 3] = P(k, 0);
        odom_aft_mapped_.pose.covariance[i * 6 + 4] = P(k, 1);
        odom_aft_mapped_.pose.covariance[i * 6 + 5] = P(k, 2);
    }

    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(odom_aft_mapped_.pose.pose.position.x, odom_aft_mapped_.pose.pose.position.y,
                                    odom_aft_mapped_.pose.pose.position.z));
    q.setW(odom_aft_mapped_.pose.pose.orientation.w);
    q.setX(odom_aft_mapped_.pose.pose.orientation.x);
    q.setY(odom_aft_mapped_.pose.pose.orientation.y);
    q.setZ(odom_aft_mapped_.pose.pose.orientation.z);
    transform.setRotation(q);

    tf::StampedTransform sensor2tug;
    try {
        tf_listener_.waitForTransform(lidar_frame_, base_link_frame_, ros::Time(0), ros::Duration(3.0));
        tf_listener_.lookupTransform(lidar_frame_, base_link_frame_, ros::Time(0), sensor2tug);

        tf::Transform odom2tug = transform * sensor2tug;
        br.sendTransform(
            tf::StampedTransform(odom2tug, ros::Time().fromSec(lidar_end_time_), global_frame_, base_link_frame_));
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
    }
}

void LaserMapping::PublishFrameWorld() {
    if (!(run_in_offline_ == false && scan_pub_en_) && !pcd_save_en_) {
        return;
    }

    PointCloudType::Ptr laserCloudWorld;
    if (dense_pub_en_) {
        PointCloudType::Ptr laserCloudFullRes(scan_undistort_);
        int size = laserCloudFullRes->points.size();
        laserCloudWorld.reset(new PointCloudType(size, 1));
        for (int i = 0; i < size; i++) {
            laserCloudWorld->points[i] = PointBodyToWorld(laserCloudFullRes->points[i]);
        }
    } else {
        laserCloudWorld = scan_down_world_;
    }

    if (run_in_offline_ == false && scan_pub_en_) {
        sensor_msgs::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time_);
        laserCloudmsg.header.frame_id = global_frame_;
        pub_laser_cloud_world_.publish(laserCloudmsg);
        publish_count_ -= options::PUBFRAME_PERIOD;
    }

    /**************** save map ****************/
    // 1. make sure you have enough memories
    // 2. noted that pcd save will influence the real-time performences
    if (pcd_save_en_) {
        *pcl_wait_save_ += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num++;
        if (pcl_wait_save_->size() > 0 && pcd_save_interval_ > 0 && scan_wait_num >= pcd_save_interval_) {
            pcd_index_++;
            std::string all_points_dir(std::string(std::string(ROOT_DIR) + "PCD/scans_") + std::to_string(pcd_index_) +
                                       std::string(".pcd"));
            pcl::PCDWriter pcd_writer;
            LOG(INFO) << "current scan saved to /PCD/" << all_points_dir;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save_);
            pcl_wait_save_->clear();
            scan_wait_num = 0;
        }
    }
}

void LaserMapping::PublishFrameBody(const ros::Publisher &pub_laser_cloud_body) {
    int size = scan_undistort_->points.size();
    PointCloudType::Ptr laser_cloud_imu_body(new PointCloudType(size, 1));

    for (int i = 0; i < size; i++) {
        PointBodyLidarToIMU(&scan_undistort_->points[i], &laser_cloud_imu_body->points[i]);
    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laser_cloud_imu_body, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time_);
    laserCloudmsg.header.frame_id = base_link_frame_;
    pub_laser_cloud_body.publish(laserCloudmsg);
    publish_count_ -= options::PUBFRAME_PERIOD;
}

void LaserMapping::Savetrajectory(const std::string &traj_file) {
    std::ofstream ofs;
    ofs.open(traj_file, std::ios::out);
    if (!ofs.is_open()) {
        LOG(ERROR) << "Failed to open traj_file: " << traj_file;
        return;
    }

    ofs << "#timestamp x y z q_x q_y q_z q_w" << std::endl;
    for (const auto &p : path_.poses) {
        ofs << std::fixed << std::setprecision(6) << p.header.stamp.toSec() << " " << std::setprecision(15)
            << p.pose.position.x << " " << p.pose.position.y << " " << p.pose.position.z << " " << p.pose.orientation.x
            << " " << p.pose.orientation.y << " " << p.pose.orientation.z << " " << p.pose.orientation.w << std::endl;
    }

    ofs.close();
}

///////////////////////////  private method /////////////////////////////////////////////////////////////////////
template <typename T>
void LaserMapping::SetPosestamp(T &out) {
    out.pose.position.x = state_point_.pos(0);
    out.pose.position.y = state_point_.pos(1);
    out.pose.position.z = state_point_.pos(2);
    out.pose.orientation.x = state_point_.rot.coeffs()[0];
    out.pose.orientation.y = state_point_.rot.coeffs()[1];
    out.pose.orientation.z = state_point_.rot.coeffs()[2];
    out.pose.orientation.w = state_point_.rot.coeffs()[3];
}

PointType LaserMapping::PointBodyToWorld(const PointType &pi) {
    common::V3D p_body(pi.x, pi.y, pi.z);
    common::V3D p_global(state_point_.rot * (state_point_.offset_R_L_I * p_body + state_point_.offset_T_L_I) +
                         state_point_.pos);
    PointType po;
    po.x = p_global(0);
    po.y = p_global(1);
    po.z = p_global(2);
    po.intensity = pi.intensity;
    return po;
}

void LaserMapping::PointBodyLidarToIMU(PointType const *const pi, PointType *const po) {
    common::V3D p_body_lidar(pi->x, pi->y, pi->z);
    common::V3D p_body_imu(state_point_.offset_R_L_I * p_body_lidar + state_point_.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void LaserMapping::Finish() {
    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. pcd save will largely influence the real-time performences **/
    if (pcl_wait_save_->size() > 0 && pcd_save_en_) {
        std::string file_name = std::string("scans.pcd");
        std::string all_points_dir(std::string(std::string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        LOG(INFO) << "current scan saved to /PCD/" << file_name;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save_);
    }

    LOG(INFO) << "finish done";
}
}  // namespace faster_lio
