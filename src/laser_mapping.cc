#include "laser_mapping.h"

#include <std_msgs/Float64.h>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tf/transform_broadcaster.h>
#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <numeric>

#include "profiling.h"

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

bool LaserMapping::LoadParams() {
    // get params from param server
    int ivox_nearby_type;
    double gyr_cov, acc_cov, b_gyr_cov, b_acc_cov;
    common::V3D lidar_T_wrt_IMU;
    common::M3D lidar_R_wrt_IMU;

    pnh_.param<std::string>("base_link_frame", base_link_frame_, "base_footprint_tug");
    pnh_.param<std::string>("lidar_frame", lidar_frame_, "main_sensor_lidar");
    pnh_.param<std::string>("global_frame", global_frame_, "world");
    pnh_.param<std::string>("trajectory_file", trajectory_file_, "");
    nh_.param<bool>("publish/scan_publish_en", scan_pub_en_, true);
    nh_.param<bool>("publish/dense_publish_en", dense_pub_en_, false);
    nh_.param<bool>("publish/scan_bodyframe_pub_en", scan_body_pub_en_, true);

    nh_.param<int>("max_iteration", options::NUM_MAX_ITERATIONS, 4);
    nh_.param<float>("esti_plane_threshold", options::ESTI_PLANE_THRESHOLD, 0.1);
    nh_.param<float>("filter_size_surf", filter_size_surf_, 0.5f);
    nh_.param<double>("filter_size_map", filter_size_map_min_, 0.0);
    nh_.param<double>("mapping/gyr_cov", gyr_cov, 0.1);
    nh_.param<double>("mapping/acc_cov", acc_cov, 0.1);
    nh_.param<double>("mapping/b_gyr_cov", b_gyr_cov, 0.0001);
    nh_.param<double>("mapping/b_acc_cov", b_acc_cov, 0.0001);
    nh_.param<double>("preprocess/blind", preprocess_->Blind(), 0.01);
    nh_.param<int>("point_filter_num", preprocess_->PointFilterNum(), 2);
    int tbb_num_threads = 0;
    nh_.param<int>("tbb_num_threads", tbb_num_threads, 0);
    if (tbb_num_threads > 0) {
        tbb_control_ = std::make_unique<tbb::global_control>(tbb::global_control::max_allowed_parallelism,
                                                             tbb_num_threads);
        ROS_INFO_STREAM("TBB limited to " << tbb_num_threads << " threads");
    }
    nh_.param<bool>("mapping/extrinsic_est_en", extrinsic_est_en_, true);
    nh_.param<std::vector<double>>("mapping/extrinsic_T", extrinT_, std::vector<double>());
    nh_.param<std::vector<double>>("mapping/extrinsic_R", extrinR_, std::vector<double>());

    nh_.param<float>("ivox_grid_resolution", ivox_options_.resolution_, 0.2);
    nh_.param<int>("ivox_nearby_type", ivox_nearby_type, 18);
    nh_.param<float>("nn_reuse_distance", nn_reuse_distance_, nn_reuse_distance_);
    int ivox_capacity = static_cast<int>(ivox_options_.capacity_);
    nh_.param<int>("ivox_capacity", ivox_capacity, ivox_capacity);  // max voxels kept, least recently used dropped
    ivox_options_.capacity_ = static_cast<std::size_t>(std::max(ivox_capacity, 1));

    if (ivox_nearby_type == 0) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::CENTER;
    } else if (ivox_nearby_type == 6) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY6;
    } else if (ivox_nearby_type == 18) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    } else if (ivox_nearby_type == 26) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY26;
    } else {
        ROS_WARN_STREAM("unknown ivox_nearby_type, use NEARBY18");
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    }

    path_.header.stamp = ros::Time::now();
    path_.header.frame_id = global_frame_;

    lidar_T_wrt_IMU = common::VecFromArray<double>(extrinT_);
    lidar_R_wrt_IMU = common::MatFromArray<double>(extrinR_);

    p_imu_->SetExtrinsic(lidar_T_wrt_IMU, lidar_R_wrt_IMU);
    p_imu_->SetGyrCov(common::V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu_->SetAccCov(common::V3D(acc_cov, acc_cov, acc_cov));
    p_imu_->SetGyrBiasCov(common::V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu_->SetAccBiasCov(common::V3D(b_acc_cov, b_acc_cov, b_acc_cov));
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
    pub_odom_aft_mapped_ = pnh_.advertise<nav_msgs::Odometry>("odometry", 100);
    pub_path_ = pnh_.advertise<nav_msgs::Path>("trajectory", 100);
    pub_cond_number = pnh_.advertise<std_msgs::Float64>("condition_number", 100);

    start_lio_service_ = pnh_.advertiseService("start_lidar_odom", &LaserMapping::startLIO, this);
    stop_lio_service_ = pnh_.advertiseService("stop_lidar_odom", &LaserMapping::stopLIO, this);
    save_trajectory_service_ = pnh_.advertiseService("save_trajectory", &LaserMapping::saveTrajectory, this);
}

LaserMapping::LaserMapping() {
    preprocess_.reset(new PointCloudPreprocess());
    p_imu_.reset(new ImuProcess());
}

bool LaserMapping::startLIO(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    path_.poses.clear();
    trajectory_.clear();
    lidar_odom_ = true;
    ROS_INFO("Starting Lidar Odometry ..............!");
    return true;
}

bool LaserMapping::stopLIO(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    lidar_odom_ = false;
    return true;
}

bool LaserMapping::saveTrajectory(faster_lio::SaveTrajectory::Request &req,
                                  faster_lio::SaveTrajectory::Response &res) {
    const std::filesystem::path file = req.file_path.empty() ? trajectory_file_ : req.file_path;
    if (file.empty()) {
        res.success = false;
        res.message = "no file_path given and ~trajectory_file is not set";
        return true;
    }

    std::error_code ec;
    if (file.has_parent_path()) {
        std::filesystem::create_directories(file.parent_path(), ec);
    }
    std::FILE *f = std::fopen(file.c_str(), "w");
    if (f == nullptr) {
        res.success = false;
        res.message = "cannot open " + file.string() + " for writing";
        return true;
    }
    std::fprintf(f, "# timestamp tx ty tz qx qy qz qw\n");
    for (const auto &p : trajectory_) {
        std::fprintf(f, "%.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f\n", p.stamp, p.pos.x(), p.pos.y(), p.pos.z(),
                     p.rot.x(), p.rot.y(), p.rot.z(), p.rot.w());
    }
    res.success = std::fclose(f) == 0;
    res.message = res.success ? "saved " + std::to_string(trajectory_.size()) + " poses to " + file.string()
                              : "error writing " + file.string();
    ROS_INFO_STREAM(res.message);
    return true;
}

void LaserMapping::Run() {
    PROFILE_SCOPE("LaserMapping::Run");
    if (!SyncPackages()) {
        return;
    }
    obs_model_calls_ = 0;

    /// IMU process, kf prediction, undistortion
    p_imu_->Process(measures_, kf_, scan_undistort_);
    if (scan_undistort_->empty() || (scan_undistort_ == nullptr)) {
        ROS_WARN_STREAM("No point, skip this scan!");
        return;
    }

    if (!lidar_odom_) {
        VoxelDownsample(*scan_undistort_, filter_size_surf_, *scan_down_body_);
        scan_down_world_->clear();
        scan_down_world_->reserve(scan_down_body_->size());
        std::for_each(scan_down_body_->begin(), scan_down_body_->end(),
                      [&](const auto &point) { scan_down_world_->push_back(PointBodyToWorld(point)); });

        PublishOdometry(pub_odom_aft_mapped_);
        PublishKeypoints(keypoints_pub_);
        path_.poses.clear();
        PublishPath(pub_path_);
        flg_first_scan_ = true;
        ProfileScan();
        return;
    }

    /// the first scan
    if (flg_first_scan_) {
        MapPointVector first_points;
        first_points.reserve(scan_undistort_->size());
        for (const auto &pt : *scan_undistort_) {
            first_points.emplace_back(pt.x, pt.y, pt.z);
        }
        ivox_->AddPoints(first_points);
        first_lidar_time_ = measures_.lidar_bag_time_;
        flg_first_scan_ = false;
        return;
    }
    flg_EKF_inited_ = (measures_.lidar_bag_time_ - first_lidar_time_) >= options::INIT_TIME;

    /// downsample
    {
        PROFILE_SCOPE("downsample");
        VoxelDownsample(*scan_undistort_, filter_size_surf_, *scan_down_body_);
    }

    int cur_pts = scan_down_body_->size();
    if (cur_pts < 5) {
        lidar_odom_ = false;
        ROS_WARN_STREAM("Too few points, skip this scan!" << scan_undistort_->size() << ", " << scan_down_body_->size());
        return;
    }
    scan_down_world_->resize(cur_pts);
    nearest_points_.resize(cur_pts);
    residuals_.resize(cur_pts, 0);
    point_selected_surf_.resize(cur_pts, true);
    plane_coef_.resize(cur_pts, common::V4F::Zero());
    search_pos_.resize(cur_pts);
    plane_ok_.resize(cur_pts, 0);
    searched_this_scan_ = false;
    nn_searches_ = 0;

    // ICP and iterated Kalman filter update
    {
        PROFILE_SCOPE("IEKF update");
        // iterated state estimation
        double solve_H_time = 0;
        // update the observation model, will call nn and point-to-plane residual computation
        kf_.update_iterated_dyn_share_modified(options::LASER_POINT_COV, solve_H_time);
        // save the state
        state_point_ = kf_.get_x();
    }

    // update local map
    {
        PROFILE_SCOPE("map incremental");
        MapIncremental();
    }

    trajectory_.push_back({lidar_end_time_, state_point_.pos, Eigen::Quaterniond(state_point_.rot)});

    // publish
    PublishConditionNumber();
    PublishKeypoints(keypoints_pub_);
    PublishPath(pub_path_);
    PublishOdometry(pub_odom_aft_mapped_);
    if (scan_pub_en_) {
        PublishFrameWorld();
        if (scan_body_pub_en_) {
            PublishFrameBody(pub_laser_cloud_body_);
        }
    }
    ProfileScan();
}

void LaserMapping::ProfileScan() {
    PROFILE_PLOT("scan points (preprocessed)", static_cast<int64_t>(measures_.lidar_ ? measures_.lidar_->size() : 0));
    PROFILE_PLOT("scan points (downsampled)", static_cast<int64_t>(scan_down_body_->size()));
    PROFILE_PLOT("effective features", static_cast<int64_t>(effect_feat_num_));
    PROFILE_PLOT("IEKF iterations", static_cast<int64_t>(obs_model_calls_));
    PROFILE_PLOT("nn searches", static_cast<int64_t>(nn_searches_.load()));
    PROFILE_PLOT("map voxels", static_cast<int64_t>(ivox_->NumValidGrids()));
    PROFILE_PLOT("lidar buffer", static_cast<int64_t>(lidar_buffer_.size()));
    profiling::PlotProcessStats();
    PROFILE_FRAME();
}

void LaserMapping::StandardPCLCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    PROFILE_SCOPE("lidar callback");
    mtx_buffer_.lock();
    if (msg->header.stamp.toSec() < last_timestamp_lidar_) {
        ROS_ERROR_STREAM("lidar loop back, clear buffer");
        lidar_buffer_.clear();
    }

    CloudPtr ptr(new PointCloudType());
    preprocess_->Process(msg, ptr);
    lidar_buffer_.push_back(ptr);
    time_buffer_.push_back(msg->header.stamp.toSec());
    last_timestamp_lidar_ = msg->header.stamp.toSec();

    mtx_buffer_.unlock();
}

void LaserMapping::IMUCallBack(const sensor_msgs::Imu::ConstPtr &msg_in) {
    PROFILE_SCOPE("imu callback");
    const double timestamp = msg_in->header.stamp.toSec();

    mtx_buffer_.lock();
    if (timestamp < last_timestamp_imu_) {
        ROS_WARN_STREAM("imu loop back, clear buffer");
        imu_buffer_.clear();
    }

    last_timestamp_imu_ = timestamp;
    imu_buffer_.emplace_back(msg_in);
    mtx_buffer_.unlock();
}

bool LaserMapping::SyncPackages() {
    PROFILE_SCOPE("sync packages");
    if (lidar_buffer_.empty() || imu_buffer_.empty()) {
        return false;
    }

    /*** push a lidar scan ***/
    if (!lidar_pushed_) {
        measures_.lidar_ = lidar_buffer_.front();
        measures_.lidar_bag_time_ = time_buffer_.front();

        if (measures_.lidar_->size() <= 1) {
            ROS_WARN_STREAM("Too few input point cloud!");
            lidar_end_time_ = measures_.lidar_bag_time_ + lidar_mean_scantime_;
        } else if (measures_.lidar_->back().time / double(1000) < 0.5 * lidar_mean_scantime_) {
            lidar_end_time_ = measures_.lidar_bag_time_ + lidar_mean_scantime_;
        } else {
            scan_num_++;
            lidar_end_time_ = measures_.lidar_bag_time_ + measures_.lidar_->back().time / double(1000);
            lidar_mean_scantime_ +=
                (measures_.lidar_->back().time / double(1000) - lidar_mean_scantime_) / scan_num_;
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

void LaserMapping::MapIncremental() {
    MapPointVector points_to_add;
    MapPointVector point_no_need_downsample;

    int cur_pts = scan_down_body_->size();
    points_to_add.reserve(cur_pts);
    point_no_need_downsample.reserve(cur_pts);

    std::vector<size_t> index(cur_pts);
    std::iota(index.begin(), index.end(), 0.0);

    // same transform as PointBodyToWorld, composed once per scan instead of per point
    const common::M3D R_wl = state_point_.rot.toRotationMatrix() * state_point_.offset_R_L_I.toRotationMatrix();
    const common::V3D t_wl = state_point_.rot * state_point_.offset_T_L_I + state_point_.pos;

    std::for_each(index.begin(), index.end(), [&](const size_t &i) {
        /* transform to world frame */
        const PointType &point_body = (*scan_down_body_)[i];
        const common::V3D p_world = R_wl * common::V3D(point_body.x, point_body.y, point_body.z) + t_wl;
        PointType &point_world = (*scan_down_world_)[i];
        point_world = PointType();
        point_world.x = p_world.x();
        point_world.y = p_world.y();
        point_world.z = p_world.z();
        point_world.intensity = point_body.intensity;

        /* decide if need add to map */
        if (!nearest_points_[i].empty() && flg_EKF_inited_) {
            const MapPointVector &points_near = nearest_points_[i];

            Eigen::Vector3f center =
                ((point_world.pos() / filter_size_map_min_).array().floor() + 0.5) * filter_size_map_min_;

            Eigen::Vector3f dis_2_center = points_near[0].pos() - center;

            if (fabs(dis_2_center.x()) > 0.5 * filter_size_map_min_ &&
                fabs(dis_2_center.y()) > 0.5 * filter_size_map_min_ &&
                fabs(dis_2_center.z()) > 0.5 * filter_size_map_min_) {
                point_no_need_downsample.emplace_back(point_world.x, point_world.y, point_world.z);
                return;
            }

            // TODO delete this and tbbfy the loop based on num points
            bool need_add = true;
            float dist = common::calc_dist(point_world.pos(), center);
            if (points_near.size() >= options::NUM_MATCH_POINTS) {
                for (int readd_i = 0; readd_i < options::NUM_MATCH_POINTS; readd_i++) {
                    if (common::calc_dist(points_near[readd_i].pos(), center) < dist + 1e-6) {
                        need_add = false;
                        break;
                    }
                }
            }
            if (need_add) {
                points_to_add.emplace_back(point_world.x, point_world.y, point_world.z);
            }
        } else {
            points_to_add.emplace_back(point_world.x, point_world.y, point_world.z);
        }
    });

    {
        PROFILE_SCOPE("ivox add points");
        ivox_->AddPoints(points_to_add);
        ivox_->AddPoints(point_no_need_downsample);
    }
}

void LaserMapping::PublishConditionNumber() {
    PROFILE_SCOPE("publish condition number");
    if (!cond_jtj_valid_) {
        return;
    }
    /// Extract only the translation part becoming 3x3 = C
    const Eigen::Matrix<double, 3, 3> C = cond_jtj_.topLeftCorner<3, 3>();
    /// CTC is symmetric, so its eigenvalues are real
    const Eigen::Matrix<double, 3, 3> CTC = C.transpose() * C;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(CTC, Eigen::EigenvaluesOnly);
    const Eigen::Vector3d &eigenvalues = solver.eigenvalues();

    /// Compute condition number
    /// Adding a small constant to the denominator to avoid dividing by a very small number
    const auto condition_number = sqrt(eigenvalues.maxCoeff() / (eigenvalues.minCoeff() + 1e-7));

    std_msgs::Float64 msg;
    msg.data = condition_number;
    pub_cond_number.publish(msg);
}

/**
 * Lidar point cloud registration
 * will be called by the eskf custom observation model
 * compute point-to-plane residual here
 * @param s kf state
 * @param ekfom_data H matrix
 */
void LaserMapping::ObsModel(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) {
    PROFILE_SCOPE("ObsModel");
    ++obs_model_calls_;
    int cnt_pts = scan_down_body_->size();
    cond_jtj_valid_ = false;

    /// Computes point to plane distances
    auto R_wl = (s.rot * s.offset_R_L_I).cast<float>();
    auto t_wl = (s.rot * s.offset_T_L_I + s.pos).cast<float>();
    // The map does not change between IEKF iterations, so a point that moved less than nn_reuse_distance_
    // since its last search keeps its neighbours and plane; only its residual is recomputed.
    const bool may_reuse = searched_this_scan_ && nn_reuse_distance_ > 0;
    const float reuse_dist2 = nn_reuse_distance_ * nn_reuse_distance_;

    tbb::parallel_for(tbb::blocked_range<int>(0, cnt_pts), [&](tbb::blocked_range<int> r) {
        int searched = 0;
        for (auto i = r.begin(); i < r.end(); ++i) {
            // TODO: these non const should die
            const PointType &point_body = (*scan_down_body_)[i];

            /* transform to world frame */
            common::V3F p_body = point_body.pos();
            PointType point_world = PointType();
            point_world.pos() = R_wl * p_body + t_wl;
            point_world.intensity = point_body.intensity;
            (*scan_down_world_)[i] = point_world;

            if (ekfom_data.converge) {
                const common::V3F p_world = point_world.pos();
                if (may_reuse && (p_world - search_pos_[i]).squaredNorm() < reuse_dist2) {
                    point_selected_surf_[i] = plane_ok_[i];
                } else {
                    /** Find the closest surfaces in the map **/
                    // search straight into the per-point buffer, it keeps its capacity across scans
                    MapPointVector &points_near = nearest_points_[i];
                    ivox_->GetClosestPoint(MapPointType(point_world.x, point_world.y, point_world.z),
                                           points_near, options::NUM_MATCH_POINTS);
                    point_selected_surf_[i] = points_near.size() >= options::MIN_NUM_MATCH_POINTS;
                    if (point_selected_surf_[i]) {
                        point_selected_surf_[i] =
                            common::esti_plane(plane_coef_[i], points_near, options::ESTI_PLANE_THRESHOLD);
                    }
                    plane_ok_[i] = point_selected_surf_[i];
                    search_pos_[i] = p_world;
                    ++searched;
                }
            }

            if (point_selected_surf_[i]) {
                float pd2 = plane_coef_[i].dot(common::V4F(point_world.x, point_world.y, point_world.z, 1.0f));

                bool valid_corr = p_body.norm() > 81 * pd2 * pd2;
                if (valid_corr) {
                    point_selected_surf_[i] = true;
                    residuals_[i] = pd2;
                }
            }
        }
        nn_searches_ += searched;
    });
    if (ekfom_data.converge) {
        searched_this_scan_ = true;
    }

    effect_feat_num_ = 0;

    corr_pts_.resize(cnt_pts);
    corr_norm_.resize(cnt_pts);
    for (int i = 0; i < cnt_pts; i++) {
        if (point_selected_surf_[i]) {
            corr_norm_[effect_feat_num_] = plane_coef_[i];
            const PointType &p = (*scan_down_body_)[i];
            corr_pts_[effect_feat_num_] = common::V4F(p.x, p.y, p.z, residuals_[i]);

            effect_feat_num_++;
        }
    }
    corr_pts_.resize(effect_feat_num_);
    corr_norm_.resize(effect_feat_num_);

    if (effect_feat_num_ < 1) {
        ekfom_data.valid = false;
        ekfom_data.has_HTH = false;
        ROS_WARN_STREAM("No Effective Points!");
        return;
    }

    /*** Measurement Jacobian H (one 1x12 row per point) and measurements h, accumulated straight into
     *   H^T H and H^T h. The filter only needs the full H when it has fewer rows than states. ***/
    const bool need_full_h = effect_feat_num_ < state_ikfom::DOF;
    if (need_full_h) {
        ekfom_data.h_x = Eigen::MatrixXd::Zero(effect_feat_num_, 12);  // 23
        ekfom_data.h.resize(effect_feat_num_);
    }

    /// Rotation lidar to IMU
    const common::M3F off_R = s.offset_R_L_I.toRotationMatrix().cast<float>();
    /// Translation lidar to IMU
    const common::V3F off_t = s.offset_T_L_I.cast<float>();
    const common::M3F Rt = s.rot.toRotationMatrix().transpose().cast<float>();

    // fixed-size chunks summed in order afterwards, so the result does not depend on the thread count
    constexpr int kChunk = 256;
    const int num_chunks = (effect_feat_num_ + kChunk - 1) / kChunk;
    std::vector<Eigen::Matrix<double, 12, 12>, Eigen::aligned_allocator<Eigen::Matrix<double, 12, 12>>>
        chunk_HTH(num_chunks);
    std::vector<Eigen::Matrix<double, 12, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 12, 1>>>
        chunk_HTh(num_chunks);

    tbb::parallel_for(tbb::blocked_range<int>(0, num_chunks), [&](tbb::blocked_range<int> r) {
        for (int c = r.begin(); c < r.end(); ++c) {
            Eigen::Matrix<double, 12, 12> HTH = Eigen::Matrix<double, 12, 12>::Zero();
            Eigen::Matrix<double, 12, 1> HTh = Eigen::Matrix<double, 12, 1>::Zero();
            const int end = std::min(effect_feat_num_, (c + 1) * kChunk);
            for (int i = c * kChunk; i < end; ++i) {
                common::V3F point_this_be = corr_pts_[i].head<3>();
                common::M3F point_be_crossmat = SKEW_SYM_MATRIX(point_this_be);
                common::V3F point_this = off_R * point_this_be + off_t;
                common::M3F point_crossmat = SKEW_SYM_MATRIX(point_this);

                /*** get the normal vector of closest surface/corner ***/
                common::V3F norm_vec = corr_norm_[i].head<3>();

                /*** calculate the Measurement Jacobian matrix H ***/
                common::V3F C(Rt * norm_vec);
                common::V3F A(point_crossmat * C);

                Eigen::Matrix<double, 12, 1> J;
                if (extrinsic_est_en_) {
                    common::V3F B(point_be_crossmat * off_R.transpose() * C);
                    J << norm_vec.cast<double>(), A.cast<double>(), B.cast<double>(), C.cast<double>();
                } else {
                    J << norm_vec.cast<double>(), A.cast<double>(), Eigen::Matrix<double, 6, 1>::Zero();
                }

                /*** Measurement: distance to the closest surface/corner ***/
                const double z = -corr_pts_[i][3];

                HTH.noalias() += J * J.transpose();
                HTh.noalias() += J * z;
                if (need_full_h) {
                    ekfom_data.h_x.row(i) = J.transpose();
                    ekfom_data.h(i) = z;
                }
            }
            chunk_HTH[c] = HTH;
            chunk_HTh[c] = HTh;
        }
    });

    ekfom_data.HTH.setZero();
    ekfom_data.HTh.setZero();
    for (int c = 0; c < num_chunks; ++c) {
        ekfom_data.HTH += chunk_HTH[c];
        ekfom_data.HTh += chunk_HTh[c];
    }
    ekfom_data.dof = effect_feat_num_;
    ekfom_data.has_HTH = !need_full_h;

    /// JtJ of the rotation/position columns, published once per scan after the filter converged
    cond_jtj_ = ekfom_data.HTH.topLeftCorner<6, 6>();
    cond_jtj_valid_ = true;
}

/////////////////////////////////////  publishing ////////////////////////////////////////////////////////////

void LaserMapping::PublishPath(const ros::Publisher pub_path) {
    PROFILE_SCOPE("publish path");
    SetPosestamp(msg_body_pose_);
    msg_body_pose_.header.stamp = ros::Time().fromSec(lidar_end_time_);
    msg_body_pose_.header.frame_id = global_frame_;

    /*** if path is too large, the rvis will crash ***/
    path_.poses.push_back(msg_body_pose_);
    // the whole path is serialized on every publish, so skip it when nobody listens
    if (pub_path.getNumSubscribers() > 0) {
        pub_path.publish(path_);
    }
}

void LaserMapping::PublishKeypoints(const ros::Publisher &pubLaserCloudFull) {
    PROFILE_SCOPE("publish keypoints");
    // ROS_INFO("Internally the keypoints size is %zu", feats_down_body->size());
    if (pubLaserCloudFull.getNumSubscribers() == 0) {
        return;
    }
    sensor_msgs::PointCloud2 laserCloudmsg;
    ToRosMsg(*scan_down_world_, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time_);
    laserCloudmsg.header.frame_id = global_frame_;
    pubLaserCloudFull.publish(laserCloudmsg);
}
void LaserMapping::PublishOdometry(const ros::Publisher &pub_odom_aft_mapped) {
    PROFILE_SCOPE("publish odometry + tf");
    // The odometry message carries the estimated sensor pose, so it is labelled with lidar_frame_. The TF tree
    // keeps base_link as the parent of the lidar link: we broadcast global_frame_ -> base_link_frame_ and
    // downstream nodes look up anything else through TF.

    // lidar -> base_link can change at runtime, so look it up every scan without blocking and keep the last good one
    try {
        tf_listener_.lookupTransform(lidar_frame_, base_link_frame_, ros::Time(0), lidar_to_base_);
        has_lidar_to_base_ = true;
    } catch (const tf::TransformException &ex) {
        ROS_WARN_THROTTLE(5.0, "%s%s", has_lidar_to_base_ ? "using last known lidar->base_link, " : "", ex.what());
    }

    static tf::TransformBroadcaster br;
    const ros::Time stamp = ros::Time().fromSec(lidar_end_time_);
    odom_aft_mapped_.header.stamp = stamp;
    odom_aft_mapped_.header.frame_id = global_frame_;
    odom_aft_mapped_.child_frame_id = lidar_frame_;

    if (!lidar_odom_) {
        // odometry stopped: the base sits at the origin, the sensor pose follows from the mount
        tf::Transform base_to_lidar = tf::Transform::getIdentity();
        if (has_lidar_to_base_) {
            base_to_lidar = lidar_to_base_.inverse();
        }
        br.sendTransform(tf::StampedTransform(tf::Transform::getIdentity(), stamp, global_frame_, base_link_frame_));

        tf::poseTFToMsg(base_to_lidar, odom_aft_mapped_.pose.pose);
        odom_aft_mapped_.pose.covariance.fill(0.0);
        pub_odom_aft_mapped.publish(odom_aft_mapped_);
        return;
    }

    SetPosestamp(odom_aft_mapped_.pose);
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
    pub_odom_aft_mapped.publish(odom_aft_mapped_);

    if (!has_lidar_to_base_) {
        return;
    }
    tf::Transform odom_to_lidar;
    tf::poseMsgToTF(odom_aft_mapped_.pose.pose, odom_to_lidar);
    br.sendTransform(tf::StampedTransform(odom_to_lidar * lidar_to_base_, stamp, global_frame_, base_link_frame_));
}

void LaserMapping::PublishFrameWorld() {
    CloudPtr laserCloudWorld;
    if (dense_pub_en_) {
        CloudPtr laserCloudFullRes(scan_undistort_);
        int size = laserCloudFullRes->size();
        laserCloudWorld.reset(new PointCloudType(size));
        for (int i = 0; i < size; i++) {
            (*laserCloudWorld)[i] = PointBodyToWorld((*laserCloudFullRes)[i]);
        }
    } else {
        laserCloudWorld = scan_down_world_;
    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    ToRosMsg(*laserCloudWorld, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time_);
    laserCloudmsg.header.frame_id = global_frame_;
    pub_laser_cloud_world_.publish(laserCloudmsg);
}

void LaserMapping::PublishFrameBody(const ros::Publisher &pub_laser_cloud_body) {
    int size = scan_undistort_->size();
    CloudPtr laser_cloud_imu_body(new PointCloudType(size));

    for (int i = 0; i < size; i++) {
        PointBodyLidarToIMU(&(*scan_undistort_)[i], &(*laser_cloud_imu_body)[i]);
    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    ToRosMsg(*laser_cloud_imu_body, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time_);
    laserCloudmsg.header.frame_id = base_link_frame_;
    pub_laser_cloud_body.publish(laserCloudmsg);
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

}  // namespace faster_lio
