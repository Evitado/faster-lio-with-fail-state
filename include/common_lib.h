#ifndef COMMON_LIB_H
#define COMMON_LIB_H

#include <sensor_msgs/Imu.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <boost/array.hpp>
#include <deque>

#include "faster_lio/Pose6D.h"
#include "options.h"
#include "point_cloud.h"
#include "so3_math.h"

namespace faster_lio::common {

constexpr double G_m_s2 = 9.81;  // Gravity const in GuangDong/China

template <typename S>
inline Eigen::Matrix<S, 3, 1> VecFromArray(const std::vector<double> &v) {
    return Eigen::Matrix<S, 3, 1>(v[0], v[1], v[2]);
}

template <typename S>
inline Eigen::Matrix<S, 3, 1> VecFromArray(const boost::array<S, 3> &v) {
    return Eigen::Matrix<S, 3, 1>(v[0], v[1], v[2]);
}

template <typename S>
inline Eigen::Matrix<S, 3, 3> MatFromArray(const std::vector<double> &v) {
    Eigen::Matrix<S, 3, 3> m;
    m << v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8];
    return m;
}

template <typename S>
inline Eigen::Matrix<S, 3, 3> MatFromArray(const boost::array<S, 9> &v) {
    Eigen::Matrix<S, 3, 3> m;
    m << v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8];
    return m;
}

using Pose6D = faster_lio::Pose6D;
using V3D = Eigen::Vector3d;
using M3D = Eigen::Matrix3d;
using V3F = Eigen::Vector3f;
using V4F = Eigen::Vector4f;
using M3F = Eigen::Matrix3f;

using VV4F = std::vector<V4F, Eigen::aligned_allocator<V4F>>;

const M3D Eye3d = M3D::Identity();
const V3D Zero3d(0, 0, 0);

/// sync imu and lidar measurements
struct MeasureGroup {
    MeasureGroup() { this->lidar_ = std::make_shared<PointCloudType>(); };

    double lidar_bag_time_ = 0;
    double lidar_end_time_ = 0;
    CloudPtr lidar_ = nullptr;
    std::deque<sensor_msgs::Imu::ConstPtr> imu_;
};

/**
 * set a pose 6d from ekf status
 * @tparam T
 * @param t
 * @param a
 * @param g
 * @param v
 * @param p
 * @param R
 * @return
 */
template <typename T>
Pose6D set_pose6d(const double t, const Eigen::Matrix<T, 3, 1> &a, const Eigen::Matrix<T, 3, 1> &g,
                  const Eigen::Matrix<T, 3, 1> &v, const Eigen::Matrix<T, 3, 1> &p, const Eigen::Matrix<T, 3, 3> &R) {
    Pose6D rot_kp;
    rot_kp.offset_time = t;
    for (int i = 0; i < 3; i++) {
        rot_kp.acc[i] = a(i);
        rot_kp.gyr[i] = g(i);
        rot_kp.vel[i] = v(i);
        rot_kp.pos[i] = p(i);
        for (int j = 0; j < 3; j++) rot_kp.rot[i * 3 + j] = R(i, j);
    }
    return rot_kp;
}

/// squared distance
inline float calc_dist(const Eigen::Vector3f &p1, const Eigen::Vector3f &p2) { return (p1 - p2).squaredNorm(); }

/**
 * estimate a plane
 * @tparam T
 * @param pca_result
 * @param point
 * @param threshold
 * @return
 */
template <typename T, typename PointVec>
inline bool esti_plane(Eigen::Matrix<T, 4, 1> &pca_result, const PointVec &point, const T &threshold = 0.1f) {
    if (point.size() < options::MIN_NUM_MATCH_POINTS) {
        return false;
    }

    // least squares for n in A * n = -1 via the 3x3 normal equations, in double for conditioning
    Eigen::Matrix3d AtA = Eigen::Matrix3d::Zero();
    Eigen::Vector3d Atb = Eigen::Vector3d::Zero();
    for (const auto &p : point) {
        const Eigen::Vector3d a(p.x, p.y, p.z);
        AtA.noalias() += a * a.transpose();
        Atb -= a;
    }
    const Eigen::Matrix<T, 3, 1> normvec = AtA.ldlt().solve(Atb).template cast<T>();
    if (!normvec.allFinite() || normvec.squaredNorm() == 0) {
        return false;  // degenerate (e.g. collinear) neighbourhood
    }

    T n = normvec.norm();
    pca_result(0) = normvec(0) / n;
    pca_result(1) = normvec(1) / n;
    pca_result(2) = normvec(2) / n;
    pca_result(3) = 1.0 / n;

    for (const auto &p : point) {
        if (fabs(pca_result.dot(Eigen::Matrix<T, 4, 1>(p.x, p.y, p.z, 1))) > threshold) {
            return false;
        }
    }
    return true;
}

}  // namespace faster_lio::common
#endif
