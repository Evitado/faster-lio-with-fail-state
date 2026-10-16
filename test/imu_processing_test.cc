// imu_processing.hpp and use-ikfom.hpp define non-inline functions: include them in this one translation unit only
#include "imu_processing.hpp"

#include <gtest/gtest.h>
#include <Eigen/Geometry>

#include <vector>

namespace faster_lio {
namespace {

using common::M3D;
using common::V3D;
using KF = esekfom::esekf<state_ikfom, 12, input_ikfom>;

constexpr double kImuDt = 0.005;  // 200 Hz
constexpr double kScan = 0.1;     // scan duration [s]
constexpr double kT0 = 1000.0;    // start of the scan under test

const V3D kGyrBias(0.01, -0.02, 0.005);
const V3D kStaticAcc(0, 0, common::G_m_s2);

void InitFilter(KF &kf) {
    std::vector<double> epsi(23, 0.001);
    kf.init_dyn_share(
        get_f, df_dx, df_dw, [](state_ikfom &, esekfom::dyn_share_datastruct<double> &) {}, 4, epsi.data());
}

sensor_msgs::Imu::ConstPtr Imu(double t, const V3D &acc, const V3D &gyr) {
    sensor_msgs::Imu::Ptr imu(new sensor_msgs::Imu());
    imu->header.stamp = ros::Time(t);
    imu->linear_acceleration.x = acc.x();
    imu->linear_acceleration.y = acc.y();
    imu->linear_acceleration.z = acc.z();
    imu->angular_velocity.x = gyr.x();
    imu->angular_velocity.y = gyr.y();
    imu->angular_velocity.z = gyr.z();
    return imu;
}

/// scan [begin, begin + kScan] with imu samples in (begin, begin + kScan]
common::MeasureGroup Scan(double begin, const V3D &acc, const V3D &gyr, PointCloudType points = {}) {
    common::MeasureGroup meas;
    meas.lidar_bag_time_ = begin;
    meas.lidar_end_time_ = begin + kScan;
    *meas.lidar_ = std::move(points);
    for (int i = 1; i <= int(std::round(kScan / kImuDt)); ++i) {
        meas.imu_.push_back(Imu(begin + i * kImuDt, acc, gyr));
    }
    return meas;
}

PointType Pt(float x, float y, float z, float time_ms, float intensity = 0) {
    PointType p;
    p.set_pos({x, y, z});
    p.time = time_ms;
    p.intensity = intensity;
    return p;
}

/// processor and filter after the IMU init on a static IMU (acc kStaticAcc, gyro kGyrBias), which ends at
/// kT0 - kScan; then a warm-up scan up to kT0 with this acc / gyro, so the scan under test starts in steady motion
struct Fixture {
    ImuProcess imu;
    KF kf;
    CloudPtr out = std::make_shared<PointCloudType>();

    explicit Fixture(const V3D &acc = kStaticAcc, const V3D &gyr = kGyrBias) {
        InitFilter(kf);
        imu.SetAccCov(V3D(0.1, 0.1, 0.1));
        imu.SetGyrCov(V3D(0.1, 0.1, 0.1));
        common::MeasureGroup init;
        init.lidar_bag_time_ = kT0 - 2 * kScan;
        init.lidar_end_time_ = kT0 - kScan;
        for (int i = MAX_INI_COUNT + 5; i >= 0; --i) {
            init.imu_.push_back(Imu(kT0 - kScan - i * kImuDt, kStaticAcc, kGyrBias));
        }
        imu.Process(init, kf, out);
        imu.Process(Scan(kT0 - kScan, acc, gyr), kf, out);
    }
};

TEST(ImuProcess, InitEstimatesGravityAndGyroBias) {
    ImuProcess imu;
    KF kf;
    InitFilter(kf);
    imu.SetExtrinsic(V3D(0.1, 0.2, 0.3), Eigen::AngleAxisd(0.5, V3D::UnitZ()).toRotationMatrix());
    common::MeasureGroup init;
    for (int i = 0; i <= MAX_INI_COUNT + 5; ++i) {
        init.imu_.push_back(Imu(10 + i * kImuDt, V3D(0.3, -0.2, 9.7), kGyrBias));
    }
    CloudPtr out = std::make_shared<PointCloudType>(1, Pt(1, 2, 3, 0));
    imu.Process(init, kf, out);

    const state_ikfom &s = kf.get_x();
    const V3D grav(s.grav[0], s.grav[1], s.grav[2]);
    // the S2 manifold has a fixed length (9.809 m/s^2, use-ikfom.hpp), not G_m_s2 (9.81) the accelerations are
    // scaled to
    EXPECT_NEAR(grav.norm(), 9.809, 1e-9);
    EXPECT_TRUE(grav.normalized().isApprox(-V3D(0.3, -0.2, 9.7).normalized(), 1e-9));
    EXPECT_TRUE(s.bg.isApprox(kGyrBias, 1e-9));
    EXPECT_TRUE(s.offset_T_L_I.isApprox(V3D(0.1, 0.2, 0.3)));
    EXPECT_TRUE(s.offset_R_L_I.toRotationMatrix().isApprox(Eigen::AngleAxisd(0.5, V3D::UnitZ()).toRotationMatrix()));

    KF expected;
    ImuProcess::SetInitialCovariance(expected);
    EXPECT_TRUE(kf.get_P() == expected.get_P());

    // the init does not touch the output cloud
    ASSERT_EQ(out->size(), 1u);
    EXPECT_EQ(out->front().pos(), Eigen::Vector3f(1, 2, 3));
}

TEST(ImuProcess, InitNeedsEnoughSamples) {
    ImuProcess imu;
    KF kf;
    InitFilter(kf);
    common::MeasureGroup few;
    for (int i = 0; i < 5; ++i) few.imu_.push_back(Imu(10 + i * kImuDt, kStaticAcc, V3D::Zero()));
    few.lidar_bag_time_ = 10;
    few.lidar_end_time_ = 10 + kScan;
    CloudPtr out = std::make_shared<PointCloudType>();

    // still initializing: the scan's points are not undistorted into the output
    *few.lidar_ = {Pt(1, 0, 0, 50)};
    imu.Process(few, kf, out);
    EXPECT_TRUE(out->empty());
}

TEST(ImuProcess, NoImuIsANoOp) {
    Fixture f;
    const state_ikfom before = f.kf.get_x();
    common::MeasureGroup meas;
    *meas.lidar_ = {Pt(1, 2, 3, 10)};
    *f.out = {Pt(9, 9, 9, 0)};
    f.imu.Process(meas, f.kf, f.out);
    ASSERT_EQ(f.out->size(), 1u);
    EXPECT_EQ(f.out->front().pos(), Eigen::Vector3f(9, 9, 9));
    EXPECT_TRUE(f.kf.get_x().pos == before.pos);
}

TEST(ImuProcess, StaticScanIsUnchangedAndSortedByTime) {
    Fixture f;
    // out of time order, two points share a time stamp
    const PointCloudType points{Pt(1, 2, 3, 80, 1), Pt(-4, 5, 6, 10, 2), Pt(7, -8, 9, 50, 3), Pt(0, 1, 2, 10, 4),
                                Pt(3, 3, 3, 100, 5)};
    f.imu.Process(Scan(kT0, kStaticAcc, kGyrBias, points), f.kf, f.out);

    ASSERT_EQ(f.out->size(), points.size());
    const float expected_intensity[] = {2, 4, 3, 1, 5};  // stable sort by time
    for (size_t i = 0; i < f.out->size(); ++i) {
        const PointType &p = (*f.out)[i];
        EXPECT_EQ(p.intensity, expected_intensity[i]) << i;
        const PointType &in = points[int(expected_intensity[i]) - 1];
        EXPECT_EQ(p.time, in.time);
        EXPECT_TRUE(p.pos().isApprox(in.pos(), 1e-5f)) << p.pos().transpose() << " vs " << in.pos().transpose();
    }
    // nearly no motion while static: the 9.81 vs 9.809 m/s^2 gravity mismatch leaves 1 mm/s^2 of acceleration,
    // which over the 0.2 s since the init gives 0.2 mm/s and 0.02 mm
    EXPECT_LT(f.kf.get_x().pos.norm(), 1e-4);
    EXPECT_LT(f.kf.get_x().vel.norm(), 1e-3);
}

TEST(ImuProcess, ConstantYawRateIsCompensated) {
    const double w = 1.0;  // rad/s about z
    Fixture f(kStaticAcc, kGyrBias + V3D(0, 0, w));
    PointCloudType points;
    for (int i = 1; i <= 20; ++i) {
        points.push_back(Pt(10, 0.5f * i, -1, 5.0f * i));
    }
    f.imu.Process(Scan(kT0, kStaticAcc, kGyrBias + V3D(0, 0, w), points), f.kf, f.out);

    ASSERT_EQ(f.out->size(), points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        // a point taken at t is seen from the scan end, kScan - t later: rotated back by w * (kScan - t)
        const double t = points[i].time / 1000.0;
        const V3D expected = Eigen::AngleAxisd(w * (t - kScan), V3D::UnitZ()) * points[i].pos().cast<double>();
        EXPECT_TRUE(((*f.out)[i].pos().cast<double>() - expected).norm() < 1e-4)
            << "t " << t << ": " << (*f.out)[i].pos().transpose() << " vs " << expected.transpose();
    }
    // the point at the scan end stays where it is
    EXPECT_TRUE(f.out->back().pos().isApprox(points.back().pos(), 1e-6f));
}

TEST(ImuProcess, ConstantAccelerationIsCompensated) {
    const double a = 2.0;  // m/s^2 along x, no rotation
    const V3D acc = kStaticAcc + V3D(a, 0, 0);
    Fixture f(acc, kGyrBias);
    const double v0 = f.kf.get_x().vel.x();  // speed at the scan start after the warm-up
    ASSERT_GT(v0, 0.1);
    PointCloudType points;
    for (int i = 1; i <= 20; ++i) {
        points.push_back(Pt(5, -2, 0.5f * i, 5.0f * i));
    }
    f.imu.Process(Scan(kT0, acc, kGyrBias, points), f.kf, f.out);

    const auto x = [&](double t) { return v0 * t + 0.5 * a * t * t; };
    ASSERT_EQ(f.out->size(), points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        const double t = points[i].time / 1000.0;
        const V3D expected = points[i].pos().cast<double>() + V3D(x(t) - x(kScan), 0, 0);
        // the filter integrates position with Euler steps, the undistortion exactly: some mm apart
        EXPECT_TRUE(((*f.out)[i].pos().cast<double>() - expected).norm() < 2e-3)
            << "t " << t << ": " << (*f.out)[i].pos().transpose() << " vs " << expected.transpose();
    }
}

TEST(ImuProcess, ExtrinsicIsApplied) {
    // lidar mounted 1 m ahead of the IMU along x, rotating about the IMU z axis: a lidar point also moves by the
    // lever arm, so the compensation differs from the one without the offset
    const double w = 1.0;
    const V3D gyr = kGyrBias + V3D(0, 0, w);
    const V3D lever(1, 0, 0);
    const PointCloudType points{Pt(10, 0, 0, 10), Pt(10, 0, 0, 100)};

    Fixture without(kStaticAcc, gyr);
    without.imu.Process(Scan(kT0, kStaticAcc, gyr, points), without.kf, without.out);

    ImuProcess imu;
    KF kf;
    InitFilter(kf);
    imu.SetExtrinsic(lever, M3D::Identity());
    CloudPtr out = std::make_shared<PointCloudType>();
    common::MeasureGroup init;
    for (int i = MAX_INI_COUNT + 5; i >= 0; --i) init.imu_.push_back(Imu(kT0 - kScan - i * kImuDt, kStaticAcc, kGyrBias));
    imu.Process(init, kf, out);
    imu.Process(Scan(kT0 - kScan, kStaticAcc, gyr), kf, out);
    imu.Process(Scan(kT0, kStaticAcc, gyr, points), kf, out);

    ASSERT_EQ(out->size(), 2u);
    // in the IMU frame the point is at lever + p; rotate that back by w * (t - kScan) and return to the lidar frame
    const double t = 0.01;
    const V3D expected =
        Eigen::AngleAxisd(w * (t - kScan), V3D::UnitZ()) * (lever + points[0].pos().cast<double>()) - lever;
    EXPECT_TRUE((out->front().pos().cast<double>() - expected).norm() < 1e-4)
        << out->front().pos().transpose() << " vs " << expected.transpose();
    EXPECT_GT((out->front().pos() - without.out->front().pos()).norm(), 0.05f);
    EXPECT_TRUE(out->back().pos().isApprox(points.back().pos(), 1e-6f));
}

}  // namespace
}  // namespace faster_lio

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
