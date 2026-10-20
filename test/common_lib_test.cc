#include "common_lib.h"
#include "so3_math.h"

#include <gtest/gtest.h>
#include <Eigen/Geometry>

#include <random>
#include <vector>

namespace faster_lio {
namespace {

using common::esti_plane;
using common::V3D;

std::vector<MapPointType> PlanePoints(const Eigen::Vector3f &normal, float d, int n, std::mt19937 &rng,
                                      float noise = 0) {
    // two in-plane directions and the plane point closest to the origin
    const Eigen::Vector3f nu = normal.normalized();
    const Eigen::Vector3f u = nu.unitOrthogonal();
    const Eigen::Vector3f v = nu.cross(u);
    const Eigen::Vector3f origin = -d / normal.norm() * nu;
    std::uniform_real_distribution<float> coord(-1.0f, 1.0f);
    std::uniform_real_distribution<float> off(-noise, noise);
    std::vector<MapPointType> points;
    for (int i = 0; i < n; ++i) {
        points.emplace_back(Eigen::Vector3f(origin + coord(rng) * u + coord(rng) * v + off(rng) * nu));
    }
    return points;
}

// ---------------------------------------------------------------------------------------------------------------
// esti_plane

TEST(EstiPlane, FitsPointsOnAPlane) {
    std::mt19937 rng(1);
    const Eigen::Vector3f normals[] = {{0, 0, 1}, {1, 0, 0}, {1, 2, -3}, {-0.3f, 0.1f, 0.9f}};
    for (const auto &normal : normals) {
        for (const float d : {-5.0f, 1.0f, 20.0f}) {
            const auto points = PlanePoints(normal, d, 5, rng);
            Eigen::Vector4f plane;
            ASSERT_TRUE(esti_plane(plane, points, 0.1f)) << normal.transpose() << " d " << d;
            EXPECT_NEAR(plane.head<3>().norm(), 1.0f, 1e-5f);
            // same plane up to sign
            const Eigen::Vector4f expected = Eigen::Vector4f(normal.x(), normal.y(), normal.z(), d) / normal.norm();
            const float sign = plane.head<3>().dot(expected.head<3>()) > 0 ? 1.0f : -1.0f;
            EXPECT_TRUE((plane - sign * expected).cwiseAbs().maxCoeff() < 1e-3f)
                << "got " << plane.transpose() << " expected " << expected.transpose();
            for (const auto &p : points) {
                EXPECT_NEAR(plane.dot(Eigen::Vector4f(p.x, p.y, p.z, 1)), 0.0f, 1e-4f);
            }
        }
    }
}

TEST(EstiPlane, WorksInDouble) {
    std::mt19937 rng(2);
    const auto points = PlanePoints({0, 1, 1}, 3, 5, rng);
    Eigen::Vector4d plane;
    ASSERT_TRUE(esti_plane(plane, points, 0.1));
    for (const auto &p : points) {
        EXPECT_NEAR(plane.dot(Eigen::Vector4d(p.x, p.y, p.z, 1)), 0.0, 1e-5);
    }
}

TEST(EstiPlane, TooFewPoints) {
    Eigen::Vector4f plane;
    EXPECT_FALSE(esti_plane(plane, std::vector<MapPointType>{}, 0.1f));
    EXPECT_FALSE(esti_plane(plane, std::vector<MapPointType>{MapPointType(1, 0, 1), MapPointType(0, 1, 1)}, 0.1f));
    // three is enough
    EXPECT_TRUE(esti_plane(
        plane, std::vector<MapPointType>{MapPointType(1, 0, 1), MapPointType(0, 1, 1), MapPointType(1, 1, 1)}, 0.1f));
}

TEST(EstiPlane, PointsOffThePlaneAreRejected) {
    std::mt19937 rng(3);
    auto points = PlanePoints({0, 0, 1}, -2, 5, rng);
    Eigen::Vector4f plane;
    ASSERT_TRUE(esti_plane(plane, points, 0.1f));
    points[2].z += 0.5f;  // one outlier
    EXPECT_FALSE(esti_plane(plane, points, 0.1f));
    // the same outlier is accepted with a looser threshold
    EXPECT_TRUE(esti_plane(plane, points, 1.0f));
}

TEST(EstiPlane, NoiseWithinThresholdIsAccepted) {
    std::mt19937 rng(4);
    const auto points = PlanePoints({1, 1, 0}, 4, 5, rng, 0.01f);
    Eigen::Vector4f plane;
    EXPECT_TRUE(esti_plane(plane, points, 0.1f));
}

TEST(EstiPlane, CollinearPointsGiveSomePlaneThroughTheLine) {
    // collinear points do not determine a plane (every plane containing the line fits); the normal equations are
    // singular but LDLT still returns one of these planes, so the fit is accepted with an arbitrary normal
    std::vector<MapPointType> points;
    for (int i = 0; i < 5; ++i) points.emplace_back(1.0f + i, 2.0f + 2 * i, 3.0f - i);
    Eigen::Vector4f plane;
    ASSERT_TRUE(esti_plane(plane, points, 0.1f));
    EXPECT_NEAR(plane.head<3>().norm(), 1.0f, 1e-5f);
    for (const auto &p : points) {
        EXPECT_NEAR(plane.dot(Eigen::Vector4f(p.x, p.y, p.z, 1)), 0.0f, 1e-3f);
    }
}

TEST(EstiPlane, AllPointsAtOneSpotAreRejected) {
    const std::vector<MapPointType> points(5, MapPointType(0, 0, 0));
    Eigen::Vector4f plane;
    EXPECT_FALSE(esti_plane(plane, points, 0.1f));
}

TEST(EstiPlane, PlaneThroughTheOriginIsRejected) {
    // A n = -1 has no solution for a plane with d = 0: the fit is ill-posed, it must not report a plane
    std::mt19937 rng(5);
    const auto points = PlanePoints({0, 0, 1}, 0, 5, rng);
    Eigen::Vector4f plane;
    EXPECT_FALSE(esti_plane(plane, points, 0.1f));
}

// ---------------------------------------------------------------------------------------------------------------
// so3_math

TEST(SkewSymMatrix, IsTheCrossProduct) {
    const V3D v(1, -2, 3.5);
    const V3D u(-0.5, 4, 2);
    EXPECT_TRUE(SKEW_SYM_MATRIX(v) * u == v.cross(u));
    EXPECT_TRUE(SKEW_SYM_MATRIX(v).transpose() == -SKEW_SYM_MATRIX(v));
}

TEST(Exp, ZeroRateIsIdentity) {
    EXPECT_TRUE(Exp(V3D(0, 0, 0), 0.1) == common::M3D::Identity());
    EXPECT_TRUE(Exp(V3D(1e-9, 0, 0), 1.0) == common::M3D::Identity());
}

TEST(Exp, MatchesAngleAxis) {
    std::mt19937 rng(6);
    std::uniform_real_distribution<double> c(-3, 3);
    for (int i = 0; i < 100; ++i) {
        const V3D w(c(rng), c(rng), c(rng));
        const double dt = 0.01 * (i + 1);
        const common::M3D R = Exp(w, dt);
        const common::M3D expected = Eigen::AngleAxisd(w.norm() * dt, w.normalized()).toRotationMatrix();
        EXPECT_TRUE(R.isApprox(expected, 1e-9)) << R << "\n" << expected;
        EXPECT_TRUE((R * R.transpose()).isApprox(common::M3D::Identity(), 1e-9));
        EXPECT_NEAR(R.determinant(), 1.0, 1e-9);
    }
}

TEST(Exp, NegativeDtIsTheInverse) {
    const V3D w(0.3, -0.2, 0.5);
    EXPECT_TRUE((Exp(w, 0.4) * Exp(w, -0.4)).isApprox(common::M3D::Identity(), 1e-12));
}

// ---------------------------------------------------------------------------------------------------------------
// conversions

TEST(SetPose6d, RoundTrip) {
    const V3D a(1, 2, 3), g(4, 5, 6), v(7, 8, 9), p(-1, -2, -3);
    const common::M3D R = Eigen::AngleAxisd(0.7, V3D(1, 2, 3).normalized()).toRotationMatrix();
    const common::Pose6D pose = common::set_pose6d(0.25, a, g, v, p, R);
    EXPECT_EQ(pose.offset_time, 0.25);
    EXPECT_EQ(common::VecFromArray(pose.acc), a);
    EXPECT_EQ(common::VecFromArray(pose.gyr), g);
    EXPECT_EQ(common::VecFromArray(pose.vel), v);
    EXPECT_EQ(common::VecFromArray(pose.pos), p);
    EXPECT_EQ(common::MatFromArray(pose.rot), R);
    EXPECT_EQ(pose.rot[1], R(0, 1));  // row-major
}

TEST(FromArray, StdVector) {
    EXPECT_EQ(common::VecFromArray<double>(std::vector<double>{1, 2, 3}), V3D(1, 2, 3));
    common::M3D m;
    m << 1, 2, 3, 4, 5, 6, 7, 8, 9;
    EXPECT_EQ(common::MatFromArray<double>(std::vector<double>{1, 2, 3, 4, 5, 6, 7, 8, 9}), m);
}

}  // namespace
}  // namespace faster_lio

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
