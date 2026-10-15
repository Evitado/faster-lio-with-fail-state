#include "point_cloud.h"

#include <gtest/gtest.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <random>
#include <tuple>

namespace faster_lio {
namespace {

PointType MakePoint(float x, float y, float z, float intensity = 0, float time = 0) {
    PointType p;
    p.x = x;
    p.y = y;
    p.z = z;
    p.intensity = intensity;
    p.time = time;
    return p;
}

void ExpectPointNear(const PointType &actual, const PointType &expected, float tol = 1e-5f) {
    EXPECT_NEAR(actual.x, expected.x, tol);
    EXPECT_NEAR(actual.y, expected.y, tol);
    EXPECT_NEAR(actual.z, expected.z, tol);
    EXPECT_NEAR(actual.intensity, expected.intensity, tol);
    EXPECT_NEAR(actual.time, expected.time, tol);
}

/// straightforward voxel grid: global grid with boundaries at multiples of leaf, mean of every field per voxel,
/// voxels ordered z, then y, then x (the order of pcl::VoxelGrid's linear voxel index)
PointCloudType ReferenceDownsample(const PointCloudType &in, float leaf) {
    std::map<std::tuple<int, int, int>, std::array<double, 6>> voxels;  // (k, j, i) -> sums of x, y, z, i, t, count
    for (const auto &p : in) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        const auto key = std::make_tuple(int(std::floor(p.z / leaf)), int(std::floor(p.y / leaf)),
                                         int(std::floor(p.x / leaf)));
        auto &sum = voxels[key];
        sum[0] += p.x;
        sum[1] += p.y;
        sum[2] += p.z;
        sum[3] += p.intensity;
        sum[4] += p.time;
        sum[5] += 1;
    }
    PointCloudType out;
    for (const auto &[key, sum] : voxels) {
        out.push_back(MakePoint(sum[0] / sum[5], sum[1] / sum[5], sum[2] / sum[5], sum[3] / sum[5], sum[4] / sum[5]));
    }
    return out;
}

// ---------------------------------------------------------------------------------------------------------------
// PointType / MapPointType

TEST(PointType, DefaultsToZero) {
    const PointType p;
    ExpectPointNear(p, MakePoint(0, 0, 0, 0, 0), 0);
}

TEST(PointType, SetPosChangesOnlyPosition) {
    PointType p = MakePoint(1, 2, 3, 40, 5);
    p.set_pos(Eigen::Vector3f(-1.5f, 0.25f, 7));
    ExpectPointNear(p, MakePoint(-1.5f, 0.25f, 7, 40, 5), 0);
    EXPECT_EQ(p.pos(), Eigen::Vector3f(-1.5f, 0.25f, 7));
}

TEST(PointType, PosIsACopy) {
    const PointType p = MakePoint(1, 2, 3);
    Eigen::Vector3f v = p.pos();
    v.x() = 100;
    EXPECT_EQ(p.x, 1);
}

TEST(MapPointType, ConstructorsAgree) {
    const MapPointType from_floats(1, -2, 3.5f);
    const MapPointType from_vector(Eigen::Vector3f(1, -2, 3.5f));
    EXPECT_EQ(from_floats.pos(), Eigen::Vector3f(1, -2, 3.5f));
    EXPECT_EQ(from_vector.pos(), from_floats.pos());
}

// ---------------------------------------------------------------------------------------------------------------
// VoxelDownsample

TEST(VoxelDownsample, EmptyInputClearsOutput) {
    PointCloudType out{MakePoint(1, 1, 1)};
    VoxelDownsample({}, 0.5f, out);
    EXPECT_TRUE(out.empty());
}

TEST(VoxelDownsample, NonPositiveOrNanLeafKeepsCloud) {
    const PointCloudType in{MakePoint(0, 0, 0, 1, 2), MakePoint(0.01f, 0, 0, 3, 4)};
    for (const float leaf : {0.0f, -1.0f, std::numeric_limits<float>::quiet_NaN()}) {
        PointCloudType out{MakePoint(9, 9, 9)};
        VoxelDownsample(in, leaf, out);
        ASSERT_EQ(out.size(), in.size()) << "leaf " << leaf;
        for (size_t i = 0; i < in.size(); ++i) {
            ExpectPointNear(out[i], in[i], 0);
        }
    }
}

TEST(VoxelDownsample, SingleVoxelAveragesEveryField) {
    const PointCloudType in{MakePoint(0.1f, 0.2f, 0.3f, 10, 1), MakePoint(0.3f, 0.4f, 0.5f, 20, 3),
                            MakePoint(0.2f, 0.0f, 0.1f, 30, 5)};
    PointCloudType out;
    VoxelDownsample(in, 1.0f, out);
    ASSERT_EQ(out.size(), 1u);
    ExpectPointNear(out[0], MakePoint(0.2f, 0.2f, 0.3f, 20, 3));
}

TEST(VoxelDownsample, VoxelBoundariesAtMultiplesOfLeaf) {
    PointCloudType out;
    // 0.2 apart but on both sides of x = 1: two voxels
    VoxelDownsample({MakePoint(0.9f, 0, 0), MakePoint(1.1f, 0, 0)}, 1.0f, out);
    EXPECT_EQ(out.size(), 2u);
    // 0.8 apart inside [0, 1): one voxel
    VoxelDownsample({MakePoint(0.1f, 0, 0), MakePoint(0.9f, 0, 0)}, 1.0f, out);
    EXPECT_EQ(out.size(), 1u);
    // across zero: [-1, 0) and [0, 1)
    VoxelDownsample({MakePoint(-0.1f, 0, 0), MakePoint(0.1f, 0, 0)}, 1.0f, out);
    EXPECT_EQ(out.size(), 2u);
}

TEST(VoxelDownsample, OutputOrderedZThenYThenX) {
    // one point per voxel, given in an order unrelated to the voxel index
    const PointCloudType in{MakePoint(0.5f, 0.5f, 1.5f), MakePoint(1.5f, 0.5f, 0.5f), MakePoint(0.5f, 1.5f, 0.5f),
                            MakePoint(0.5f, 0.5f, 0.5f)};
    PointCloudType out;
    VoxelDownsample(in, 1.0f, out);
    ASSERT_EQ(out.size(), 4u);
    ExpectPointNear(out[0], MakePoint(0.5f, 0.5f, 0.5f));
    ExpectPointNear(out[1], MakePoint(1.5f, 0.5f, 0.5f));
    ExpectPointNear(out[2], MakePoint(0.5f, 1.5f, 0.5f));
    ExpectPointNear(out[3], MakePoint(0.5f, 0.5f, 1.5f));
}

TEST(VoxelDownsample, SkipsNonFinitePoints) {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    const PointCloudType in{MakePoint(nan, 0, 0), MakePoint(0.2f, 0.2f, 0.2f, 4), MakePoint(0, inf, 0),
                            MakePoint(0.4f, 0.4f, 0.4f, 8), MakePoint(0, 0, -inf)};
    PointCloudType out;
    VoxelDownsample(in, 1.0f, out);
    ASSERT_EQ(out.size(), 1u);
    ExpectPointNear(out[0], MakePoint(0.3f, 0.3f, 0.3f, 6));
}

TEST(VoxelDownsample, OnlyNonFinitePointsGivesEmptyOutput) {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    PointCloudType out{MakePoint(1, 1, 1)};
    VoxelDownsample({MakePoint(nan, nan, nan), MakePoint(std::numeric_limits<float>::infinity(), 0, 0)}, 1.0f, out);
    EXPECT_TRUE(out.empty());
}

TEST(VoxelDownsample, MatchesReferenceOnRandomClouds) {
    std::mt19937 rng(42);
    std::uniform_real_distribution<float> coord(-20.0f, 20.0f);
    std::uniform_real_distribution<float> unit(0.0f, 100.0f);
    for (const float leaf : {0.1f, 0.5f, 1.0f, 3.0f}) {
        PointCloudType in;
        for (int i = 0; i < 5000; ++i) {
            in.push_back(MakePoint(coord(rng), coord(rng), coord(rng) * 0.2f, unit(rng), unit(rng)));
        }
        PointCloudType out;
        VoxelDownsample(in, leaf, out);
        const PointCloudType expected = ReferenceDownsample(in, leaf);
        ASSERT_EQ(out.size(), expected.size()) << "leaf " << leaf;
        for (size_t i = 0; i < out.size(); ++i) {
            ExpectPointNear(out[i], expected[i], 1e-3f);
        }
    }
}

// ---------------------------------------------------------------------------------------------------------------
// ToRosMsg

TEST(ToRosMsg, FieldLayout) {
    sensor_msgs::PointCloud2 msg;
    ToRosMsg(PointCloudType(3), msg);

    ASSERT_EQ(msg.fields.size(), 4u);
    const char *names[] = {"x", "y", "z", "intensity"};
    for (size_t i = 0; i < 4; ++i) {
        EXPECT_EQ(msg.fields[i].name, names[i]);
        EXPECT_EQ(msg.fields[i].offset, 4 * i);
        EXPECT_EQ(msg.fields[i].datatype, sensor_msgs::PointField::FLOAT32);
        EXPECT_EQ(msg.fields[i].count, 1u);
    }
    EXPECT_EQ(msg.height, 1u);
    EXPECT_EQ(msg.width, 3u);
    EXPECT_EQ(msg.point_step, 16u);
    EXPECT_EQ(msg.row_step, 48u);
    EXPECT_EQ(msg.data.size(), 48u);
    EXPECT_FALSE(msg.is_bigendian);
}

TEST(ToRosMsg, ValuesReadBack) {
    const PointCloudType cloud{MakePoint(1, 2, 3, 4, 99), MakePoint(-5.5f, 0, 1e6f, 0.25f, 99)};
    sensor_msgs::PointCloud2 msg;
    ToRosMsg(cloud, msg);

    sensor_msgs::PointCloud2ConstIterator<float> x(msg, "x"), y(msg, "y"), z(msg, "z"), intensity(msg, "intensity");
    for (const auto &p : cloud) {
        EXPECT_EQ(*x, p.x);
        EXPECT_EQ(*y, p.y);
        EXPECT_EQ(*z, p.z);
        EXPECT_EQ(*intensity, p.intensity);
        ++x, ++y, ++z, ++intensity;
    }
    EXPECT_FALSE(x != x.end());
}

TEST(ToRosMsg, EmptyCloud) {
    sensor_msgs::PointCloud2 msg;
    ToRosMsg({}, msg);
    EXPECT_EQ(msg.fields.size(), 4u);
    EXPECT_EQ(msg.width, 0u);
    EXPECT_EQ(msg.row_step, 0u);
    EXPECT_TRUE(msg.data.empty());
}

TEST(ToRosMsg, ReusedMessageIsOverwritten) {
    sensor_msgs::PointCloud2 msg;
    ToRosMsg(PointCloudType(10), msg);
    msg.header.frame_id = "kept";
    ToRosMsg({MakePoint(7, 8, 9, 1)}, msg);

    EXPECT_EQ(msg.fields.size(), 4u);
    EXPECT_EQ(msg.width, 1u);
    EXPECT_EQ(msg.data.size(), 16u);
    EXPECT_EQ(msg.header.frame_id, "kept");  // the header is the caller's
    sensor_msgs::PointCloud2ConstIterator<float> z(msg, "z");
    EXPECT_EQ(*z, 9);
}

}  // namespace
}  // namespace faster_lio

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
