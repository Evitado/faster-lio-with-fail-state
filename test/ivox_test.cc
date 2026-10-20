#include "ivox3d/ivox3d.h"
#include "point_cloud.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <set>
#include <tuple>

namespace faster_lio {
namespace {

using IVoxType = IVox<3, MapPointType>;
using NearbyType = IVoxType::NearbyType;

IVoxType::Options MakeOptions(float resolution, NearbyType nearby = NearbyType::NEARBY6,
                              size_t capacity = 1000000) {
    IVoxType::Options options;
    options.resolution_ = resolution;
    options.nearby_type_ = nearby;
    options.capacity_ = capacity;
    return options;
}

MapPointType P(float x, float y, float z) { return MapPointType(x, y, z); }

/// voxel key the way IVox computes it (multiplying by the float inverse resolution, for identical rounding)
std::tuple<int, int, int> VoxelOf(const MapPointType &p, float resolution) {
    const float inv = 1.0 / resolution;
    return {int(std::round(p.x * inv)), int(std::round(p.y * inv)), int(std::round(p.z * inv))};
}

/// offsets of the voxels searched around the query's voxel: all with |dx| + |dy| + |dz| <= reach (CENTER 0,
/// NEARBY6 1, NEARBY18 2, NEARBY26 3)
std::set<std::tuple<int, int, int>> NearbyOffsets(NearbyType type) {
    const int reach = type == NearbyType::CENTER ? 0 : type == NearbyType::NEARBY6 ? 1 : type == NearbyType::NEARBY18 ? 2 : 3;
    std::set<std::tuple<int, int, int>> offsets;
    for (int x = -1; x <= 1; ++x) {
        for (int y = -1; y <= 1; ++y) {
            for (int z = -1; z <= 1; ++z) {
                if (std::abs(x) + std::abs(y) + std::abs(z) <= reach) offsets.insert({x, y, z});
            }
        }
    }
    return offsets;
}

/// all points in the nearby voxels within max_range, the k nearest sorted by distance
MapPointVector ReferenceClosest(const MapPointVector &cloud, const MapPointType &q, float resolution, NearbyType type,
                                int k, double max_range) {
    const auto [qx, qy, qz] = VoxelOf(q, resolution);
    const auto offsets = NearbyOffsets(type);
    std::vector<std::pair<float, MapPointType>> candidates;
    for (const auto &p : cloud) {
        const auto [px, py, pz] = VoxelOf(p, resolution);
        if (!offsets.count({px - qx, py - qy, pz - qz})) continue;
        const float d = (p.pos() - q.pos()).squaredNorm();
        if (d >= float(max_range * max_range)) continue;
        candidates.emplace_back(d, p);
    }
    std::stable_sort(candidates.begin(), candidates.end(),
                     [](const auto &a, const auto &b) { return a.first < b.first; });
    MapPointVector out;
    for (int i = 0; i < std::min<int>(k, candidates.size()); ++i) {
        out.push_back(candidates[i].second);
    }
    return out;
}

float Dist2(const MapPointType &a, const MapPointType &b) { return (a.pos() - b.pos()).squaredNorm(); }

TEST(IVox, EmptyMapFindsNothing) {
    IVoxType ivox(MakeOptions(0.5f));
    MapPointVector out{P(1, 1, 1)};
    EXPECT_FALSE(ivox.GetClosestPoint(P(0, 0, 0), out));
    EXPECT_TRUE(out.empty());
    EXPECT_EQ(ivox.NumValidGrids(), 0u);
}

TEST(IVox, VoxelsAreCenteredOnMultiplesOfResolution) {
    IVoxType ivox(MakeOptions(1.0f));
    ivox.AddPoints({P(0.49f, 0, 0), P(-0.49f, 0, 0)});
    EXPECT_EQ(ivox.NumValidGrids(), 1u);
    ivox.AddPoints({P(0.51f, 0, 0)});
    EXPECT_EQ(ivox.NumValidGrids(), 2u);
    ivox.AddPoints({P(0, 0, -0.51f), P(0, 1.2f, 0)});
    EXPECT_EQ(ivox.NumValidGrids(), 4u);
}

TEST(IVox, SkipsNonFinitePoints) {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    IVoxType ivox(MakeOptions(0.5f));
    ivox.AddPoints({P(nan, 0, 0), P(0, inf, 0), P(0, 0, -inf), P(nan, nan, nan)});
    EXPECT_EQ(ivox.NumValidGrids(), 0u);

    ivox.AddPoints({P(0.1f, 0, 0), P(nan, 0, 0)});
    EXPECT_EQ(ivox.NumValidGrids(), 1u);
    MapPointVector out;
    ASSERT_TRUE(ivox.GetClosestPoint(P(0, 0, 0), out, 5));
    ASSERT_EQ(out.size(), 1u);
    EXPECT_TRUE(out[0].pos().allFinite());
    EXPECT_EQ(out[0].pos(), Eigen::Vector3f(0.1f, 0, 0));
}

TEST(IVox, NonPositiveMaxNumFindsNothing) {
    IVoxType ivox(MakeOptions(0.5f));
    ivox.AddPoints({P(0, 0, 0)});
    MapPointVector out{P(1, 1, 1)};
    EXPECT_FALSE(ivox.GetClosestPoint(P(0, 0, 0), out, 0));
    EXPECT_TRUE(out.empty());
    EXPECT_FALSE(ivox.GetClosestPoint(P(0, 0, 0), out, -3));
    EXPECT_TRUE(out.empty());
}

TEST(IVox, MaxNumIsClampedTo16) {
    IVoxType ivox(MakeOptions(10.0f, NearbyType::CENTER));
    MapPointVector cloud;
    for (int i = 0; i < 40; ++i) cloud.push_back(P(0.1f * i, 0, 0));
    ivox.AddPoints(cloud);
    MapPointVector out;
    ASSERT_TRUE(ivox.GetClosestPoint(P(0, 0, 0), out, 100, 100.0));
    ASSERT_EQ(out.size(), 16u);
    for (int i = 0; i < 16; ++i) {
        EXPECT_FLOAT_EQ(out[i].x, 0.1f * i);
    }
}

TEST(IVox, ResultsSortedByDistance) {
    IVoxType ivox(MakeOptions(10.0f, NearbyType::CENTER));
    ivox.AddPoints({P(3, 0, 0), P(0, -1, 0), P(0, 0, 2), P(0.5f, 0, 0), P(-4, 0, 0)});
    MapPointVector out;
    ASSERT_TRUE(ivox.GetClosestPoint(P(0, 0, 0), out, 4));
    ASSERT_EQ(out.size(), 4u);
    EXPECT_EQ(out[0].pos(), Eigen::Vector3f(0.5f, 0, 0));
    EXPECT_EQ(out[1].pos(), Eigen::Vector3f(0, -1, 0));
    EXPECT_EQ(out[2].pos(), Eigen::Vector3f(0, 0, 2));
    EXPECT_EQ(out[3].pos(), Eigen::Vector3f(3, 0, 0));
}

TEST(IVox, MaxRangeIsExclusive) {
    IVoxType ivox(MakeOptions(10.0f, NearbyType::CENTER));
    ivox.AddPoints({P(1, 0, 0), P(2, 0, 0), P(3, 0, 0)});
    MapPointVector out;
    ASSERT_TRUE(ivox.GetClosestPoint(P(0, 0, 0), out, 5, 2.0));
    ASSERT_EQ(out.size(), 1u);  // the point at exactly 2 m is out of range
    EXPECT_EQ(out[0].pos(), Eigen::Vector3f(1, 0, 0));
    EXPECT_FALSE(ivox.GetClosestPoint(P(0, 0, 0), out, 5, 0.5));
    EXPECT_TRUE(out.empty());
}

TEST(IVox, SearchesOnlyTheNearbyVoxels) {
    // query in voxel (0, 0, 0); a point in the face neighbour (1, 0, 0) and one in the edge neighbour (1, 1, 0)
    const MapPointVector cloud{P(1.0f, 0, 0), P(1.0f, 1.0f, 0)};
    for (const auto &[type, expected] : {std::pair{NearbyType::CENTER, 0u}, std::pair{NearbyType::NEARBY6, 1u},
                                         std::pair{NearbyType::NEARBY18, 2u}, std::pair{NearbyType::NEARBY26, 2u}}) {
        IVoxType ivox(MakeOptions(1.0f, type));
        ivox.AddPoints(cloud);
        MapPointVector out;
        ivox.GetClosestPoint(P(0.3f, 0.3f, 0), out, 5, 10.0);
        EXPECT_EQ(out.size(), expected) << "nearby type " << int(type);
    }
}

TEST(IVox, MatchesBruteForceOnRandomClouds) {
    std::mt19937 rng(3);
    std::uniform_real_distribution<float> coord(-3.0f, 3.0f);
    for (const NearbyType type : {NearbyType::CENTER, NearbyType::NEARBY6, NearbyType::NEARBY18, NearbyType::NEARBY26}) {
        for (const float resolution : {0.2f, 0.5f, 1.0f}) {
            MapPointVector cloud;
            for (int i = 0; i < 4000; ++i) cloud.push_back(P(coord(rng), coord(rng), coord(rng)));
            IVoxType ivox(MakeOptions(resolution, type));
            // added in batches like the mapping does
            for (size_t i = 0; i < cloud.size(); i += 500) {
                ivox.AddPoints(MapPointVector(cloud.begin() + i, cloud.begin() + std::min(cloud.size(), i + 500)));
            }
            for (int query = 0; query < 300; ++query) {
                const MapPointType q = P(coord(rng), coord(rng), coord(rng));
                for (const auto &[k, max_range] : {std::pair{5, 5.0}, std::pair{1, 5.0}, std::pair{16, 0.3}}) {
                    MapPointVector out;
                    const bool found = ivox.GetClosestPoint(q, out, k, max_range);
                    const MapPointVector expected = ReferenceClosest(cloud, q, resolution, type, k, max_range);
                    ASSERT_EQ(found, !expected.empty());
                    ASSERT_EQ(out.size(), expected.size())
                        << "type " << int(type) << " resolution " << resolution << " k " << k;
                    for (size_t i = 0; i < out.size(); ++i) {
                        // equal distances may come in another order: compare the distances
                        ASSERT_FLOAT_EQ(Dist2(out[i], q), Dist2(expected[i], q)) << "neighbour " << i;
                    }
                }
            }
        }
    }
}

TEST(IVox, CapacityEvictsTheOldestVoxel) {
    IVoxType ivox(MakeOptions(1.0f, NearbyType::CENTER, 4));
    ivox.AddPoints({P(0, 0, 0), P(10, 0, 0), P(20, 0, 0)});
    EXPECT_EQ(ivox.NumValidGrids(), 3u);
    ivox.AddPoints({P(30, 0, 0)});  // the 4th voxel reaches capacity: the oldest one, at 0, goes
    EXPECT_EQ(ivox.NumValidGrids(), 3u);

    MapPointVector out;
    EXPECT_FALSE(ivox.GetClosestPoint(P(0, 0, 0), out));
    EXPECT_TRUE(ivox.GetClosestPoint(P(10, 0, 0), out));

    // adding to the voxel at 10 makes it the most recent: the voxel at 20 is evicted next
    ivox.AddPoints({P(10.1f, 0, 0)});
    ivox.AddPoints({P(40, 0, 0)});
    EXPECT_EQ(ivox.NumValidGrids(), 3u);
    EXPECT_FALSE(ivox.GetClosestPoint(P(20, 0, 0), out));
    ASSERT_TRUE(ivox.GetClosestPoint(P(10, 0, 0), out));
    EXPECT_EQ(out.size(), 2u);
    EXPECT_TRUE(ivox.GetClosestPoint(P(30, 0, 0), out));
    EXPECT_TRUE(ivox.GetClosestPoint(P(40, 0, 0), out));
}

}  // namespace
}  // namespace faster_lio

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
