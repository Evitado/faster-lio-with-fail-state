//
// Created by xiang on 2021/9/16.
//

#ifndef FASTER_LIO_IVOX3D_H
#define FASTER_LIO_IVOX3D_H

#include <ros/console.h>
#include <algorithm>
#include <vector>

#include "ivox3d_node.hpp"
#include "voxel_table.h"

namespace faster_lio {

/// voxel hash map of points with least-recently-used voxel eviction, queried for k nearest neighbours
template <int dim, typename PointType>
class IVox {
   public:
    using KeyType = Eigen::Matrix<int, dim, 1>;
    using PtType = Eigen::Matrix<float, dim, 1>;
    using NodeType = IVoxNode<PointType, dim>;
    using PointVector = std::vector<PointType>;

    enum class NearbyType {
        CENTER,  // center only
        NEARBY6,
        NEARBY18,
        NEARBY26,
    };

    struct Options {
        float resolution_ = 0.2;                        // ivox resolution
        float inv_resolution_ = 10.0;                   // inverse resolution
        NearbyType nearby_type_ = NearbyType::NEARBY6;  // nearby range
        std::size_t capacity_ = 1000000;                // capacity
    };

    explicit IVox(Options options) : options_(options) {
        options_.inv_resolution_ = 1.0 / options_.resolution_;
        GenerateNearbyGrids();
    }

    void AddPoints(const PointVector& points_to_add);

    /// up to max_num nearest points within max_range, sorted by distance
    bool GetClosestPoint(const PointType& pt, PointVector& closest_pt, int max_num = 5, double max_range = 5.0);

    /// get number of valid grids
    size_t NumValidGrids() const;

   private:
    /// generate the nearby grids according to the given options
    void GenerateNearbyGrids();

    /// position to grid
    KeyType Pos2Grid(const PtType& pt) const;

    Options options_;
    VoxelTable<dim, NodeType> grids_;    // voxels by key, with least-recently-used eviction
    std::vector<KeyType> nearby_grids_;  // nearbys
};

template <int dim, typename PointType>
bool IVox<dim, PointType>::GetClosestPoint(const PointType& pt, PointVector& closest_pt, int max_num,
                                         double max_range) {
    closest_pt.clear();
    // Exact top-K over the nearby voxels, kept sorted by insertion. A nearby voxel is skipped when its box is
    // already farther away than the current K-th best, so it cannot contribute a closer point.
    constexpr int kMaxK = 16;
    const int k = std::min(max_num, kMaxK);
    if (k <= 0) {
        return false;
    }

    float best_dist[kMaxK];
    const PointType* best_pt[kMaxK];
    int num_found = 0;

    const PtType p = ToEigen<float, dim>(pt);
    const KeyType key = Pos2Grid(p);
    const float max_range2 = static_cast<float>(max_range * max_range);
    const float half_res = 0.5f * options_.resolution_;

    for (const KeyType& delta : nearby_grids_) {
        const KeyType dkey = key + delta;

        // squared distance from the query to this voxel's box (voxel dkey covers dkey*res +- res/2)
        float box_dist2 = 0;
        for (int d = 0; d < dim; ++d) {
            const float c = dkey[d] * options_.resolution_;
            const float excess = std::abs(p[d] - c) - half_res;
            if (excess > 0) {
                box_dist2 += excess * excess;
            }
        }
        if (box_dist2 >= max_range2 || (num_found == k && box_dist2 >= best_dist[k - 1])) {
            continue;
        }

        const int voxel = grids_.Find(dkey);
        if (voxel < 0) {
            continue;
        }

        grids_.Node(voxel).ForEachPoint([&](const PointType& cand) {
            const float d = (cand.pos() - p).squaredNorm();
            if (d >= max_range2 || (num_found == k && d >= best_dist[k - 1])) {
                return;
            }
            int pos = num_found < k ? num_found++ : k - 1;
            while (pos > 0 && best_dist[pos - 1] > d) {
                best_dist[pos] = best_dist[pos - 1];
                best_pt[pos] = best_pt[pos - 1];
                --pos;
            }
            best_dist[pos] = d;
            best_pt[pos] = &cand;
        });
    }

    for (int i = 0; i < num_found; ++i) {
        closest_pt.emplace_back(*best_pt[i]);
    }
    return num_found > 0;
}

template <int dim, typename PointType>
size_t IVox<dim, PointType>::NumValidGrids() const {
    return grids_.Size();
}

template <int dim, typename PointType>
void IVox<dim, PointType>::GenerateNearbyGrids() {
    if (options_.nearby_type_ == NearbyType::CENTER) {
        nearby_grids_.emplace_back(KeyType::Zero());
    } else if (options_.nearby_type_ == NearbyType::NEARBY6) {
        nearby_grids_ = {KeyType(0, 0, 0),  KeyType(-1, 0, 0), KeyType(1, 0, 0), KeyType(0, 1, 0),
                         KeyType(0, -1, 0), KeyType(0, 0, -1), KeyType(0, 0, 1)};
    } else if (options_.nearby_type_ == NearbyType::NEARBY18) {
        nearby_grids_ = {KeyType(0, 0, 0),  KeyType(-1, 0, 0), KeyType(1, 0, 0),   KeyType(0, 1, 0),
                         KeyType(0, -1, 0), KeyType(0, 0, -1), KeyType(0, 0, 1),   KeyType(1, 1, 0),
                         KeyType(-1, 1, 0), KeyType(1, -1, 0), KeyType(-1, -1, 0), KeyType(1, 0, 1),
                         KeyType(-1, 0, 1), KeyType(1, 0, -1), KeyType(-1, 0, -1), KeyType(0, 1, 1),
                         KeyType(0, -1, 1), KeyType(0, 1, -1), KeyType(0, -1, -1)};
    } else if (options_.nearby_type_ == NearbyType::NEARBY26) {
        nearby_grids_ = {KeyType(0, 0, 0),   KeyType(-1, 0, 0),  KeyType(1, 0, 0),   KeyType(0, 1, 0),
                         KeyType(0, -1, 0),  KeyType(0, 0, -1),  KeyType(0, 0, 1),   KeyType(1, 1, 0),
                         KeyType(-1, 1, 0),  KeyType(1, -1, 0),  KeyType(-1, -1, 0), KeyType(1, 0, 1),
                         KeyType(-1, 0, 1),  KeyType(1, 0, -1),  KeyType(-1, 0, -1), KeyType(0, 1, 1),
                         KeyType(0, -1, 1),  KeyType(0, 1, -1),  KeyType(0, -1, -1), KeyType(1, 1, 1),
                         KeyType(-1, 1, 1),  KeyType(1, -1, 1),  KeyType(1, 1, -1),  KeyType(-1, -1, 1),
                         KeyType(-1, 1, -1), KeyType(1, -1, -1), KeyType(-1, -1, -1)};
    } else {
        ROS_ERROR_STREAM("Unknown nearby_type!");
    }
}

template <int dim, typename PointType>
void IVox<dim, PointType>::AddPoints(const PointVector& points_to_add) {
    std::for_each(points_to_add.begin(), points_to_add.end(), [this](const auto& pt) {
        auto key = Pos2Grid(ToEigen<float, dim>(pt));

        const int voxel = grids_.Find(key);
        if (voxel < 0) {
            PointType center;
            center.pos() = key.template cast<float>() * options_.resolution_;

            const int added = grids_.Insert(key, NodeType(center, options_.resolution_));
            grids_.Node(added).InsertPoint(pt);

            if (grids_.Size() >= options_.capacity_) {
                grids_.EvictLeastRecent();
            }
        } else {
            grids_.Node(voxel).InsertPoint(pt);
            grids_.Touch(voxel);
        }
    });
}

template <int dim, typename PointType>
Eigen::Matrix<int, dim, 1> IVox<dim, PointType>::Pos2Grid(const IVox::PtType& pt) const {
    return (pt * options_.inv_resolution_).array().round().template cast<int>();
}

}  // namespace faster_lio

#endif
