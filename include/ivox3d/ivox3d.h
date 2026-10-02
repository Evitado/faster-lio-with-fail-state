//
// Created by xiang on 2021/9/16.
//

#ifndef FASTER_LIO_IVOX3D_H
#define FASTER_LIO_IVOX3D_H

#include <glog/logging.h>
#include <algorithm>
#include <list>
#include <numeric>
#include <thread>

#include "eigen_types.h"
#include "ivox3d_node.hpp"
#include "voxel_table.h"

namespace faster_lio {

enum class IVoxNodeType {
    DEFAULT,  // linear ivox
    PHC,      // phc ivox
};

/// traits for NodeType
template <IVoxNodeType node_type, typename PointT, int dim>
struct IVoxNodeTypeTraits {};

template <typename PointT, int dim>
struct IVoxNodeTypeTraits<IVoxNodeType::DEFAULT, PointT, dim> {
    using NodeType = IVoxNode<PointT, dim>;
};

template <typename PointT, int dim>
struct IVoxNodeTypeTraits<IVoxNodeType::PHC, PointT, dim> {
    using NodeType = IVoxNodePhc<PointT, dim>;
};

template <int dim = 3, IVoxNodeType node_type = IVoxNodeType::DEFAULT, typename PointType = pcl::PointXYZ>
class IVox {
   public:
    using KeyType = Eigen::Matrix<int, dim, 1>;
    using PtType = Eigen::Matrix<float, dim, 1>;
    using NodeType = typename IVoxNodeTypeTraits<node_type, PointType, dim>::NodeType;
    using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;
    using DistPoint = typename NodeType::DistPoint;

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

    /**
     * constructor
     * @param options  ivox options
     */
    explicit IVox(Options options) : options_(options) {
        options_.inv_resolution_ = 1.0 / options_.resolution_;
        GenerateNearbyGrids();
    }

    /**
     * clear eveything
     */
    inline void Reset() {
        grids_.Clear();
        nearby_grids_.clear();
    }
    /**
     * add points
     * @param points_to_add
     */
    void AddPoints(const PointVector& points_to_add);

    /// get nn
    bool GetClosestPoint(const PointType& pt, PointType& closest_pt);

    /// get nn with condition
    bool GetClosestPoint(const PointType& pt, PointVector& closest_pt, int max_num = 5, double max_range = 5.0);

    /// get nn in cloud
    bool GetClosestPoint(const PointVector& cloud, PointVector& closest_cloud);

    /// get number of points
    size_t NumPoints() const;

    /// get number of valid grids
    size_t NumValidGrids() const;

    /// get statistics of the points
    std::vector<float> StatGridPoints() const;

   private:
    /// knn via per-voxel candidate lists, used for node types without ForEachPoint
    bool GetClosestPointGeneric(const PointType& pt, PointVector& closest_pt, int max_num, double max_range);

    /// generate the nearby grids according to the given options
    void GenerateNearbyGrids();

    /// position to grid
    KeyType Pos2Grid(const PtType& pt) const;

    Options options_;
    VoxelTable<dim, NodeType> grids_;    // voxels by key, with least-recently-used eviction
    std::vector<KeyType> nearby_grids_;  // nearbys
};

template <int dim, IVoxNodeType node_type, typename PointType>
bool IVox<dim, node_type, PointType>::GetClosestPoint(const PointType& pt, PointType& closest_pt) {
    std::vector<DistPoint> candidates;
    auto key = Pos2Grid(ToEigen<float, dim>(pt));
    std::for_each(nearby_grids_.begin(), nearby_grids_.end(), [&key, &candidates, &pt, this](const KeyType& delta) {
        auto dkey = key + delta;
        const int voxel = grids_.Find(dkey);
        if (voxel >= 0) {
            DistPoint dist_point;
            bool found = grids_.Node(voxel).NNPoint(pt, dist_point);
            if (found) {
                candidates.emplace_back(dist_point);
            }
        }
    });

    if (candidates.empty()) {
        return false;
    }

    auto iter = std::min_element(candidates.begin(), candidates.end());
    closest_pt = iter->Get();
    return true;
}

template <int dim, IVoxNodeType node_type, typename PointType>
bool IVox<dim, node_type, PointType>::GetClosestPoint(const PointType& pt, PointVector& closest_pt, int max_num,
                                                      double max_range) {
    closest_pt.clear();
    if constexpr (node_type != IVoxNodeType::DEFAULT) {
        return GetClosestPointGeneric(pt, closest_pt, max_num, max_range);
    } else {
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
                const float d = (cand.getVector3fMap() - p).squaredNorm();
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
}

template <int dim, IVoxNodeType node_type, typename PointType>
bool IVox<dim, node_type, PointType>::GetClosestPointGeneric(const PointType& pt, PointVector& closest_pt,
                                                             int max_num, double max_range) {
    std::vector<DistPoint> candidates;
    candidates.reserve(max_num * nearby_grids_.size());

    auto key = Pos2Grid(ToEigen<float, dim>(pt));
    for (const KeyType& delta : nearby_grids_) {
        auto dkey = key + delta;
        const int voxel = grids_.Find(dkey);
        if (voxel >= 0) {
            grids_.Node(voxel).KNNPointByCondition(candidates, pt, max_num, max_range);
        }
    }

    if (candidates.empty()) {
        return false;
    }

    if (candidates.size() > max_num) {
        std::nth_element(candidates.begin(), candidates.begin() + max_num - 1, candidates.end());
        candidates.resize(max_num);
    }
    std::nth_element(candidates.begin(), candidates.begin(), candidates.end());

    for (auto& it : candidates) {
        closest_pt.emplace_back(it.Get());
    }
    return closest_pt.empty() == false;
}

template <int dim, IVoxNodeType node_type, typename PointType>
size_t IVox<dim, node_type, PointType>::NumValidGrids() const {
    return grids_.Size();
}

template <int dim, IVoxNodeType node_type, typename PointType>
void IVox<dim, node_type, PointType>::GenerateNearbyGrids() {
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
        LOG(ERROR) << "Unknown nearby_type!";
    }
}

template <int dim, IVoxNodeType node_type, typename PointType>
void IVox<dim, node_type, PointType>::AddPoints(const PointVector& points_to_add) {
    std::for_each(points_to_add.begin(), points_to_add.end(), [this](const auto& pt) {
        auto key = Pos2Grid(ToEigen<float, dim>(pt));

        const int voxel = grids_.Find(key);
        if (voxel < 0) {
            PointType center;
            center.getVector3fMap() = key.template cast<float>() * options_.resolution_;

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

template <int dim, IVoxNodeType node_type, typename PointType>
Eigen::Matrix<int, dim, 1> IVox<dim, node_type, PointType>::Pos2Grid(const IVox::PtType& pt) const {
    return (pt * options_.inv_resolution_).array().round().template cast<int>();
}

template <int dim, IVoxNodeType node_type, typename PointType>
std::vector<float> IVox<dim, node_type, PointType>::StatGridPoints() const {
    int num = grids_.Size(), valid_num = 0, max = 0, min = 100000000;
    int sum = 0, sum_square = 0;
    grids_.ForEachNode([&](const NodeType& node) {
        int s = node.Size();
        valid_num += s > 0;
        max = s > max ? s : max;
        min = s < min ? s : min;
        sum += s;
        sum_square += s * s;
    });
    float ave = float(sum) / num;
    float stddev = num > 1 ? sqrt((float(sum_square) - num * ave * ave) / (num - 1)) : 0;
    return std::vector<float>{valid_num, ave, max, min, stddev};
}

}  // namespace faster_lio

#endif
