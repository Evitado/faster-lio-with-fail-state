#ifndef FASTER_LIO_POINT_CLOUD_H
#define FASTER_LIO_POINT_CLOUD_H

#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

namespace faster_lio {

/// scan point: position, intensity and its time since the scan start
struct PointType {
    float x = 0, y = 0, z = 0;
    float intensity = 0;
    float time = 0;  // [ms]

    Eigen::Map<Eigen::Vector3f> pos() { return Eigen::Map<Eigen::Vector3f>(&x); }
    Eigen::Map<const Eigen::Vector3f> pos() const { return Eigen::Map<const Eigen::Vector3f>(&x); }
};

/// local map point, position only
struct MapPointType {
    float x = 0, y = 0, z = 0;

    MapPointType() = default;
    MapPointType(float px, float py, float pz) : x(px), y(py), z(pz) {}

    Eigen::Map<Eigen::Vector3f> pos() { return Eigen::Map<Eigen::Vector3f>(&x); }
    Eigen::Map<const Eigen::Vector3f> pos() const { return Eigen::Map<const Eigen::Vector3f>(&x); }
};

using PointCloudType = std::vector<PointType>;
using CloudPtr = std::shared_ptr<PointCloudType>;
using MapPointVector = std::vector<MapPointType>;

/**
 * Voxel grid downsampling, same result as pcl::VoxelGrid: every occupied voxel of side `leaf` (grid anchored at the
 * cloud's bounding box) is replaced by the mean of its points, all fields averaged, output ordered by voxel index.
 */
inline void VoxelDownsample(const PointCloudType &in, float leaf, PointCloudType &out) {
    out.clear();
    if (in.empty()) {
        return;
    }
    const float inv_leaf = 1.0f / leaf;

    Eigen::Array3f min_p = Eigen::Array3f::Constant(std::numeric_limits<float>::max());
    Eigen::Array3f max_p = Eigen::Array3f::Constant(std::numeric_limits<float>::lowest());
    for (const auto &p : in) {
        if (!p.pos().allFinite()) continue;
        min_p = min_p.min(p.pos().array());
        max_p = max_p.max(p.pos().array());
    }
    const Eigen::Array3i min_b = (min_p * inv_leaf).floor().cast<int>();
    const Eigen::Array3i max_b = (max_p * inv_leaf).floor().cast<int>();
    const Eigen::Array3i div = max_b - min_b + 1;
    const uint64_t mul_y = uint64_t(div.x());
    const uint64_t mul_z = mul_y * uint64_t(div.y());

    std::vector<std::pair<uint64_t, uint32_t>> keys;  // (voxel index, point index)
    keys.reserve(in.size());
    for (uint32_t i = 0; i < in.size(); ++i) {
        const auto &p = in[i];
        if (!p.pos().allFinite()) continue;
        const Eigen::Array3i ijk = (p.pos().array() * inv_leaf).floor().cast<int>() - min_b;
        keys.emplace_back(uint64_t(ijk.x()) + uint64_t(ijk.y()) * mul_y + uint64_t(ijk.z()) * mul_z, i);
    }
    std::sort(keys.begin(), keys.end());

    for (size_t begin = 0; begin < keys.size();) {
        size_t end = begin;
        float sx = 0, sy = 0, sz = 0, si = 0, st = 0;
        for (; end < keys.size() && keys[end].first == keys[begin].first; ++end) {
            const auto &p = in[keys[end].second];
            sx += p.x;
            sy += p.y;
            sz += p.z;
            si += p.intensity;
            st += p.time;
        }
        const float n = float(end - begin);
        out.push_back(PointType{sx / n, sy / n, sz / n, si / n, st / n});
        begin = end;
    }
}

/// flat PointCloud2 with float32 x, y, z, intensity
inline void ToRosMsg(const PointCloudType &cloud, sensor_msgs::PointCloud2 &msg) {
    msg.fields.clear();
    for (const char *name : {"x", "y", "z", "intensity"}) {
        sensor_msgs::PointField field;
        field.name = name;
        field.offset = 4 * msg.fields.size();
        field.datatype = sensor_msgs::PointField::FLOAT32;
        field.count = 1;
        msg.fields.push_back(field);
    }
    msg.height = 1;
    msg.width = cloud.size();
    msg.is_bigendian = false;
    msg.is_dense = false;
    msg.point_step = 16;
    msg.row_step = msg.point_step * msg.width;
    msg.data.resize(msg.row_step);
    uint8_t *dst = msg.data.data();
    for (const auto &p : cloud) {
        std::memcpy(dst, &p.x, 16);  // x, y, z, intensity are the first four floats of PointType
        dst += 16;
    }
}

}  // namespace faster_lio

#endif  // FASTER_LIO_POINT_CLOUD_H
