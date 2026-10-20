#include "pointcloud_preprocess.h"
#include "profiling.h"

#include <ros/console.h>
#include <cmath>
#include <cstring>

namespace faster_lio {

double PointCloudPreprocess::Process(const sensor_msgs::PointCloud2::ConstPtr &msg, CloudPtr &pcl_out) {
    PROFILE_SCOPE("preprocess");
    const double scan_end = Ouster128Handler(msg);
    // hand the result over instead of copying it; cloud_out_ is cleared at the start of every handler
    pcl_out->swap(cloud_out_);
    return scan_end;
}


namespace {
/// byte offset of a 4-byte field with this name and datatype, or -1; a field with another datatype, or one that does
/// not fit inside a point, counts as missing
int FieldOffset(const sensor_msgs::PointCloud2 &msg, const char *name, uint8_t datatype) {
    for (const auto &field : msg.fields) {
        if (field.name == name && field.datatype == datatype && field.count >= 1 &&
            size_t(field.offset) + 4 <= msg.point_step) {
            return field.offset;
        }
    }
    return -1;
}

bool HasField(const sensor_msgs::PointCloud2 &msg, const char *name) {
    return std::any_of(msg.fields.begin(), msg.fields.end(), [&](const auto &field) { return field.name == name; });
}

// a point time beyond this is garbage (e.g. a missing or corrupt 't'): scans last ~0.1 s [ns]
constexpr uint32_t MAX_POINT_TIME_NS = 1'000'000'000;

template <typename T>
T ReadField(const uint8_t *point, int offset) {
    T value;
    std::memcpy(&value, point + offset, sizeof(T));
    return value;
}
}  // namespace

double PointCloudPreprocess::Ouster128Handler(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    using sensor_msgs::PointField;
    const int off_x = FieldOffset(*msg, "x", PointField::FLOAT32);
    const int off_y = FieldOffset(*msg, "y", PointField::FLOAT32);
    const int off_z = FieldOffset(*msg, "z", PointField::FLOAT32);
    const size_t num_points = size_t(msg->width) * msg->height;
    const bool layout_ok = !msg->is_bigendian && off_x >= 0 && off_y >= 0 && off_z >= 0 &&
                           msg->data.size() >= size_t(msg->row_step) * msg->height &&
                           size_t(msg->point_step) * msg->width <= msg->row_step;
    cloud_out_.clear();
    if (!layout_ok) {
        ROS_ERROR_THROTTLE(5.0, "unsupported point cloud layout: need little-endian float32 x, y, z");
        return 0.0;
    }

    const int off_intensity = FieldOffset(*msg, "intensity", PointField::FLOAT32);
    // When the merged cloud has no 't' field, deskewing is done upstream; treat all points as simultaneous.
    const int off_t = FieldOffset(*msg, "t", PointField::UINT32);
    if (off_t < 0 && HasField(*msg, "t")) {
        ROS_WARN_THROTTLE(5.0, "point cloud field 't' is not a uint32 [ns] inside the point, scans are not deskewed");
    }
    const double blind2 = blind_ * blind_;
    cloud_out_.reserve(num_points / point_filter_num_ + 1);

    // Read the kept points (every point_filter_num_-th in row-major order) straight from the message buffer. An
    // organized ouster cloud is visited column by column, so the output is roughly in time order (a destaggered
    // cloud still differs in time within a column; undistortion sorts what is not).
    const size_t width = msg->width;
    const size_t height = msg->height;
    uint32_t scan_end_ns = 0;
    size_t bad_times = 0;
    for (size_t k = 0; k < num_points; ++k) {
        const size_t row = k % height;
        const size_t col = k / height;
        const size_t i = row * width + col;
        const uint8_t *point = msg->data.data() + row * msg->row_step + col * msg->point_step;
        const float x = ReadField<float>(point, off_x);
        const float y = ReadField<float>(point, off_y);
        const float z = ReadField<float>(point, off_z);
        const double range = double(x) * x + double(y) * y + double(z) * z;
        if (!std::isfinite(range)) continue;  // NaN / inf points: no position, and their time may be garbage too

        // the scan end over every valid return, also the skipped and blind ones, so it does not depend on the filters
        const uint32_t t_ns = off_t >= 0 ? ReadField<uint32_t>(point, off_t) : 0;
        if (t_ns > MAX_POINT_TIME_NS) {
            ++bad_times;
            continue;
        }
        scan_end_ns = std::max(scan_end_ns, t_ns);
        if (i % point_filter_num_ != 0 || range < blind2) continue;

        PointType added_pt;
        added_pt.x = x;
        added_pt.y = y;
        added_pt.z = z;
        added_pt.intensity = off_intensity >= 0 ? ReadField<float>(point, off_intensity) : 0.f;
        added_pt.time = t_ns / 1e6;  // time unit: ms

        cloud_out_.push_back(added_pt);
    }
    if (bad_times > 0) {
        ROS_WARN_THROTTLE(5.0, "dropped %zu points with a time past %.1f s in the scan", bad_times,
                          MAX_POINT_TIME_NS / 1e9);
    }
    return scan_end_ns / 1e6;  // ms, like PointType::time
}

}  // namespace faster_lio
