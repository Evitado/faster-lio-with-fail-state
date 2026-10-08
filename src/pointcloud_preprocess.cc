#include "pointcloud_preprocess.h"
#include "profiling.h"

#include <ros/console.h>
#include <cstring>

namespace faster_lio {

void PointCloudPreprocess::Process(const sensor_msgs::PointCloud2::ConstPtr &msg, CloudPtr &pcl_out) {
    PROFILE_SCOPE("preprocess");
    Ouster128Handler(msg);
    // hand the result over instead of copying it; cloud_out_ is cleared at the start of every handler
    pcl_out->swap(cloud_out_);
}


namespace {
/// byte offset of a field with this name and datatype, or -1; a field with another datatype counts as missing
int FieldOffset(const sensor_msgs::PointCloud2 &msg, const char *name, uint8_t datatype) {
    for (const auto &field : msg.fields) {
        if (field.name == name && field.datatype == datatype && field.count >= 1) {
            return field.offset;
        }
    }
    return -1;
}

template <typename T>
T ReadField(const uint8_t *point, int offset) {
    T value;
    std::memcpy(&value, point + offset, sizeof(T));
    return value;
}
}  // namespace

void PointCloudPreprocess::Ouster128Handler(const sensor_msgs::PointCloud2::ConstPtr &msg) {
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
        return;
    }

    const int off_intensity = FieldOffset(*msg, "intensity", PointField::FLOAT32);
    // When the merged cloud has no 't' field, deskewing is done upstream; treat all points as simultaneous.
    const int off_t = FieldOffset(*msg, "t", PointField::UINT32);
    const double blind2 = blind_ * blind_;
    cloud_out_.reserve(num_points / point_filter_num_ + 1);

    // Read the kept points (every point_filter_num_-th in row-major order) straight from the message buffer. An
    // organized ouster cloud is visited column by column: all points of a column share one time stamp, so the
    // output is already in time order and undistortion does not have to sort it.
    const size_t width = msg->width;
    const size_t height = msg->height;
    for (size_t k = 0; k < num_points; ++k) {
        const size_t row = k % height;
        const size_t col = k / height;
        const size_t i = row * width + col;
        if (i % point_filter_num_ != 0) continue;
        const uint8_t *point = msg->data.data() + row * msg->row_step + col * msg->point_step;
        const float x = ReadField<float>(point, off_x);
        const float y = ReadField<float>(point, off_y);
        const float z = ReadField<float>(point, off_z);

        double range = x * x + y * y + z * z;
        if (range < blind2) continue;

        PointType added_pt;
        added_pt.x = x;
        added_pt.y = y;
        added_pt.z = z;
        added_pt.intensity = off_intensity >= 0 ? ReadField<float>(point, off_intensity) : 0.f;
        added_pt.time = off_t >= 0 ? ReadField<uint32_t>(point, off_t) / 1e6 : 0.0;  // time unit: ms

        cloud_out_.push_back(added_pt);
    }
}

}  // namespace faster_lio
