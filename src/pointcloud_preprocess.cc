#include "pointcloud_preprocess.h"
#include "profiling.h"

#include <glog/logging.h>

#include <cstring>

namespace faster_lio {

void PointCloudPreprocess::Set(LidarType lid_type, double bld, int pfilt_num) {
    lidar_type_ = lid_type;
    blind_ = bld;
    point_filter_num_ = pfilt_num;
}

void PointCloudPreprocess::Process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudType::Ptr &pcl_out) {
    PROFILE_SCOPE("preprocess");
    switch (lidar_type_) {
        case LidarType::OUST64:
            Oust64Handler(msg);
            break;

        case LidarType::VELO32:
            VelodyneHandler(msg);
            break;

        default:
            LOG(ERROR) << "Error LiDAR Type";
            break;
    }
    // hand the result over instead of copying it; cloud_out_ is cleared at the start of every handler
    pcl_out->swap(cloud_out_);
}


namespace {
/// byte offset of a field with this name and datatype, or -1; like pcl's field mapping, a field with another
/// datatype counts as missing
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

void PointCloudPreprocess::Oust64Handler(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    using sensor_msgs::PointField;
    const int off_x = FieldOffset(*msg, "x", PointField::FLOAT32);
    const int off_y = FieldOffset(*msg, "y", PointField::FLOAT32);
    const int off_z = FieldOffset(*msg, "z", PointField::FLOAT32);
    const size_t num_points = size_t(msg->width) * msg->height;
    const bool layout_ok = !msg->is_bigendian && off_x >= 0 && off_y >= 0 && off_z >= 0 &&
                           msg->data.size() >= size_t(msg->row_step) * msg->height &&
                           size_t(msg->point_step) * msg->width <= msg->row_step;
    if (!layout_ok) {
        Oust64HandlerPcl(msg);
        return;
    }

    cloud_out_.clear();
    cloud_full_.clear();
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
        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        added_pt.curvature = off_t >= 0 ? ReadField<uint32_t>(point, off_t) / 1e6 : 0.0;  // curvature unit: ms

        cloud_out_.points.push_back(added_pt);
    }
}

void PointCloudPreprocess::Oust64HandlerPcl(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    cloud_out_.clear();
    cloud_full_.clear();
    pcl::PointCloud<ouster_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.size();
    cloud_out_.reserve(plsize);

    // When the merged cloud has no 't' field, deskewing is done upstream; treat all points as simultaneous.
    bool has_t_field = false;
    for (const auto &field : msg->fields) {
        if (field.name == "t") { has_t_field = true; break; }
    }

    for (int i = 0; i < pl_orig.points.size(); i++) {
        if (i % point_filter_num_ != 0) continue;

        double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                       pl_orig.points[i].z * pl_orig.points[i].z;

        if (range < (blind_ * blind_)) continue;

        PointType added_pt;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        added_pt.curvature = has_t_field ? pl_orig.points[i].t / 1e6 : 0.0;  // curvature unit: ms

        cloud_out_.points.push_back(added_pt);
    }
}

void PointCloudPreprocess::VelodyneHandler(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    cloud_out_.clear();
    cloud_full_.clear();

    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    cloud_out_.reserve(plsize);

    /*** These variables only works when no point timestamps given ***/
    double omega_l = 3.61;  // scan angular velocity
    std::vector<bool> is_first(num_scans_, true);
    std::vector<double> yaw_fp(num_scans_, 0.0);    // yaw of first scan point
    std::vector<float> yaw_last(num_scans_, 0.0);   // yaw of last scan point
    std::vector<float> time_last(num_scans_, 0.0);  // last offset time
    /*****************************************************************/

    if (pl_orig.points[plsize - 1].time > 0) {
        given_offset_time_ = true;
    } else {
        given_offset_time_ = false;
        double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
        double yaw_end = yaw_first;
        int layer_first = pl_orig.points[0].ring;
        for (uint i = plsize - 1; i > 0; i--) {
            if (pl_orig.points[i].ring == layer_first) {
                yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
                break;
            }
        }
    }

    for (int i = 0; i < plsize; i++) {
        PointType added_pt;

        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.curvature = pl_orig.points[i].time * time_scale_;  // curvature unit: ms

        if (!given_offset_time_) {
            int layer = pl_orig.points[i].ring;
            double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

            if (is_first[layer]) {
                yaw_fp[layer] = yaw_angle;
                is_first[layer] = false;
                added_pt.curvature = 0.0;
                yaw_last[layer] = yaw_angle;
                time_last[layer] = added_pt.curvature;
                continue;
            }

            // compute offset time
            if (yaw_angle <= yaw_fp[layer]) {
                added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l;
            } else {
                added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
            }

            if (added_pt.curvature < time_last[layer]) added_pt.curvature += 360.0 / omega_l;

            yaw_last[layer] = yaw_angle;
            time_last[layer] = added_pt.curvature;
        }

        if (i % point_filter_num_ == 0) {
            if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > (blind_ * blind_)) {
                cloud_out_.points.push_back(added_pt);
            }
        }
    }
}

}  // namespace faster_lio
