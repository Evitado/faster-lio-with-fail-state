#ifndef FASTER_LIO_POINTCLOUD_PROCESSING_H
#define FASTER_LIO_POINTCLOUD_PROCESSING_H

#include <sensor_msgs/PointCloud2.h>
#include <algorithm>
#include "point_cloud.h"
#include "common_lib.h"

namespace faster_lio {

/**
 * point cloud preprocess
 * converts an ouster-layout PointCloud2 into the internal point type: range filter, decimation, per-point time
 */
class PointCloudPreprocess {
   public:
    /// blind: points closer than this [m] are dropped; point_filter_num: keep every n-th point (values < 1 keep all)
    PointCloudPreprocess(double blind, int point_filter_num)
        : point_filter_num_(std::max(point_filter_num, 1)), blind_(blind) {}

    /// returns the scan end [ms after the header stamp]: the latest per-point time of the whole message, NaN and
    /// blind points included, so it does not depend on which points survive the filters; 0 without a 't' field
    double Process(const sensor_msgs::PointCloud2::ConstPtr &msg, CloudPtr &pcl_out);

   private:
    double Ouster128Handler(const sensor_msgs::PointCloud2::ConstPtr &msg);

    PointCloudType cloud_out_;

    const int point_filter_num_;
    const double blind_;
};
}  // namespace faster_lio

#endif
