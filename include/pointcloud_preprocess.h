#ifndef FASTER_LIO_POINTCLOUD_PROCESSING_H
#define FASTER_LIO_POINTCLOUD_PROCESSING_H

#include <sensor_msgs/PointCloud2.h>

#include "common_lib.h"

namespace faster_lio {

/**
 * point cloud preprocess
 * converts an ouster-layout PointCloud2 into the internal point type: range filter, decimation, per-point time
 */
class PointCloudPreprocess {
   public:
    void Process(const sensor_msgs::PointCloud2::ConstPtr &msg, CloudPtr &pcl_out);

    // accessors
    double &Blind() { return blind_; }
    int &PointFilterNum() { return point_filter_num_; }

   private:
    void Ouster128Handler(const sensor_msgs::PointCloud2::ConstPtr &msg);

    PointCloudType cloud_out_;

    int point_filter_num_ = 1;
    double blind_ = 0.01;
};
}  // namespace faster_lio

#endif
