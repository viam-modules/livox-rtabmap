#pragma once

#include <memory>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/OdometryICP.h>
#include <rtabmap/core/SensorData.h>

class SlamPipeline {
public:
    SlamPipeline();
    ~SlamPipeline();

    bool init();

    // Process a new lidar frame. Returns true if odometry succeeded.
    bool processCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, uint64_t timestamp_ns);

    // Get the current accumulated map
    pcl::PointCloud<pcl::PointXYZI>::Ptr getMap() const;

    // Get the current pose
    rtabmap::Transform getPose() const;

    int getMapSize() const;
    int getFrameCount() const { return frame_count_; }

private:
    std::unique_ptr<rtabmap::OdometryICP> odom_;
    std::unique_ptr<rtabmap::Rtabmap> rtabmap_;
    rtabmap::Transform current_pose_;
    int frame_count_ = 0;
};
