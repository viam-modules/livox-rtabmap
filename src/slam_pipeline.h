#pragma once

#include <memory>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/SensorData.h>

#include <nlohmann/json.hpp>

class SlamPipeline {
public:
    SlamPipeline();
    ~SlamPipeline();

    bool init(const nlohmann::json &config);
    std::string getDatabasePath() const { return db_path_; }

    // Process a new lidar frame. Returns true if odometry succeeded.
    bool processCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, uint64_t timestamp_ns);

    // Get the current pose
    rtabmap::Transform getPose() const;

    int getMapSize() const;
    int getFrameCount() const { return frame_count_; }

private:
    std::unique_ptr<rtabmap::Odometry> odom_;
    std::unique_ptr<rtabmap::Rtabmap> rtabmap_;
    rtabmap::Transform current_pose_;
    int frame_count_ = 0;
    std::string db_path_;
};
