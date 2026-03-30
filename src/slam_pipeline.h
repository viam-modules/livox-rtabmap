#pragma once

#include <memory>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/IMU.h>

#include <nlohmann/json.hpp>

#include "imu_reader.h"

class SlamPipeline {
public:
    SlamPipeline();
    ~SlamPipeline();

    bool init(const nlohmann::json &config);
    std::string getDatabasePath() const { return db_path_; }

    // Process a new lidar frame. Returns true if odometry succeeded.
    bool processCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, uint64_t timestamp_ns);

    // Feed an IMU reading to the odometry as a motion prior.
    // Should be called before the next processCloud call.
    void processImu(const ImuReading &imu);

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

    // Last known IMU state — updated incrementally since each Viam reading
    // only carries one measurement type at a time.
    cv::Vec3d last_gyro_{0, 0, 0};
    cv::Vec3d last_accel_{0, 0, 9.81};
    cv::Vec4d last_orientation_{0, 0, 0, 1}; // identity quaternion
    bool has_gyro_ = false;
    bool has_accel_ = false;
    bool has_orientation_ = false;
};
