#pragma once

#include <atomic>
#include <memory>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/IMU.h>

#include <nlohmann/json.hpp>

class SlamPipeline {
public:
    SlamPipeline();
    ~SlamPipeline();

    bool init(const nlohmann::json &config);
    std::string getDatabasePath() const { return db_path_; }

    // Process a new lidar frame. Returns true if odometry succeeded.
    bool processCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, uint64_t timestamp_ns);

    // Feed IMU data for motion prior (called at 200Hz)
    void processIMU(float gyro_x, float gyro_y, float gyro_z,
                    float acc_x, float acc_y, float acc_z,
                    uint64_t timestamp_ns);

    // Get the current pose
    rtabmap::Transform getPose() const;

    int getMapSize() const;
    int getFrameCount() const { return frame_count_; }

    // Reconstruct accumulated map from database
    pcl::PointCloud<pcl::PointXYZI>::Ptr rebuildMap() const;

private:
    std::unique_ptr<rtabmap::Odometry> odom_;
    std::unique_ptr<rtabmap::Rtabmap> rtabmap_;
    rtabmap::Transform current_pose_;
    int frame_count_ = 0;
    std::string db_path_;
    bool use_imu_ = false;
    float max_range_ = 0;
    float max_accel_ = 0; // m/s², 0 = disabled
    std::atomic<float> current_accel_{0};
    rtabmap::IMU last_imu_;
};
