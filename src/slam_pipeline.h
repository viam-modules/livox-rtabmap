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

#include "imu_reader.h"

class SlamPipeline {
public:
    SlamPipeline();
    ~SlamPipeline();

    bool init(const nlohmann::json &config);
    std::string getDatabasePath() const { return db_path_; }

    // Process a new lidar frame. Returns true if odometry succeeded.
    // filtered_cloud is set to the range-filtered cloud for display.
    bool processCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, uint64_t timestamp_ns,
                      pcl::PointCloud<pcl::PointXYZI>::Ptr *filtered_cloud = nullptr);

    // Feed IMU data for motion prior (called at 200Hz)
    void processIMU(float gyro_x, float gyro_y, float gyro_z,
                    float acc_x, float acc_y, float acc_z,
                    uint64_t timestamp_ns);

    // Feed an IMU reading to the odometry as a motion prior.
    // Should be called before the next processCloud call.
    void processImu(const ImuReading &imu);

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

    // Live Livox IMU path (processIMU — called at 200Hz from LivoxReceiver)
    bool use_imu_ = false;
    float min_range_ = 0;
    float max_range_ = 0;
    float max_accel_ = 0; // m/s², 0 = disabled
    float accel_holdoff_ = 1.0; // seconds
    std::atomic<float> current_accel_{0};
    std::atomic<double> last_high_accel_time_{0};
    rtabmap::IMU last_imu_;

    // Viam playback IMU path (processImu — called from PcdPlayer)
    // Each Viam reading carries one measurement type at a time.
    cv::Vec3d last_gyro_{0, 0, 0};
    cv::Vec3d last_accel_{0, 0, 9.81};
    cv::Vec4d last_orientation_{0, 0, 0, 1}; // identity quaternion
    bool has_gyro_ = false;
    bool has_accel_ = false;
    bool has_orientation_ = false;
};
