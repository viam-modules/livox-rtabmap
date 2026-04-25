#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <opencv2/core.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <rtabmap/core/Transform.h>

#include <viam/sdk/common/instance.hpp>
#include <viam/sdk/components/base.hpp>
#include <viam/sdk/components/camera.hpp>
#include <viam/sdk/components/movement_sensor.hpp>
#include <viam/sdk/robot/client.hpp>

#include "livox_receiver.h"  // FrameCallback, IMUCallback, LivoxIMU

struct RGBDFrame {
    cv::Mat rgb;    // CV_8UC3 BGR, may be empty
    cv::Mat depth;  // CV_32FC1 meters, may be empty
    double fx = 0, fy = 0, cx = 0, cy = 0;
    int width = 0, height = 0;
    uint64_t timestamp_ns = 0;
};
using RGBDCallback = std::function<void(const RGBDFrame &)>;

// Pulls live pointcloud and IMU data from a Viam machine via the Viam C++ SDK.
// Implements the same FrameCallback / IMUCallback interface as LivoxReceiver.
//
// Credentials are read from env vars VIAM_API_KEY / VIAM_API_KEY_ID (with
// config values as fallback) — use the same .env file as other Viam modules.
class ViamClient {
public:
    struct Config {
        std::string address;      // e.g. "my-robot.abc123.viam.cloud" or "localhost:8080"
        std::string api_key;      // fallback if VIAM_API_KEY not set
        std::string api_key_id;   // fallback if VIAM_API_KEY_ID not set
        bool insecure = false;    // true = no credentials, allow plaintext (local connections)
        std::string lidar_name;   // Camera component name for pointcloud
        std::string imu_name;     // MovementSensor component name ("" = disabled)
        std::string base_name;    // Base component name ("" = no base control)
        std::string rgb_name;       // Camera component name for RGBD ("" = disabled)
        std::string planning_frame; // Viam frame used as map origin ("" = skip frame query)
        int cloud_hz = 10;          // target pointcloud poll rate (Hz)
        int imu_hz = 100;           // target IMU poll rate (Hz)
        int rgb_hz = 15;            // target RGBD poll rate (Hz)
    };

    explicit ViamClient(const Config &cfg);
    ~ViamClient();

    // Start polling. Returns false if the initial connection fails.
    bool start(FrameCallback frame_cb, IMUCallback imu_cb = nullptr, RGBDCallback rgbd_cb = nullptr);
    void stop();

    // Validate that all configured sensors and frame transforms resolved.
    // Call after start(). Prints a summary table and returns false if anything
    // that was configured (non-empty name) failed to connect or resolve.
    bool validateFrameSystem() const;

    // Send a velocity command to the base component. No-op if base_name is empty.
    void sendBaseVelocity(float linear_mps, float angular_rps);

    // Pose of the lidar expressed in the planning frame (map origin).
    // Valid only if planning_frame is set and the frame query succeeded.
    rtabmap::Transform getLidarInPlanningFrame() const { return lidar_in_planning_frame_; }

    // Pose of the RGB camera expressed in the lidar frame (camera extrinsics).
    // Valid only if rgb_name and planning_frame are set and the frame query succeeded.
    rtabmap::Transform getCameraToLidar() const { return camera_to_lidar_; }

    // Pose of the IMU expressed in the lidar frame.
    // Valid only if imu_name and planning_frame are set and the frame query succeeded.
    rtabmap::Transform getImuToLidar() const { return imu_to_lidar_; }

private:
    // Instance MUST be the first member: constructed before any SDK objects,
    // destroyed after all SDK objects (C++ reverses declaration order on destroy).
    viam::sdk::Instance inst_;

    Config cfg_;
    FrameCallback frame_cb_;
    IMUCallback imu_cb_;
    RGBDCallback rgbd_cb_;
    std::atomic<bool> running_{false};
    std::thread cloud_thread_;
    std::thread imu_thread_;
    std::thread rgbd_thread_;

    // Single shared connection — all loops use these handles.
    // Protected by machine_mu_; copy the shared_ptr locally before doing I/O.
    std::mutex machine_mu_;
    std::shared_ptr<viam::sdk::RobotClient> machine_;
    std::shared_ptr<viam::sdk::Camera>          camera_;
    std::shared_ptr<viam::sdk::Camera>          rgb_cam_;
    std::shared_ptr<viam::sdk::MovementSensor>  sensor_;
    std::shared_ptr<viam::sdk::Base>            base_;

    void cloudLoop();
    void imuLoop();
    void rgbdLoop();
    // Rebuild machine_ + all component handles. Must be called with machine_mu_ held.
    bool reconnect();
    // Query get_pose() for lidar and camera. Called once from reconnect().
    void queryFrameSystem();

    rtabmap::Transform lidar_in_planning_frame_;  // null if not queried
    rtabmap::Transform camera_to_lidar_;          // null if not queried
    rtabmap::Transform imu_to_lidar_;             // null if not queried

    static pcl::PointCloud<pcl::PointXYZI>::Ptr parsePCD(
        const std::vector<uint8_t> &bytes);
};
