#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <viam/sdk/common/instance.hpp>
#include <viam/sdk/components/base.hpp>
#include <viam/sdk/components/camera.hpp>
#include <viam/sdk/components/movement_sensor.hpp>
#include <viam/sdk/robot/client.hpp>

#include "livox_receiver.h"  // FrameCallback, IMUCallback, LivoxIMU

// Pulls live pointcloud and IMU data from a Viam machine via the Viam C++ SDK.
// Implements the same FrameCallback / IMUCallback interface as LivoxReceiver.
//
// Credentials are read from env vars VIAM_API_KEY / VIAM_API_KEY_ID (with
// config values as fallback) — use the same .env file as other Viam modules.
class ViamClient {
public:
    struct Config {
        std::string address;      // e.g. "my-robot.abc123.viam.cloud"
        std::string api_key;      // fallback if VIAM_API_KEY not set
        std::string api_key_id;   // fallback if VIAM_API_KEY_ID not set
        std::string lidar_name;   // Camera component name for pointcloud
        std::string imu_name;     // MovementSensor component name ("" = disabled)
        std::string base_name;    // Base component name ("" = no base control)
        int cloud_hz = 10;        // target pointcloud poll rate (Hz)
        int imu_hz = 100;         // target IMU poll rate (Hz)
    };

    explicit ViamClient(const Config &cfg);
    ~ViamClient();

    // Start polling. Returns false if the initial connection fails.
    bool start(FrameCallback frame_cb, IMUCallback imu_cb = nullptr);
    void stop();

    // Send a velocity command to the base component. No-op if base_name is empty.
    void sendBaseVelocity(float linear_mps, float angular_rps);

private:
    // Instance MUST be the first member: constructed before any SDK objects,
    // destroyed after all SDK objects (C++ reverses declaration order on destroy).
    viam::sdk::Instance inst_;

    Config cfg_;
    FrameCallback frame_cb_;
    IMUCallback imu_cb_;
    std::atomic<bool> running_{false};
    std::thread cloud_thread_;
    std::thread imu_thread_;

    // Single shared connection — all loops use these handles.
    // Protected by machine_mu_; copy the shared_ptr locally before doing I/O.
    std::mutex machine_mu_;
    std::shared_ptr<viam::sdk::RobotClient> machine_;
    std::shared_ptr<viam::sdk::Camera>          camera_;
    std::shared_ptr<viam::sdk::MovementSensor>  sensor_;
    std::shared_ptr<viam::sdk::Base>            base_;

    void cloudLoop();
    void imuLoop();
    // Rebuild machine_ + all component handles. Must be called with machine_mu_ held.
    bool reconnect();

    static pcl::PointCloud<pcl::PointXYZI>::Ptr parsePCD(
        const std::vector<uint8_t> &bytes);
};
