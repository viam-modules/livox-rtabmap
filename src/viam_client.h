#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <viam/sdk/common/instance.hpp>

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
        int cloud_hz = 10;        // target pointcloud poll rate (Hz)
        int imu_hz = 100;         // target IMU poll rate (Hz)
    };

    explicit ViamClient(const Config &cfg);
    ~ViamClient();

    // Start polling. Returns false if the initial connection fails.
    bool start(FrameCallback frame_cb, IMUCallback imu_cb = nullptr);
    void stop();

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

    void cloudLoop();
    void imuLoop();

    static pcl::PointCloud<pcl::PointXYZI>::Ptr parsePCD(
        const std::vector<uint8_t> &bytes);
};
