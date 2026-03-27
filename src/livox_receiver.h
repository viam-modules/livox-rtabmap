#pragma once

#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Callback type: receives a complete frame as PCL cloud (meters + intensity)
using FrameCallback = std::function<void(pcl::PointCloud<pcl::PointXYZI>::Ptr, uint64_t timestamp_ns)>;

class LivoxReceiver {
public:
    LivoxReceiver(const std::string &sensor_ip, const std::string &host_ip);
    ~LivoxReceiver();

    // Start receiving data. Calls frame_cb on each complete frame (~10Hz).
    bool start(FrameCallback frame_cb);
    void stop();

    // Called from C callbacks — must be public
    void handlePointCloud(uint32_t handle, uint8_t dev_type, void *data);
    void handleInfoChange(uint32_t handle, void *info);

private:
    std::string sensor_ip_;
    std::string host_ip_;
    std::string config_path_;
    FrameCallback frame_cb_;
    bool running_ = false;

    // Frame accumulation (called from SDK callback thread)
    std::mutex mu_;
    std::vector<pcl::PointXYZI> writing_;
    uint64_t write_timestamp_ = 0;
    bool initialized_ = false;

    std::string generateConfig();
};
