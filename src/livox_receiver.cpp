#include "livox_receiver.h"

#include <cstring>
#include <fstream>
#include <iostream>

#include "livox_lidar_api.h"
#include "livox_lidar_def.h"

// Global pointer for C callbacks to reach the receiver instance
static LivoxReceiver *g_receiver = nullptr;

// C callback trampolines
static void pointCloudCb(const uint32_t handle, const uint8_t dev_type,
                          LivoxLidarEthernetPacket *data, void *) {
    if (data && g_receiver) {
        g_receiver->handlePointCloud(handle, dev_type, data);
    }
}

static void imuCb(const uint32_t handle, const uint8_t dev_type,
                   LivoxLidarEthernetPacket *data, void *) {
    if (data && g_receiver) {
        g_receiver->handleIMU(handle, dev_type, data);
    }
}

static void infoChangeCb(const uint32_t handle, const LivoxLidarInfo *info, void *) {
    if (info && g_receiver) {
        g_receiver->handleInfoChange(handle, (void *)info);
    }
}

LivoxReceiver::LivoxReceiver(const std::string &sensor_ip, const std::string &host_ip)
    : sensor_ip_(sensor_ip), host_ip_(host_ip) {}

LivoxReceiver::~LivoxReceiver() {
    stop();
}

bool LivoxReceiver::start(FrameCallback frame_cb, IMUCallback imu_cb) {
    frame_cb_ = frame_cb;
    imu_cb_ = imu_cb;
    g_receiver = this;

    config_path_ = generateConfig();
    if (config_path_.empty()) {
        std::cerr << "Failed to generate config\n";
        return false;
    }

    std::cout << "[Livox] Initializing SDK...\n";
    if (!LivoxLidarSdkInit(config_path_.c_str(), host_ip_.c_str(), nullptr)) {
        std::cerr << "[Livox] SDK init failed\n";
        return false;
    }

    SetLivoxLidarPointCloudCallBack(pointCloudCb, nullptr);
    SetLivoxLidarImuDataCallback(imuCb, nullptr);
    SetLivoxLidarInfoChangeCallback(infoChangeCb, nullptr);

    std::cout << "[Livox] Starting SDK...\n";
    if (!LivoxLidarSdkStart()) {
        std::cerr << "[Livox] SDK start failed\n";
        LivoxLidarSdkUninit();
        return false;
    }

    running_ = true;
    std::cout << "[Livox] Receiving data from " << sensor_ip_ << "\n";
    return true;
}

void LivoxReceiver::stop() {
    if (running_) {
        running_ = false;
        LivoxLidarSdkUninit();
        g_receiver = nullptr;
        std::cout << "[Livox] Stopped.\n";
    }
}

void LivoxReceiver::handlePointCloud(uint32_t handle, uint8_t dev_type, void *raw) {
    auto *pkt = static_cast<LivoxLidarEthernetPacket *>(raw);
    if (pkt->data_type != kLivoxLidarCartesianCoordinateHighData) return;

    uint64_t ts;
    memcpy(&ts, pkt->timestamp, sizeof(ts));
    uint16_t dot_num = pkt->dot_num;

    auto *points = reinterpret_cast<LivoxLidarCartesianHighRawPoint *>(pkt->data);

    std::lock_guard<std::mutex> lock(mu_);

    // Time-based framing: 100ms per frame (~10Hz)
    constexpr uint64_t kFrameIntervalNs = 100'000'000;
    if (initialized_ && ts - write_timestamp_ >= kFrameIntervalNs) {
        // Frame complete — build PCL cloud and deliver
        auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        cloud->reserve(writing_.size());
        for (const auto &p : writing_) {
            cloud->push_back(p);
        }
        cloud->width = cloud->size();
        cloud->height = 1;
        cloud->is_dense = true;

        uint64_t frame_ts = write_timestamp_;
        writing_.clear();
        write_timestamp_ = ts;

        // Deliver outside lock would be better but callback should be fast
        if (frame_cb_) {
            frame_cb_(cloud, frame_ts);
        }
    }

    initialized_ = true;
    if (writing_.empty()) {
        write_timestamp_ = ts;
    }

    // Accumulate points (convert mm int32 → meters float)
    for (uint16_t i = 0; i < dot_num; i++) {
        pcl::PointXYZI pt;
        pt.x = static_cast<float>(points[i].x) / 1000.0f;
        pt.y = static_cast<float>(points[i].y) / 1000.0f;
        pt.z = static_cast<float>(points[i].z) / 1000.0f;
        pt.intensity = static_cast<float>(points[i].reflectivity);
        writing_.push_back(pt);
    }
}

void LivoxReceiver::handleInfoChange(uint32_t handle, void *raw) {
    auto *info = static_cast<LivoxLidarInfo *>(raw);
    char sn[17] = {};
    memcpy(sn, info->sn, 16);
    std::cout << "[Livox] Device connected: handle=" << handle
              << " sn=" << sn << "\n";

    // Enable normal work mode + IMU
    SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, nullptr, nullptr);
    EnableLivoxLidarImuData(handle, nullptr, nullptr);
}

void LivoxReceiver::handleIMU(uint32_t handle, uint8_t dev_type, void *raw) {
    auto *pkt = static_cast<LivoxLidarEthernetPacket *>(raw);
    if (pkt->data_type != kLivoxLidarImuData) return;

    uint64_t ts;
    memcpy(&ts, pkt->timestamp, sizeof(ts));

    auto *imu = reinterpret_cast<LivoxLidarImuRawPoint *>(pkt->data);
    if (imu_cb_) {
        imu_cb_(LivoxIMU{
            imu->gyro_x, imu->gyro_y, imu->gyro_z,
            imu->acc_x, imu->acc_y, imu->acc_z,
            ts
        });
    }
}

std::string LivoxReceiver::generateConfig() {
    std::string path = "/tmp/livox_rtabmap_config.json";
    std::ofstream f(path);
    if (!f) return "";

    f << R"({
  "MID360": {
    "lidar_net_info": {
      "cmd_data_port": 56100,
      "push_msg_port": 56200,
      "point_data_port": 56300,
      "imu_data_port": 56400,
      "log_data_port": 56500
    },
    "host_net_info": [{
      "host_ip": ")" << host_ip_ << R"(",
      "cmd_data_port": 56101,
      "push_msg_port": 56201,
      "point_data_port": 56301,
      "imu_data_port": 56401,
      "log_data_port": 56501,
      "lidar_ip": [")" << sensor_ip_ << R"("]
    }]
  }
})";

    return path;
}
