#include "viam_client.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <thread>
#include <unistd.h>

#include <pcl/io/pcd_io.h>

#include <viam/sdk/components/camera.hpp>
#include <viam/sdk/components/movement_sensor.hpp>
#include <viam/sdk/robot/client.hpp>

using namespace viam::sdk;

// Helper: open a fresh connection using the client's config.
static std::shared_ptr<RobotClient> connect(const ViamClient::Config &cfg) {
    ViamChannel::Options channel_opts;
    std::cerr << "[VIAM] connecting: address=" << cfg.address
              << " key_id=" << (cfg.api_key_id.empty() ? "(empty)" : cfg.api_key_id)
              << " key=" << (cfg.api_key.empty() ? "(empty)" : "(set)") << "\n";
    channel_opts.set_entity(cfg.api_key_id);
    channel_opts.set_credentials(Credentials("api-key", cfg.api_key));
    Options options(0, std::move(channel_opts));
    return RobotClient::at_address(cfg.address, options);
}

ViamClient::ViamClient(const Config &cfg) : cfg_(cfg) {}

ViamClient::~ViamClient() { stop(); }

bool ViamClient::start(FrameCallback frame_cb, IMUCallback imu_cb) {
    frame_cb_ = std::move(frame_cb);
    imu_cb_   = std::move(imu_cb);

    // Verify connectivity and component names before spawning threads.
    try {
        auto machine = connect(cfg_);
        machine->resource_by_name<Camera>(cfg_.lidar_name);
        std::cout << "[VIAM] Connected to " << cfg_.address << "\n";
    } catch (const std::exception &e) {
        std::cerr << "[VIAM] Connection failed: " << e.what() << "\n";
        return false;
    }

    running_ = true;
    cloud_thread_ = std::thread(&ViamClient::cloudLoop, this);
    if (imu_cb_ && !cfg_.imu_name.empty()) {
        imu_thread_ = std::thread(&ViamClient::imuLoop, this);
    }
    return true;
}

void ViamClient::stop() {
    running_ = false;
    if (cloud_thread_.joinable()) cloud_thread_.join();
    if (imu_thread_.joinable())   imu_thread_.join();
}

void ViamClient::cloudLoop() {
    const auto interval = std::chrono::microseconds(1'000'000 / cfg_.cloud_hz);

    std::shared_ptr<RobotClient> machine;
    std::shared_ptr<Camera> camera;

    auto reconnect = [&]() -> bool {
        try {
            machine = connect(cfg_);
            camera  = machine->resource_by_name<Camera>(cfg_.lidar_name);
            return true;
        } catch (const std::exception &e) {
            std::cerr << "[VIAM] cloud reconnect failed: " << e.what() << "\n";
            return false;
        }
    };

    if (!reconnect()) { running_ = false; return; }

    while (running_) {
        auto t0 = std::chrono::steady_clock::now();

        try {
            auto result = camera->get_point_cloud("pointcloud/pcd");
            uint64_t ts_ns = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::system_clock::now().time_since_epoch())
                .count());

            auto cloud = parsePCD(result.pc);
            if (cloud && !cloud->empty()) {
                frame_cb_(cloud, ts_ns);
            }
        } catch (const std::exception &e) {
            std::cerr << "[VIAM] get_point_cloud: " << e.what()
                      << " — reconnecting\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            reconnect();
        }

        std::this_thread::sleep_until(t0 + interval);
    }
}

void ViamClient::imuLoop() {
    const auto interval = std::chrono::microseconds(1'000'000 / cfg_.imu_hz);

    std::shared_ptr<RobotClient> machine;
    std::shared_ptr<MovementSensor> sensor;

    auto reconnect = [&]() -> bool {
        try {
            machine = connect(cfg_);
            sensor  = machine->resource_by_name<MovementSensor>(cfg_.imu_name);
            return true;
        } catch (const std::exception &e) {
            std::cerr << "[VIAM] IMU reconnect failed: " << e.what() << "\n";
            return false;
        }
    };

    if (!reconnect()) return;

    constexpr double kDeg2Rad  = M_PI / 180.0;
    constexpr double kMps2ToG  = 1.0 / 9.80665;

    while (running_) {
        auto t0 = std::chrono::steady_clock::now();

        try {
            // get_angular_velocity → deg/s; get_linear_acceleration → m/s²
            auto av = sensor->get_angular_velocity();
            auto la = sensor->get_linear_acceleration();

            uint64_t ts_ns = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::system_clock::now().time_since_epoch())
                .count());

            LivoxIMU imu;
            imu.gyro_x = static_cast<float>(av.x() * kDeg2Rad);
            imu.gyro_y = static_cast<float>(av.y() * kDeg2Rad);
            imu.gyro_z = static_cast<float>(av.z() * kDeg2Rad);
            // LivoxIMU accel field is in g-force (processIMU multiplies by 9.8 internally)
            imu.acc_x = static_cast<float>(la.x() * kMps2ToG);
            imu.acc_y = static_cast<float>(la.y() * kMps2ToG);
            imu.acc_z = static_cast<float>(la.z() * kMps2ToG);
            imu.timestamp_ns = ts_ns;

            imu_cb_(imu);
        } catch (const std::exception &e) {
            std::cerr << "[VIAM] IMU read: " << e.what()
                      << " — reconnecting\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            reconnect();
        }

        std::this_thread::sleep_until(t0 + interval);
    }
}

pcl::PointCloud<pcl::PointXYZI>::Ptr ViamClient::parsePCD(
    const std::vector<uint8_t> &bytes) {
    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

    std::string tmp = std::string("/tmp/viam_pcd_") +
                      std::to_string(getpid()) + ".pcd";
    {
        std::ofstream f(tmp, std::ios::binary);
        if (!f) {
            std::cerr << "[VIAM] Cannot write temp PCD\n";
            return cloud;
        }
        f.write(reinterpret_cast<const char *>(bytes.data()), bytes.size());
    }

    if (pcl::io::loadPCDFile(tmp, *cloud) < 0) {
        std::cerr << "[VIAM] Failed to parse PCD (" << bytes.size() << " bytes)\n";
    }

    std::remove(tmp.c_str());
    return cloud;
}
