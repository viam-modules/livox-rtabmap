#include "viam_client.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <thread>
#include <unistd.h>

#include <pcl/io/pcd_io.h>

#include <viam/sdk/components/base.hpp>
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

bool ViamClient::reconnect() {
    // Must be called with machine_mu_ held.
    try {
        machine_ = connect(cfg_);
        camera_  = machine_->resource_by_name<Camera>(cfg_.lidar_name);
        if (!cfg_.imu_name.empty())
            sensor_ = machine_->resource_by_name<MovementSensor>(cfg_.imu_name);
        if (!cfg_.base_name.empty())
            base_ = machine_->resource_by_name<Base>(cfg_.base_name);
        std::cout << "[VIAM] Connected to " << cfg_.address << "\n";
        return true;
    } catch (const std::exception &e) {
        std::cerr << "[VIAM] reconnect failed: " << e.what() << "\n";
        machine_.reset(); camera_.reset(); sensor_.reset(); base_.reset();
        return false;
    }
}

bool ViamClient::start(FrameCallback frame_cb, IMUCallback imu_cb) {
    frame_cb_ = std::move(frame_cb);
    imu_cb_   = std::move(imu_cb);

    {
        std::lock_guard<std::mutex> lock(machine_mu_);
        if (!reconnect()) return false;
    }

    running_ = true;
    cloud_thread_ = std::thread(&ViamClient::cloudLoop, this);
    if (imu_cb_ && !cfg_.imu_name.empty())
        imu_thread_ = std::thread(&ViamClient::imuLoop, this);
    return true;
}

void ViamClient::stop() {
    running_ = false;
    if (cloud_thread_.joinable()) cloud_thread_.join();
    if (imu_thread_.joinable())   imu_thread_.join();
}

void ViamClient::cloudLoop() {
    const auto interval = std::chrono::microseconds(1'000'000 / cfg_.cloud_hz);

    while (running_) {
        auto t0 = std::chrono::steady_clock::now();

        std::shared_ptr<Camera> cam;
        {
            std::lock_guard<std::mutex> lock(machine_mu_);
            cam = camera_;
            if (!cam && !reconnect()) {
                std::this_thread::sleep_until(t0 + interval);
                continue;
            }
            cam = camera_;
        }

        try {
            auto t_fetch = std::chrono::steady_clock::now();
            auto result = cam->get_point_cloud("pointcloud/pcd");
            auto t_parse = std::chrono::steady_clock::now();

            uint64_t ts_ns = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::system_clock::now().time_since_epoch())
                .count());

            auto cloud = parsePCD(result.pc);
            auto t_cb = std::chrono::steady_clock::now();

            if (cloud && !cloud->empty()) frame_cb_(cloud, ts_ns);
            auto t_done = std::chrono::steady_clock::now();

            auto ms = [](auto a, auto b) {
                return std::chrono::duration_cast<std::chrono::milliseconds>(b - a).count();
            };
            std::cerr << "[VIAM] fetch=" << ms(t_fetch, t_parse)
                      << "ms  parse=" << ms(t_parse, t_cb)
                      << "ms  slam=" << ms(t_cb, t_done)
                      << "ms  total=" << ms(t_fetch, t_done) << "ms"
                      << "  pts=" << (cloud ? cloud->size() : 0) << "\n";
        } catch (const std::exception &e) {
            std::cerr << "[VIAM] get_point_cloud: " << e.what() << " — reconnecting\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            std::lock_guard<std::mutex> lock(machine_mu_);
            reconnect();
        }

        std::this_thread::sleep_until(t0 + interval);
    }
}

void ViamClient::imuLoop() {
    const auto interval = std::chrono::microseconds(1'000'000 / cfg_.imu_hz);
    constexpr double kDeg2Rad = M_PI / 180.0;
    constexpr double kMps2ToG = 1.0 / 9.80665;

    while (running_) {
        auto t0 = std::chrono::steady_clock::now();

        std::shared_ptr<MovementSensor> sens;
        {
            std::lock_guard<std::mutex> lock(machine_mu_);
            sens = sensor_;
            if (!sens && !reconnect()) {
                std::this_thread::sleep_until(t0 + interval);
                continue;
            }
            sens = sensor_;
        }

        try {
            // get_angular_velocity → deg/s; get_linear_acceleration → m/s²
            auto av = sens->get_angular_velocity();
            auto la = sens->get_linear_acceleration();
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
            std::cerr << "[VIAM] IMU read: " << e.what() << " — reconnecting\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            std::lock_guard<std::mutex> lock(machine_mu_);
            reconnect();
        }

        std::this_thread::sleep_until(t0 + interval);
    }
}

void ViamClient::sendBaseVelocity(float linear_mps, float angular_rps) {
    if (cfg_.base_name.empty()) return;

    // set_velocity takes mm/s (linear) and deg/s (angular).
    // Forward motion is linear Y; rotation is angular Z.
    Vector3 linear_vec{};
    linear_vec.set_y(static_cast<double>(linear_mps) * 1000.0);
    Vector3 angular_vec{};
    angular_vec.set_z(static_cast<double>(angular_rps) * (180.0 / M_PI));

    std::shared_ptr<Base> base;
    {
        std::lock_guard<std::mutex> lock(machine_mu_);
        if (!base_ && !reconnect()) return;
        base = base_;
    }

    try {
        base->set_velocity(linear_vec, angular_vec);
    } catch (const std::exception &e) {
        std::cerr << "[VIAM] set_velocity failed: " << e.what() << " — reconnecting\n";
        std::lock_guard<std::mutex> lock(machine_mu_);
        if (reconnect()) {
            try { base_->set_velocity(linear_vec, angular_vec); }
            catch (const std::exception &e2) {
                std::cerr << "[VIAM] set_velocity retry failed: " << e2.what() << "\n";
            }
        }
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
