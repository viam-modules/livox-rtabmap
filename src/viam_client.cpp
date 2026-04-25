#include "viam_client.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <thread>
#include <unistd.h>

#include <pcl/io/pcd_io.h>

#include <opencv2/imgcodecs.hpp>

#include <viam/sdk/components/base.hpp>
#include <viam/sdk/components/camera.hpp>
#include <viam/sdk/components/movement_sensor.hpp>
#include <viam/sdk/robot/client.hpp>

using namespace viam::sdk;

// Convert a Viam pose (mm, axis-angle degrees) to an rtabmap Transform (m, rotation matrix).
static rtabmap::Transform poseToTransform(const pose &p) {
    float x = static_cast<float>(p.coordinates.x / 1000.0);
    float y = static_cast<float>(p.coordinates.y / 1000.0);
    float z = static_cast<float>(p.coordinates.z / 1000.0);
    double half = p.theta * M_PI / 360.0;  // theta is degrees, half-angle
    float qw = static_cast<float>(std::cos(half));
    float s  = static_cast<float>(std::sin(half));
    float qx = s * static_cast<float>(p.orientation.o_x);
    float qy = s * static_cast<float>(p.orientation.o_y);
    float qz = s * static_cast<float>(p.orientation.o_z);
    return rtabmap::Transform(x, y, z, qx, qy, qz, qw);
}

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
        if (!cfg_.rgb_name.empty())
            rgb_cam_ = machine_->resource_by_name<Camera>(cfg_.rgb_name);
        std::cout << "[VIAM] Connected to " << cfg_.address << "\n";
        queryFrameSystem();
        return true;
    } catch (const std::exception &e) {
        std::cerr << "[VIAM] reconnect failed: " << e.what() << "\n";
        machine_.reset(); camera_.reset(); sensor_.reset(); base_.reset(); rgb_cam_.reset();
        return false;
    }
}

bool ViamClient::start(FrameCallback frame_cb, IMUCallback imu_cb, RGBDCallback rgbd_cb) {
    frame_cb_ = std::move(frame_cb);
    imu_cb_   = std::move(imu_cb);
    rgbd_cb_  = std::move(rgbd_cb);

    {
        std::lock_guard<std::mutex> lock(machine_mu_);
        if (!reconnect()) return false;
    }

    running_ = true;
    cloud_thread_ = std::thread(&ViamClient::cloudLoop, this);
    if (imu_cb_ && !cfg_.imu_name.empty())
        imu_thread_ = std::thread(&ViamClient::imuLoop, this);
    if (rgbd_cb_ && !cfg_.rgb_name.empty())
        rgbd_thread_ = std::thread(&ViamClient::rgbdLoop, this);
    return true;
}

void ViamClient::stop() {
    running_ = false;
    if (cloud_thread_.joinable()) cloud_thread_.join();
    if (imu_thread_.joinable())   imu_thread_.join();
    if (rgbd_thread_.joinable())  rgbd_thread_.join();
}

void ViamClient::cloudLoop() {
    const auto interval = std::chrono::microseconds(1'000'000 / cfg_.cloud_hz);
    // Cooldown between reconnect attempts — prevents rapid reconnect storms that
    // crash the Viam SDK's RobotClient destructor (actuator-stop spam over WiFi).
    auto last_reconnect = std::chrono::steady_clock::time_point{};
    constexpr int kReconnectCooldownSec = 5;

    while (running_) {
        auto t0 = std::chrono::steady_clock::now();

        std::shared_ptr<Camera> cam;
        {
            std::lock_guard<std::mutex> lock(machine_mu_);
            cam = camera_;
            if (!cam) {
                auto since = std::chrono::duration_cast<std::chrono::seconds>(
                    t0 - last_reconnect).count();
                if (since < kReconnectCooldownSec) {
                    std::cerr << "[VIAM] reconnect cooldown (" << since << "s < "
                              << kReconnectCooldownSec << "s), waiting\n";
                    std::this_thread::sleep_until(t0 + interval);
                    continue;
                }
                if (!reconnect()) {
                    std::this_thread::sleep_until(t0 + interval);
                    continue;
                }
                last_reconnect = std::chrono::steady_clock::now();
                cam = camera_;
            }
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

            // Skip clouds that are too small to produce valid ICP correspondences.
            // A fresh connection or a corrupted PCD can return a handful of points
            // which causes ICP to fail, resetting the odometry and spiralling.
            constexpr size_t kMinPoints = 1000;
            if (cloud && cloud->size() >= kMinPoints) frame_cb_(cloud, ts_ns);
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
            std::cerr << "[VIAM] cloud fetch error: " << e.what() << "\n";
            auto since = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - last_reconnect).count();
            if (since >= kReconnectCooldownSec) {
                std::lock_guard<std::mutex> lock(machine_mu_);
                camera_.reset();  // cleared; top of loop will reconnect after cooldown check
                last_reconnect = std::chrono::steady_clock::now();
            } else {
                std::cerr << "[VIAM] reconnect cooldown (" << since << "s < "
                          << kReconnectCooldownSec << "s), skipping reconnect\n";
            }
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
        }
        if (!sens) {
            // cloudLoop owns reconnects — wait for it to restore sensor_
            std::this_thread::sleep_until(t0 + interval);
            continue;
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
            std::cerr << "[VIAM] IMU read error: " << e.what() << " — clearing handle, cloudLoop will reconnect\n";
            std::lock_guard<std::mutex> lock(machine_mu_);
            sensor_.reset();
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

bool ViamClient::validateFrameSystem() const {
    std::lock_guard<std::mutex> lock(const_cast<std::mutex &>(machine_mu_));

    bool ok = true;

    // Print one validation row and accumulate failures.
    auto row = [&](const std::string &label, const std::string &name,
                   bool configured, bool resolved) {
        const char *status = !configured ? " -- " : (resolved ? " OK " : "FAIL");
        std::cout << "  [" << status << "]  " << label;
        if (configured) std::cout << "  " << name;
        else            std::cout << "  (not configured)";
        std::cout << "\n";
        if (configured && !resolved) ok = false;
    };

    std::cout << "[VIAM] Validation:\n";

    // Component handles
    row("lidar  ", cfg_.lidar_name, true,                   camera_  != nullptr);
    row("imu    ", cfg_.imu_name,   !cfg_.imu_name.empty(), sensor_  != nullptr);
    row("base   ", cfg_.base_name,  !cfg_.base_name.empty(), base_   != nullptr);
    row("rgb    ", cfg_.rgb_name,   !cfg_.rgb_name.empty(), rgb_cam_ != nullptr);

    // Frame transforms
    bool have_frame = !cfg_.planning_frame.empty();
    row("lidar→'" + (have_frame ? cfg_.planning_frame : "?") + "'",
        "",       have_frame,                    !lidar_in_planning_frame_.isNull());
    row("cam→lidar  ", "",
        have_frame && !cfg_.rgb_name.empty(),    !camera_to_lidar_.isNull());
    row("imu→lidar  ", "",
        have_frame && !cfg_.imu_name.empty(),    !imu_to_lidar_.isNull());

    if (ok) std::cout << "[VIAM] All configured sensors/frames OK\n";
    else    std::cerr << "[VIAM] Validation FAILED — check component names and frame system\n";

    return ok;
}

void ViamClient::queryFrameSystem() {
    // Must be called with machine_mu_ held (called from reconnect()).
    if (cfg_.planning_frame.empty() || !machine_) return;

    try {
        auto pif = machine_->get_pose(cfg_.lidar_name, cfg_.planning_frame, {}, {});
        lidar_in_planning_frame_ = poseToTransform(pif.pose);
        std::cout << "[VIAM] Lidar in planning frame '" << cfg_.planning_frame << "': "
                  << lidar_in_planning_frame_.prettyPrint() << "\n";
    } catch (const std::exception &e) {
        std::cerr << "[VIAM] get_pose(" << cfg_.lidar_name << ", " << cfg_.planning_frame
                  << ") failed: " << e.what() << "\n";
    }

    if (!cfg_.rgb_name.empty()) {
        try {
            auto pif = machine_->get_pose(cfg_.rgb_name, cfg_.lidar_name, {}, {});
            camera_to_lidar_ = poseToTransform(pif.pose);
            std::cout << "[VIAM] Camera '" << cfg_.rgb_name << "' in lidar frame: "
                      << camera_to_lidar_.prettyPrint() << "\n";
        } catch (const std::exception &e) {
            std::cerr << "[VIAM] get_pose(" << cfg_.rgb_name << ", " << cfg_.lidar_name
                      << ") failed: " << e.what() << "\n";
        }
    }

    if (!cfg_.imu_name.empty()) {
        try {
            auto pif = machine_->get_pose(cfg_.imu_name, cfg_.lidar_name, {}, {});
            imu_to_lidar_ = poseToTransform(pif.pose);
            std::cout << "[VIAM] IMU '" << cfg_.imu_name << "' in lidar frame: "
                      << imu_to_lidar_.prettyPrint() << "\n";
        } catch (const std::exception &e) {
            std::cerr << "[VIAM] get_pose(" << cfg_.imu_name << ", " << cfg_.lidar_name
                      << ") failed: " << e.what() << "\n";
        }
    }
}

void ViamClient::rgbdLoop() {
    const auto interval = std::chrono::microseconds(1'000'000 / cfg_.rgb_hz);

    double fx = 0, fy = 0, cx = 0, cy = 0;
    int width = 0, height = 0;
    bool have_intrinsics = false;

    while (running_) {
        auto t0 = std::chrono::steady_clock::now();

        std::shared_ptr<Camera> cam;
        {
            std::lock_guard<std::mutex> lock(machine_mu_);
            cam = rgb_cam_;
        }
        if (!cam) {
            // cloudLoop owns reconnects — wait for it to restore rgb_cam_
            std::this_thread::sleep_until(t0 + interval);
            continue;
        }

        if (!have_intrinsics) {
            try {
                auto props = cam->get_properties();
                fx = props.intrinsic_parameters.focal_x_px;
                fy = props.intrinsic_parameters.focal_y_px;
                cx = props.intrinsic_parameters.center_x_px;
                cy = props.intrinsic_parameters.center_y_px;
                width  = props.intrinsic_parameters.width_px;
                height = props.intrinsic_parameters.height_px;
                have_intrinsics = true;
                std::cout << "[VIAM] RGBD intrinsics: fx=" << fx << " fy=" << fy
                          << " cx=" << cx << " cy=" << cy
                          << " size=" << width << "x" << height << "\n";
            } catch (const std::exception &e) {
                std::cerr << "[VIAM] RGBD get_properties: " << e.what() << "\n";
                std::this_thread::sleep_until(t0 + interval);
                continue;
            }
        }

        try {
            auto images = cam->get_images();
            uint64_t ts_ns = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count());

            cv::Mat rgb, depth;

            for (const auto &img : images.images) {
                if (img.mime_type.find("dep") != std::string::npos) {
                    try {
                        Camera::depth_map dm = Camera::decode_depth_map(img.bytes);
                        int h = static_cast<int>(dm.shape(0));
                        int w = static_cast<int>(dm.shape(1));
                        cv::Mat depth_mm(h, w, CV_16UC1);
                        for (int r = 0; r < h; r++)
                            for (int c = 0; c < w; c++)
                                depth_mm.at<uint16_t>(r, c) = dm(r, c);
                        depth_mm.convertTo(depth, CV_32FC1, 0.001f);
                    } catch (const std::exception &e) {
                        std::cerr << "[VIAM] depth decode: " << e.what() << "\n";
                    }
                } else {
                    std::vector<uint8_t> buf(img.bytes.begin(), img.bytes.end());
                    cv::Mat decoded = cv::imdecode(buf, cv::IMREAD_COLOR);
                    if (!decoded.empty()) rgb = decoded;
                }
            }

            if (!rgb.empty() || !depth.empty()) {
                RGBDFrame frame;
                frame.rgb   = rgb;
                frame.depth = depth;
                frame.fx = fx; frame.fy = fy;
                frame.cx = cx; frame.cy = cy;
                frame.width  = width;
                frame.height = height;
                frame.timestamp_ns = ts_ns;
                rgbd_cb_(frame);
            }
        } catch (const std::exception &e) {
            std::cerr << "[RGBD] read error: " << e.what() << " — clearing handle, cloudLoop will reconnect\n";
            {
                std::lock_guard<std::mutex> lock(machine_mu_);
                rgb_cam_.reset();
            }
            have_intrinsics = false;  // re-fetch intrinsics on next good connection
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
