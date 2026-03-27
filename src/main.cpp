#include <atomic>
#include <cmath>
#include <csignal>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <QApplication>
#include <QTimer>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <rtabmap/gui/CloudViewer.h>
#include <rtabmap/core/Transform.h>

#include <nlohmann/json.hpp>

#include "livox_receiver.h"
#include "slam_pipeline.h"

using json = nlohmann::json;

static std::atomic<bool> running{true};

static void signalHandler(int) {
    running = false;
}

// HSV to RGB helper (h: 0-360, s: 0-1, v: 0-1)
static void hsv2rgb(float h, float s, float v, uint8_t &r, uint8_t &g, uint8_t &b) {
    float c = v * s;
    float x = c * (1 - std::fabs(std::fmod(h / 60.0f, 2) - 1));
    float m = v - c;
    float rf, gf, bf;
    if (h < 60)      { rf = c; gf = x; bf = 0; }
    else if (h < 120) { rf = x; gf = c; bf = 0; }
    else if (h < 180) { rf = 0; gf = c; bf = x; }
    else if (h < 240) { rf = 0; gf = x; bf = c; }
    else if (h < 300) { rf = x; gf = 0; bf = c; }
    else              { rf = c; gf = 0; bf = x; }
    r = (uint8_t)((rf + m) * 255);
    g = (uint8_t)((gf + m) * 255);
    b = (uint8_t)((bf + m) * 255);
}

// Per-point data for the accumulated map
struct MapPoint {
    float x, y, z;
    float intensity;
    int frame_id;  // which frame added this point
};

int main(int argc, char *argv[]) {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    std::string config_path = "config/default.json";
    if (argc > 1 && argv[1][0] != '-') {
        config_path = argv[1];
    }

    std::ifstream f(config_path);
    if (!f) {
        std::cerr << "Cannot open config: " << config_path << "\n";
        return 1;
    }
    json config = json::parse(f);

    std::string sensor_ip = config.value("sensor_ip", "192.168.1.196");
    std::string host_ip = config.value("host_ip", "192.168.1.10");
    bool headless = config.value("headless", false);
    float map_voxel = config.value("map_voxel_size", 0.03f);
    int downsample_interval = config.value("map_downsample_interval", 50);
    std::string color_mode = config.value("color_mode", "intensity");
    bool show_trajectory = config.value("show_trajectory", true);

    auto map_color_arr = config.value("map_color", std::vector<int>{180, 180, 180});
    auto scan_color_arr = config.value("scan_color", std::vector<int>{0, 255, 0});
    int map_point_size = config.value("map_point_size", 1);
    int scan_point_size = config.value("scan_point_size", 3);
    QColor map_color(map_color_arr[0], map_color_arr[1], map_color_arr[2]);
    QColor scan_color(scan_color_arr[0], scan_color_arr[1], scan_color_arr[2]);

    std::cout << "Livox → RTAB-Map SLAM\n"
              << "  Config: " << config_path << "\n"
              << "  Sensor: " << sensor_ip << "\n"
              << "  Host:   " << host_ip << "\n"
              << "  Mode:   " << (headless ? "headless" : "GUI") << "\n\n";

    SlamPipeline slam;
    if (!slam.init(config)) {
        std::cerr << "Failed to init SLAM pipeline\n";
        return 1;
    }

    if (headless) {
        LivoxReceiver receiver(sensor_ip, host_ip);
        bool ok = receiver.start(
            [&slam](pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, uint64_t ts) {
                slam.processCloud(cloud, ts);
            },
            [&slam](const LivoxIMU &imu) {
                slam.processIMU(imu.gyro_x, imu.gyro_y, imu.gyro_z,
                                imu.acc_x, imu.acc_y, imu.acc_z,
                                imu.timestamp_ns);
            });
        if (!ok) { std::cerr << "Failed to start Livox receiver\n"; return 1; }
        while (running) std::this_thread::sleep_for(std::chrono::seconds(1));
        receiver.stop();
        return 0;
    }

    // GUI mode
    QApplication app(argc, argv);
    rtabmap::CloudViewer viewer;
    viewer.setWindowTitle("Livox Mid-360 SLAM");
    viewer.setBackgroundColor(QColor(20, 20, 20));
    viewer.setGridShown(true);
    viewer.resize(1280, 720);
    float cam_dist = config.value("camera_distance", 10.0f);
    float d = cam_dist / 1.732f;
    viewer.setCameraPosition(d, d, d, 0, 0, 0, 0, 0, 1);
    viewer.show();

    std::mutex cloud_mu;
    pcl::PointCloud<pcl::PointXYZI>::Ptr latest_cloud;
    rtabmap::Transform latest_pose;
    std::vector<MapPoint> map_points;
    std::vector<std::pair<float, float>> trajectory_xz; // for trajectory line
    int frame_count = 0;
    int max_frame_id = 0;

    bool load_previous = config.value("load_previous_map", false);
    if (load_previous) {
        auto rebuilt = slam.rebuildMap();
        if (rebuilt->size() > 0) {
            std::cout << "[VIEWER] Loaded " << rebuilt->size() << " points from previous session\n";
            for (const auto &p : *rebuilt) {
                map_points.push_back({p.x, p.y, p.z, p.intensity, 0});
            }
        }
    }

    LivoxReceiver receiver(sensor_ip, host_ip);
    bool ok = receiver.start(
        [&](pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, uint64_t ts) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr filtered;
            bool success = slam.processCloud(cloud, ts, &filtered);
            if (success) {
                std::lock_guard<std::mutex> lock(cloud_mu);
                latest_cloud = filtered ? filtered : cloud;
                latest_pose = slam.getPose();
                frame_count++;
            }
        },
        [&](const LivoxIMU &imu) {
            slam.processIMU(imu.gyro_x, imu.gyro_y, imu.gyro_z,
                            imu.acc_x, imu.acc_y, imu.acc_z,
                            imu.timestamp_ns);
        });
    if (!ok) { std::cerr << "Failed to start Livox receiver\n"; return 1; }

    // Config live reload
    auto last_config_write = std::filesystem::last_write_time(config_path);
    QTimer config_timer;
    QObject::connect(&config_timer, &QTimer::timeout, [&]() {
        try {
            auto now = std::filesystem::last_write_time(config_path);
            if (now == last_config_write) return;
            last_config_write = now;
            std::ifstream rf(config_path);
            if (!rf) return;
            json new_config = json::parse(rf);
            map_voxel = new_config.value("map_voxel_size", map_voxel);
            downsample_interval = new_config.value("map_downsample_interval", downsample_interval);
            color_mode = new_config.value("color_mode", color_mode);
            show_trajectory = new_config.value("show_trajectory", show_trajectory);
            auto mc = new_config.value("map_color", std::vector<int>{map_color.red(), map_color.green(), map_color.blue()});
            auto sc = new_config.value("scan_color", std::vector<int>{scan_color.red(), scan_color.green(), scan_color.blue()});
            map_color = QColor(mc[0], mc[1], mc[2]);
            scan_color = QColor(sc[0], sc[1], sc[2]);
            map_point_size = new_config.value("map_point_size", map_point_size);
            scan_point_size = new_config.value("scan_point_size", scan_point_size);
            std::cout << "[CONFIG] Reloaded " << config_path << "\n";
        } catch (const std::exception &e) {
            std::cerr << "[CONFIG] Reload error: " << e.what() << "\n";
        }
    });
    config_timer.start(2000);

    QTimer update_timer;
    QObject::connect(&update_timer, &QTimer::timeout, [&]() {
        if (!running) { app.quit(); return; }

        std::lock_guard<std::mutex> lock(cloud_mu);
        if (!latest_cloud) return;

        max_frame_id = frame_count;

        // Transform and accumulate points
        for (const auto &p : *latest_cloud) {
            float x = p.x, y = p.y, z = p.z;
            const float *d = latest_pose.data();
            MapPoint mp;
            mp.x = d[0]*x + d[1]*y + d[2]*z + d[3];
            mp.y = d[4]*x + d[5]*y + d[6]*z + d[7];
            mp.z = d[8]*x + d[9]*y + d[10]*z + d[11];
            mp.intensity = p.intensity;
            mp.frame_id = frame_count;
            map_points.push_back(mp);
        }

        // Track trajectory
        float tx = latest_pose.x(), ty = latest_pose.y(), tz = latest_pose.z();
        trajectory_xz.push_back({tx, ty});

        // Build colored cloud for display
        auto display_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        display_cloud->reserve(map_points.size());

        if (color_mode == "age") {
            // Blue (old) → Red (new)
            for (const auto &mp : map_points) {
                float age = (max_frame_id > 1) ? (float)mp.frame_id / max_frame_id : 1.0f;
                float hue = (1.0f - age) * 240.0f;
                uint8_t r, g, b;
                hsv2rgb(hue, 1.0f, 1.0f, r, g, b);
                pcl::PointXYZRGB rp;
                rp.x = mp.x; rp.y = mp.y; rp.z = mp.z;
                rp.r = r; rp.g = g; rp.b = b;
                display_cloud->push_back(rp);
            }
        } else if (color_mode == "height") {
            // Find Z range then map to rainbow
            float zmin = 1e9, zmax = -1e9;
            for (const auto &mp : map_points) {
                if (mp.z < zmin) zmin = mp.z;
                if (mp.z > zmax) zmax = mp.z;
            }
            float zrange = zmax - zmin;
            if (zrange < 0.01f) zrange = 1.0f;
            for (const auto &mp : map_points) {
                float t = (mp.z - zmin) / zrange; // 0 = low, 1 = high
                float hue = (1.0f - t) * 240.0f;  // blue (low) → red (high)
                uint8_t r, g, b;
                hsv2rgb(hue, 1.0f, 1.0f, r, g, b);
                pcl::PointXYZRGB rp;
                rp.x = mp.x; rp.y = mp.y; rp.z = mp.z;
                rp.r = r; rp.g = g; rp.b = b;
                display_cloud->push_back(rp);
            }
        } else if (color_mode == "flat") {
            for (const auto &mp : map_points) {
                pcl::PointXYZRGB rp;
                rp.x = mp.x; rp.y = mp.y; rp.z = mp.z;
                rp.r = map_color.red(); rp.g = map_color.green(); rp.b = map_color.blue();
                display_cloud->push_back(rp);
            }
        } else {
            // Intensity mode — map intensity to grayscale RGB
            for (const auto &mp : map_points) {
                uint8_t v = std::min(255, std::max(0, (int)mp.intensity));
                pcl::PointXYZRGB rp;
                rp.x = mp.x; rp.y = mp.y; rp.z = mp.z;
                rp.r = v; rp.g = v; rp.b = v;
                display_cloud->push_back(rp);
            }
        }
        display_cloud->width = display_cloud->size();
        display_cloud->height = 1;

        // Voxel downsample periodically — filter map_points via PointXYZI to preserve intensity
        if (frame_count % downsample_interval == 0 && map_points.size() > 100000) {
            auto xyzi = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
            xyzi->reserve(map_points.size());
            for (const auto &mp : map_points) {
                pcl::PointXYZI p;
                p.x = mp.x; p.y = mp.y; p.z = mp.z;
                p.intensity = mp.intensity;
                xyzi->push_back(p);
            }
            xyzi->width = xyzi->size(); xyzi->height = 1;

            pcl::VoxelGrid<pcl::PointXYZI> voxel;
            voxel.setInputCloud(xyzi);
            voxel.setLeafSize(map_voxel, map_voxel, map_voxel);
            auto filtered = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
            voxel.filter(*filtered);

            map_points.clear();
            map_points.reserve(filtered->size());
            for (const auto &p : *filtered) {
                map_points.push_back({p.x, p.y, p.z, p.intensity, frame_count});
            }

            // Rebuild display cloud from filtered map_points
            display_cloud->clear();
            display_cloud->reserve(map_points.size());
            for (const auto &mp : map_points) {
                pcl::PointXYZRGB rp;
                rp.x = mp.x; rp.y = mp.y; rp.z = mp.z;
                if (color_mode == "age") {
                    float age = (max_frame_id > 1) ? (float)mp.frame_id / max_frame_id : 1.0f;
                    hsv2rgb((1.0f - age) * 240.0f, 1.0f, 1.0f, rp.r, rp.g, rp.b);
                } else if (color_mode == "height") {
                    // will be slightly off since we lost zmin/zmax, but acceptable
                    rp.r = rp.g = rp.b = std::min(255, std::max(0, (int)mp.intensity));
                } else if (color_mode == "flat") {
                    rp.r = map_color.red(); rp.g = map_color.green(); rp.b = map_color.blue();
                } else {
                    uint8_t v = std::min(255, std::max(0, (int)mp.intensity));
                    rp.r = v; rp.g = v; rp.b = v;
                }
                display_cloud->push_back(rp);
            }
            display_cloud->width = display_cloud->size(); display_cloud->height = 1;
        }

        viewer.addCloud("map", display_cloud);
        viewer.setCloudPointSize("map", map_point_size);

        // Current scan in scan_color
        auto scan_rgb = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        scan_rgb->reserve(latest_cloud->size());
        for (const auto &p : *latest_cloud) {
            pcl::PointXYZRGB rp;
            rp.x = p.x; rp.y = p.y; rp.z = p.z;
            rp.r = scan_color.red(); rp.g = scan_color.green(); rp.b = scan_color.blue();
            scan_rgb->push_back(rp);
        }
        scan_rgb->width = scan_rgb->size(); scan_rgb->height = 1;
        viewer.addCloud("scan", scan_rgb, latest_pose);
        viewer.setCloudPointSize("scan", scan_point_size);

        // Draw trajectory
        if (show_trajectory && trajectory_xz.size() > 1) {
            viewer.removeCloud("trajectory");
            auto traj_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
            for (size_t i = 0; i < trajectory_xz.size(); i++) {
                pcl::PointXYZRGB tp;
                tp.x = trajectory_xz[i].first;
                tp.y = trajectory_xz[i].second;
                tp.z = 0;  // project onto ground plane
                tp.r = 255; tp.g = 50; tp.b = 255; // magenta trajectory
                traj_cloud->push_back(tp);
            }
            traj_cloud->width = traj_cloud->size(); traj_cloud->height = 1;
            viewer.addCloud("trajectory", traj_cloud);
            viewer.setCloudPointSize("trajectory", 6);
        }

        latest_cloud.reset();

        viewer.setWindowTitle(QString("Livox SLAM | Frame %1 | Map: %2 pts | %3")
            .arg(frame_count)
            .arg(map_points.size())
            .arg(QString::fromStdString(color_mode)));

        viewer.refreshView();
    });
    update_timer.start(33);

    int ret = app.exec();
    running = false;
    receiver.stop();

    std::cout << "\nFinal: " << slam.getFrameCount() << " frames, "
              << map_points.size() << " map points\n"
              << "Database saved to: " << slam.getDatabasePath() << "\n";
    return ret;
}
