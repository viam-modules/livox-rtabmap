#include <atomic>
#include <csignal>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include <QApplication>
#include <QTimer>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

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

int main(int argc, char *argv[]) {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    // Load config file (first arg, or default path)
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

    auto map_color_arr = config.value("map_color", std::vector<int>{180, 180, 180});
    auto scan_color_arr = config.value("scan_color", std::vector<int>{0, 255, 0});
    QColor map_color(map_color_arr[0], map_color_arr[1], map_color_arr[2]);
    QColor scan_color(scan_color_arr[0], scan_color_arr[1], scan_color_arr[2]);

    std::cout << "Livox → RTAB-Map SLAM\n"
              << "  Config: " << config_path << "\n"
              << "  Sensor: " << sensor_ip << "\n"
              << "  Host:   " << host_ip << "\n"
              << "  Mode:   " << (headless ? "headless" : "GUI") << "\n\n";

    // Init SLAM pipeline
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
            });
        if (!ok) {
            std::cerr << "Failed to start Livox receiver\n";
            return 1;
        }

        while (running) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
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
    // 45° isometric: camera at equal x, y, z offset, looking at origin, Z up
    float d = cam_dist / 1.732f; // divide by sqrt(3) so total distance = cam_dist
    viewer.setCameraPosition(d, d, d,    // camera position
                             0, 0, 0,    // focal point (origin)
                             0, 0, 1);   // up vector (Z up, matches lidar frame)
    viewer.show();

    std::mutex cloud_mu;
    pcl::PointCloud<pcl::PointXYZI>::Ptr latest_cloud;
    rtabmap::Transform latest_pose;
    pcl::PointCloud<pcl::PointXYZI>::Ptr accumulated_map;
    bool load_previous = config.value("load_previous_map", false);
    if (load_previous) {
        accumulated_map = slam.rebuildMap();
        if (accumulated_map->size() > 0) {
            std::cout << "[VIEWER] Loaded " << accumulated_map->size() << " points from previous session\n";
            viewer.addCloud("map", accumulated_map);
            viewer.refreshView();
        }
    } else {
        accumulated_map = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    }
    int frame_count = 0;

    LivoxReceiver receiver(sensor_ip, host_ip);
    bool ok = receiver.start(
        [&](pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, uint64_t ts) {
            bool success = slam.processCloud(cloud, ts);
            if (success) {
                std::lock_guard<std::mutex> lock(cloud_mu);
                latest_cloud = cloud;
                latest_pose = slam.getPose();
                frame_count++;
            }
        });

    if (!ok) {
        std::cerr << "Failed to start Livox receiver\n";
        return 1;
    }

    // Config live reload — check every 2 seconds
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
            auto mc = new_config.value("map_color", std::vector<int>{map_color.red(), map_color.green(), map_color.blue()});
            auto sc = new_config.value("scan_color", std::vector<int>{scan_color.red(), scan_color.green(), scan_color.blue()});
            map_color = QColor(mc[0], mc[1], mc[2]);
            scan_color = QColor(sc[0], sc[1], sc[2]);

            std::cout << "[CONFIG] Reloaded " << config_path << "\n";
        } catch (const std::exception &e) {
            std::cerr << "[CONFIG] Reload error: " << e.what() << "\n";
        }
    });
    config_timer.start(2000);

    QTimer update_timer;
    QObject::connect(&update_timer, &QTimer::timeout, [&]() {
        if (!running) {
            app.quit();
            return;
        }

        std::lock_guard<std::mutex> lock(cloud_mu);
        if (!latest_cloud) return;

        for (const auto &p : *latest_cloud) {
            pcl::PointXYZI tp;
            float x = p.x, y = p.y, z = p.z;
            const float *data = latest_pose.data();
            tp.x = data[0]*x + data[1]*y + data[2]*z + data[3];
            tp.y = data[4]*x + data[5]*y + data[6]*z + data[7];
            tp.z = data[8]*x + data[9]*y + data[10]*z + data[11];
            tp.intensity = p.intensity;
            accumulated_map->push_back(tp);
        }

        accumulated_map->width = accumulated_map->size();
        accumulated_map->height = 1;

        if (frame_count % downsample_interval == 0 && accumulated_map->size() > 100000) {
            pcl::VoxelGrid<pcl::PointXYZI> voxel;
            voxel.setInputCloud(accumulated_map);
            voxel.setLeafSize(map_voxel, map_voxel, map_voxel);
            pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);
            voxel.filter(*filtered);
            accumulated_map = filtered;
        }

        if (color_mode == "flat") {
            // Convert to XYZRGB for flat coloring
            auto map_rgb = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
            map_rgb->reserve(accumulated_map->size());
            for (const auto &p : *accumulated_map) {
                pcl::PointXYZRGB rp;
                rp.x = p.x; rp.y = p.y; rp.z = p.z;
                rp.r = map_color.red(); rp.g = map_color.green(); rp.b = map_color.blue();
                map_rgb->push_back(rp);
            }
            map_rgb->width = map_rgb->size(); map_rgb->height = 1;
            viewer.addCloud("map", map_rgb);

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
        } else {
            // Intensity coloring (default) — pass default QColor so viewer uses intensity handler
            viewer.addCloud("map", accumulated_map);
            viewer.addCloud("scan", latest_cloud, latest_pose);
        }
        latest_cloud.reset();

        viewer.setWindowTitle(QString("Livox SLAM | Frame %1 | Map: %2 pts")
            .arg(frame_count)
            .arg(accumulated_map->size()));

        viewer.refreshView();
    });
    update_timer.start(33);

    int ret = app.exec();

    running = false;
    receiver.stop();

    std::cout << "\nFinal: " << slam.getFrameCount() << " frames, "
              << accumulated_map->size() << " map points\n"
              << "Database saved to: " << slam.getDatabasePath() << "\n";

    return ret;
}
