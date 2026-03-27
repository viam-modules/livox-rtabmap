#include <atomic>
#include <csignal>
#include <cstring>
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
    viewer.show();

    std::mutex cloud_mu;
    pcl::PointCloud<pcl::PointXYZI>::Ptr latest_cloud;
    rtabmap::Transform latest_pose;
    pcl::PointCloud<pcl::PointXYZI>::Ptr accumulated_map(new pcl::PointCloud<pcl::PointXYZI>);
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

        viewer.addCloud("map", accumulated_map, rtabmap::Transform::getIdentity(), QColor(180, 180, 180));
        viewer.addCloud("scan", latest_cloud, latest_pose, QColor(0, 255, 0));
        latest_cloud.reset();

        viewer.setWindowTitle(QString("Livox SLAM | Frame %1 | Map: %2 pts")
            .arg(frame_count)
            .arg(accumulated_map->size()));
    });
    update_timer.start(33);

    int ret = app.exec();

    running = false;
    receiver.stop();

    std::cout << "\nFinal: " << slam.getFrameCount() << " frames, "
              << accumulated_map->size() << " map points\n";

    return ret;
}
