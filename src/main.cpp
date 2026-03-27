#include <atomic>
#include <csignal>
#include <cstring>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include <QApplication>
#include <QTimer>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <rtabmap/gui/CloudViewer.h>
#include <rtabmap/core/Transform.h>

#include "livox_receiver.h"
#include "slam_pipeline.h"

static std::atomic<bool> running{true};

static void signalHandler(int) {
    running = false;
}

static void printUsage(const char *prog) {
    std::cerr << "Usage: " << prog << " --sensor-ip <ip> --host-ip <ip> [--headless]\n";
}

int main(int argc, char *argv[]) {
    std::string sensor_ip = "192.168.1.196";
    std::string host_ip = "192.168.1.10";
    bool headless = false;

    // Parse args before QApplication consumes them
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--sensor-ip") == 0 && i + 1 < argc) {
            sensor_ip = argv[++i];
        } else if (strcmp(argv[i], "--host-ip") == 0 && i + 1 < argc) {
            host_ip = argv[++i];
        } else if (strcmp(argv[i], "--headless") == 0) {
            headless = true;
        }
    }

    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    std::cout << "Livox → RTAB-Map SLAM\n"
              << "  Sensor: " << sensor_ip << "\n"
              << "  Host:   " << host_ip << "\n"
              << "  Mode:   " << (headless ? "headless" : "GUI") << "\n\n";

    // Init SLAM pipeline
    SlamPipeline slam;
    if (!slam.init()) {
        std::cerr << "Failed to init SLAM pipeline\n";
        return 1;
    }

    if (headless) {
        // Headless mode — no Qt, just process and print
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

    // GUI mode — Qt runs on main thread, Livox SDK on background thread
    QApplication app(argc, argv);

    rtabmap::CloudViewer viewer;
    viewer.setWindowTitle("Livox Mid-360 SLAM");
    viewer.setBackgroundColor(QColor(20, 20, 20));
    viewer.setGridShown(true);
    viewer.resize(1280, 720);
    viewer.show();

    // Shared state between SDK callback thread and Qt main thread
    std::mutex cloud_mu;
    pcl::PointCloud<pcl::PointXYZI>::Ptr latest_cloud;
    rtabmap::Transform latest_pose;
    pcl::PointCloud<pcl::PointXYZI>::Ptr accumulated_map(new pcl::PointCloud<pcl::PointXYZI>);
    int frame_count = 0;

    // Start Livox receiver — callback runs on SDK thread
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

    // Timer to update the viewer from the main thread (30Hz)
    QTimer update_timer;
    QObject::connect(&update_timer, &QTimer::timeout, [&]() {
        if (!running) {
            app.quit();
            return;
        }

        std::lock_guard<std::mutex> lock(cloud_mu);
        if (!latest_cloud) return;

        // Transform current cloud by pose and add to accumulated map
        for (const auto &p : *latest_cloud) {
            pcl::PointXYZI tp;
            float x = p.x, y = p.y, z = p.z;
            // Apply pose transform
            const float *data = latest_pose.data();
            tp.x = data[0]*x + data[1]*y + data[2]*z + data[3];
            tp.y = data[4]*x + data[5]*y + data[6]*z + data[7];
            tp.z = data[8]*x + data[9]*y + data[10]*z + data[11];
            tp.intensity = p.intensity;
            accumulated_map->push_back(tp);
        }

        accumulated_map->width = accumulated_map->size();
        accumulated_map->height = 1;

        // Update viewer with accumulated map
        viewer.addCloud("map", accumulated_map, rtabmap::Transform::getIdentity(), QColor(180, 180, 180));

        // Show current scan in a different color
        viewer.addCloud("scan", latest_cloud, latest_pose, QColor(0, 255, 0));

        latest_cloud.reset();

        viewer.setWindowTitle(QString("Livox SLAM | Frame %1 | Map: %2 pts")
            .arg(frame_count)
            .arg(accumulated_map->size()));
    });
    update_timer.start(33); // ~30Hz

    int ret = app.exec();

    running = false;
    receiver.stop();

    std::cout << "\nFinal: " << slam.getFrameCount() << " frames, "
              << accumulated_map->size() << " map points\n";

    return ret;
}
