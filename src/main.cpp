#include <atomic>
#include <csignal>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

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

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--sensor-ip") == 0 && i + 1 < argc) {
            sensor_ip = argv[++i];
        } else if (strcmp(argv[i], "--host-ip") == 0 && i + 1 < argc) {
            host_ip = argv[++i];
        } else if (strcmp(argv[i], "--headless") == 0) {
            headless = true;
        } else {
            printUsage(argv[0]);
            return 1;
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

    // Init Livox receiver — feed frames into SLAM pipeline
    LivoxReceiver receiver(sensor_ip, host_ip);
    bool ok = receiver.start(
        [&slam](pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, uint64_t ts) {
            slam.processCloud(cloud, ts);
        });

    if (!ok) {
        std::cerr << "Failed to start Livox receiver\n";
        return 1;
    }

    std::cout << "Running... Press Ctrl+C to stop.\n\n";

    while (running) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    std::cout << "\nShutting down...\n";
    receiver.stop();

    std::cout << "Final stats: " << slam.getFrameCount() << " frames, "
              << slam.getMapSize() << " map nodes\n";

    return 0;
}
