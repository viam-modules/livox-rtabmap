#include "pcd_player.h"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <string>
#include <thread>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace fs = std::filesystem;

bool PcdPlayer::load(const std::string &dir) {
    files_.clear();
    stopped_ = false;

    if (!fs::is_directory(dir)) {
        std::cerr << "[PcdPlayer] not a directory: " << dir << "\n";
        return false;
    }

    for (const auto &entry : fs::directory_iterator(dir)) {
        if (entry.path().extension() != ".pcd") continue;

        PcdFile f;
        f.path = entry.path().string();

        std::string stem = entry.path().stem().string();
        try {
            f.timestamp_ns = static_cast<uint64_t>(std::stoull(stem));
        } catch (...) {
            f.timestamp_ns = 0;
        }

        files_.push_back(f);
    }

    if (files_.empty()) {
        std::cerr << "[PcdPlayer] no .pcd files found in " << dir << "\n";
        return false;
    }

    std::sort(files_.begin(), files_.end(), [](const PcdFile &a, const PcdFile &b) {
        if (a.timestamp_ns != b.timestamp_ns) return a.timestamp_ns < b.timestamp_ns;
        return a.path < b.path;
    });

    if (files_[0].timestamp_ns == 0) {
        uint64_t ts = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());
        for (auto &f : files_) {
            f.timestamp_ns = ts;
            ts += 100'000'000;
        }
    }

    std::cout << "[PcdPlayer] loaded " << files_.size() << " PCD file(s) from " << dir << "\n";
    return true;
}

bool PcdPlayer::loadImu(const std::string &dir) {
    return imu_reader_.load(dir);
}

void PcdPlayer::play(FrameCallback pcdCb, ImuCallback imuCb, int delay_ms) {
    const auto &imu = imu_reader_.readings();
    size_t imu_idx = 0;

    int pcd_idx = 0;
    for (const auto &f : files_) {
        if (stopped_) break;

        // Flush all IMU readings that occurred before this PCD frame
        if (imuCb) {
            while (imu_idx < imu.size() && imu[imu_idx].timestamp_ns <= f.timestamp_ns) {
                imuCb(imu[imu_idx]);
                imu_idx++;
            }
        }

        auto cloud_xyz = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(f.path, *cloud_xyz) != 0) {
            std::cerr << "[PcdPlayer] failed to load: " << f.path << "\n";
            continue;
        }
        auto cloud_xyzi = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        cloud_xyzi->reserve(cloud_xyz->size());
        for (const auto &p : *cloud_xyz) {
            cloud_xyzi->emplace_back(p.x, p.y, p.z, 0.0f);
        }
        cloud_xyzi->width = cloud_xyzi->size();
        cloud_xyzi->height = 1;
        pcdCb(cloud_xyzi, f.timestamp_ns);

        if (++pcd_idx % 10 == 0) {
            std::cout << "[PcdPlayer] played " << pcd_idx << " / " << files_.size() << " frames\n";
        }

        if (delay_ms > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
        }
    }
    std::cout << "[PcdPlayer] playback complete (" << pcd_idx << " frames, "
              << imu_idx << " IMU readings)\n";
}
