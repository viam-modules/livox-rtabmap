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

        // Try to parse timestamp from filename: {timestamp_ns}_{component}.pcd
        std::string stem = entry.path().stem().string();
        try {
            f.timestamp_ns = static_cast<uint64_t>(std::stoull(stem));
        } catch (...) {
            // Non-numeric prefix — timestamp will be fixed up after sorting
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

    // If timestamps weren't parseable, assign synthetic 10 Hz timestamps
    if (files_[0].timestamp_ns == 0) {
        uint64_t ts = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());
        for (auto &f : files_) {
            f.timestamp_ns = ts;
            ts += 100'000'000; // 100ms = 10 Hz
        }
    }

    std::cout << "[PcdPlayer] loaded " << files_.size() << " PCD file(s) from " << dir << "\n";
    return true;
}

void PcdPlayer::play(FrameCallback cb, int delay_ms) {
    int idx = 0;
    for (const auto &f : files_) {
        if (stopped_) break;

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
        cb(cloud_xyzi, f.timestamp_ns);

        if (++idx % 10 == 0) {
            std::cout << "[PcdPlayer] played " << idx << " / " << files_.size() << " frames\n";
        }

        if (delay_ms > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
        }
    }
    std::cout << "[PcdPlayer] playback complete (" << idx << " frames)\n";
}
