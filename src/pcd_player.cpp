#include "pcd_player.h"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <map>
#include <string>
#include <thread>

#include <opencv2/imgcodecs.hpp>

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

bool PcdPlayer::loadImu(const std::string &dir, bool use_orientation) {
    return imu_reader_.load(dir, use_orientation);
}

bool PcdPlayer::loadRgb(const std::string &dir) {
    rgb_files_.clear();

    if (!fs::is_directory(dir)) {
        std::cerr << "[PcdPlayer] rgb dir not found: " << dir << "\n";
        return false;
    }

    // Collect color and depth files by timestamp prefix.
    // Filenames from fetch-data: {ts}_color.{ext} and {ts}_depth.dep
    std::map<uint64_t, RGBDFile> by_ts;

    for (const auto &entry : fs::directory_iterator(dir)) {
        std::string stem = entry.path().stem().string();   // e.g. "1234567890_color"
        std::string ext  = entry.path().extension().string(); // e.g. ".jpeg"

        uint64_t ts = 0;
        try { ts = static_cast<uint64_t>(std::stoull(stem)); }
        catch (...) { continue; }
        if (ts == 0) continue;

        // Determine type from suffix after the first underscore
        auto under = stem.find('_');
        std::string suffix = (under != std::string::npos) ? stem.substr(under + 1) : "";

        auto &rf = by_ts[ts];
        rf.timestamp_ns = ts;
        if (suffix == "color") {
            rf.color_path = entry.path().string();
        } else if (suffix == "depth") {
            rf.depth_path = entry.path().string();
        }
    }

    for (auto &[ts, rf] : by_ts) {
        if (!rf.color_path.empty() || !rf.depth_path.empty())
            rgb_files_.push_back(rf);
    }

    std::sort(rgb_files_.begin(), rgb_files_.end(),
              [](const RGBDFile &a, const RGBDFile &b) { return a.timestamp_ns < b.timestamp_ns; });

    std::cout << "[PcdPlayer] loaded " << rgb_files_.size() << " RGBD frame(s) from " << dir << "\n";
    return !rgb_files_.empty();
}

void PcdPlayer::play(FrameCallback pcdCb, ImuCallback imuCb, int delay_ms, RGBDCallback rgbdCb) {
    const auto &imu = imu_reader_.readings();
    size_t imu_idx = 0;
    size_t rgb_idx = 0;  // index of the next undelivered RGBD frame

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

        // Advance RGBD index to the frame closest to this PCD timestamp
        if (rgbdCb && !rgb_files_.empty()) {
            while (rgb_idx + 1 < rgb_files_.size() &&
                   rgb_files_[rgb_idx + 1].timestamp_ns <= f.timestamp_ns) {
                rgb_idx++;
            }
            const auto &rf = rgb_files_[rgb_idx];
            RGBDReading reading;
            reading.timestamp_ns = rf.timestamp_ns;
            if (!rf.color_path.empty())
                reading.rgb = cv::imread(rf.color_path, cv::IMREAD_COLOR);
            if (!rf.depth_path.empty()) {
                // .dep is the Viam raw depth format — decode via libviamcamera if available,
                // otherwise treat as 16-bit PNG (millimeters) and convert to float meters.
                cv::Mat raw = cv::imread(rf.depth_path, cv::IMREAD_ANYDEPTH);
                if (!raw.empty()) {
                    raw.convertTo(reading.depth, CV_32FC1, 0.001f);
                }
            }
            rgbdCb(reading);
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
