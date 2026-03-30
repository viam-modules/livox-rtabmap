#pragma once

#include <functional>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "imu_reader.h"

// Replays a directory of PCD files (from the fetch-data tool) as if they were
// live Livox frames. Files must be named {timestamp_ns}_{component}.pcd so they
// sort chronologically. Optionally interleaves IMU readings between frames.
class PcdPlayer {
public:
    using FrameCallback = std::function<void(pcl::PointCloud<pcl::PointXYZI>::Ptr, uint64_t)>;
    using ImuCallback   = std::function<void(const ImuReading &)>;

    // Scans dir for *.pcd files and sorts them.
    // Returns false if no files found.
    bool load(const std::string &dir);

    // Optionally load IMU data to interleave during playback.
    // Returns false if no IMU files found (non-fatal — playback still works).
    bool loadImu(const std::string &dir);

    int frameCount() const { return static_cast<int>(files_.size()); }

    // Calls pcdCb for each frame and (optionally) imuCb for each IMU reading,
    // all in chronological order. Blocks until done (or stop() called).
    // delay_ms: milliseconds to wait between PCD frames (0 = as fast as possible).
    void play(FrameCallback pcdCb, ImuCallback imuCb = nullptr, int delay_ms = 0);

    void stop() { stopped_ = true; }

private:
    struct PcdFile {
        std::string path;
        uint64_t timestamp_ns = 0;
    };

    std::vector<PcdFile> files_;
    ImuReader imu_reader_;
    bool stopped_ = false;
};
