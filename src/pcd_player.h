#pragma once

#include <functional>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Replays a directory of PCD files (from the fetch-data tool) as if they were
// live Livox frames. Files must be named {timestamp_ns}_{component}.pcd so they
// sort chronologically.
class PcdPlayer {
public:
    using FrameCallback = std::function<void(pcl::PointCloud<pcl::PointXYZI>::Ptr, uint64_t)>;

    // Scans dir for *.pcd files and sorts them.
    // Returns false if no files found.
    bool load(const std::string &dir);

    int frameCount() const { return static_cast<int>(files_.size()); }

    // Calls callback for each frame in order. Blocks until done (or stop() called).
    // delay_ms: milliseconds to wait between frames (0 = as fast as possible).
    void play(FrameCallback cb, int delay_ms = 0);

    void stop() { stopped_ = true; }

private:
    struct PcdFile {
        std::string path;
        uint64_t timestamp_ns = 0; // parsed from filename, or derived from order
    };

    std::vector<PcdFile> files_;
    bool stopped_ = false;
};
