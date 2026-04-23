#pragma once

#include <opencv2/core.hpp>
#include <utility>
#include <vector>

struct PlannerConfig {
    float inflation_radius = 0.3f;  // metres — expand obstacles by robot footprint radius
    bool  allow_unknown    = false; // treat unknown cells as traversable
};

class Planner {
public:
    // A* path plan on an occupancy grid (CV_8SC1: -1=unknown, 0=free, 100=occupied).
    // Returns world-space (x, y) waypoints from start to goal, simplified via line-of-sight.
    // Returns empty vector if no path exists.
    static std::vector<std::pair<float,float>> plan(
        const cv::Mat &grid,
        float xMin, float yMin, float cellSize,
        std::pair<float,float> start,
        std::pair<float,float> goal,
        const PlannerConfig &cfg = PlannerConfig{});
};
