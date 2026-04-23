#pragma once

#include <atomic>
#include <functional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <rtabmap/core/Transform.h>

class Navigator {
public:
    struct Config {
        float arrival_radius    = 0.5f;   // m — distance to switch to next waypoint
        float heading_threshold = 0.15f;  // rad — below this, drive forward
        float forward_speed     = 0.3f;   // m/s
        float angular_gain      = 1.5f;   // (rad/s) / rad of error
        float angular_max       = 0.8f;   // rad/s clamp
        float update_hz         = 10.0f;
    };

    Navigator();
    explicit Navigator(const Config &cfg);
    ~Navigator();

    void setWaypoints(const std::vector<std::pair<float, float>> &waypoints);
    bool start(std::function<rtabmap::Transform()> get_pose);
    void stop();
    bool isRunning() const { return running_; }

private:
    void loop();

    Config cfg_;
    std::vector<std::pair<float, float>> waypoints_;
    std::function<rtabmap::Transform()> get_pose_;
    std::atomic<bool> running_{false};
    std::thread thread_;
};
