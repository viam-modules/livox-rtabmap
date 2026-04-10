#include "navigator.h"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <thread>

Navigator::Navigator() {}
Navigator::Navigator(const Config &cfg) : cfg_(cfg) {}

Navigator::~Navigator() { stop(); }

void Navigator::setWaypoints(const std::vector<std::pair<float, float>> &waypoints) {
    waypoints_ = waypoints;
}

bool Navigator::start(std::function<rtabmap::Transform()> get_pose) {
    if (running_ || waypoints_.empty()) return false;
    get_pose_ = std::move(get_pose);
    running_ = true;
    thread_ = std::thread(&Navigator::loop, this);
    return true;
}

void Navigator::stop() {
    running_ = false;
    if (thread_.joinable()) thread_.join();
}

static float normalizeAngle(float a) {
    while (a >  M_PI) a -= 2 * M_PI;
    while (a < -M_PI) a += 2 * M_PI;
    return a;
}

void Navigator::loop() {
    const auto interval = std::chrono::duration<float>(1.0f / cfg_.update_hz);
    size_t wp_idx = 0;

    std::cout << "[NAV] Starting — " << waypoints_.size() << " waypoint(s)\n";

    while (running_ && wp_idx < waypoints_.size()) {
        auto t0 = std::chrono::steady_clock::now();

        rtabmap::Transform pose = get_pose_();
        if (pose.isNull()) {
            std::this_thread::sleep_until(t0 + interval);
            continue;
        }

        float roll, pitch, yaw;
        pose.getEulerAngles(roll, pitch, yaw);

        float rx = pose.x(), ry = pose.y();
        auto [wx, wy] = waypoints_[wp_idx];

        float dx   = wx - rx;
        float dy   = wy - ry;
        float dist = std::sqrt(dx * dx + dy * dy);

        if (dist < cfg_.arrival_radius) {
            std::cout << "[NAV] Reached waypoint " << (wp_idx + 1)
                      << "/" << waypoints_.size() << "\n";
            wp_idx++;
            if (wp_idx >= waypoints_.size()) break;
            std::this_thread::sleep_until(t0 + interval);
            continue;
        }

        float target_heading = std::atan2(dy, dx);
        float heading_error  = normalizeAngle(target_heading - yaw);

        float linear  = 0.0f;
        float angular = 0.0f;

        if (std::abs(heading_error) > cfg_.heading_threshold) {
            // Spin in place to align heading
            angular = std::clamp(cfg_.angular_gain * heading_error,
                                 -cfg_.angular_max, cfg_.angular_max);
        } else {
            // Drive forward
            linear = cfg_.forward_speed;
        }

        std::cout << "[NAV] wp=" << (wp_idx + 1)
                  << " dist=" << std::fixed << std::setprecision(2) << dist << "m"
                  << " heading_err=" << std::setprecision(2) << heading_error << "rad"
                  << " → linear=" << linear << " angular=" << angular << "\n";

        std::this_thread::sleep_until(t0 + interval);
    }

    if (running_) {
        std::cout << "[NAV] All waypoints reached. Stopping.\n";
        std::cout << "[NAV] CMD final → linear=0 angular=0\n";
    } else {
        std::cout << "[NAV] Stopped by user. CMD → linear=0 angular=0\n";
    }
    running_ = false;
}
