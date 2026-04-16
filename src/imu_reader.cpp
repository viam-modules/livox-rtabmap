#include "imu_reader.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>

#include <nlohmann/json.hpp>

using json = nlohmann::json;
namespace fs = std::filesystem;

static constexpr double DEG_TO_RAD = M_PI / 180.0;
static constexpr double POLE_EPSILON = 1e-4;

// Convert Viam OrientationVectorDegrees to quaternion (qx, qy, qz, qw).
//
// Viam's OV is NOT standard axis-angle. (ox, oy, oz) is a point on the unit
// sphere (the direction the sensor points), and theta is an in-line rotation
// around that direction. Conversion uses ZYZ Euler angles, matching the RDK:
//   lat = acos(oz)
//   lon = atan2(oy, ox)   (0 at poles)
//   q   = qZ(lon) * qY(lat) * qZ(theta)
static void viamOVDToQuat(double ox, double oy, double oz, double theta_deg,
                          double &qx, double &qy, double &qz, double &qw) {
    // Normalize direction vector
    double norm = std::sqrt(ox*ox + oy*oy + oz*oz);
    if (norm < 1e-9) { qx = 0; qy = 0; qz = 0; qw = 1; return; }
    ox /= norm; oy /= norm; oz /= norm;

    double theta = theta_deg * DEG_TO_RAD;
    double lat   = std::acos(std::max(-1.0, std::min(1.0, oz)));
    double lon   = (1.0 - std::abs(oz) > POLE_EPSILON) ? std::atan2(oy, ox) : 0.0;

    // Build quaternions for each ZYZ component: q = (x, y, z, w)
    auto qZ = [](double a, double (&q)[4]) {
        q[0] = 0; q[1] = 0; q[2] = std::sin(a/2); q[3] = std::cos(a/2);
    };
    auto qY = [](double a, double (&q)[4]) {
        q[0] = 0; q[1] = std::sin(a/2); q[2] = 0; q[3] = std::cos(a/2);
    };
    // Hamilton product: out = a * b
    auto qMul = [](const double a[4], const double b[4], double out[4]) {
        out[0] = a[3]*b[0] + a[0]*b[3] + a[1]*b[2] - a[2]*b[1];
        out[1] = a[3]*b[1] - a[0]*b[2] + a[1]*b[3] + a[2]*b[0];
        out[2] = a[3]*b[2] + a[0]*b[1] - a[1]*b[0] + a[2]*b[3];
        out[3] = a[3]*b[3] - a[0]*b[0] - a[1]*b[1] - a[2]*b[2];
    };

    double q1[4], q2[4], q3[4], tmp[4], result[4];
    qZ(lon,   q1);
    qY(lat,   q2);
    qZ(theta, q3);
    qMul(q1, q2, tmp);
    qMul(tmp, q3, result);

    qx = result[0]; qy = result[1]; qz = result[2]; qw = result[3];
}

bool ImuReader::load(const std::string &dir, bool use_orientation) {
    readings_.clear();

    if (!fs::is_directory(dir)) {
        std::cerr << "[ImuReader] not a directory: " << dir << "\n";
        return false;
    }

    int loaded = 0, skipped = 0;
    for (const auto &entry : fs::directory_iterator(dir)) {
        if (entry.path().extension() != ".json") continue;

        std::ifstream f(entry.path());
        if (!f) { skipped++; continue; }

        json j;
        try { j = json::parse(f); } catch (...) { skipped++; continue; }

        // Timestamp from filename: {timestamp_ns}_{component}.json
        uint64_t ts = 0;
        try { ts = std::stoull(entry.path().stem().string()); } catch (...) {}
        if (ts == 0) { skipped++; continue; }

        const auto &data = j.value("data", json::object());
        ImuReading r;
        r.timestamp_ns = ts;

        // Angular velocity: stored as {"angular_velocity": {"x", "y", "z"}} in deg/s
        if (data.contains("angular_velocity")) {
            const auto &av = data["angular_velocity"];
            r.has_gyro = true;
            r.gx = av.value("x", 0.0) * DEG_TO_RAD;
            r.gy = av.value("y", 0.0) * DEG_TO_RAD;
            r.gz = av.value("z", 0.0) * DEG_TO_RAD;
        }

        // Linear acceleration: stored as {"linear_acceleration": {"x", "y", "z"}} in m/s²
        if (data.contains("linear_acceleration")) {
            const auto &la = data["linear_acceleration"];
            r.has_accel = true;
            r.ax = la.value("x", 0.0);
            r.ay = la.value("y", 0.0);
            r.az = la.value("z", 0.0);
        }

        // Orientation: disabled by default — Viam's frame convention doesn't match RTAB-Map's
        if (use_orientation && data.contains("orientation")) {
            const auto &o = data["orientation"];
            r.has_orientation = true;
            viamOVDToQuat(
                o.value("o_x",   0.0),
                o.value("o_y",   0.0),
                o.value("o_z",   1.0),
                o.value("theta", 0.0),
                r.qx, r.qy, r.qz, r.qw);
        }

        if (!r.has_gyro && !r.has_accel && !r.has_orientation) { skipped++; continue; }

        readings_.push_back(r);
        loaded++;
    }

    std::sort(readings_.begin(), readings_.end(), [](const ImuReading &a, const ImuReading &b) {
        return a.timestamp_ns < b.timestamp_ns;
    });

    // Merge readings with complementary fields that arrive close together.
    // Viam data capture stores each SDK method (gyro, accel, orientation) as a
    // separate file — this re-pairs them into synchronized readings like the
    // live path does. Stop a group when a duplicate field type is seen (new sample).
    const uint64_t kMergeWindowNs = 5'000'000; // 5ms — well under the 10ms IMU period
    std::vector<ImuReading> merged;
    merged.reserve(readings_.size());
    for (size_t i = 0; i < readings_.size(); ) {
        ImuReading m = readings_[i];
        size_t j = i + 1;
        while (j < readings_.size() &&
               readings_[j].timestamp_ns - readings_[i].timestamp_ns <= kMergeWindowNs) {
            const auto &r = readings_[j];
            // Stop if this reading duplicates a field we already have — it's a new sample
            if ((r.has_gyro        && m.has_gyro)        ||
                (r.has_accel       && m.has_accel)       ||
                (r.has_orientation && m.has_orientation)) break;
            if (r.has_gyro)        { m.has_gyro = true;  m.gx = r.gx; m.gy = r.gy; m.gz = r.gz; }
            if (r.has_accel)       { m.has_accel = true; m.ax = r.ax; m.ay = r.ay; m.az = r.az; }
            if (r.has_orientation) { m.has_orientation = true; m.qx = r.qx; m.qy = r.qy; m.qz = r.qz; m.qw = r.qw; }
            j++;
        }
        merged.push_back(m);
        i = j;
    }

    int paired = 0;
    for (const auto &r : merged) if (r.has_gyro && r.has_accel) paired++;

    std::cout << "[ImuReader] loaded " << loaded << " reading(s) from " << dir;
    if (skipped) std::cout << " (" << skipped << " skipped)";
    std::cout << " → " << merged.size() << " merged (" << paired << " with gyro+accel)\n";

    readings_ = std::move(merged);
    return !readings_.empty();
}
