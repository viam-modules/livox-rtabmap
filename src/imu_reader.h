#pragma once

#include <cstdint>
#include <string>
#include <vector>

// One raw measurement from a Viam IMU JSON file.
// Each file contains exactly one measurement type.
struct ImuReading {
    uint64_t timestamp_ns = 0;

    bool has_gyro = false;
    double gx = 0, gy = 0, gz = 0; // rad/s (converted from Viam's deg/s)

    bool has_accel = false;
    double ax = 0, ay = 0, az = 0; // m/s²

    bool has_orientation = false;
    double qx = 0, qy = 0, qz = 0, qw = 1; // quaternion (converted from Viam axis-angle)
};

class ImuReader {
public:
    // Scans dir for *.json files, parses and sorts by timestamp.
    // Returns false if no files found.
    // use_orientation: include Viam orientation quaternion — disabled by default because
    // Viam's OrientationVectorDegrees frame convention doesn't match RTAB-Map's world frame.
    bool load(const std::string &dir, bool use_orientation = false);

    const std::vector<ImuReading> &readings() const { return readings_; }

private:
    std::vector<ImuReading> readings_;
};
