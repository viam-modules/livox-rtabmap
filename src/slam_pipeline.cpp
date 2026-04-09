#include "slam_pipeline.h"

#include <chrono>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <iostream>

#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/OdometryInfo.h>
#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/Link.h>
#include <rtabmap/core/IMU.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/utilite/ULogger.h>

#include <opencv2/core.hpp>

#include "imu_reader.h"

using json = nlohmann::json;

SlamPipeline::SlamPipeline() {}

SlamPipeline::~SlamPipeline() {
    if (rtabmap_) {
        rtabmap_->close();
    }
}

bool SlamPipeline::init(const json &config) {
    json icp = config.value("icp", json::object());
    json rtab = config.value("rtabmap", json::object());

    rtabmap::ParametersMap params;
    // 0=Frame-to-Map (more robust, resists ghosts), 1=Frame-to-Frame
    params.insert({rtabmap::Parameters::kOdomStrategy(),
        std::to_string(icp.value("odom_strategy", 0))});
    params.insert({rtabmap::Parameters::kRegStrategy(), "1"});
    // Kalman filter smooths odometry (0=none, 1=Kalman, 2=Particle)
    params.insert({rtabmap::Parameters::kOdomFilteringStrategy(),
        std::to_string(icp.value("filtering_strategy", 1))});
    params.insert({rtabmap::Parameters::kOdomGuessMotion(), "true"});
    params.insert({rtabmap::Parameters::kOdomKalmanProcessNoise(),
        std::to_string(icp.value("kalman_process_noise", 0.001))});
    params.insert({rtabmap::Parameters::kOdomKalmanMeasurementNoise(),
        std::to_string(icp.value("kalman_measurement_noise", 0.01))});

    // ICP parameters from config
    params.insert({rtabmap::Parameters::kIcpPointToPlane(),
        icp.value("point_to_plane", true) ? "true" : "false"});
    params.insert({rtabmap::Parameters::kIcpPointToPlaneK(),
        std::to_string(icp.value("point_to_plane_k", 10))});
    params.insert({rtabmap::Parameters::kIcpPointToPlaneRadius(),
        std::to_string(icp.value("point_to_plane_radius", 0.0))});
    params.insert({rtabmap::Parameters::kIcpPointToPlaneGroundNormalsUp(),
        std::to_string(icp.value("point_to_plane_ground_normals_up", 0.8))});
    params.insert({rtabmap::Parameters::kIcpPointToPlaneMinComplexity(),
        std::to_string(icp.value("point_to_plane_min_complexity", 0.02))});
    params.insert({rtabmap::Parameters::kIcpVoxelSize(),
        std::to_string(icp.value("voxel_size", 0.03))});
    params.insert({rtabmap::Parameters::kIcpMaxCorrespondenceDistance(),
        std::to_string(icp.value("max_correspondence_distance", 0.25))});
    params.insert({rtabmap::Parameters::kIcpCorrespondenceRatio(),
        std::to_string(icp.value("correspondence_ratio", 0.3))});
    params.insert({rtabmap::Parameters::kIcpReciprocalCorrespondences(),
        icp.value("reciprocal_correspondences", true) ? "true" : "false"});
    params.insert({rtabmap::Parameters::kIcpIterations(),
        std::to_string(icp.value("iterations", 40))});
    params.insert({rtabmap::Parameters::kIcpEpsilon(),
        std::to_string(icp.value("epsilon", 0.0005))});
    params.insert({rtabmap::Parameters::kIcpMaxTranslation(),
        std::to_string(icp.value("max_translation", 0.5))});
    params.insert({rtabmap::Parameters::kIcpMaxRotation(),
        std::to_string(icp.value("max_rotation", 0.3))});
    params.insert({rtabmap::Parameters::kIcpOutlierRatio(),
        std::to_string(icp.value("outlier_ratio", 0.85))});

    // Frame-to-Map parameters
    params.insert({rtabmap::Parameters::kOdomF2MScanMaxSize(),
        std::to_string(icp.value("f2m_scan_max_size", 15000))});
    params.insert({rtabmap::Parameters::kOdomF2MScanSubtractRadius(),
        std::to_string(icp.value("f2m_scan_subtract_radius", 0.03))});

    // RTAB-Map parameters from config
    params.insert({rtabmap::Parameters::kRtabmapDetectionRate(),
        std::to_string(rtab.value("detection_rate", 0))});
    params.insert({rtabmap::Parameters::kRGBDProximityBySpace(),
        rtab.value("proximity_by_space", true) ? "true" : "false"});
    params.insert({rtabmap::Parameters::kRGBDLinearUpdate(),
        std::to_string(rtab.value("linear_update", 0.1))});
    params.insert({rtabmap::Parameters::kRGBDAngularUpdate(),
        std::to_string(rtab.value("angular_update", 0.1))});

    odom_.reset(rtabmap::Odometry::create(params));
    if (!odom_) {
        std::cerr << "[SLAM] Failed to create odometry\n";
        return false;
    }

    use_imu_ = config.value("use_imu_prior", false);
    min_range_ = config.value("min_range", 0.0f);
    max_range_ = config.value("max_range", 0.0f);
    max_accel_ = config.value("max_accel", 0.0f);
    accel_holdoff_ = config.value("accel_holdoff", 1.0f);

    db_path_ = config.value("database_path", "");
    if (db_path_.empty()) {
        auto now = std::chrono::system_clock::now();
        auto t = std::chrono::system_clock::to_time_t(now);
        char buf[32];
        std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", std::localtime(&t));
        db_path_ = std::string("slam_") + buf + ".db";
    }

    rtabmap_ = std::make_unique<rtabmap::Rtabmap>();
    rtabmap_->init(params, db_path_);

    // Occupancy grid maker — 2D projection with ray tracing for free space
    rtabmap::ParametersMap grid_params;
    grid_params.insert({rtabmap::Parameters::kGridCellSize(),
        std::to_string(config.value("grid_resolution", 0.05f))});
    grid_params.insert({rtabmap::Parameters::kGridRayTracing(), "true"});
    grid_params.insert({rtabmap::Parameters::kGrid3D(), "false"});
    grid_maker_.parseParameters(grid_params);
    occ_grid_ = std::make_unique<rtabmap::OccupancyGrid>(&grid_cache_, grid_params);

    std::cout << "[SLAM] Pipeline initialized\n"
              << "  Database: " << db_path_ << "\n"
              << "  ICP: voxel=" << icp.value("voxel_size", 0.03)
              << " corr=" << icp.value("max_correspondence_distance", 0.25)
              << " iter=" << icp.value("iterations", 40) << "\n";
    return true;
}

bool SlamPipeline::processCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, uint64_t timestamp_ns,
                                pcl::PointCloud<pcl::PointXYZI>::Ptr *filtered_cloud) {
    std::lock_guard<std::mutex> lock(slam_mutex_);
    if (!odom_ || !rtabmap_) return false;

    if (max_accel_ > 0) {
        double scan_time = static_cast<double>(timestamp_ns) / 1e9;
        double time_since_high = scan_time - last_high_accel_time_.load();
        if (current_accel_.load() > max_accel_ || time_since_high < accel_holdoff_) {
            if (current_accel_.load() > max_accel_) {
                std::cerr << "[SLAM] Scan rejected: accel " << current_accel_.load()
                          << " m/s² > max " << max_accel_ << "\n";
            } else {
                std::cerr << "[SLAM] Scan rejected: holdoff " << std::fixed
                          << std::setprecision(1) << time_since_high << "s / "
                          << accel_holdoff_ << "s\n";
            }
            return false;
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz(new pcl::PointCloud<pcl::PointXYZ>);
    auto xyzi_filtered = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    xyz->reserve(cloud->size());
    xyzi_filtered->reserve(cloud->size());
    float min_r2 = min_range_ * min_range_;
    float max_r2 = max_range_ * max_range_;
    for (const auto &p : *cloud) {
        float r2 = p.x*p.x + p.y*p.y + p.z*p.z;
        if (min_range_ > 0 && r2 < min_r2) continue;
        if (max_range_ > 0 && r2 > max_r2) continue;
        xyz->push_back(pcl::PointXYZ(p.x, p.y, p.z));
        xyzi_filtered->push_back(p);
    }
    if (filtered_cloud) {
        *filtered_cloud = xyzi_filtered;
    }

    double stamp = static_cast<double>(timestamp_ns) / 1e9;
    rtabmap::LaserScan scan = rtabmap::util3d::laserScanFromPointCloud(*xyz);
    rtabmap::SensorData data(scan, cv::Mat(), cv::Mat(), rtabmap::CameraModel(), 0, stamp);

    rtabmap::OdometryInfo odom_info;
    rtabmap::Transform pose = odom_->process(data, &odom_info);

    if (pose.isNull()) {
        std::cerr << "[SLAM] Odometry failed on frame " << frame_count_ << "\n";
        return false;
    }

    current_pose_ = pose;
    frame_count_++;

    rtabmap_->process(data, pose);

    // Add local occupancy grid for this node if a new node was created
    int nodeId = rtabmap_->getLastLocationId();
    if (nodeId > 0 && nodeId != last_grid_node_id_) {
        last_grid_node_id_ = nodeId;
        cv::Mat ground, obstacles, empty;
        cv::Point3f viewPoint(0, 0, 0);
        grid_maker_.createLocalMap(scan, pose, ground, obstacles, empty, viewPoint);
        grid_cache_.add(nodeId, ground, obstacles, empty,
                        grid_maker_.getCellSize(), viewPoint);
    }

    if (frame_count_ % 10 == 0) {
        std::cout << "[SLAM] Frame " << frame_count_
                  << " | pose: " << pose.prettyPrint() << "\n";
    }

    return true;
}

void SlamPipeline::processImu(const ImuReading &imu) {
    std::lock_guard<std::mutex> lock(slam_mutex_);
    if (!odom_) return;

    // Update last-known state — each Viam reading carries one measurement type
    if (imu.has_gyro)        { last_gyro_        = {imu.gx, imu.gy, imu.gz}; has_gyro_        = true; }
    if (imu.has_accel)       { last_accel_       = {imu.ax, imu.ay, imu.az}; has_accel_       = true; }
    if (imu.has_orientation) { last_orientation_ = {imu.qx, imu.qy, imu.qz, imu.qw}; has_orientation_ = true; }

    // Update accel deviation for scan rejection (same logic as processIMU)
    if (imu.has_accel) {
        const double g = 9.80665;
        double mag = std::sqrt(imu.ax*imu.ax + imu.ay*imu.ay + imu.az*imu.az);
        float accel_dev = static_cast<float>(std::abs(mag - g));
        current_accel_.store(accel_dev);
        if (max_accel_ > 0 && accel_dev > max_accel_) {
            double stamp = static_cast<double>(imu.timestamp_ns) / 1e9;
            last_high_accel_time_.store(stamp);
        }
    }

    // Need at least gyro or accel to be useful for odometry pre-integration
    if (!use_imu_ || (!has_gyro_ && !has_accel_)) return;

    cv::Mat cov3 = cv::Mat::eye(3, 3, CV_64FC1) * 1e-3;
    cv::Mat cov3_large = cv::Mat::eye(3, 3, CV_64FC1) * 9999.0;

    rtabmap::IMU rtImu(
        last_orientation_,
        has_orientation_ ? cov3 : cov3_large,
        last_gyro_,
        has_gyro_ ? cov3 : cov3_large,
        last_accel_,
        has_accel_ ? cov3 : cov3_large
    );

    double stamp = static_cast<double>(imu.timestamp_ns) / 1e9;
    rtabmap::SensorData data;
    data.setStamp(stamp);
    data.setIMU(rtImu);
    odom_->process(data, nullptr);
}

rtabmap::Transform SlamPipeline::getPose() const {
    return current_pose_;
}

int SlamPipeline::getMapSize() const {
    if (rtabmap_) {
        return rtabmap_->getLastLocationId();
    }
    return 0;
}

void SlamPipeline::processIMU(float gyro_x, float gyro_y, float gyro_z,
                              float acc_x, float acc_y, float acc_z,
                              uint64_t timestamp_ns) {
    const double g = 9.80665;
    ImuReading imu;
    imu.timestamp_ns = timestamp_ns;
    imu.has_gyro  = true;
    imu.gx = gyro_x; imu.gy = gyro_y; imu.gz = gyro_z; // rad/s
    imu.has_accel = true;
    imu.ax = acc_x * g; imu.ay = acc_y * g; imu.az = acc_z * g; // g → m/s²
    processImu(imu);
}

cv::Mat SlamPipeline::getOccupancyGrid(float &xMin, float &yMin, float &cellSize) {
    std::lock_guard<std::mutex> lock(slam_mutex_);
    if (!rtabmap_ || !occ_grid_) return cv::Mat();

    std::map<int, rtabmap::Transform> poses;
    std::multimap<int, rtabmap::Link> constraints;
    rtabmap_->getGraph(poses, constraints, /*optimized=*/true, /*global=*/true);

    occ_grid_->update(poses);

    cellSize = occ_grid_->getCellSize();
    return occ_grid_->getMap(xMin, yMin);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr SlamPipeline::rebuildMap(int map_id) const {
    auto map = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    if (!rtabmap_) return map;

    std::map<int, rtabmap::Signature> signatures;
    std::map<int, rtabmap::Transform> poses;
    std::multimap<int, rtabmap::Link> constraints;
    rtabmap_->get3DMap(signatures, poses, constraints, true, true);

    int included = 0;
    for (auto &[id, pose] : poses) {
        if (pose.isNull()) continue;
        auto it = signatures.find(id);
        if (it == signatures.end()) continue;
        if (map_id >= 0 && it->second.mapId() != map_id) continue;

        rtabmap::LaserScan scan = it->second.sensorData().laserScanRaw();
        if (scan.isEmpty()) {
            it->second.sensorData().uncompressData(nullptr, nullptr, &scan);
        }
        if (scan.isEmpty()) continue;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = rtabmap::util3d::laserScanToPointCloud(scan, pose);
        for (const auto &p : *cloud) {
            pcl::PointXYZI pi;
            pi.x = p.x; pi.y = p.y; pi.z = p.z;
            pi.intensity = 128;
            map->push_back(pi);
        }
        included++;
    }

    map->width = map->size();
    map->height = 1;
    std::cout << "[SLAM] Rebuilt map: " << map->size() << " points from "
              << included << " nodes"
              << (map_id >= 0 ? " (map " + std::to_string(map_id) + ")" : " (all sessions)")
              << "\n";
    return map;
}

int SlamPipeline::largestMapId() const {
    if (!rtabmap_) return -1;
    std::map<int, rtabmap::Signature> signatures;
    std::map<int, rtabmap::Transform> poses;
    std::multimap<int, rtabmap::Link> constraints;
    rtabmap_->get3DMap(signatures, poses, constraints, false, true);

    std::map<int, int> counts;
    for (const auto &[id, sig] : signatures) counts[sig.mapId()]++;

    int best_id = -1, best_n = 0;
    for (const auto &[mid, n] : counts) {
        if (n > best_n) { best_n = n; best_id = mid; }
    }
    return best_id;
}

int SlamPipeline::lastMapId() const {
    if (!rtabmap_) return -1;
    std::map<int, rtabmap::Signature> signatures;
    std::map<int, rtabmap::Transform> poses;
    std::multimap<int, rtabmap::Link> constraints;
    rtabmap_->get3DMap(signatures, poses, constraints, false, true);

    int last = -1;
    for (const auto &[id, sig] : signatures) last = std::max(last, sig.mapId());
    return last;
}
