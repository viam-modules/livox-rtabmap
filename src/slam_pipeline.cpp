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
#include <rtabmap/core/odometry/OdometryF2M.h>
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
    params.insert({rtabmap::Parameters::kOdomGuessMotion(),
        icp.value("guess_motion", false) ? "true" : "false"});
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
    // Ground vehicle constraint: restrict odometry to XY plane (no Z drift, no roll/pitch)
    params.insert({rtabmap::Parameters::kOdomHolonomic(),
        icp.value("holonomic", true) ? "true" : "false"});

    // Graph optimizer parameters
    {
        json opt = config.value("optimizer", json::object());
        // 0=TORO, 1=g2o, 2=GTSAM — g2o required for robust mode
        params.insert({rtabmap::Parameters::kOptimizerStrategy(),
            std::to_string(opt.value("strategy", 1))});
        params.insert({rtabmap::Parameters::kOptimizerIterations(),
            std::to_string(opt.value("iterations", 100))});
        // Robust kernel down-weights bad loop closures instead of letting them warp the map
        params.insert({rtabmap::Parameters::kOptimizerRobust(),
            opt.value("robust", true) ? "true" : "false"});
        // Re-run ICP on every consecutive node pair during refineLinks() to correct odometry drift
        params.insert({rtabmap::Parameters::kRGBDNeighborLinkRefining(),
            opt.value("neighbor_link_refining", false) ? "true" : "false"});
    }

    // RTAB-Map parameters from config
    params.insert({rtabmap::Parameters::kRtabmapDetectionRate(),
        std::to_string(rtab.value("detection_rate", 0))});
    params.insert({rtabmap::Parameters::kRGBDProximityBySpace(),
        rtab.value("proximity_by_space", true) ? "true" : "false"});
    params.insert({rtabmap::Parameters::kRGBDLinearUpdate(),
        std::to_string(rtab.value("linear_update", 0.1))});
    params.insert({rtabmap::Parameters::kRGBDAngularUpdate(),
        std::to_string(rtab.value("angular_update", 0.1))});
    // Localization-only mode: match against existing map, don't add new nodes
    // and open the database read-only so the file isn't modified on exit.
    if (config.value("localize_only", false)) {
        params.insert({rtabmap::Parameters::kMemIncrementalMemory(), "false"});
        params.insert({rtabmap::Parameters::kMemLocalizationReadOnly(), "true"});
        // start_at_origin (RGBD/StartAtOrigin):
        //   true  — ignore the last-saved localization pose from the DB.
        //           Needed for setInitialPose() (from initial_pose config) to
        //           actually stick; otherwise rtabmap snaps the correction back
        //           to the stored pose on the first frame.
        //   false — restore the last-saved pose (rtabmap's default).
        // Default: true when initial_pose is configured, false otherwise.
        bool default_start_at_origin =
            config.contains("initial_pose") && !config["initial_pose"].is_null();
        bool start_at_origin = config.value("start_at_origin", default_start_at_origin);
        params.insert({rtabmap::Parameters::kRGBDStartAtOrigin(),
                       start_at_origin ? "true" : "false"});
    }

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
    odom_fail_reset_threshold_ = config.value("odom_fail_reset_threshold", 2);

    // Sensor → base_link extrinsic transform
    {
        json ex = config.value("extrinsics", json::object());
        float tx    = ex.value("x",     0.0f);
        float ty    = ex.value("y",     0.0f);
        float tz    = ex.value("z",     0.0f);
        float roll  = ex.value("roll",  0.0f);
        float pitch = ex.value("pitch", 0.0f);
        float yaw   = ex.value("yaw",   0.0f);
        lidar_to_base_ = rtabmap::Transform(tx, ty, tz, roll, pitch, yaw);
        std::cout << "[SLAM] Lidar extrinsics: xyz=(" << tx << "," << ty << "," << tz
                  << ") rpy=(" << roll << "," << pitch << "," << yaw << ")\n";
    }

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
    // Proof that DB contents are loaded into rtabmap's working memory:
    // getWMSize() reads directly from rtabmap's Memory class. This number
    // is what "match candidates" the SLAM layer has available right now.
    // In localize_only mode this stays constant for the whole session
    // (IncrementalMemory=false prevents growth).
    std::cout << "[SLAM] Rtabmap working memory after init: "
              << rtabmap_->getWMSize() << " nodes from "
              << (db_path_.empty() ? "(none)" : db_path_) << "\n";

    // Initial pose guess for localization. When localizing in a pre-built map
    // rtabmap needs to know roughly where in the map we're starting, otherwise
    // it has to search the whole graph. Accepts either:
    //   "initial_pose": {"x":..,"y":..,"z":..,"roll":..,"pitch":..,"yaw":..}
    // or a 12-element row-major matrix (rtabmap::Transform data layout).
    if (config.contains("initial_pose") && !config["initial_pose"].is_null()) {
        const auto &ip = config["initial_pose"];
        rtabmap::Transform init;
        if (ip.is_object()) {
            float x     = ip.value("x",     0.0f);
            float y     = ip.value("y",     0.0f);
            float z     = ip.value("z",     0.0f);
            float roll  = ip.value("roll",  0.0f);
            float pitch = ip.value("pitch", 0.0f);
            float yaw   = ip.value("yaw",   0.0f);
            init = rtabmap::Transform(x, y, z, roll, pitch, yaw);
        } else if (ip.is_array() && ip.size() == 12) {
            std::array<float, 12> m{};
            for (size_t i = 0; i < 12; i++) m[i] = ip[i].get<float>();
            init = rtabmap::Transform(m[0], m[1], m[2], m[3],
                                      m[4], m[5], m[6], m[7],
                                      m[8], m[9], m[10], m[11]);
        }
        if (!init.isNull()) {
            // Seed odometry immediately so frame 0 starts at this pose.
            odom_->reset(init);
            // Stash for loadMap() to re-apply after the database is loaded —
            // calling setInitialPose before the map is loaded has no effect.
            initial_pose_ = init;
            // Also apply here in case loadMap is never called (pure mapping mode).
            rtabmap_->setInitialPose(init);
            std::cout << "[SLAM] Initial pose set: " << init.prettyPrint() << "\n";
        }
    }

    // Occupancy grid maker — 2D projection with ray tracing for free space
    rtabmap::ParametersMap grid_params;
    grid_params.insert({rtabmap::Parameters::kGridCellSize(),
        std::to_string(config.value("grid_resolution", 0.05f))});
    grid_params.insert({rtabmap::Parameters::kGridRayTracing(), "true"});
    grid_params.insert({rtabmap::Parameters::kGrid3D(), "false"});
    // Height filtering
    grid_params.insert({rtabmap::Parameters::kGridMinGroundHeight(),
        std::to_string(config.value("grid_min_height", 0.0f))});
    grid_params.insert({rtabmap::Parameters::kGridMaxObstacleHeight(),
        std::to_string(config.value("grid_max_height", 0.0f))});
    // Range, segmentation, noise
    grid_params.insert({rtabmap::Parameters::kGridRangeMax(),
        std::to_string(config.value("grid_range_max", 5.0f))});
    grid_params.insert({rtabmap::Parameters::kGridNormalsSegmentation(),
        config.value("grid_normals_segmentation", true) ? "true" : "false"});
    grid_params.insert({rtabmap::Parameters::kGridNoiseFilteringRadius(),
        std::to_string(config.value("grid_noise_radius", 0.0f))});
    grid_maker_.parseParameters(grid_params);
    occ_grid_ = std::make_unique<rtabmap::OccupancyGrid>(&grid_cache_, grid_params);

    std::cout << "[SLAM] Pipeline initialized\n"
              << "  Database: " << db_path_ << "\n"
              << "  Mode: " << (config.value("localize_only", false) ? "LOCALIZE" : "MAPPING") << "\n"
              << "  ICP: voxel=" << icp.value("voxel_size", 0.03)
              << " corr=" << icp.value("max_correspondence_distance", 0.25)
              << " iter=" << icp.value("iterations", 40) << "\n";
    return true;
}

bool SlamPipeline::processCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, uint64_t timestamp_ns,
                                pcl::PointCloud<pcl::PointXYZI>::Ptr *filtered_cloud) {
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

    auto xyzi_filtered = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    xyzi_filtered->reserve(cloud->size());
    float min_r2 = min_range_ * min_range_;
    float max_r2 = max_range_ * max_range_;
    for (const auto &p : *cloud) {
        float r2 = p.x*p.x + p.y*p.y + p.z*p.z;
        if (min_range_ > 0 && r2 < min_r2) continue;
        if (max_range_ > 0 && r2 > max_r2) continue;
        xyzi_filtered->push_back(p);
    }

    // Pre-transform filtered cloud into base_link frame so the viewer and
    // rtabmap see the same geometry. If we left it in raw sensor frame and
    // the viewer multiplied it by latest_pose (which is the base_link pose
    // in the map frame), the live cloud would be shifted by lidar_to_base_
    // relative to everything else (the local map, the loaded map, etc.).
    if (!lidar_to_base_.isIdentity() && !lidar_to_base_.isNull()) {
        auto transformed = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        transformed->reserve(xyzi_filtered->size());
        const float *t = lidar_to_base_.data();
        for (const auto &p : *xyzi_filtered) {
            pcl::PointXYZI tp = p;
            tp.x = t[0]*p.x + t[1]*p.y + t[2]*p.z + t[3];
            tp.y = t[4]*p.x + t[5]*p.y + t[6]*p.z + t[7];
            tp.z = t[8]*p.x + t[9]*p.y + t[10]*p.z + t[11];
            transformed->push_back(tp);
        }
        transformed->width = transformed->size();
        transformed->height = 1;
        xyzi_filtered = transformed;
    }

    if (filtered_cloud) {
        *filtered_cloud = xyzi_filtered;
    }

    double stamp = static_cast<double>(timestamp_ns) / 1e9;
    // xyzi_filtered is already in base_link frame, so pass identity here
    // (do NOT double-apply lidar_to_base_).
    rtabmap::LaserScan scan = rtabmap::util3d::laserScanFromPointCloud(*xyzi_filtered);
    rtabmap::SensorData data(scan, cv::Mat(), cv::Mat(), rtabmap::CameraModel(), 0, stamp);

    rtabmap::Transform pose;
    int nodeId = -1;
    {
        std::lock_guard<std::mutex> lock(slam_mutex_);

        rtabmap::OdometryInfo odom_info;
        pose = odom_->process(data, &odom_info);

        if (pose.isNull()) {
            odom_fail_count_++;
            std::cerr << "[SLAM] Odometry failed on frame " << frame_count_
                      << " (consecutive: " << odom_fail_count_ << ")"
                      << " icp_ratio=" << odom_info.reg.icpInliersRatio
                      << " corr=" << odom_info.reg.icpCorrespondences
                      << " rms=" << odom_info.reg.icpRMS;
            if (!odom_info.reg.rejectedMsg.empty())
                std::cerr << " [" << odom_info.reg.rejectedMsg << "]";
            std::cerr << "\n";
            if (odom_fail_count_ >= odom_fail_reset_threshold_) {
                std::cerr << "[SLAM] Resetting odometry after " << odom_fail_count_ << " failures\n";
                odom_->reset();
                odom_fail_count_ = 0;
            }
            return false;
        }
        odom_fail_count_ = 0;

        frame_count_++;

        rtabmap_->process(data, pose);
        nodeId = rtabmap_->getLastLocationId();

        // Store the map-frame pose, not the raw odom pose. In mapping mode
        // mapCorrection is identity so this is a no-op; in localize-only mode
        // it's the transform that aligns odometry to the loaded map.
        rtabmap::Transform correction = rtabmap_->getMapCorrection();
        current_pose_ = correction.isNull() ? pose : (correction * pose);

        // Localization state: correction is identity until rtabmap matches a
        // loaded-map node. Once set it stays (even through intermittent odom
        // failures), so this is a latch, not a per-frame presence signal.
        bool locked_now = !correction.isNull() && !correction.isIdentity();
        // In LiDAR-only localization, matches almost always come through the
        // proximity detection pathway (geometric), not the visual loop-closure
        // pathway. Read both so we don't miss the match.
        int loop_id = rtabmap_->getLoopClosureId();
        int prox_id = rtabmap_->getStatistics().proximityDetectionId();
        int match_id = loop_id > 0 ? loop_id : prox_id;
        const char *match_kind = loop_id > 0 ? "loop" : "proximity";

        if (locked_now && !localized_) {
            std::cout << "[SLAM] *** LOCALIZED *** via " << match_kind
                      << " match to node " << match_id
                      << " | correction: " << correction.prettyPrint() << "\n";
            localized_ = true;
        } else if (match_id > 0) {
            std::cout << "[SLAM] " << match_kind << " match this frame: node "
                      << match_id;
            if (loop_id > 0) std::cout << " (score " << std::fixed
                                       << std::setprecision(3)
                                       << rtabmap_->getLoopClosureValue() << ")";
            std::cout << "\n";
        }

        // Cache matched node pose (from loaded_poses_) for viewer rendering.
        // Lock order: slam_mutex_ (held) → grid_mutex_, matches loadMap.
        if (match_id > 0) {
            std::lock_guard<std::mutex> glock(grid_mutex_);
            auto it = loaded_poses_.find(match_id);
            if (it != loaded_poses_.end()) {
                last_closure_id_ = match_id;
                last_closure_pose_ = it->second;
            }
        }

        if (frame_count_ % 10 == 0) {
            std::cout << "[SLAM] Frame " << frame_count_
                      << " | wm=" << rtabmap_->getWMSize()
                      << " | " << (localized_ ? "LOCALIZED" : "SEARCHING")
                      << " | odom: " << pose.prettyPrint()
                      << " | correction: " << (correction.isNull() ? std::string("null")
                                              : correction.isIdentity() ? std::string("identity")
                                              : correction.prettyPrint())
                      << " | icp_ratio=" << std::fixed << std::setprecision(2)
                      << odom_info.reg.icpInliersRatio
                      << " corr=" << odom_info.reg.icpCorrespondences << "\n";
        }
    } // slam_mutex_ released

    // Add local occupancy grid tile — grid_mutex_ only, slam_mutex_ no longer held
    if (nodeId > 0) {
        std::lock_guard<std::mutex> glock(grid_mutex_);
        if (nodeId != last_grid_node_id_) {
            last_grid_node_id_ = nodeId;
            cv::Mat ground, obstacles, empty;
            cv::Point3f viewPoint(0, 0, 0);
            grid_maker_.createLocalMap(scan, pose, ground, obstacles, empty, viewPoint);
            grid_cache_.add(nodeId, ground, obstacles, empty,
                            grid_maker_.getCellSize(), viewPoint);
        }
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

    // Don't pass orientation to RTAB-Map — Viam's orientation frame convention
    // doesn't reliably match RTAB-Map's expected world frame (ENU), causing the
    // entire map to rotate. Gyro+accel integration is sufficient for motion prior.
    rtabmap::IMU rtImu(
        last_orientation_,
        cov3_large,  // always ignore absolute orientation
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
    std::lock_guard<std::mutex> lock(slam_mutex_);
    return current_pose_;
}

bool SlamPipeline::isLocalized() const {
    std::lock_guard<std::mutex> lock(slam_mutex_);
    return localized_;
}

std::pair<int, rtabmap::Transform> SlamPipeline::getLastLoopClosure() const {
    std::lock_guard<std::mutex> lock(slam_mutex_);
    return {last_closure_id_, last_closure_pose_};
}

pcl::PointCloud<pcl::PointXYZI>::Ptr SlamPipeline::getLocalMap() const {
    auto out = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    std::lock_guard<std::mutex> lock(slam_mutex_);
    if (!odom_) return out;
    auto *f2m = dynamic_cast<const rtabmap::OdometryF2M *>(odom_.get());
    if (!f2m) return out; // only F2M maintains a rolling local map

    const rtabmap::Signature &sig = f2m->getMap();
    rtabmap::LaserScan scan = sig.sensorData().laserScanRaw();
    if (scan.isEmpty()) return out;

    // util3d gives us XYZ; copy into XYZI with intensity=128.
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz =
        rtabmap::util3d::laserScanToPointCloud(scan);
    out->reserve(xyz->size());
    for (const auto &p : *xyz) {
        pcl::PointXYZI pi;
        pi.x = p.x; pi.y = p.y; pi.z = p.z;
        pi.intensity = 128.0f;
        out->push_back(pi);
    }
    out->width = out->size();
    out->height = 1;
    return out;
}

rtabmap::Transform SlamPipeline::getMapCorrection() const {
    std::lock_guard<std::mutex> lock(slam_mutex_);
    if (!rtabmap_) return rtabmap::Transform::getIdentity();
    return rtabmap_->getMapCorrection();
}

std::vector<std::pair<float,float>> SlamPipeline::getTrajectory() const {
    std::lock_guard<std::mutex> lock(grid_mutex_);
    std::vector<std::pair<float,float>> traj;
    traj.reserve(loaded_poses_.size());
    for (const auto &[id, pose] : loaded_poses_) {
        traj.push_back({pose.x(), pose.y()});
    }
    return traj;
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
    // Snapshot current session poses under slam_mutex_ only
    std::map<int, rtabmap::Transform> current_poses;
    {
        std::lock_guard<std::mutex> lock(slam_mutex_);
        if (!rtabmap_) return cv::Mat();
        std::multimap<int, rtabmap::Link> constraints;
        rtabmap_->getGraph(current_poses, constraints, /*optimized=*/true, /*global=*/true);
        if (current_poses.empty()) {
            rtabmap_->getGraph(current_poses, constraints, /*optimized=*/false, /*global=*/true);
        }
    } // slam_mutex_ released

    // Assemble and update grid under grid_mutex_ only
    std::lock_guard<std::mutex> glock(grid_mutex_);
    if (!occ_grid_) return cv::Mat();

    // Start with loaded map poses, overlay current session on top
    std::map<int, rtabmap::Transform> poses = loaded_poses_;
    for (auto &[id, p] : current_poses) poses[id] = p;

    occ_grid_->update(poses);
    cv::Mat result = occ_grid_->getMap(xMin, yMin);

    static bool grid_debug_printed = false;
    if (!grid_debug_printed) {
        grid_debug_printed = true;
        std::cout << "[GRID] poses=" << poses.size()
                  << " cache=" << grid_cache_.size()
                  << " map=" << result.cols << "x" << result.rows << "\n";
    }

    cellSize = occ_grid_->getCellSize();
    return result;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr SlamPipeline::loadMap(int map_id) {
    auto map = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    if (!rtabmap_) return map;

    // Snapshot the database under slam_mutex_ only
    std::map<int, rtabmap::Signature> signatures;
    std::map<int, rtabmap::Transform> poses;
    {
        std::lock_guard<std::mutex> lock(slam_mutex_);
        std::multimap<int, rtabmap::Link> constraints;
        // Prefer optimized poses; fall back to raw
        rtabmap_->get3DMap(signatures, poses, constraints, true, true);
        if (poses.empty()) {
            rtabmap_->get3DMap(signatures, poses, constraints, false, true);
            std::cout << "[SLAM] Using raw odometry poses (no saved optimization found)\n";
        } else {
            std::cout << "[SLAM] Using optimized poses (" << poses.size() << " nodes)\n";
        }

        // Dump node ID range so matches can be confirmed against this set.
        // Any subsequent "match to node N" log must have N in this range —
        // new nodes can't appear because IncrementalMemory=false.
        if (!poses.empty()) {
            int min_id = poses.begin()->first;
            int max_id = poses.begin()->first;
            for (auto &[id, _] : poses) {
                if (id < min_id) min_id = id;
                if (id > max_id) max_id = id;
            }
            std::cout << "[SLAM] Loaded map node IDs: [" << min_id << ".." << max_id
                      << "] (" << poses.size() << " total)\n";
        }

        // Re-apply configured initial pose now that the map is loaded. Calling
        // setInitialPose before loadMap has no effect — rtabmap needs the
        // graph present to seed the localization search near the given pose.
        if (!initial_pose_.isNull()) {
            rtabmap_->setInitialPose(initial_pose_);
            std::cout << "[SLAM] Re-applied initial pose to loaded map: "
                      << initial_pose_.prettyPrint() << "\n";
        }
    } // slam_mutex_ released

    // --- Pass 1: build display point cloud (no locks needed — local work) ---
    std::map<int, rtabmap::Transform> new_loaded_poses;
    int included = 0;
    for (auto &[id, sig] : signatures) {
        rtabmap::Transform pose = poses.count(id) ? poses.at(id) : sig.getPose();
        if (pose.isNull()) continue;
        if (map_id >= 0 && sig.mapId() != map_id) continue;

        rtabmap::LaserScan scan = sig.sensorData().laserScanRaw();
        if (scan.isEmpty()) {
            sig.sensorData().uncompressData(nullptr, nullptr, &scan);
        }
        if (scan.isEmpty()) continue;

        new_loaded_poses[id] = pose;

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
    std::cout << "[SLAM] Loaded map: " << map->size() << " points from "
              << included << " nodes"
              << (map_id >= 0 ? " (map " + std::to_string(map_id) + ")" : " (all sessions)")
              << "\n";

    // --- Pass 2: populate grid cache and assemble grid under grid_mutex_ only ---
    {
        std::lock_guard<std::mutex> glock(grid_mutex_);
        loaded_poses_ = std::move(new_loaded_poses);

        int grid_nodes = 0;
        for (auto &[id, sig] : signatures) {
            rtabmap::Transform pose = poses.count(id) ? poses.at(id) : sig.getPose();
            if (pose.isNull()) continue;
            if (map_id >= 0 && sig.mapId() != map_id) continue;
            if (grid_cache_.find(id) != grid_cache_.end()) continue;

            rtabmap::LaserScan scan = sig.sensorData().laserScanRaw();
            if (scan.isEmpty()) {
                sig.sensorData().uncompressData(nullptr, nullptr, &scan);
            }
            if (scan.isEmpty()) continue;

            try {
                cv::Mat ground, obstacles, empty;
                cv::Point3f viewPoint(0, 0, 0);
                grid_maker_.createLocalMap(scan, pose, ground, obstacles, empty, viewPoint);
                grid_cache_.add(id, ground, obstacles, empty, grid_maker_.getCellSize(), viewPoint);
                last_grid_node_id_ = id;
                grid_nodes++;
            } catch (const std::exception &e) {
                std::cerr << "[SLAM] Grid cache skipped node " << id << ": " << e.what() << "\n";
            }
        }
        std::cout << "[SLAM] Grid cache populated: " << grid_nodes << " nodes\n";

        if (!loaded_poses_.empty()) {
            std::cout << "[SLAM] Pre-assembling occupancy grid...\n";
            occ_grid_->update(loaded_poses_);
            std::cout << "[SLAM] Occupancy grid ready\n";
        }
    } // grid_mutex_ released

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

std::pair<int,int> SlamPipeline::postProcess(const json &config) {
    if (!rtabmap_) return {0, 0};
    std::lock_guard<std::mutex> lock(slam_mutex_);

    json pp = config.value("post_process", json::object());
    bool detect_loops  = pp.value("detect_loops",  true);
    bool refine        = pp.value("refine_links",   true);
    float cluster_radius = pp.value("loop_cluster_radius", 1.0f);
    float cluster_angle  = pp.value("loop_cluster_angle",  static_cast<float>(M_PI / 6.0));
    int   iterations     = pp.value("loop_iterations", 1);

    int loops_found   = 0;
    int links_refined = 0;

    if (detect_loops) {
        std::cout << "[POST] Detecting loop closures "
                  << "(radius=" << cluster_radius << "m, iter=" << iterations << ")...\n";
        loops_found = rtabmap_->detectMoreLoopClosures(
            cluster_radius, cluster_angle, iterations,
            /*intraSession=*/true, /*interSession=*/true);
        std::cout << "[POST] " << loops_found << " loop closure(s) added\n";
    }

    if (refine) {
        std::cout << "[POST] Refining links...\n";
        links_refined = rtabmap_->refineLinks();
        std::cout << "[POST] " << links_refined << " link(s) refined\n";
    }

    return {loops_found, links_refined};
}
