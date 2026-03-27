#include "slam_pipeline.h"

#include <iostream>

#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/OdometryInfo.h>
#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/utilite/ULogger.h>

#include <opencv2/core.hpp>

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
    params.insert({rtabmap::Parameters::kOdomStrategy(), "1"});
    params.insert({rtabmap::Parameters::kRegStrategy(), "1"});

    // ICP parameters from config
    params.insert({rtabmap::Parameters::kIcpPointToPlane(),
        icp.value("point_to_plane", true) ? "true" : "false"});
    params.insert({rtabmap::Parameters::kIcpVoxelSize(),
        std::to_string(icp.value("voxel_size", 0.03))});
    params.insert({rtabmap::Parameters::kIcpMaxCorrespondenceDistance(),
        std::to_string(icp.value("max_correspondence_distance", 0.25))});
    params.insert({rtabmap::Parameters::kIcpIterations(),
        std::to_string(icp.value("iterations", 40))});
    params.insert({rtabmap::Parameters::kIcpEpsilon(),
        std::to_string(icp.value("epsilon", 0.0005))});
    params.insert({rtabmap::Parameters::kIcpMaxTranslation(),
        std::to_string(icp.value("max_translation", 0.5))});
    params.insert({rtabmap::Parameters::kIcpMaxRotation(),
        std::to_string(icp.value("max_rotation", 0.3))});

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

    rtabmap_ = std::make_unique<rtabmap::Rtabmap>();
    rtabmap_->init(params);

    std::cout << "[SLAM] Pipeline initialized\n"
              << "  ICP: voxel=" << icp.value("voxel_size", 0.03)
              << " corr=" << icp.value("max_correspondence_distance", 0.25)
              << " iter=" << icp.value("iterations", 40) << "\n";
    return true;
}

bool SlamPipeline::processCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, uint64_t timestamp_ns) {
    if (!odom_ || !rtabmap_) return false;

    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz(new pcl::PointCloud<pcl::PointXYZ>);
    xyz->reserve(cloud->size());
    for (const auto &p : *cloud) {
        xyz->push_back(pcl::PointXYZ(p.x, p.y, p.z));
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

    if (frame_count_ % 10 == 0) {
        std::cout << "[SLAM] Frame " << frame_count_
                  << " | pose: " << pose.prettyPrint() << "\n";
    }

    return true;
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
