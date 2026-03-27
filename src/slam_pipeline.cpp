#include "slam_pipeline.h"

#include <iostream>

#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/OdometryInfo.h>
#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/utilite/ULogger.h>

#include <opencv2/core.hpp>

SlamPipeline::SlamPipeline() {}

SlamPipeline::~SlamPipeline() {
    if (rtabmap_) {
        rtabmap_->close();
    }
}

bool SlamPipeline::init() {
    // Configure ICP odometry
    rtabmap::ParametersMap params;
    params.insert({rtabmap::Parameters::kOdomStrategy(), "1"});  // ICP
    params.insert({rtabmap::Parameters::kRegStrategy(), "1"});   // ICP registration

    // ICP parameters tuned for Mid-360 (~20k pts, indoor/outdoor)
    params.insert({rtabmap::Parameters::kIcpPointToPlane(), "true"});
    params.insert({rtabmap::Parameters::kIcpVoxelSize(), "0.05"});                // 50mm voxel
    params.insert({rtabmap::Parameters::kIcpMaxCorrespondenceDistance(), "0.5"});  // 500mm
    params.insert({rtabmap::Parameters::kIcpIterations(), "30"});
    params.insert({rtabmap::Parameters::kIcpEpsilon(), "0.001"});

    // RTAB-Map parameters
    params.insert({rtabmap::Parameters::kRtabmapDetectionRate(), "0"});     // process all
    params.insert({rtabmap::Parameters::kRGBDProximityBySpace(), "true"});
    params.insert({rtabmap::Parameters::kRGBDLinearUpdate(), "0.1"});       // 100mm movement
    params.insert({rtabmap::Parameters::kRGBDAngularUpdate(), "0.1"});      // ~6 degrees

    odom_.reset(rtabmap::Odometry::create(params));
    if (!odom_) {
        std::cerr << "[SLAM] Failed to create odometry\n";
        return false;
    }

    rtabmap_ = std::make_unique<rtabmap::Rtabmap>();
    rtabmap_->init(params);

    std::cout << "[SLAM] Pipeline initialized (ICP odometry + RTAB-Map)\n";
    return true;
}

bool SlamPipeline::processCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, uint64_t timestamp_ns) {
    if (!odom_ || !rtabmap_) return false;

    // Convert PCL PointXYZI cloud to rtabmap LaserScan
    // rtabmap's laserScanFromPointCloud works with PointXYZ, so strip intensity
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz(new pcl::PointCloud<pcl::PointXYZ>);
    xyz->reserve(cloud->size());
    for (const auto &p : *cloud) {
        xyz->push_back(pcl::PointXYZ(p.x, p.y, p.z));
    }

    double stamp = static_cast<double>(timestamp_ns) / 1e9;
    rtabmap::LaserScan scan = rtabmap::util3d::laserScanFromPointCloud(*xyz);

    // Use the RGB-D + laser scan constructor with empty images
    rtabmap::SensorData data(scan, cv::Mat(), cv::Mat(), rtabmap::CameraModel(), 0, stamp);

    // Run odometry
    rtabmap::OdometryInfo odom_info;
    rtabmap::Transform pose = odom_->process(data, &odom_info);

    if (pose.isNull()) {
        std::cerr << "[SLAM] Odometry failed on frame " << frame_count_ << "\n";
        return false;
    }

    current_pose_ = pose;
    frame_count_++;

    // Feed into RTAB-Map for graph optimization + loop closure
    rtabmap_->process(data, pose);

    if (frame_count_ % 10 == 0) {
        std::cout << "[SLAM] Frame " << frame_count_
                  << " | pose: " << pose.prettyPrint()
                  << "\n";
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
