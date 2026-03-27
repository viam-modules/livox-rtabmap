#include "slam_pipeline.h"

#include <iostream>

#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>

#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/utilite/ULogger.h>

SlamPipeline::SlamPipeline() {}

SlamPipeline::~SlamPipeline() {
    if (rtabmap_) {
        rtabmap_->close();
    }
}

bool SlamPipeline::init() {
    // Configure ICP odometry
    rtabmap::ParametersMap odom_params;
    odom_params.insert({rtabmap::Parameters::kOdomStrategy(), "1"});  // ICP
    odom_params.insert({rtabmap::Parameters::kRegStrategy(), "1"});   // ICP registration
    // ICP parameters tuned for Mid-360 (~20k pts, indoor/outdoor)
    odom_params.insert({rtabmap::Parameters::kIcpPointToPlane(), "true"});
    odom_params.insert({rtabmap::Parameters::kIcpVoxelSize(), "0.05"});           // 50mm voxel
    odom_params.insert({rtabmap::Parameters::kIcpMaxCorrespondenceDistance(), "0.5"});  // 500mm
    odom_params.insert({rtabmap::Parameters::kIcpIterations(), "30"});
    odom_params.insert({rtabmap::Parameters::kIcpEpsilon(), "0.001"});

    odom_ = std::make_unique<rtabmap::OdometryICP>(odom_params);

    // Configure RTAB-Map
    rtabmap::ParametersMap rtabmap_params;
    rtabmap_params.insert({rtabmap::Parameters::kRtabmapDetectionRate(), "0"});  // process all
    rtabmap_params.insert({rtabmap::Parameters::kRGBDProximityBySpace(), "true"});
    rtabmap_params.insert({rtabmap::Parameters::kRGBDLinearUpdate(), "0.1"});    // 100mm movement
    rtabmap_params.insert({rtabmap::Parameters::kRGBDAngularUpdate(), "0.1"});   // ~6 degrees

    rtabmap_ = std::make_unique<rtabmap::Rtabmap>();
    rtabmap_->init(rtabmap_params);

    std::cout << "[SLAM] Pipeline initialized (ICP odometry + RTAB-Map)\n";
    return true;
}

bool SlamPipeline::processCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, uint64_t timestamp_ns) {
    if (!odom_ || !rtabmap_) return false;

    // Convert PCL cloud to rtabmap LaserScan
    // rtabmap expects laser scan data for ICP odometry
    pcl::PCLPointCloud2 pcl2;
    pcl::toPCLPointCloud2(*cloud, pcl2);

    // Create sensor data with laser scan
    double stamp = static_cast<double>(timestamp_ns) / 1e9;
    rtabmap::LaserScan scan = rtabmap::util3d::laserScanFromPointCloud(*cloud);
    rtabmap::SensorData data(scan, 0, stamp);

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
                  << " | map nodes: " << rtabmap_->getMemory()->getWorkingMem().size()
                  << "\n";
    }

    return true;
}

rtabmap::Transform SlamPipeline::getPose() const {
    return current_pose_;
}

int SlamPipeline::getMapSize() const {
    if (rtabmap_ && rtabmap_->getMemory()) {
        return rtabmap_->getMemory()->getWorkingMem().size();
    }
    return 0;
}
