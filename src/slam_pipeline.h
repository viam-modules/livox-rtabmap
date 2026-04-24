#pragma once

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <string>

#include <opencv2/core.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/IMU.h>
#include <rtabmap/core/LocalGrid.h>
#include <rtabmap/core/LocalGridMaker.h>
#include <rtabmap/core/global_map/OccupancyGrid.h>

#include <nlohmann/json.hpp>

#include "imu_reader.h"

class SlamPipeline {
public:
    SlamPipeline();
    ~SlamPipeline();

    bool init(const nlohmann::json &config);
    std::string getDatabasePath() const { return db_path_; }

    // Process a new lidar frame. Returns true if odometry succeeded.
    // filtered_cloud is set to the range-filtered cloud for display.
    bool processCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, uint64_t timestamp_ns,
                      pcl::PointCloud<pcl::PointXYZI>::Ptr *filtered_cloud = nullptr);

    // Feed IMU data for motion prior (called at 200Hz)
    void processIMU(float gyro_x, float gyro_y, float gyro_z,
                    float acc_x, float acc_y, float acc_z,
                    uint64_t timestamp_ns);

    // Feed an IMU reading to the odometry as a motion prior.
    // Should be called before the next processCloud call.
    void processImu(const ImuReading &imu);

    // Get the current pose
    rtabmap::Transform getPose() const;

    // Get the trajectory as (x, y) pairs in node-ID order (chronological).
    // After loadMap(), returns optimized poses. During live/playback, returns odometry poses.
    std::vector<std::pair<float,float>> getTrajectory() const;

    int getMapSize() const;
    int getFrameCount() const { return frame_count_; }

    // True once rtabmap has produced a non-identity mapCorrection (i.e. it
    // matched a loaded-map node). Latches — doesn't flip back on drift.
    bool isLocalized() const;

    // The last map node we loop-closed to. Returns (-1, null transform) if
    // no match has happened yet. The pose is the node's optimized pose in
    // the map frame, suitable for rendering directly in the viewer.
    std::pair<int, rtabmap::Transform> getLastLoopClosure() const;

    // Reconstruct accumulated map from database and populate the occupancy grid cache.
    // map_id = -1 loads all sessions; otherwise only that session.
    pcl::PointCloud<pcl::PointXYZI>::Ptr loadMap(int map_id = -1);

    // Return the map_id with the most nodes, or the highest map_id.
    int largestMapId() const;
    int lastMapId() const;

    // Run post-processing: loop closure detection, link refinement, graph optimization.
    // Returns {loops_found, links_refined}.
    std::pair<int,int> postProcess(const nlohmann::json &config);

    // Assemble the 2D occupancy grid from the current SLAM graph.
    // Returns a CV_8SC1 mat: -1=unknown, 0=free, 100=occupied.
    // xMin/yMin are the world-space origin of the grid in metres.
    cv::Mat getOccupancyGrid(float &xMin, float &yMin, float &cellSize);

private:
    std::unique_ptr<rtabmap::Odometry> odom_;
    std::unique_ptr<rtabmap::Rtabmap> rtabmap_;
    rtabmap::Transform current_pose_;
    rtabmap::Transform initial_pose_; // null if not configured
    int frame_count_ = 0;
    std::string db_path_;
    // Localization state against loaded map: true once mapCorrection has been
    // set to a non-identity transform by rtabmap. Tracked so we can log
    // transitions (SEARCHING → LOCALIZED → LOST) instead of silently drifting.
    bool localized_ = false;
    // Latched last successful loop closure: node id in the loaded DB and its
    // pose in the map frame (looked up from loaded_poses_). -1 / null until
    // first match. Protected by slam_mutex_.
    int last_closure_id_ = -1;
    rtabmap::Transform last_closure_pose_;

    // Guards odom_, rtabmap_, current_pose_, frame_count_
    mutable std::mutex slam_mutex_;
    // Guards grid_maker_, grid_cache_, occ_grid_, loaded_poses_, last_grid_node_id_
    mutable std::mutex grid_mutex_;

    // Occupancy grid
    rtabmap::LocalGridMaker grid_maker_;
    rtabmap::LocalGridCache grid_cache_;
    std::unique_ptr<rtabmap::OccupancyGrid> occ_grid_;
    int last_grid_node_id_ = -1;

    // Live Livox IMU path (processIMU — called at 200Hz from LivoxReceiver)
    bool use_imu_ = false;
    float min_range_ = 0;
    float max_range_ = 0;
    float max_accel_ = 0; // m/s², 0 = disabled
    float accel_holdoff_ = 1.0; // seconds
    std::atomic<float> current_accel_{0};
    std::atomic<double> last_high_accel_time_{0};

    // Consecutive odometry failure tracking — reset odometry after too many failures
    int odom_fail_count_ = 0;
    int odom_fail_reset_threshold_ = 5;

    // Sensor → base_link transform (extrinsics)
    rtabmap::Transform lidar_to_base_;

    // Poses from the loaded map session (populated by loadMap, used by getOccupancyGrid)
    std::map<int, rtabmap::Transform> loaded_poses_;

    // Viam playback IMU path (processImu — called from PcdPlayer)
    // Each Viam reading carries one measurement type at a time.
    cv::Vec3d last_gyro_{0, 0, 0};
    cv::Vec3d last_accel_{0, 0, 9.81};
    cv::Vec4d last_orientation_{0, 0, 0, 1}; // identity quaternion
    bool has_gyro_ = false;
    bool has_accel_ = false;
    bool has_orientation_ = false;
};
