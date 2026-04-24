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
#include <rtabmap/core/CameraModel.h>
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

    // Feed the latest RGBD frame (called from ViamClient's rgbd thread).
    // Stored and consumed on the next processCloud() call.
    void setLatestRGBD(const cv::Mat &rgb, const cv::Mat &depth,
                       double fx, double fy, double cx, double cy,
                       int width, int height);

    // Set the camera-to-lidar extrinsic transform (from Viam frame system).
    // Must be called before the first setLatestRGBD() for it to take effect.
    void setCameraToLidar(const rtabmap::Transform &t);

    // Set the initial SLAM pose from the Viam planning frame.
    // Overrides any initial_pose in the config. Call after init().
    void setInitialPose(const rtabmap::Transform &t);

    // Set the IMU-to-lidar extrinsic transform (from Viam frame system).
    void setImuToLidar(const rtabmap::Transform &t);

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

    // True if rtabmap got a proximity/loop match within the last
    // `staleness_frames` frames. Flips back to false when matches stop
    // happening, so it reflects live state, not history.
    bool isLocalized(int staleness_frames = 3) const;

    // Number of processCloud calls since the last successful match.
    // 0 means "matched this frame". Grows unboundedly while lost.
    int framesSinceMatch() const;

    // The last map node we loop-closed to. Returns (-1, null transform) if
    // no match has happened yet. The pose is the node's optimized pose in
    // the map frame, suitable for rendering directly in the viewer.
    std::pair<int, rtabmap::Transform> getLastLoopClosure() const;

    // The F2M odometry's rolling local map — the reference cloud odometry
    // ICPs each new scan against. Returned in odom frame. Empty if odometry
    // isn't F2M (odom_strategy != 0) or hasn't accumulated scans yet.
    pcl::PointCloud<pcl::PointXYZI>::Ptr getLocalMap() const;

    // Odom→map transform applied by rtabmap during localization.
    // Identity until localization locks on.
    rtabmap::Transform getMapCorrection() const;

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
    // Frames since rtabmap last produced a proximity/loop match. 0 means a
    // match happened on the most recent processCloud call. -1 means "never
    // matched in this session" — use ever_localized_ in callers to branch.
    int frames_since_match_ = -1;
    // Latched-at-first-match tracking just for the one-time "*** LOCALIZED ***"
    // transition log. Set to true forever after the first match; the live
    // indicator uses frames_since_match_ instead.
    bool ever_localized_ = false;
    // Last successful match node id and its pose in the map frame (from
    // loaded_poses_). -1 / null until first match. Protected by slam_mutex_.
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

    bool localize_only_ = false;

    // Latest RGBD frame from the RGB camera (updated by setLatestRGBD, consumed by processCloud)
    mutable std::mutex rgbd_mutex_;
    cv::Mat latest_rgb_;
    cv::Mat latest_depth_;
    rtabmap::CameraModel camera_model_;
    rtabmap::Transform cam_to_lidar_;  // set by setCameraToLidar(), default identity
    bool has_rgbd_ = false;

    // IMU-to-lidar extrinsic (set by setImuToLidar, default identity)
    // Protected by slam_mutex_ since processImu() reads it.
    rtabmap::Transform imu_to_lidar_;

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
