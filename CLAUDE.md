# Livox Mid-360 → RTAB-Map Live SLAM

Standalone C++ application that bridges Livox-SDK2 point clouds into RTAB-Map for live 3D SLAM with Qt visualization. No ROS dependency.

## Architecture

```
Livox-SDK2 callbacks (200k pts/sec UDP)
    ↓
Frame accumulation (100ms → ~20k pts/frame at 10Hz)
    ↓
Range filtering (min_range / max_range)
    ↓
Accel gating (reject scans during high acceleration + holdoff)
    ↓
rtabmap::SensorData (LaserScan + optional IMU)
    ↓
rtabmap::Odometry (Frame-to-Map ICP with Kalman filter)
    ↓
rtabmap::Rtabmap (graph SLAM, loop closure)
    ↓
rtabmap::CloudViewer (live Qt 3D visualization)
    ↓
rtabmap database (.db file, persisted on exit)
```

## Project Structure

```
livox-rtab-map/
├── CLAUDE.md
├── README.md               # full parameter documentation
├── CMakeLists.txt           # build config, finds Livox-SDK2 + rtabmap + Qt + PCL + nlohmann_json
├── src/
│   ├── main.cpp             # entry point: config loading, Qt viewer, frame accumulation + coloring
│   ├── livox_receiver.h     # Livox-SDK2 wrapper: point cloud + IMU callbacks, frame accumulation
│   ├── livox_receiver.cpp
│   ├── slam_pipeline.h      # rtabmap wrapper: odometry + SLAM + range filter + accel gating
│   └── slam_pipeline.cpp
├── config/
│   └── default.json         # all parameters (IPs, ICP, rtabmap, display, filtering)
└── third_party/
    └── Livox-SDK2/           # git submodule
```

## Data Flow

### Livox Receiver (`livox_receiver.h/cpp`)
- Inits Livox-SDK2 with generated JSON config (sensor IP, host IP, ports, unicast)
- Registers point cloud + IMU callbacks
- Accumulates points into frames using 100ms time window
- Converts `LivoxLidarCartesianHighRawPoint` (int32 mm) → PCL `PointXYZI` (float meters)
- Delivers IMU at 200Hz (gyro rad/s + accel in g) via separate callback
- Signals new frame via FrameCallback to SLAM pipeline

### SLAM Pipeline (`slam_pipeline.h/cpp`)
- Loads all parameters from config JSON
- Creates `rtabmap::Odometry` via factory (strategy 0=F2M or 1=F2F)
- Creates `rtabmap::Rtabmap` with database persistence
- Per frame:
  1. Check accel gating (reject if above threshold or in holdoff period)
  2. Filter points by min/max range
  3. Convert to `rtabmap::LaserScan`
  4. Attach IMU data if `use_imu_prior` is enabled
  5. Process through odometry → get pose estimate
  6. Feed (SensorData, pose) into Rtabmap → graph update + loop closure
  7. Return filtered cloud + pose to viewer
- Can rebuild accumulated map from database on startup (`load_previous_map`)

### Main (`main.cpp`)
- Loads JSON config (first CLI arg or `config/default.json`)
- Headless mode: just runs SLAM pipeline with console output
- GUI mode:
  - Creates `rtabmap::CloudViewer` with configurable camera position
  - Connects Livox receiver → SLAM pipeline → viewer
  - Accumulates transformed points into map (respects `map_add_interval`)
  - Voxel downsamples periodically (preserves intensity)
  - Colors map via configurable mode (intensity/height/age/flat)
  - Shows current scan overlay + trajectory
  - Live-reloads display parameters from config every 2 seconds

## Config System

All parameters in `config/default.json`. See README.md for full documentation.

**Live-reloadable** (no restart needed):
- color_mode, map_color, scan_color
- map_point_size, scan_point_size
- map_voxel_size, map_downsample_interval, map_add_interval
- show_trajectory

**Requires restart**:
- sensor_ip, host_ip, headless, database_path
- All ICP parameters (odom_strategy, voxel_size, correspondence_distance, etc.)
- All rtabmap parameters (detection_rate, linear_update, etc.)
- min_range, max_range, max_accel, accel_holdoff, use_imu_prior

## Dependencies

- **Livox-SDK2** — git submodule at `third_party/Livox-SDK2`
- **RTAB-Map** — build from source (not in Ubuntu 24.04 repos)
- **PCL** — `sudo apt install libpcl-dev`
- **Qt5** — `sudo apt install qtbase5-dev`
- **nlohmann-json** — `sudo apt install nlohmann-json3-dev`
- **OpenMPI** — `sudo apt install libopenmpi-dev` (needed by PCL/VTK)
- **CMake 3.14+**

## Build

```bash
git submodule update --init
mkdir build && cd build
cmake ..
make -j$(nproc)
```

Note: Livox-SDK2 needs `-Wno-error -include cstdint` for GCC 13+. This is handled in CMakeLists.txt.

## Key rtabmap API

```cpp
// Odometry via factory (respects Odom/Strategy param)
odom_.reset(rtabmap::Odometry::create(params));

// Create SensorData with laser scan (no images for LiDAR)
rtabmap::LaserScan scan = rtabmap::util3d::laserScanFromPointCloud(*xyz);
rtabmap::SensorData data(scan, cv::Mat(), cv::Mat(), rtabmap::CameraModel(), 0, stamp);
data.setIMU(imu); // optional

// Process
rtabmap::Transform pose = odom_->process(data, &odom_info);
rtabmap_->process(data, pose);

// Rebuild map from database
rtabmap_->get3DMap(signatures, poses, constraints, true, true);
```

## TODO

### Residual color mode
Add a `"residual"` color_mode that colors each point by its distance to the nearest neighbor in the previous frame's cloud. Green = well-matched (low residual), red = poorly matched (high residual). This would show exactly where ICP alignment is failing. Requires building a KD-tree of the reference cloud each frame and querying each new point against it — expensive (~20k queries × ~200k reference points), so probably needs to be opt-in and might need to run on a downsampled version. Could use `pcl::KdTreeFLANN` for the lookup.
