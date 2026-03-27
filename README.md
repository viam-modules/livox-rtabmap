# Livox Mid-360 → RTAB-Map Live SLAM

Standalone C++ application that feeds Livox Mid-360 LiDAR point clouds into RTAB-Map for live ICP-based SLAM with a 3D viewer.

## Build

```bash
# Dependencies (Ubuntu 24.04)
sudo apt install libpcl-dev qtbase5-dev libopenmpi-dev nlohmann-json3-dev cmake

# Build rtabmap from source (not in Ubuntu 24.04 repos)
git clone --depth 1 https://github.com/introlab/rtabmap.git /tmp/rtabmap
cd /tmp/rtabmap && mkdir build && cd build && cmake .. && make -j$(nproc) && sudo make install && sudo ldconfig

# Build this project
git submodule update --init
mkdir build && cd build && cmake .. && make -j$(nproc)
```

## Usage

```bash
# GUI mode (default config)
./build/livox_rtabmap

# GUI mode with custom config
./build/livox_rtabmap /path/to/config.json

# Headless mode (set "headless": true in config)
```

The viewer shows the accumulated map in real time. Mouse to rotate/zoom/pan. Close the window or Ctrl+C to stop. The SLAM database is saved automatically on exit.

## Configuration

All parameters are in `config/default.json`. Edit and restart — no recompile needed.

### Network

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `sensor_ip` | string | `"192.168.1.196"` | IP address of the Livox Mid-360 |
| `host_ip` | string | `"192.168.1.10"` | IP address of the host machine on the same subnet as the lidar |

### General

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `headless` | bool | `false` | If true, run without the Qt 3D viewer (console output only) |
| `database_path` | string | `"livox_slam.db"` | Path to the rtabmap database file. SLAM state (poses, graph, sensor data) is saved here on exit and reloaded on next launch. Relative to the working directory |
| `load_previous_map` | bool | `false` | If true, reconstruct and display the point cloud map from the existing database on startup. If false, start with an empty viewer (database is still loaded by rtabmap internally) |
| `use_imu_prior` | bool | `false` | If true, feed Mid-360 IMU data (200Hz gyro + accelerometer) into rtabmap's odometry as a motion prior. Helps ICP converge during fast motion by giving it a starting guess close to the true pose |

### Map Display

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `min_range` | float | `0` | Minimum point range in meters. Points closer than this are discarded before ICP. 0 = no limit. Useful for removing self-reflections or near-field noise |
| `max_range` | float | `0` | Maximum point range in meters. Points farther than this are discarded before ICP. 0 = no limit. Useful for removing noisy far-field points that hurt alignment |
| `max_accel` | float | `0` | Maximum acceleration deviation from gravity in m/s². If the IMU reads acceleration above this threshold, the entire scan is rejected. 0 = no limit. Prevents adding blurry scans during fast motion or impacts |
| `accel_holdoff` | float | `1.0` | Seconds to continue rejecting scans after acceleration drops below `max_accel`. Allows the sensor to stabilize before resuming mapping. Only active when `max_accel` > 0 |
| `map_voxel_size` | float | `0.03` | Voxel grid filter size in meters for the accumulated map. Each cube of this side length keeps one point (at the centroid). Smaller = denser map, more memory. Larger = sparser, faster |
| `map_downsample_interval` | int | `50` | Number of frames between voxel grid downsampling passes on the accumulated map. At 10Hz frame rate, 50 = every 5 seconds |
| `camera_distance` | float | `10.0` | Initial camera distance from origin in meters. Larger = more zoomed out. Adjust based on the size of the environment you're mapping |
| `color_mode` | string | `"intensity"` | How to color map points. `"intensity"`: grayscale from reflectivity. `"flat"`: solid color from `map_color`. `"age"`: blue (old) → red (new) to visualize convergence. `"height"`: blue (low) → red (high) rainbow by Z coordinate |
| `map_color` | [R,G,B] | `[180, 180, 180]` | RGB color for accumulated map points when `color_mode` is `"flat"` (0-255 per channel) |
| `scan_color` | [R,G,B] | `[0, 255, 0]` | RGB color for the current scan overlay (0-255 per channel) |
| `map_point_size` | int | `1` | Pixel size of accumulated map points in the viewer |
| `scan_point_size` | int | `3` | Pixel size of current scan points in the viewer. Larger than map points makes the latest scan easy to distinguish |
| `show_trajectory` | bool | `true` | Show the odometry trajectory as yellow points projected onto the ground plane. Useful for spotting drift — a straight walk should show a straight line |

### ICP Odometry (`icp` section)

These control how consecutive lidar scans are aligned to estimate frame-to-frame motion.

| Parameter | Type | Default | Unit | Description |
|-----------|------|---------|------|-------------|
| `odom_strategy` | int | `0` | — | 0=Frame-to-Map (matches each scan against a local map of recent scans, more robust against ghost maps), 1=Frame-to-Frame (matches consecutive scans only, faster but more drift-prone) |
| `filtering_strategy` | int | `1` | — | Odometry smoothing filter. 0=none, 1=Kalman filter (smooths jumps), 2=particle filter |
| `kalman_process_noise` | float | `0.001` | — | Kalman filter process noise covariance. Higher = trusts measurements more (less smoothing). Lower = smoother but more lag |
| `kalman_measurement_noise` | float | `0.01` | — | Kalman filter measurement noise covariance. Higher = trusts predictions more (more smoothing). Lower = follows ICP output more closely |
| `point_to_plane` | bool | `true` | — | Use point-to-plane ICP (more accurate for flat surfaces) vs point-to-point |
| `point_to_plane_k` | int | `10` | — | Number of neighbors used to compute normals for point-to-plane. Higher = smoother normals but slower |
| `point_to_plane_radius` | float | `0.0` | meters | Search radius for normal computation. 0 = use K neighbors only |
| `point_to_plane_ground_normals_up` | float | `0.8` | — | Fix flipped normals on ground surfaces for ring-like 3D LiDARs like the Mid-360. 0 = disabled, 1 = only perfectly vertical normals. 0.8 is recommended |
| `point_to_plane_min_complexity` | float | `0.02` | — | Minimum structural complexity (0-1) to use point-to-plane. Below this, falls back to point-to-point. Prevents bad alignment in featureless areas like long corridors |
| `voxel_size` | float | `0.03` | meters | Downsample input scans to this voxel size before ICP alignment. Smaller = more precise but slower. 0.03 = 30mm |
| `max_correspondence_distance` | float | `0.25` | meters | Maximum distance between two points to be considered a correspondence pair. Too large = wrong matches. Too small = not enough matches. 0.25 = 250mm |
| `correspondence_ratio` | float | `0.3` | — | Minimum ratio of matched correspondences to total points to accept the transform. Higher = stricter, rejects bad alignments where most points didn't match. 0.3 = at least 30% of points must match |
| `reciprocal_correspondences` | bool | `true` | — | Both points must agree they're each other's closest neighbor. Reduces false matches |
| `iterations` | int | `40` | — | Maximum ICP optimization iterations per frame. More = better convergence, slower |
| `epsilon` | float | `0.0005` | meters | Convergence threshold. ICP stops early if improvement is below this. 0.0005 = 0.5mm |
| `max_translation` | float | `0.5` | meters | Reject odometry results with translation larger than this between consecutive frames. Prevents jumps from bad ICP convergence. 0.5 = 500mm |
| `max_rotation` | float | `0.3` | radians | Reject odometry results with rotation larger than this between consecutive frames. 0.3 = ~17 degrees |
| `outlier_ratio` | float | `0.85` | — | Ratio of points to keep after trimming outliers (0-1). 0.85 = discard the 15% worst matches before computing the alignment. Helps reject spurious correspondences |
| `f2m_scan_max_size` | int | `15000` | — | Maximum points in the Frame-to-Map local reference map. Larger = more reference coverage for better matching but slower. Should be close to your scan size (~20k for Mid-360) |
| `f2m_scan_subtract_radius` | float | `0.03` | meters | When adding a new scan to the local map, remove existing points within this radius to prevent density buildup. Should roughly match voxel_size |

### RTAB-Map (`rtabmap` section)

These control the graph SLAM backend (loop closure, map management).

| Parameter | Type | Default | Unit | Description |
|-----------|------|---------|------|-------------|
| `detection_rate` | int | `0` | Hz | Rate limit for processing frames into the SLAM graph. 0 = process every frame. Set to e.g. 1 to only add one node per second |
| `proximity_by_space` | bool | `true` | — | Enable spatial proximity detection for loop closure candidates. When true, nearby poses in space (not just in sequence) are checked for loop closures |
| `linear_update` | float | `0.1` | meters | Minimum translation since last node to add a new node to the graph. 0.1 = 100mm. Prevents adding redundant nodes when stationary |
| `angular_update` | float | `0.1` | radians | Minimum rotation since last node to add a new node to the graph. 0.1 = ~6 degrees |
