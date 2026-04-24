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

For SSH X forwarding, you need XQuartz on macOS and software rendering:
```bash
ssh -Y host
LIBGL_ALWAYS_SOFTWARE=1 ./build/livox_rtabmap
```

All parameters in the **Map Display** section are live-reloadable — edit the config JSON while the app is running and changes take effect within 2 seconds. ICP and RTAB-Map parameters require a restart.

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
| `load_map` | null/string/int | `null` | Load and display a previous session's map on startup. See **Map Loading** section below |
| `use_imu_prior` | bool | `false` | If true, feed Mid-360 IMU data (200Hz gyro + accelerometer) into rtabmap's odometry as a motion prior. Helps ICP converge during fast motion by giving it a starting guess close to the true pose |

### Map Display

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `min_range` | float | `0` | Minimum point range in meters. Points closer than this are discarded before ICP. 0 = no limit. Useful for removing self-reflections or near-field noise |
| `max_range` | float | `0` | Maximum point range in meters. Points farther than this are discarded before ICP. 0 = no limit. Useful for removing noisy far-field points that hurt alignment |
| `max_accel` | float | `0` | Maximum acceleration deviation from gravity in m/s². If the IMU reads acceleration above this threshold, the entire scan is rejected. 0 = no limit. Prevents adding blurry scans during fast motion or impacts |
| `accel_holdoff` | float | `1.0` | Seconds to continue rejecting scans after acceleration drops below `max_accel`. Allows the sensor to stabilize before resuming mapping. Only active when `max_accel` > 0 |
| `map_add_interval` | int | `1` | Only add every Nth frame to the accumulated map. 1 = every frame, 5 = every 5th frame. Reduces map density growth. Odometry still runs on every frame. Live-reloadable |
| `map_voxel_size` | float | `0.03` | Voxel grid filter size in meters for the accumulated map. Each cube of this side length keeps one point (at the centroid). Smaller = denser map, more memory. Larger = sparser, faster. Live-reloadable |
| `map_downsample_interval` | int | `50` | Number of frames between voxel grid downsampling passes on the accumulated map. At 10Hz frame rate, 50 = every 5 seconds. Live-reloadable |
| `camera_distance` | float | `10.0` | Initial camera distance from origin in meters. Larger = more zoomed out. Adjust based on the size of the environment you're mapping |
| `color_mode` | string | `"intensity"` | How to color map points. `"intensity"`: auto-normalized blue→red rainbow by reflectivity. `"flat"`: solid color from `map_color`. `"age"`: blue (old) → red (new) to visualize convergence. `"height"`: blue (low) → red (high) rainbow by Z coordinate. Live-reloadable |
| `map_color` | [R,G,B] | `[180, 180, 180]` | RGB color for accumulated map points when `color_mode` is `"flat"` (0-255 per channel). Live-reloadable |
| `scan_color` | [R,G,B] | `[0, 255, 0]` | RGB color for the current scan overlay (0-255 per channel). Live-reloadable |
| `map_point_size` | int | `1` | Pixel size of accumulated map points in the viewer. Live-reloadable |
| `scan_point_size` | int | `3` | Pixel size of current scan points in the viewer. Larger than map points makes the latest scan easy to distinguish. Live-reloadable |
| `show_trajectory` | bool | `true` | Show the odometry trajectory as magenta points projected onto the ground plane. Useful for spotting drift — a straight walk should show a straight line. Live-reloadable |

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

### Map Loading

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `load_map` | null / string / int | `null` | Load a previous session on startup. `null` = fresh start. `"largest"` = load the session with the most nodes. `"last"` = load the most recently created session. Integer = load a specific `map_id` (use `tools/prune-db` to list IDs) |
| `localize_only` | bool | `false` | If true, disables adding new nodes — RTAB-Map only localizes against the loaded map. Requires `load_map` to be set. Note: with LiDAR-only, the robot must start near the same origin as the loaded map for localization to lock on |
| `initial_pose` | null / object / [12 floats] | `null` | Initial pose guess to seed rtabmap's localization in a loaded map. When set, rtabmap starts searching near this pose instead of from map origin — much faster lock-on when you don't start at the original origin. Object form: `{"x":..,"y":..,"z":..,"roll":..,"pitch":..,"yaw":..}` (meters/radians). Array form: 12-element row-major transform matrix |
| `start_at_origin` | bool | `true` if `initial_pose` set, else `false` | Maps to RTAB-Map's `RGBD/StartAtOrigin`. When `true`, rtabmap ignores the last-saved localization pose from the database and starts at map origin (or at `initial_pose`). When `false`, rtabmap restores the pose from where it left off last session — which will override `initial_pose`. Only meaningful in `localize_only` mode |

### Extrinsics

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `extrinsics.x/y/z` | float | `0.0` | Translation from LiDAR sensor to robot base_link origin, in meters |
| `extrinsics.roll/pitch/yaw` | float | `0.0` | Rotation from LiDAR sensor to robot base_link, in radians |

### Post-Processing (`post_process` section, playback only)

Runs automatically after all frames are ingested in playback mode. Improves loop closure and map accuracy.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `detect_loops` | bool | `true` | Search all node pairs spatially for loop closures missed during live processing |
| `refine_links` | bool | `true` | Re-run ICP on all detected loop closure links to tighten alignment |
| `loop_cluster_radius` | float | `1.0` | meters — max spatial distance between nodes to consider as a loop closure candidate. Increase if odometry drift is significant |
| `loop_cluster_angle` | float | `0.5236` | radians (~30°) — max angular difference between nodes to consider as a candidate |
| `loop_iterations` | int | `1` | Number of detection passes. Each pass can find new loops gated by previously uncorrected poses |

---

## Viam Client

The Viam data source streams live point clouds and IMU data from a Viam-connected robot instead of a directly-attached LiDAR.

### Build with Viam support

```bash
# Install Viam C++ SDK and deps
make setup-viam

# Build with Viam enabled
make viam
```

### Credentials

Create `tools/fetch-data/.env` (copy from `.env.example`) and fill in your Viam credentials:

```bash
cp tools/fetch-data/.env.example tools/fetch-data/.env
```

```ini
VIAM_API_KEY=your_api_key_here
VIAM_API_KEY_ID=your_api_key_id_here
```

The remaining fields (`VIAM_ORG_ID`, `VIAM_MACHINE_ID`, etc.) are only needed for `tools/fetch-data`.

### Configure

Edit `config/viam.json`:

```json
{
  "data_source": "viam",
  "viam": {
    "address":    "your-machine.viam.cloud",
    "lidar_name": "livox-pc",
    "imu_name":   "livox-imu",
    "cloud_hz":   10,
    "imu_hz":     100
  },
  ...
}
```

| Field | Description |
|-------|-------------|
| `address` | Your machine's Viam cloud address (from the Viam app → Connect tab) |
| `lidar_name` | Name of the point cloud component in your Viam config |
| `imu_name` | Name of the IMU/movement sensor component. Leave empty to disable IMU |
| `cloud_hz` | Point cloud polling rate in Hz |
| `imu_hz` | IMU polling rate in Hz |

### Run

```bash
# Source credentials and run
make run-viam ENV_FILE=tools/fetch-data/.env

# Or with a custom env file
make run-viam ENV_FILE=/path/to/.env
```

---

## Fetch Tool (`tools/fetch-data`)

Downloads point cloud (PCD) and IMU data from Viam's data platform for offline playback.

### Setup

```bash
cd tools/fetch-data
cp .env.example .env
# Fill in all fields in .env
```

### Usage

```bash
cd tools/fetch-data
go run . --out ../../data/my_session

# Fetch last 2 hours only
go run . --out ../../data/my_session --since 2h

# Fetch a specific time range
go run . --out ../../data/my_session --start "2025-03-27T10:00:00Z" --end "2025-03-27T11:00:00Z"

# Parallel downloads (default 4 workers)
go run . --out ../../data/my_session --workers 8
```

Downloaded PCD files go to `<out>/pcd/` and IMU data to `<out>/imu.jsonl`. Point to them in `config/playback.json`:

```json
{
  "playback_dir": "data/my_session/pcd",
  "imu_dir": "data/my_session"
}
```

---

## Database Tools (`tools/prune-db`)

```bash
# List all sessions in a database
tools/prune-db path/to/map.db

# Delete sessions with fewer than N nodes
tools/prune-db path/to/map.db --min 50

# Keep only one session, delete everything else
tools/prune-db path/to/map.db --keep 3

# Skip confirmation prompt
tools/prune-db path/to/map.db --keep 3 -y
```
