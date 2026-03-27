# Livox Mid-360 → RTAB-Map Live SLAM

Standalone C++ application that bridges Livox-SDK2 point clouds into RTAB-Map for live 3D SLAM with Qt visualization.

## Architecture

```
Livox-SDK2 callbacks (200k pts/sec UDP)
    ↓
Frame accumulation (100ms → ~20k pts/frame at 10Hz)
    ↓
rtabmap::SensorData (point cloud + timestamp)
    ↓
rtabmap::OdometryICP (scan-to-scan or scan-to-map lidar odometry)
    ↓
rtabmap::Rtabmap (graph SLAM, loop closure detection)
    ↓
rtabmap::CloudViewer / MainWindow (live Qt 3D visualization)
```

## Project Structure

```
livox-rtab-map/
├── CLAUDE.md
├── CMakeLists.txt          # build config, finds Livox-SDK2 + rtabmap + Qt + PCL
├── src/
│   ├── main.cpp            # entry point: init SDK, start rtabmap, run Qt event loop
│   ├── livox_receiver.h    # Livox-SDK2 wrapper: callbacks, frame accumulation
│   ├── livox_receiver.cpp
│   ├── slam_pipeline.h     # rtabmap wrapper: odometry + SLAM + map management
│   └── slam_pipeline.cpp
└── config/
    └── mid360_config.json  # Livox SDK2 network config (generated or static)
```

## Data Flow

### Livox Receiver (`livox_receiver.h/cpp`)
- Inits Livox-SDK2 with JSON config (sensor IP, host IP, ports)
- Registers point cloud callback
- Accumulates points into frames using 100ms time window (same as Viam module)
- Converts `LivoxLidarCartesianHighRawPoint` (int32 mm) → PCL `PointXYZI` (float meters)
- Signals new frame via callback to SLAM pipeline

### SLAM Pipeline (`slam_pipeline.h/cpp`)
- Creates `rtabmap::OdometryICP` for scan matching
  - Parameters: voxel size, max correspondence distance, ICP iterations
- Creates `rtabmap::Rtabmap` for graph SLAM
  - Parameters: memory management, loop closure threshold, map update rate
- On each frame:
  1. Wrap PCL cloud in `rtabmap::SensorData` with timestamp
  2. Process through odometry → get pose estimate
  3. Feed (SensorData, pose) into Rtabmap → map update + loop closure check
- Exposes current map and trajectory for visualization

### Main (`main.cpp`)
- Parse CLI args (sensor IP, host IP)
- Init Livox receiver
- Init SLAM pipeline
- Create `rtabmap::CloudViewer` (lightweight) or `rtabmap::MainWindow` (full GUI)
- Connect frame callback → SLAM pipeline → viewer update
- Run Qt event loop

## Dependencies

- **Livox-SDK2** — git submodule at `third_party/Livox-SDK2` (same as viam-livox-mid360)
- **RTAB-Map** — system install (`sudo apt install librtabmap-dev` or build from source)
- **PCL** — required by rtabmap (`sudo apt install libpcl-dev`)
- **Qt5** — for rtabmap GUI (`sudo apt install qtbase5-dev`)
- **CMake 3.14+**

## Build

```bash
# Install dependencies (Ubuntu 24.04)
sudo apt install librtabmap-dev libpcl-dev qtbase5-dev cmake

# Build
mkdir build && cd build
cmake ..
make -j$(nproc)

# Run
./livox_rtabmap --sensor-ip 192.168.1.196 --host-ip 192.168.1.10
```

## Key rtabmap API Usage

```cpp
// Create odometry
rtabmap::OdometryICP odom;
rtabmap::ParametersMap odomParams;
odomParams.insert({rtabmap::Parameters::kOdomStrategy(), "1"}); // ICP
odom.parseParameters(odomParams);

// Create SLAM
rtabmap::Rtabmap rtabmap;
rtabmap.init();

// Per frame:
rtabmap::SensorData data(cloud, timestamp);
rtabmap::OdometryInfo info;
rtabmap::Transform pose = odom.process(data, &info);
if (!pose.isNull()) {
    rtabmap.process(data, pose);
}
```

## Configuration

### Livox SDK2 (same as viam-livox-mid360)
```json
{
  "MID360": {
    "lidar_net_info": { ... },
    "host_net_info": [{
      "host_ip": "192.168.1.10",
      "lidar_ip": ["192.168.1.196"],
      ...
    }]
  }
}
```

### RTAB-Map Parameters (tuned for Mid-360)
- `Odom/Strategy`: 1 (ICP)
- `OdomICP/VoxelSize`: 0.05 (50mm voxel downsampling)
- `OdomICP/MaxCorrespondenceDistance`: 0.5 (500mm)
- `Rtabmap/DetectionRate`: 1.0 (process every frame)
- `RGBD/ProximityBySpace`: true (spatial proximity for loop closure)
