#include <atomic>
#include <cmath>
#include <csignal>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <QApplication>
#include <QSurfaceFormat>
#include <QVTKOpenGLNativeWidget.h>

#include <vtkAxesActor.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkRendererCollection.h>
#include <vtkSmartPointer.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <vtkCoordinate.h>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsPixmapItem>
#include <QGraphicsEllipseItem>
#include <QGraphicsTextItem>
#include <QHBoxLayout>
#include <QImage>
#include <QLabel>
#include <QMouseEvent>
#include <QPixmap>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>
#include <QWheelEvent>
#include <QWidget>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <rtabmap/gui/CloudViewer.h>
#include <rtabmap/core/Transform.h>

#include <nlohmann/json.hpp>

#include "livox_receiver.h"
#include "navigator.h"
#include "planner.h"
#include "pcd_player.h"
#include "slam_pipeline.h"
#ifdef HAVE_VIAM_SDK
#include "viam_client.h"
#endif

using json = nlohmann::json;

static std::atomic<bool> running{true};

static void signalHandler(int) {
    running = false;
}

// HSV to RGB helper (h: 0-360, s: 0-1, v: 0-1)
static void hsv2rgb(float h, float s, float v, uint8_t &r, uint8_t &g, uint8_t &b) {
    float c = v * s;
    float x = c * (1 - std::fabs(std::fmod(h / 60.0f, 2) - 1));
    float m = v - c;
    float rf, gf, bf;
    if (h < 60)      { rf = c; gf = x; bf = 0; }
    else if (h < 120) { rf = x; gf = c; bf = 0; }
    else if (h < 180) { rf = 0; gf = c; bf = x; }
    else if (h < 240) { rf = 0; gf = x; bf = c; }
    else if (h < 300) { rf = x; gf = 0; bf = c; }
    else              { rf = c; gf = 0; bf = x; }
    r = (uint8_t)((rf + m) * 255);
    g = (uint8_t)((gf + m) * 255);
    b = (uint8_t)((bf + m) * 255);
}

// Per-point data for the accumulated map
struct MapPoint {
    float x, y, z;
    float intensity;
    int frame_id;  // which frame added this point
};

int main(int argc, char *argv[]) {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    // VTK's QVTKOpenGLNativeWidget (used by rtabmap::CloudViewer) requires
    // the default surface format to be set before QApplication — otherwise
    // the OpenGL context lacks VAO support and the first paint dereferences
    // a null GL function pointer (crashes on macOS).
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());

    std::string config_path = "config/default.json";
    if (argc > 1 && argv[1][0] != '-') {
        config_path = argv[1];
    }

    std::ifstream f(config_path);
    if (!f) {
        std::cerr << "Cannot open config: " << config_path << "\n";
        return 1;
    }
    json config = json::parse(f);

    std::string sensor_ip = config.value("sensor_ip", "192.168.1.196");
    std::string host_ip = config.value("host_ip", "192.168.1.10");
    std::string playback_dir = config.value("playback_dir", "");
    std::string imu_dir = config.value("imu_dir", "");
    int playback_delay_ms = config.value("playback_delay_ms", 0);
    bool headless = config.value("headless", false);
    std::string data_source = config.value("data_source", "livox");
    float map_voxel = config.value("map_voxel_size", 0.03f);
    int downsample_interval = config.value("map_downsample_interval", 50);
    std::string color_mode = config.value("color_mode", "intensity");
    bool show_trajectory = config.value("show_trajectory", true);
    int map_add_interval = config.value("map_add_interval", 1);
    bool add_scans_to_map = config.value("add_scans_to_map", true);

    auto map_color_arr = config.value("map_color", std::vector<int>{180, 180, 180});
    auto scan_color_arr = config.value("scan_color", std::vector<int>{0, 255, 0});
    int map_point_size = config.value("map_point_size", 1);
    int scan_point_size = config.value("scan_point_size", 3);
    QColor map_color(map_color_arr[0], map_color_arr[1], map_color_arr[2]);
    QColor scan_color(scan_color_arr[0], scan_color_arr[1], scan_color_arr[2]);

    std::cout << "Livox → RTAB-Map SLAM\n"
              << "  Config: " << config_path << "\n";
    if (!playback_dir.empty()) {
        std::cout << "  Mode:   playback from " << playback_dir << "\n";
    } else if (data_source == "viam") {
        auto &vc = config.at("viam");
        std::cout << "  Source: viam (" << vc.value("address", "") << ")\n"
                  << "  Lidar:  " << vc.value("lidar_name", "") << "\n"
                  << "  IMU:    " << vc.value("imu_name", "(none)") << "\n"
                  << "  Mode:   " << (headless ? "headless" : "GUI") << "\n";
    } else {
        std::cout << "  Sensor: " << sensor_ip << "\n"
                  << "  Host:   " << host_ip << "\n"
                  << "  Mode:   " << (headless ? "headless" : "GUI") << "\n";
    }
    std::cout << "\n";

    SlamPipeline slam;
    if (!slam.init(config)) {
        std::cerr << "Failed to init SLAM pipeline\n";
        return 1;
    }

    // Combine mode: post-process an existing multi-session database
    if (data_source == "combine") {
        bool post_processing_needed =
            config.value("post_process", json::object()).value("detect_loops", true) ||
            config.value("post_process", json::object()).value("refine_links",  true);

        if (headless) {
            std::cout << "Loading existing map...\n";
            slam.loadMap(-1);
            if (post_processing_needed) slam.postProcess(config);
            std::cout << "Done. Database: " << slam.getDatabasePath() << "\n";
            return 0;
        }

        QApplication app(argc, argv);

        rtabmap::CloudViewer viewer;
        viewer.setWindowTitle("Livox SLAM (combine)");
        viewer.setBackgroundColor(QColor(20, 20, 20));
        viewer.setGridShown(true);
        viewer.resize(1280, 720);
        float cam_dist = config.value("camera_distance", 10.0f);
        float d = cam_dist / 1.732f;
        viewer.setCameraPosition(d, d, d, 0, 0, 0, 0, 0, 1);
        viewer.show();

        std::mutex cloud_mu;
        pcl::PointCloud<pcl::PointXYZI>::Ptr accumulated_map(new pcl::PointCloud<pcl::PointXYZI>);
        std::atomic<bool> post_processing{false};
        std::atomic<bool> post_process_done{false};
        std::atomic<bool> map_reloaded{false};

        // Load existing sessions for initial display
        {
            auto initial = slam.loadMap(-1);
            std::lock_guard<std::mutex> lk(cloud_mu);
            accumulated_map = initial;
        }
        viewer.addCloud("map", accumulated_map);
        viewer.refreshView();

        // Occupancy grid window
        QWidget grid_win;
        grid_win.setWindowTitle("Occupancy Grid (combine)");
        grid_win.resize(700, 700);
        auto *cm_vbox    = new QVBoxLayout(&grid_win);
        cm_vbox->setContentsMargins(4, 4, 4, 4);
        auto *cm_scene   = new QGraphicsScene(&grid_win);
        auto *cm_view    = new QGraphicsView(cm_scene, &grid_win);
        auto *cm_pmap    = cm_scene->addPixmap(QPixmap());
        cm_view->setDragMode(QGraphicsView::ScrollHandDrag);
        cm_view->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
        cm_view->setRenderHint(QPainter::SmoothPixmapTransform);
        cm_view->setBackgroundBrush(QColor(80, 80, 80));
        cm_view->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        cm_vbox->addWidget(cm_view);

        struct CmWheelFilter : QObject {
            QGraphicsView *v;
            CmWheelFilter(QGraphicsView *v, QObject *p) : QObject(p), v(v) {}
            bool eventFilter(QObject *, QEvent *e) override {
                if (e->type() == QEvent::Wheel) {
                    double f = static_cast<QWheelEvent*>(e)->angleDelta().y() > 0 ? 1.15 : 1.0/1.15;
                    v->scale(f, f);
                    return true;
                }
                return false;
            }
        };
        cm_view->viewport()->installEventFilter(new CmWheelFilter(cm_view, &grid_win));
        grid_win.show();

        // Combine thread: post-process, then reload with optimized poses
        std::thread combine_thread([&]() {
            if (!post_processing_needed) return;
            post_processing = true;
            slam.postProcess(config);
            std::cout << "Reloading map with optimized poses...\n";
            auto opt = slam.loadMap(-1);
            {
                std::lock_guard<std::mutex> lk(cloud_mu);
                accumulated_map = opt;
            }
            map_reloaded = true;
            post_processing = false;
            post_process_done = true;
            std::cout << "Combine done. Database: " << slam.getDatabasePath() << "\n";
        });

        // Grid timer
        bool grid_refreshed_after_postprocess = false;
        QTimer cm_grid_timer;
        QObject::connect(&cm_grid_timer, &QTimer::timeout, [&]() {
            if (post_processing) return;
            if (post_process_done && grid_refreshed_after_postprocess) return;
            float xMin, yMin, cellSize;
            cv::Mat grid = slam.getOccupancyGrid(xMin, yMin, cellSize);
            if (grid.empty()) return;
            QImage img(grid.cols, grid.rows, QImage::Format_RGB888);
            for (int r = 0; r < grid.rows; r++) {
                for (int c = 0; c < grid.cols; c++) {
                    auto v = grid.at<int8_t>(r, c);
                    QRgb px;
                    if (v < 0)       px = qRgb(128, 128, 128);
                    else if (v == 0) px = qRgb(240, 240, 240);
                    else             px = qRgb(20,  20,  20);
                    img.setPixel(c, grid.rows - 1 - r, px);
                }
            }
            cm_pmap->setPixmap(QPixmap::fromImage(img));
            cm_scene->setSceneRect(img.rect());
            if (post_process_done) grid_refreshed_after_postprocess = true;
        });
        cm_grid_timer.start(2000);

        // CloudViewer update timer
        QTimer cm_update_timer;
        QObject::connect(&cm_update_timer, &QTimer::timeout, [&]() {
            if (!running) { app.quit(); return; }
            if (map_reloaded.exchange(false)) {
                std::lock_guard<std::mutex> lk(cloud_mu);
                viewer.addCloud("map", accumulated_map);
                QString status = post_process_done ? " | Combine done" : " | Combining...";
                viewer.setWindowTitle(QString("Livox SLAM (combine) | Map: %1 pts%2")
                    .arg(accumulated_map->size()).arg(status));
                viewer.refreshView();
            }
        });
        cm_update_timer.start(100);

        int ret = app.exec();
        running = false;
        combine_thread.join();
        return ret;
    }

    // Playback mode: replay downloaded PCD files through the SLAM pipeline
    if (!playback_dir.empty()) {
        PcdPlayer player;
        if (!player.load(playback_dir)) {
            std::cerr << "Failed to load PCD files from: " << playback_dir << "\n";
            return 1;
        }
        if (!imu_dir.empty()) {
            player.loadImu(imu_dir, config.value("imu_use_orientation", false)); // non-fatal if missing
        }

        auto imuCb = [&slam](const ImuReading &imu) {
            slam.processImu(imu);
        };

        if (headless) {
            player.play(
                [&](pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, uint64_t ts) {
                    if (!running) { player.stop(); return; }
                    slam.processCloud(cloud, ts);
                },
                imuCb,
                playback_delay_ms);

            std::cout << "\nPlayback done: " << slam.getFrameCount() << " frames processed\n";
            if (running &&
                (config.value("post_process", json::object()).value("detect_loops", true) ||
                 config.value("post_process", json::object()).value("refine_links",  true))) {
                slam.postProcess(config);
            }
            std::cout << "Database saved to: " << slam.getDatabasePath() << "\n";
            return 0;
        }

        // GUI playback — spin up Qt viewer, play in background thread
        QApplication app(argc, argv);

        rtabmap::CloudViewer viewer;
        viewer.setWindowTitle("Livox SLAM (playback)");
        viewer.setBackgroundColor(QColor(20, 20, 20));
        viewer.setGridShown(true);
        viewer.resize(1280, 720);
        float cam_dist = config.value("camera_distance", 10.0f);
        float d = cam_dist / 1.732f;
        viewer.setCameraPosition(d, d, d, 0, 0, 0, 0, 0, 1);
        viewer.show();

        std::mutex cloud_mu;
        pcl::PointCloud<pcl::PointXYZI>::Ptr latest_cloud;
        rtabmap::Transform latest_pose;
        pcl::PointCloud<pcl::PointXYZI>::Ptr accumulated_map(new pcl::PointCloud<pcl::PointXYZI>);
        std::vector<std::pair<float,float>> trajectory_xz;
        int frame_count = 0;
        bool playback_done = false;

        bool post_processing_needed =
            config.value("post_process", json::object()).value("detect_loops", true) ||
            config.value("post_process", json::object()).value("refine_links",  true);
        std::atomic<bool> post_processing{false};
        std::atomic<bool> post_process_done{false};
        std::atomic<bool> map_reloaded{false};

        std::thread play_thread([&]() {
            player.play(
                [&](pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, uint64_t ts) {
                    if (!running) { player.stop(); return; }
                    if (slam.processCloud(cloud, ts)) {
                        std::lock_guard<std::mutex> lock(cloud_mu);
                        latest_cloud = cloud;
                        latest_pose = slam.getPose();
                        trajectory_xz.push_back({latest_pose.x(), latest_pose.y()});
                        frame_count++;
                    }
                },
                imuCb,
                playback_delay_ms);

            playback_done = true;
            std::cout << "\nPlayback done: " << slam.getFrameCount() << " frames processed\n";

            if (post_processing_needed && running) {
                post_processing = true;
                slam.postProcess(config);

                // Reload map with optimized poses — rebuilds point cloud + grid cache
                std::cout << "Reloading map with optimized poses...\n";
                auto optimized_map = slam.loadMap(-1);
                auto optimized_traj = slam.getTrajectory();
                {
                    std::lock_guard<std::mutex> lock(cloud_mu);
                    accumulated_map = optimized_map;
                    trajectory_xz   = std::move(optimized_traj);
                }
                map_reloaded = true;
                post_processing = false;  // allow grid timer to fire for final refresh
                post_process_done = true;
                std::cout << "Map reloaded. Database saved to: "
                          << slam.getDatabasePath() << "\n";
            }
        });

        // Occupancy grid window
        struct GridMeta { float xMin=0, yMin=0, cellSize=0.05f; int rows=0, cols=0; };
        std::mutex grid_meta_mu;
        GridMeta grid_meta;

        QWidget grid_win;
        grid_win.setWindowTitle("Occupancy Grid (playback)");
        grid_win.resize(700, 700);
        auto *pb_grid_vbox   = new QVBoxLayout(&grid_win);
        pb_grid_vbox->setContentsMargins(4, 4, 4, 4);
        auto *pb_scene       = new QGraphicsScene(&grid_win);
        auto *pb_grid_view   = new QGraphicsView(pb_scene, &grid_win);
        auto *pb_pmap_item   = pb_scene->addPixmap(QPixmap());
        pb_grid_view->setDragMode(QGraphicsView::ScrollHandDrag);
        pb_grid_view->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
        pb_grid_view->setRenderHint(QPainter::SmoothPixmapTransform);
        pb_grid_view->setBackgroundBrush(QColor(80, 80, 80));
        pb_grid_view->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        pb_grid_vbox->addWidget(pb_grid_view);

        struct PbWheelFilter : QObject {
            QGraphicsView *v;
            PbWheelFilter(QGraphicsView *v, QObject *p) : QObject(p), v(v) {}
            bool eventFilter(QObject *, QEvent *e) override {
                if (e->type() == QEvent::Wheel) {
                    double f = static_cast<QWheelEvent*>(e)->angleDelta().y() > 0 ? 1.15 : 1.0/1.15;
                    v->scale(f, f);
                    return true;
                }
                return false;
            }
        };
        pb_grid_view->viewport()->installEventFilter(new PbWheelFilter(pb_grid_view, &grid_win));
        grid_win.show();

        QTimer pb_grid_timer;
        bool grid_refreshed_after_postprocess = false;
        QObject::connect(&pb_grid_timer, &QTimer::timeout, [&]() {
            if (post_processing) return;
            if (post_process_done && grid_refreshed_after_postprocess) return;
            float xMin, yMin, cellSize;
            cv::Mat grid = slam.getOccupancyGrid(xMin, yMin, cellSize);
            if (grid.empty()) return;

            QImage img(grid.cols, grid.rows, QImage::Format_RGB888);
            for (int r = 0; r < grid.rows; r++) {
                for (int c = 0; c < grid.cols; c++) {
                    auto v = grid.at<int8_t>(r, c);
                    QRgb px;
                    if (v < 0)       px = qRgb(128, 128, 128);
                    else if (v == 0) px = qRgb(240, 240, 240);
                    else             px = qRgb(20,  20,  20);
                    img.setPixel(c, grid.rows - 1 - r, px);
                }
            }

            {
                std::lock_guard<std::mutex> lock(cloud_mu);
                for (const auto &[tx, ty] : trajectory_xz) {
                    int cx = (int)((tx - xMin) / cellSize);
                    int cy = grid.rows - 1 - (int)((ty - yMin) / cellSize);
                    if (cx >= 0 && cx < grid.cols && cy >= 0 && cy < grid.rows)
                        img.setPixel(cx, cy, qRgb(255, 50, 255));
                }
                if (!trajectory_xz.empty()) {
                    int cx = (int)((trajectory_xz.back().first  - xMin) / cellSize);
                    int cy = grid.rows - 1 - (int)((trajectory_xz.back().second - yMin) / cellSize);
                    for (int d = -3; d <= 3; d++) {
                        if (cx+d >= 0 && cx+d < grid.cols) img.setPixel(cx+d, cy, qRgb(0, 255, 0));
                        if (cy+d >= 0 && cy+d < grid.rows) img.setPixel(cx, cy+d, qRgb(0, 255, 0));
                    }
                }
            }

            pb_pmap_item->setPixmap(QPixmap::fromImage(img));
            pb_scene->setSceneRect(img.rect());
            if (post_process_done) grid_refreshed_after_postprocess = true;
        });
        pb_grid_timer.start(2000);

        QTimer update_timer;
        QObject::connect(&update_timer, &QTimer::timeout, [&]() {
            if (!running) {
                app.quit();
                return;
            }

            // Post-processing finished: swap in the optimized map and re-render
            if (map_reloaded.exchange(false)) {
                std::lock_guard<std::mutex> lock(cloud_mu);
                viewer.addCloud("map", accumulated_map);
                viewer.setWindowTitle(QString("Livox SLAM (playback) | Frame %1 / %2 | Map: %3 pts | Post-processing done")
                    .arg(frame_count).arg(player.frameCount()).arg(accumulated_map->size()));
                viewer.refreshView();
                return;
            }

            if (playback_done && !latest_cloud) return;
            std::lock_guard<std::mutex> lock(cloud_mu);
            if (!latest_cloud) return;

            for (const auto &p : *latest_cloud) {
                pcl::PointXYZI tp;
                float x = p.x, y = p.y, z = p.z;
                const float *data = latest_pose.data();
                tp.x = data[0]*x + data[1]*y + data[2]*z + data[3];
                tp.y = data[4]*x + data[5]*y + data[6]*z + data[7];
                tp.z = data[8]*x + data[9]*y + data[10]*z + data[11];
                tp.intensity = p.intensity;
                accumulated_map->push_back(tp);
            }
            accumulated_map->width = accumulated_map->size();
            accumulated_map->height = 1;

            if (frame_count % downsample_interval == 0 && accumulated_map->size() > 100000) {
                pcl::VoxelGrid<pcl::PointXYZI> voxel;
                voxel.setInputCloud(accumulated_map);
                voxel.setLeafSize(map_voxel, map_voxel, map_voxel);
                pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);
                voxel.filter(*filtered);
                accumulated_map = filtered;
            }

            viewer.addCloud("map", accumulated_map);
            viewer.addCloud("scan", latest_cloud, latest_pose);
            latest_cloud.reset();
            QString status;
            if (post_process_done)       status = " | Post-processing done";
            else if (post_processing)    status = " | Post-processing...";
            else if (playback_done)      status = " | Playback done";
            viewer.setWindowTitle(QString("Livox SLAM (playback) | Frame %1 / %2 | Map: %3 pts%4")
                .arg(frame_count).arg(player.frameCount()).arg(accumulated_map->size()).arg(status));
            viewer.refreshView();
        });
        update_timer.start(33);

        int ret = app.exec();
        running = false;
        player.stop();
        play_thread.join();

        if (!post_process_done && post_processing_needed) {
            // User closed window before post-processing finished — run it now.
            slam.postProcess(config);
        }
        std::cout << "Database saved to: " << slam.getDatabasePath() << "\n";
        return ret;
    }

    // Build data source start/stop for live modes (Livox or Viam)
    std::function<bool(FrameCallback, IMUCallback)> source_start;
    std::function<void()> source_stop;
    std::function<void(float, float)> nav_vel_cb; // set below if Viam base is configured

    if (data_source == "viam") {
#ifdef HAVE_VIAM_SDK
        auto &vc = config.at("viam");

        // Credentials come from env vars (same .env file as other Viam modules).
        // Config values are used as fallback if the env vars are not set.
        auto from_env = [](const char *var, const std::string &fallback) {
            const char *v = std::getenv(var);
            return (v && v[0]) ? std::string(v) : fallback;
        };

        ViamClient::Config vcfg;
        vcfg.address     = vc.value("address", "");
        vcfg.api_key     = from_env("VIAM_API_KEY",    vc.value("api_key", ""));
        vcfg.api_key_id  = from_env("VIAM_API_KEY_ID", vc.value("api_key_id", ""));
        vcfg.lidar_name  = vc.value("lidar_name", "");
        vcfg.imu_name    = vc.value("imu_name", "");
        vcfg.base_name   = vc.value("base_name", "");
        vcfg.cloud_hz    = vc.value("cloud_hz", 10);
        vcfg.imu_hz      = vc.value("imu_hz", 100);
        auto client = std::make_shared<ViamClient>(vcfg);
        source_start = [client](FrameCallback f, IMUCallback i) { return client->start(f, i); };
        source_stop  = [client]() { client->stop(); };
        if (!vcfg.base_name.empty()) {
            nav_vel_cb = [client](float l, float a) { client->sendBaseVelocity(l, a); };
        }
#else
        std::cerr << "data_source=viam but HAVE_VIAM_SDK not compiled in.\n"
                  << "Install viam-cpp-sdk and rebuild.\n";
        return 1;
#endif
    } else {
        auto receiver = std::make_shared<LivoxReceiver>(sensor_ip, host_ip);
        source_start = [receiver](FrameCallback f, IMUCallback i) { return receiver->start(f, i); };
        source_stop  = [receiver]() { receiver->stop(); };
    }

    auto frame_cb = [&slam](pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, uint64_t ts) {
        slam.processCloud(cloud, ts);
    };
    auto imu_cb = [&slam](const LivoxIMU &imu) {
        slam.processIMU(imu.gyro_x, imu.gyro_y, imu.gyro_z,
                        imu.acc_x, imu.acc_y, imu.acc_z,
                        imu.timestamp_ns);
    };

    // Live mode — headless
    if (headless) {
        if (!source_start(frame_cb, imu_cb)) {
            std::cerr << "Failed to start data source\n"; return 1;
        }
        while (running) std::this_thread::sleep_for(std::chrono::seconds(1));
        source_stop();
        return 0;
    }

    // GUI mode
    QApplication app(argc, argv);
    rtabmap::CloudViewer viewer;
    viewer.setWindowTitle("Livox Mid-360 SLAM");
    viewer.setBackgroundColor(QColor(20, 20, 20));
    viewer.setGridShown(true);
    viewer.resize(1280, 720);
    float cam_dist = config.value("camera_distance", 10.0f);
    float d = cam_dist / 1.732f;
    viewer.setCameraPosition(d, d, d, 0, 0, 0, 0, 0, 1);
    viewer.show();

    // World-frame orientation marker (X=red, Y=green, Z=blue) in bottom-right corner.
    // Held at function scope so it outlives the render loop.
    vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New();
    axes->SetXAxisLabelText("X");
    axes->SetYAxisLabelText("Y");
    axes->SetZAxisLabelText("Z");
    vtkSmartPointer<vtkOrientationMarkerWidget> axis_widget =
        vtkSmartPointer<vtkOrientationMarkerWidget>::New();
    axis_widget->SetOrientationMarker(axes);
    axis_widget->SetInteractor(viewer.renderWindow()->GetInteractor());
    axis_widget->SetViewport(0.82, 0.0, 1.0, 0.22); // bottom-right corner
    axis_widget->SetEnabled(1);
    axis_widget->InteractiveOff();

    // Localization status overlay — top-right. Red SEARCHING until rtabmap
    // locks onto the loaded map, then green LOCALIZED.
    vtkSmartPointer<vtkTextActor> status_text = vtkSmartPointer<vtkTextActor>::New();
    status_text->SetInput("SEARCHING");
    status_text->GetTextProperty()->SetFontSize(22);
    status_text->GetTextProperty()->SetBold(1);
    status_text->GetTextProperty()->SetColor(1.0, 0.2, 0.2); // red
    status_text->GetTextProperty()->SetJustificationToRight();
    status_text->GetTextProperty()->SetVerticalJustificationToTop();
    status_text->GetPositionCoordinate()->SetCoordinateSystemToNormalizedViewport();
    status_text->SetPosition(0.985, 0.975);
    if (auto *renderers = viewer.renderWindow()->GetRenderers()) {
        renderers->InitTraversal();
        if (auto *renderer = renderers->GetNextItem()) {
            renderer->AddActor2D(status_text);
        }
    }

    std::mutex cloud_mu;
    pcl::PointCloud<pcl::PointXYZI>::Ptr latest_cloud;
    rtabmap::Transform latest_pose;
    std::vector<MapPoint> map_points;
    std::vector<std::pair<float, float>> trajectory_xz; // for trajectory line
    int frame_count = 0;
    int max_frame_id = 0;

    // load_map: null/absent = don't load, "largest", "last", or integer map_id
    int load_map_id = -1;
    std::string load_map_desc;
    if (config.contains("load_map") && !config["load_map"].is_null()) {
        auto &lm = config["load_map"];
        if (lm.is_number_integer()) {
            load_map_id = lm.get<int>();
            load_map_desc = "map_id=" + std::to_string(load_map_id);
        } else if (lm.is_string()) {
            std::string s = lm.get<std::string>();
            if (s == "largest") {
                load_map_id = slam.largestMapId();
                load_map_desc = "largest (id=" + std::to_string(load_map_id) + ")";
            } else if (s == "last") {
                load_map_id = slam.lastMapId();
                load_map_desc = "last (id=" + std::to_string(load_map_id) + ")";
            } else {
                std::cerr << "[VIEWER] Unknown load_map value: " << s
                          << " (expected null, \"largest\", \"last\", or integer)\n";
            }
        }
    }
    if (load_map_id >= 0) {
        std::cout << "[VIEWER] Loading " << load_map_desc << "\n";
        auto rebuilt = slam.loadMap(load_map_id);
        if (rebuilt->size() > 0) {
            std::cout << "[VIEWER] Loaded " << rebuilt->size() << " points\n";
            for (const auto &p : *rebuilt) {
                map_points.push_back({p.x, p.y, p.z, p.intensity, 0});
            }
        }
    }

    bool ok = source_start(
        [&](pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, uint64_t ts) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr filtered;
            bool success = slam.processCloud(cloud, ts, &filtered);
            if (success) {
                std::lock_guard<std::mutex> lock(cloud_mu);
                latest_cloud = filtered ? filtered : cloud;
                latest_pose = slam.getPose();
                frame_count++;
            }
        },
        [&](const LivoxIMU &imu) {
            slam.processIMU(imu.gyro_x, imu.gyro_y, imu.gyro_z,
                            imu.acc_x, imu.acc_y, imu.acc_z,
                            imu.timestamp_ns);
        });
    if (!ok) { std::cerr << "Failed to start data source\n"; return 1; }

    // Config live reload
    auto last_config_write = std::filesystem::last_write_time(config_path);
    QTimer config_timer;
    QObject::connect(&config_timer, &QTimer::timeout, [&]() {
        try {
            auto now = std::filesystem::last_write_time(config_path);
            if (now == last_config_write) return;
            last_config_write = now;
            std::ifstream rf(config_path);
            if (!rf) return;
            json new_config = json::parse(rf);
            map_add_interval = new_config.value("map_add_interval", map_add_interval);
            add_scans_to_map = new_config.value("add_scans_to_map", add_scans_to_map);
            map_voxel = new_config.value("map_voxel_size", map_voxel);
            downsample_interval = new_config.value("map_downsample_interval", downsample_interval);
            color_mode = new_config.value("color_mode", color_mode);
            show_trajectory = new_config.value("show_trajectory", show_trajectory);
            auto mc = new_config.value("map_color", std::vector<int>{map_color.red(), map_color.green(), map_color.blue()});
            auto sc = new_config.value("scan_color", std::vector<int>{scan_color.red(), scan_color.green(), scan_color.blue()});
            map_color = QColor(mc[0], mc[1], mc[2]);
            scan_color = QColor(sc[0], sc[1], sc[2]);
            map_point_size = new_config.value("map_point_size", map_point_size);
            scan_point_size = new_config.value("scan_point_size", scan_point_size);
            std::cout << "[CONFIG] Reloaded " << config_path << "\n";
        } catch (const std::exception &e) {
            std::cerr << "[CONFIG] Reload error: " << e.what() << "\n";
        }
    });
    config_timer.start(2000);

    // ── Occupancy grid window ─────────────────────────────────────────────────
    // Grid metadata — updated by the grid timer, read by the click handler.
    struct GridMeta { float xMin=0, yMin=0, cellSize=0.05f; int rows=0, cols=0; };
    std::mutex grid_meta_mu;
    GridMeta grid_meta;

    // Waypoints in world space (x, y metres).
    std::vector<std::pair<float,float>> waypoints;
    // Planned path (A* output) — stored so it can be drawn on the grid.
    std::vector<std::pair<float,float>> planned_path;

    // Navigator — load from "navigator" config block
    Navigator::Config nav_cfg;
    {
        json nc = config.value("navigator", json::object());
        nav_cfg.arrival_radius    = nc.value("arrival_radius",    nav_cfg.arrival_radius);
        nav_cfg.heading_threshold = nc.value("heading_threshold", nav_cfg.heading_threshold);
        nav_cfg.forward_speed     = nc.value("forward_speed",     nav_cfg.forward_speed);
        nav_cfg.angular_gain      = nc.value("angular_gain",      nav_cfg.angular_gain);
        nav_cfg.angular_max       = nc.value("angular_max",       nav_cfg.angular_max);
        nav_cfg.update_hz         = nc.value("update_hz",         nav_cfg.update_hz);
    }
    Navigator navigator(nav_cfg);

    QWidget grid_win;
    grid_win.setWindowTitle("Occupancy Grid");
    grid_win.resize(700, 750);
    auto *grid_vbox = new QVBoxLayout(&grid_win);
    grid_vbox->setContentsMargins(4, 4, 4, 4);
    grid_vbox->setSpacing(4);

    // Scene + view
    auto *scene      = new QGraphicsScene(&grid_win);
    auto *grid_view  = new QGraphicsView(scene, &grid_win);
    auto *pmap_item  = scene->addPixmap(QPixmap());
    // Waypoint items live on the scene; we rebuild them when the list changes.
    std::vector<QGraphicsItem*> wp_items;
    std::vector<QGraphicsItem*> plan_items;

    // Pose arrow — shaft + arrowhead pointing in +X (right), rotated each tick
    QPolygonF arrow_poly;
    {
        float L = 10, W = 3.5f, hw = 7, hl = 5;
        arrow_poly << QPointF(-L/2,   -W/2)
                   << QPointF(L/2-hl, -W/2)
                   << QPointF(L/2-hl, -hw/2)
                   << QPointF(L/2,     0)
                   << QPointF(L/2-hl,  hw/2)
                   << QPointF(L/2-hl,  W/2)
                   << QPointF(-L/2,    W/2);
    }
    auto *pose_arrow = scene->addPolygon(arrow_poly,
        QPen(QColor(0, 220, 0), 1), QBrush(QColor(0, 220, 0)));
    pose_arrow->setVisible(false);

    grid_view->setDragMode(QGraphicsView::ScrollHandDrag);
    grid_view->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
    grid_view->setRenderHint(QPainter::SmoothPixmapTransform);
    grid_view->setBackgroundBrush(QColor(80, 80, 80));
    grid_view->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    // Wheel zoom via event filter
    struct WheelFilter : QObject {
        QGraphicsView *v;
        WheelFilter(QGraphicsView *v, QObject *parent) : QObject(parent), v(v) {}
        bool eventFilter(QObject *, QEvent *e) override {
            if (e->type() == QEvent::Wheel) {
                double f = static_cast<QWheelEvent*>(e)->angleDelta().y() > 0 ? 1.15 : 1.0/1.15;
                v->scale(f, f);
                return true;
            }
            return false;
        }
    };
    grid_view->viewport()->installEventFilter(new WheelFilter(grid_view, &grid_win));

    // Buttons
    auto *btn_row   = new QHBoxLayout;
    auto *clear_btn = new QPushButton("Clear Waypoints");
    auto *begin_btn = new QPushButton("Begin Navigation");
    auto *stop_btn  = new QPushButton("Stop Navigation");
    stop_btn->setEnabled(false);
    btn_row->addWidget(clear_btn);
    btn_row->addWidget(begin_btn);
    btn_row->addWidget(stop_btn);

    grid_vbox->addWidget(grid_view);
    grid_vbox->addLayout(btn_row);
    grid_win.show();

    // Helper: rebuild waypoint scene items from current list
    auto rebuild_wp_items = [&]() {
        for (auto *item : wp_items) scene->removeItem(item);
        wp_items.clear();
        std::lock_guard<std::mutex> lk(grid_meta_mu);
        for (size_t i = 0; i < waypoints.size(); i++) {
            auto [wx, wy] = waypoints[i];
            int cx = (int)((wx - grid_meta.xMin) / grid_meta.cellSize);
            int cy = grid_meta.rows - 1 - (int)((wy - grid_meta.yMin) / grid_meta.cellSize);
            float r = 6.0f;
            auto *circle = scene->addEllipse(cx-r, cy-r, 2*r, 2*r,
                QPen(Qt::yellow, 1.5), QBrush(Qt::yellow));
            auto *label = scene->addText(QString::number(i + 1));
            label->setDefaultTextColor(Qt::black);
            label->setPos(cx - r*0.5f, cy - r*1.5f);
            label->setScale(0.5);
            wp_items.push_back(circle);
            wp_items.push_back(label);
        }
    };

    // Helper: rebuild planned-path scene items (A* output)
    auto rebuild_plan_items = [&]() {
        for (auto *item : plan_items) scene->removeItem(item);
        plan_items.clear();
        if (planned_path.empty()) return;
        std::lock_guard<std::mutex> lk(grid_meta_mu);
        if (grid_meta.cellSize <= 0 || grid_meta.rows == 0) return;

        auto toScene = [&](std::pair<float,float> wp) -> QPointF {
            float sx = (wp.first  - grid_meta.xMin) / grid_meta.cellSize;
            float sy = grid_meta.rows - 1 - (wp.second - grid_meta.yMin) / grid_meta.cellSize;
            return {sx, sy};
        };

        // Lines connecting consecutive waypoints
        QPen line_pen(QColor(0, 200, 255), 1.5, Qt::DashLine);
        for (size_t i = 1; i < planned_path.size(); i++) {
            QPointF a = toScene(planned_path[i-1]);
            QPointF b = toScene(planned_path[i]);
            plan_items.push_back(scene->addLine(a.x(), a.y(), b.x(), b.y(), line_pen));
        }
        // Dot at each intermediate waypoint (skip first/last which overlap start/goal)
        QPen dot_pen(QColor(0, 200, 255), 1);
        QBrush dot_brush(QColor(0, 200, 255));
        float r = 3.0f;
        for (size_t i = 0; i < planned_path.size(); i++) {
            QPointF p = toScene(planned_path[i]);
            plan_items.push_back(
                scene->addEllipse(p.x()-r, p.y()-r, 2*r, 2*r, dot_pen, dot_brush));
        }
    };

    // Click on grid view → add waypoint
    struct ClickFilter : QObject {
        QGraphicsView *v;
        std::mutex &meta_mu;
        GridMeta &meta;
        std::vector<std::pair<float,float>> &waypoints;
        std::function<void()> on_add;
        ClickFilter(QGraphicsView *v, std::mutex &mu, GridMeta &m,
                    std::vector<std::pair<float,float>> &wp, std::function<void()> cb,
                    QObject *parent)
            : QObject(parent), v(v), meta_mu(mu), meta(m), waypoints(wp), on_add(cb) {}
        bool eventFilter(QObject *, QEvent *e) override {
            if (e->type() == QEvent::MouseButtonPress) {
                auto *me = static_cast<QMouseEvent*>(e);
                if (me->button() == Qt::LeftButton) {
                    QPointF sp = v->mapToScene(me->pos());
                    float wx, wy;
                    {
                        std::lock_guard<std::mutex> lk(meta_mu);
                        if (meta.cellSize <= 0 || meta.rows == 0) return false;
                        wx = meta.xMin + sp.x() * meta.cellSize;
                        wy = meta.yMin + (meta.rows - 1 - sp.y()) * meta.cellSize;
                    }
                    waypoints.push_back({wx, wy});
                    std::cout << "[NAV] Waypoint " << waypoints.size()
                              << " added: (" << wx << ", " << wy << ")\n";
                    on_add();
                    return true;
                }
            }
            return false;
        }
    };
    grid_view->viewport()->installEventFilter(
        new ClickFilter(grid_view, grid_meta_mu, grid_meta, waypoints, rebuild_wp_items, &grid_win));

    // Clear button
    QObject::connect(clear_btn, &QPushButton::clicked, [&]() {
        waypoints.clear();
        planned_path.clear();
        rebuild_wp_items();
        rebuild_plan_items();
    });

    // Begin navigation — run A* from current pose through each user waypoint in sequence
    QObject::connect(begin_btn, &QPushButton::clicked, [&]() {
        if (navigator.isRunning() || waypoints.empty()) return;

        json pp = config.value("planner", json::object());
        PlannerConfig plan_cfg;
        plan_cfg.inflation_radius = pp.value("inflation_radius", 0.3f);
        plan_cfg.allow_unknown    = pp.value("allow_unknown", false);

        float xMin, yMin, cellSize;
        cv::Mat grid = slam.getOccupancyGrid(xMin, yMin, cellSize);

        std::vector<std::pair<float,float>> path;

        if (grid.empty()) {
            // No grid available — fall back to straight-line waypoints
            std::cerr << "[PLAN] No occupancy grid — using direct waypoints\n";
            path = waypoints;
        } else {
            rtabmap::Transform pose = slam.getPose();
            std::pair<float,float> cur = {pose.x(), pose.z()};

            for (const auto &goal : waypoints) {
                auto segment = Planner::plan(grid, xMin, yMin, cellSize, cur, goal, plan_cfg);
                if (segment.empty()) {
                    std::cerr << "[PLAN] No path to waypoint (" << goal.first
                              << ", " << goal.second << ") — skipping\n";
                } else {
                    // Skip the first point of each segment (it's the previous goal / start)
                    for (size_t i = path.empty() ? 0 : 1; i < segment.size(); i++)
                        path.push_back(segment[i]);
                    cur = goal;
                }
            }
        }

        if (path.empty()) {
            std::cerr << "[PLAN] No valid path found\n";
            return;
        }

        planned_path = path;
        rebuild_plan_items();

        navigator.setWaypoints(path);
        navigator.start([&slam]() { return slam.getPose(); }, nav_vel_cb);
        begin_btn->setEnabled(false);
        stop_btn->setEnabled(true);
    });

    // Stop navigation
    QObject::connect(stop_btn, &QPushButton::clicked, [&]() {
        navigator.stop();
        begin_btn->setEnabled(true);
        stop_btn->setEnabled(false);
    });

    // Poll navigator state to re-enable buttons when it finishes naturally
    QTimer nav_poll;
    QObject::connect(&nav_poll, &QTimer::timeout, [&]() {
        if (!navigator.isRunning() && !begin_btn->isEnabled()) {
            begin_btn->setEnabled(true);
            stop_btn->setEnabled(false);
        }
    });
    nav_poll.start(500);

    // Grid update timer
    QTimer grid_timer;
    QObject::connect(&grid_timer, &QTimer::timeout, [&]() {
        float xMin, yMin, cellSize;
        cv::Mat grid = slam.getOccupancyGrid(xMin, yMin, cellSize);
        if (grid.empty()) return;

        // CV_8SC1: -1=unknown, 0=free, 100=occupied
        QImage img(grid.cols, grid.rows, QImage::Format_RGB888);
        for (int r = 0; r < grid.rows; r++) {
            for (int c = 0; c < grid.cols; c++) {
                auto v = grid.at<int8_t>(r, c);
                QRgb px;
                if (v < 0)       px = qRgb(128, 128, 128);  // unknown
                else if (v == 0) px = qRgb(240, 240, 240);  // free
                else             px = qRgb(20,  20,  20);   // occupied
                img.setPixel(c, grid.rows - 1 - r, px);
            }
        }

        // Overlay trajectory
        {
            std::lock_guard<std::mutex> lock(cloud_mu);
            for (const auto &[tx, ty] : trajectory_xz) {
                int cx = (int)((tx - xMin) / cellSize);
                int cy = grid.rows - 1 - (int)((ty - yMin) / cellSize);
                if (cx >= 0 && cx < grid.cols && cy >= 0 && cy < grid.rows)
                    img.setPixel(cx, cy, qRgb(255, 50, 255));
            }
        }

        // Update pose arrow position and orientation
        {
            rtabmap::Transform pose = slam.getPose();
            if (!pose.isNull()) {
                float px = (pose.x() - xMin) / cellSize;
                float py = grid.rows - 1 - (pose.y() - yMin) / cellSize;
                float roll, pitch, yaw;
                pose.getEulerAngles(roll, pitch, yaw);
                pose_arrow->setPos(px, py);
                // Qt rotation is CW in degrees; world yaw is CCW → negate
                pose_arrow->setRotation(-yaw * 180.0f / M_PI);
                pose_arrow->setVisible(true);
            }
        }

        // Update pixmap in scene (preserve view transform)
        pmap_item->setPixmap(QPixmap::fromImage(img));
        scene->setSceneRect(img.rect());

        // Update grid metadata for click mapping
        {
            std::lock_guard<std::mutex> lk(grid_meta_mu);
            grid_meta = {xMin, yMin, cellSize, grid.rows, grid.cols};
        }

        // Refresh waypoint/path positions (grid may have grown)
        rebuild_wp_items();
        rebuild_plan_items();
    });
    grid_timer.start(2000);  // 0.5 Hz — assembly is graph-wide

    QTimer update_timer;
    QObject::connect(&update_timer, &QTimer::timeout, [&]() {
        if (!running) { app.quit(); return; }

        std::lock_guard<std::mutex> lock(cloud_mu);
        if (!latest_cloud) return;

        max_frame_id = frame_count;

        // Transform and accumulate points (only every map_add_interval frames, and only if enabled)
        bool add_to_map = add_scans_to_map && ((map_add_interval <= 1) || (frame_count % map_add_interval == 0));
        if (add_to_map) for (const auto &p : *latest_cloud) {
            float x = p.x, y = p.y, z = p.z;
            const float *d = latest_pose.data();
            MapPoint mp;
            mp.x = d[0]*x + d[1]*y + d[2]*z + d[3];
            mp.y = d[4]*x + d[5]*y + d[6]*z + d[7];
            mp.z = d[8]*x + d[9]*y + d[10]*z + d[11];
            mp.intensity = p.intensity;
            mp.frame_id = frame_count;
            map_points.push_back(mp);
        }

        // Track trajectory
        float tx = latest_pose.x(), ty = latest_pose.y(), tz = latest_pose.z();
        trajectory_xz.push_back({tx, ty});

        // Voxel downsample periodically — before building display cloud
        if (frame_count % downsample_interval == 0 && map_points.size() > 100000) {
            auto xyzi = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
            xyzi->reserve(map_points.size());
            for (const auto &mp : map_points) {
                pcl::PointXYZI p;
                p.x = mp.x; p.y = mp.y; p.z = mp.z;
                p.intensity = mp.intensity;
                xyzi->push_back(p);
            }
            xyzi->width = xyzi->size(); xyzi->height = 1;

            pcl::VoxelGrid<pcl::PointXYZI> voxel;
            voxel.setInputCloud(xyzi);
            voxel.setLeafSize(map_voxel, map_voxel, map_voxel);
            auto filtered = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
            voxel.filter(*filtered);

            map_points.clear();
            map_points.reserve(filtered->size());
            for (const auto &p : *filtered) {
                map_points.push_back({p.x, p.y, p.z, p.intensity, frame_count});
            }
        }

        // Build colored cloud for display (single code path, always from map_points)
        auto display_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        display_cloud->reserve(map_points.size());

        // Pre-compute ranges for normalized colormaps
        float zmin = 1e9, zmax = -1e9, imin = 1e9, imax = -1e9;
        if (color_mode == "height" || color_mode == "intensity") {
            for (const auto &mp : map_points) {
                if (mp.z < zmin) zmin = mp.z;
                if (mp.z > zmax) zmax = mp.z;
                if (mp.intensity < imin) imin = mp.intensity;
                if (mp.intensity > imax) imax = mp.intensity;
            }
        }
        float zrange = (zmax - zmin < 0.01f) ? 1.0f : zmax - zmin;
        float irange = (imax - imin < 1.0f) ? 1.0f : imax - imin;

        for (const auto &mp : map_points) {
            pcl::PointXYZRGB rp;
            rp.x = mp.x; rp.y = mp.y; rp.z = mp.z;

            if (color_mode == "age") {
                float age = (max_frame_id > 1) ? (float)mp.frame_id / max_frame_id : 1.0f;
                hsv2rgb((1.0f - age) * 240.0f, 1.0f, 1.0f, rp.r, rp.g, rp.b);
            } else if (color_mode == "height") {
                float t = (mp.z - zmin) / zrange;
                hsv2rgb((1.0f - t) * 240.0f, 1.0f, 1.0f, rp.r, rp.g, rp.b);
            } else if (color_mode == "flat") {
                rp.r = map_color.red(); rp.g = map_color.green(); rp.b = map_color.blue();
            } else {
                float t = (mp.intensity - imin) / irange;
                hsv2rgb((1.0f - t) * 240.0f, 1.0f, 1.0f, rp.r, rp.g, rp.b);
            }
            display_cloud->push_back(rp);
        }
        display_cloud->width = display_cloud->size();
        display_cloud->height = 1;

        viewer.addCloud("map", display_cloud);
        viewer.setCloudPointSize("map", map_point_size);

        // Current scan in scan_color
        auto scan_rgb = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        scan_rgb->reserve(latest_cloud->size());
        for (const auto &p : *latest_cloud) {
            pcl::PointXYZRGB rp;
            rp.x = p.x; rp.y = p.y; rp.z = p.z;
            rp.r = scan_color.red(); rp.g = scan_color.green(); rp.b = scan_color.blue();
            scan_rgb->push_back(rp);
        }
        scan_rgb->width = scan_rgb->size(); scan_rgb->height = 1;
        viewer.addCloud("scan", scan_rgb, latest_pose);
        viewer.setCloudPointSize("scan", scan_point_size);

        // Draw trajectory
        if (show_trajectory && trajectory_xz.size() > 1) {
            viewer.removeCloud("trajectory");
            auto traj_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
            for (size_t i = 0; i < trajectory_xz.size(); i++) {
                pcl::PointXYZRGB tp;
                tp.x = trajectory_xz[i].first;
                tp.y = trajectory_xz[i].second;
                tp.z = 0;  // project onto ground plane
                tp.r = 255; tp.g = 50; tp.b = 255; // magenta trajectory
                traj_cloud->push_back(tp);
            }
            traj_cloud->width = traj_cloud->size(); traj_cloud->height = 1;
            viewer.addCloud("trajectory", traj_cloud);
            viewer.setCloudPointSize("trajectory", 6);
        }

        latest_cloud.reset();

        viewer.setWindowTitle(QString("Livox SLAM | Frame %1 | Map: %2 pts | %3")
            .arg(frame_count)
            .arg(map_points.size())
            .arg(QString::fromStdString(color_mode)));

        // Update localization state overlay
        bool loc = slam.isLocalized();
        status_text->SetInput(loc ? "LOCALIZED" : "SEARCHING");
        if (loc) status_text->GetTextProperty()->SetColor(0.2, 1.0, 0.2);
        else     status_text->GetTextProperty()->SetColor(1.0, 0.2, 0.2);

        // Highlight the last matched map node as a yellow sphere and draw a
        // line from our current pose to it, so you can see which node is
        // anchoring localization.
        auto [closure_id, closure_pose] = slam.getLastLoopClosure();
        if (closure_id > 0 && !closure_pose.isNull()) {
            viewer.addOrUpdateSphere("closure_node", closure_pose, 0.3f,
                                     QColor(255, 255, 0), /*foreground*/true);
            viewer.addOrUpdateLine("closure_link", latest_pose, closure_pose,
                                   QColor(255, 255, 0), /*arrow*/false, /*foreground*/true);
        }

        viewer.refreshView();
    });
    update_timer.start(33);

    int ret = app.exec();
    running = false;
    source_stop();

    std::cout << "\nFinal: " << slam.getFrameCount() << " frames, "
              << map_points.size() << " map points\n"
              << "Database saved to: " << slam.getDatabasePath() << "\n";
    return ret;
}
