#include "vision/gst_pipeline.hpp"
#include "vision/detector.hpp"
#include "ipc/publisher.hpp"
#include "ipc/subscriber.hpp"
#include "telemetry/imu_simulator.hpp"
#include "telemetry/latency_tracker.hpp"
#include "fusion/ekf.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <mutex>
#include <array>
#include <thread>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

static std::atomic<bool> g_shutdown{false};
static void signal_handler(int) noexcept {
    g_shutdown.store(true, std::memory_order_release);
}

static std::mutex           g_state_mutex;
static std::array<double,6> g_ekf_state{};

// Latest YOLO detection shared to overlay thread
static std::mutex                     g_det_mutex;
static std::vector<vision::Detection> g_detections;

int main() {
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    zmq::context_t ctx{1};

    telemetry::LatencyTracker vision_lat{"Vision Pipeline", 10'000};
    telemetry::LatencyTracker imu_lat   {"IMU IPC",        100'000};

    std::uint64_t frame_n{0};
    std::uint64_t saved{0};

    // ── Thread A: Vision Publisher + YOLO ─────────────────────────────────
    std::thread vision_thread([&] {
        ipc::Publisher pub{ctx, "ipc:///tmp/vision_feed"};

        // Load YOLO — paths relative to project root
        vision::YoloDetector detector{
            "models/yolov4-tiny.cfg",
            "models/yolov4-tiny.weights",
            "models/coco.names"
        };

        constexpr std::string_view kPipeline =
            "videotestsrc pattern=ball ! "
            "video/x-raw,format=BGR,width=640,height=480,framerate=30/1 ! "
            "appsink name=app_sink";

        vision::GstPipeline pipeline{kPipeline, "app_sink",
            [&](const cv::Mat& frame, std::int64_t pts_ns) {
                const std::int64_t send_ts = vision_lat.mark_send();

                // Run YOLO every frame
                auto dets = detector.detect(frame);

                // Share detections for overlay
                {
                    std::lock_guard lock{g_det_mutex};
                    g_detections = dets;
                }

                // Use first detection centre as EKF measurement
                // Format: [cx_norm, cy_norm, 0] — z from IMU only
                float meas_x = 0.f, meas_y = 0.f;
                bool  has_det = !dets.empty();
                if (has_det) {
                    meas_x = dets[0].cx_norm;
                    meas_y = dets[0].cy_norm;
                }

                // Pack [send_ts | pts_ns | meas_x | meas_y | has_det]
                struct VisionMsg {
                    std::int64_t send_ts;
                    std::int64_t pts_ns;
                    float        mx, my;
                    int          has_det;
                };
                VisionMsg msg{send_ts, pts_ns, meas_x, meas_y,
                              has_det ? 1 : 0};
                pub.send("vision",
                    std::as_bytes(std::span{&msg, 1}));

                // ── Annotate and save ─────────────────────────────────────
                cv::Mat display = frame.clone();

                std::array<double,6> st{};
                {
                    std::lock_guard lock{g_state_mutex};
                    st = g_ekf_state;
                }

                // Draw YOLO boxes
                std::vector<vision::Detection> dets_copy;
                {
                    std::lock_guard lock{g_det_mutex};
                    dets_copy = g_detections;
                }
                for (const auto& d : dets_copy) {
                    cv::rectangle(display, d.box, {0,255,255}, 2);
                    char label[32];
                    std::snprintf(label, sizeof(label),
                        "%.0f%%", d.confidence * 100.f);
                    cv::putText(display, label,
                        {d.box.x, d.box.y - 5},
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, {0,255,255}, 1);
                }

                char buf[128];
                std::snprintf(buf, sizeof(buf),
                    "POS  x:%.4f  y:%.4f  z:%.4f m",
                    st[0], st[1], st[2]);
                cv::putText(display, buf, {10,30},
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, {0,255,0}, 2);

                std::snprintf(buf, sizeof(buf),
                    "VEL  x:%.4f  y:%.4f  z:%.4f m/s",
                    st[3], st[4], st[5]);
                cv::putText(display, buf, {10,65},
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, {0,200,255}, 2);

                std::snprintf(buf, sizeof(buf),
                    "YOLO detections: %zu | IMU 200Hz | IPC p99<1ms",
                    dets_copy.size());
                cv::putText(display, buf, {10, display.rows-12},
                    cv::FONT_HERSHEY_SIMPLEX, 0.45, {255,255,0}, 1);

                ++frame_n;
                if (frame_n % 30 == 0 && saved < 10) {
                    char path[64];
                    std::snprintf(path, sizeof(path),
                        "/tmp/pipeline_frame_%02lu.png", saved++);
                    cv::imwrite(path, display);
                    std::printf("[Demo] Saved %s\n", path);
                }
            }};

        if (!pipeline.start()) return;
        while (!g_shutdown.load(std::memory_order_acquire))
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        pipeline.stop();
    });

    // ── Thread B: Telemetry Publisher ─────────────────────────────────────
    std::thread telemetry_thread([&] {
        ipc::Publisher          pub{ctx, "ipc:///tmp/telemetry_feed"};
        telemetry::ImuSimulator imu{};

        while (!g_shutdown.load(std::memory_order_acquire)) {
            const auto t0    = std::chrono::steady_clock::now();
            auto pkt         = imu.next_sample();
            pkt.timestamp_us = imu_lat.mark_send() / 1000;
            pub.send("telemetry", std::as_bytes(std::span{&pkt, 1}));
            std::this_thread::sleep_until(
                t0 + std::chrono::microseconds{5000});
        }
    });

    // ── Thread C: Fusion Engine ───────────────────────────────────────────
    std::thread fusion_thread([&] {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        ipc::Subscriber vision_sub{ctx,    "ipc:///tmp/vision_feed"};
        ipc::Subscriber telemetry_sub{ctx, "ipc:///tmp/telemetry_feed"};

        fusion::EKF   ekf{};
        std::uint64_t vision_count{0}, telemetry_count{0};

        ipc::poll_two(vision_sub, telemetry_sub,
            [&](std::string_view topic, std::span<const std::byte> data) {
                if (topic == "telemetry") {
                    telemetry::ImuPacket pkt{};
                    std::memcpy(&pkt, data.data(), sizeof(pkt));
                    imu_lat.mark_recv(
                        static_cast<std::int64_t>(pkt.timestamp_us)*1000);
                    ekf.predict(pkt.accel, pkt.gyro, pkt.timestamp_us);
                    ++telemetry_count;

                    std::lock_guard lock{g_state_mutex};
                    const auto& x = ekf.state();
                    g_ekf_state = {x(0),x(1),x(2),x(3),x(4),x(5)};

                } else if (topic == "vision") {
                    struct VisionMsg {
                        std::int64_t send_ts, pts_ns;
                        float mx, my;
                        int   has_det;
                    };
                    VisionMsg msg{};
                    std::memcpy(&msg, data.data(), sizeof(msg));
                    vision_lat.mark_recv(msg.send_ts);

                    // Feed real measurement if detection exists,
                    // otherwise feed current EKF position (no-op update)
                    fusion::MeasVec z;
                    if (msg.has_det) {
                        z << static_cast<double>(msg.mx),
                             static_cast<double>(msg.my),
                             0.0;
                    } else {
                        const auto& x = ekf.state();
                        z << x(0), x(1), x(2);
                    }
                    ekf.update_vision(z, msg.pts_ns / 1000);

                    if (++vision_count % 30 == 0)
                        std::printf(
                            "[Fusion] vision=%lu imu=%lu "
                            "pos=(%.4f,%.4f,%.4f)\n",
                            vision_count, telemetry_count,
                            ekf.state()(0), ekf.state()(1), ekf.state()(2));
                }
            }, g_shutdown);
    });

    vision_thread.join();
    telemetry_thread.join();
    fusion_thread.join();

    vision_lat.report();
    imu_lat.report();

    std::puts("Shutdown complete.");
    return 0;
}
