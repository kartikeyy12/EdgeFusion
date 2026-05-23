#pragma once
#include <cstdint>
#include <random>

namespace telemetry {

// MAVLink-compatible IMU measurement packet
struct ImuPacket {
    std::int64_t timestamp_us;  // microseconds
    float accel[3];             // m/s² — x, y, z
    float gyro[3];              // rad/s — x, y, z
};

// MEMS noise model:
//   white_noise  → zero-mean Gaussian spike each sample  ~ N(0, sigma_w²)
//   random_walk  → bias drifts slowly each sample        ~ N(0, sigma_b²·dt)
class ImuSimulator {
public:
    explicit ImuSimulator(unsigned seed = 42);
    ImuPacket next_sample();   // call at 200Hz

private:
    float apply_noise(float true_val, float& bias,
                      float sigma_w, float sigma_b);

    std::mt19937                          rng_;
    std::normal_distribution<float>       gauss_{0.0f, 1.0f};

    float accel_bias_[3]{};
    float gyro_bias_[3]{};

    static constexpr float kDt       = 1.0f / 200.0f; // seconds
    static constexpr float kSigmaW_a = 0.05f;          // accel white noise
    static constexpr float kSigmaB_a = 0.001f;         // accel bias walk
    static constexpr float kSigmaW_g = 0.005f;         // gyro white noise
    static constexpr float kSigmaB_g = 0.0001f;        // gyro bias walk
};

} // namespace telemetry
