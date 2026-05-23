#include "telemetry/imu_simulator.hpp"
#include <chrono>
#include <cmath>

namespace telemetry {

ImuSimulator::ImuSimulator(unsigned seed) : rng_{seed} {}

// ── Core noise model ──────────────────────────────────────────────────────
// true_val  → what a perfect sensor would read
// bias      → current random walk state (mutated each call)
// sigma_w   → white noise standard deviation
// sigma_b   → bias walk standard deviation per sqrt(second)
//
// Output = true_val + bias + white_noise
// Bias update: b_k = b_{k-1} + N(0, sigma_b² · dt)
float ImuSimulator::apply_noise(float true_val, float& bias,
                                float sigma_w,  float sigma_b) {
    bias += gauss_(rng_) * sigma_b * std::sqrt(kDt);
    return true_val + bias + gauss_(rng_) * sigma_w;
}

ImuPacket ImuSimulator::next_sample() {
    // True values — simulating a drone hovering with slight gravity reading
    constexpr float kTrueAccel[3] = {0.0f, 0.0f, 9.81f};
    constexpr float kTrueGyro[3]  = {0.0f, 0.0f, 0.0f};

    const auto now_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();

    ImuPacket pkt{};
    pkt.timestamp_us = now_us;

    for (int i = 0; i < 3; ++i) {
        pkt.accel[i] = apply_noise(kTrueAccel[i], accel_bias_[i],
                                   kSigmaW_a, kSigmaB_a);
        pkt.gyro[i]  = apply_noise(kTrueGyro[i],  gyro_bias_[i],
                                   kSigmaW_g, kSigmaB_g);
    }
    return pkt;
}

} // namespace telemetry
