#pragma once
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <numeric>
#include <vector>

namespace telemetry {

// High-resolution timestamp in nanoseconds
inline std::int64_t now_ns() noexcept {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

class LatencyTracker {
public:
    explicit LatencyTracker(const char* name, std::size_t reserve = 10'000)
        : name_{name}
    {
        samples_.reserve(reserve);
    }

    // Call at the send side with a token
    [[nodiscard]] std::int64_t mark_send() noexcept {
        return now_ns();
    }

    // Call at the receive side with the token from mark_send()
    void mark_recv(std::int64_t send_ts) noexcept {
        const std::int64_t latency = now_ns() - send_ts;
        if (samples_.size() < samples_.capacity())
            samples_.push_back(latency);
    }

    // Print full report — call once at shutdown
    void report() const noexcept {
        if (samples_.empty()) {
            std::printf("[%s] No samples collected.\n", name_);
            return;
        }

        std::vector<std::int64_t> sorted = samples_;
        std::sort(sorted.begin(), sorted.end());

        const double mean = static_cast<double>(
            std::accumulate(sorted.begin(), sorted.end(), std::int64_t{0}))
            / static_cast<double>(sorted.size());

        const double variance = [&] {
            double v = 0.0;
            for (auto s : sorted)
                v += (static_cast<double>(s) - mean) *
                     (static_cast<double>(s) - mean);
            return v / static_cast<double>(sorted.size());
        }();

        const auto p = [&](double pct) {
            const std::size_t idx =
                static_cast<std::size_t>(pct / 100.0 *
                static_cast<double>(sorted.size() - 1));
            return sorted[idx] / 1000.0; // ns → µs
        };

        std::printf("\n═══ Latency Report: %s ═══\n", name_);
        std::printf("  Samples : %zu\n",        sorted.size());
        std::printf("  Mean    : %.2f µs\n",    mean   / 1000.0);
        std::printf("  Min     : %.2f µs\n",    p(0));
        std::printf("  p50     : %.2f µs\n",    p(50));
        std::printf("  p95     : %.2f µs\n",    p(95));
        std::printf("  p99     : %.2f µs\n",    p(99));
        std::printf("  Max     : %.2f µs\n",    p(100));
        std::printf("  Jitter  : %.2f µs\n",    std::sqrt(variance) / 1000.0);
        std::printf("══════════════════════════════\n\n");
    }

private:
    const char*              name_;
    std::vector<std::int64_t> samples_;
};

} // namespace telemetry
