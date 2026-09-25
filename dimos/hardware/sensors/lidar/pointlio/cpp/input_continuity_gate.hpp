#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>

// Packet source timestamps, not wall-clock arrival timestamps. Caller must
// serialize observe/status and publication. No reset: recovery is a new session.
class InputContinuityGate {
public:
    enum class Stream { lidar = 0, imu = 1 };
    enum class Reason { backwards, gap };
    struct Fault {
        Stream stream;
        Reason reason;
        uint64_t previous_ns;
        uint64_t current_ns;
    };

    explicit InputContinuityGate(double maximum_gap_s) {
        // Zero is explicit opt-out; do not change legacy behavior implicitly.
        if (!std::isfinite(maximum_gap_s) || maximum_gap_s < 0 ||
            maximum_gap_s > 86400.)
            throw std::invalid_argument("maximum_input_gap_s must be finite in [0,86400]");
        enabled_ = maximum_gap_s > 0;
        maximum_gap_ns_ = static_cast<uint64_t>(maximum_gap_s * 1e9);
        if (enabled_ && maximum_gap_ns_ == 0)
            throw std::invalid_argument("maximum_input_gap_s must be at least 1ns");
    }

    bool observe(Stream stream, uint64_t timestamp_ns) {
        if (fault_) return false;
        if (!enabled_) return true;
        auto& previous = last_[static_cast<unsigned>(stream)];
        if (previous) {
            // Compare before subtraction to avoid unsigned wraparound.
            if (timestamp_ns < *previous) {
                fault_ = Fault{stream, Reason::backwards, *previous, timestamp_ns};
                return false;
            }
            if (timestamp_ns - *previous > maximum_gap_ns_) {
                fault_ = Fault{stream, Reason::gap, *previous, timestamp_ns};
                return false;
            }
        }
        previous = timestamp_ns;
        return true;
    }
    const std::optional<Fault>& fault() const { return fault_; }
private:
    bool enabled_;
    uint64_t maximum_gap_ns_;
    std::array<std::optional<uint64_t>, 2> last_{};
    std::optional<Fault> fault_;
};
