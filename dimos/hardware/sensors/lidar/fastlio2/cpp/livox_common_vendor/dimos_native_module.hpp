// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Lightweight header-only helper for dimos NativeModule C++ binaries.
// Parses --<port_name> <topic> CLI args passed by the Python NativeModule wrapper.

#pragma once

#include <atomic>
#include <map>
#include <stdexcept>
#include <string>

#include "std_msgs/Header.hpp"
#include "std_msgs/Time.hpp"

namespace dimos {

class NativeModule {
public:
    NativeModule(int argc, char** argv) {
        for (int i = 1; i < argc; ++i) {
            std::string arg(argv[i]);
            if (arg.size() > 2 && arg[0] == '-' && arg[1] == '-' && i + 1 < argc) {
                args_[arg.substr(2)] = argv[++i];
            }
        }
    }

    /// Get the full LCM channel string for a declared port.
    /// Format is "<topic>#<msg_type>", e.g. "/pointcloud#sensor_msgs.PointCloud2".
    /// This is the exact channel name used by Python LCMTransport subscribers.
    const std::string& topic(const std::string& port) const {
        auto it = args_.find(port);
        if (it == args_.end()) {
            throw std::runtime_error("NativeModule: no topic for port '" + port + "'");
        }
        return it->second;
    }

    /// Get a string arg value, or a default if not present.
    std::string arg(const std::string& key, const std::string& default_val = "") const {
        auto it = args_.find(key);
        return it != args_.end() ? it->second : default_val;
    }

    /// Get a float arg value, or a default if not present.
    float arg_float(const std::string& key, float default_val = 0.0f) const {
        auto it = args_.find(key);
        return it != args_.end() ? std::stof(it->second) : default_val;
    }

    /// Get an int arg value, or a default if not present.
    int arg_int(const std::string& key, int default_val = 0) const {
        auto it = args_.find(key);
        return it != args_.end() ? std::stoi(it->second) : default_val;
    }

    /// Check if a port/arg was provided.
    bool has(const std::string& key) const {
        return args_.count(key) > 0;
    }

private:
    std::map<std::string, std::string> args_;
};

/// Convert seconds (double) to a ROS-style Time message.
inline std_msgs::Time time_from_seconds(double t) {
    std_msgs::Time ts;
    ts.sec = static_cast<int32_t>(t);
    ts.nsec = static_cast<int32_t>((t - ts.sec) * 1e9);
    return ts;
}

/// Build a stamped Header with auto-incrementing sequence number.
inline std_msgs::Header make_header(const std::string& frame_id, double ts) {
    static std::atomic<int32_t> seq{0};
    std_msgs::Header h;
    h.seq = seq.fetch_add(1, std::memory_order_relaxed);
    h.stamp = time_from_seconds(ts);
    h.frame_id = frame_id;
    return h;
}

}  // namespace dimos
