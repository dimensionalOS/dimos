// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// LCM implementation of the Transport seam. A single receive thread runs the
// LCM handle loop and demuxes each message to the callbacks registered for its
// channel, mirroring the Rust LcmTransport. Publishing is a direct, thread-safe
// lcm_publish. The per-channel publish workers that decouple slow publishes live
// in the module runtime, not here.

#pragma once

#include <lcm/lcm-cpp.hpp>

#include <atomic>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "dimos/native/log.hpp"
#include "dimos/native/transport.hpp"
#include "dimos/native/transport_selection.hpp"

namespace dimos::native {

class LcmTransport : public Transport {
public:
    LcmTransport() {
        if (!lcm_.good()) {
            throw std::runtime_error("LcmTransport: failed to initialize LCM");
        }
    }

    ~LcmTransport() override {
        running_.store(false, std::memory_order_relaxed);
        if (recv_thread_.joinable()) {
            recv_thread_.join();
        }
    }

    LcmTransport(const LcmTransport&) = delete;
    LcmTransport& operator=(const LcmTransport&) = delete;

    void publish(const std::string& channel, std::vector<uint8_t> data) override {
        int rc = lcm_.publish(channel, data.data(), static_cast<unsigned int>(data.size()));
        if (rc != 0) {
            DIMOS_ERROR_THROTTLED(log::from_secs(1), "lcm publish failed",
                                  log::Field("channel", channel),
                                  log::Field("rc", static_cast<std::int64_t>(rc)));
        }
    }

    void subscribe(const std::string& channel, Dispatch on_msg) override {
        bool first_for_channel = false;
        {
            std::lock_guard<std::mutex> lock(routes_mu_);
            auto it = routes_.find(channel);
            std::vector<Dispatch> updated;
            if (it != routes_.end()) {
                updated = *it->second;
            } else {
                first_for_channel = true;
            }
            updated.push_back(std::move(on_msg));
            routes_[channel] = std::make_shared<const std::vector<Dispatch>>(std::move(updated));
        }
        // One LCM subscription per channel. Extra callbacks fan out in on_lcm_message.
        if (first_for_channel) {
            lcm_.subscribe(channel, &LcmTransport::on_lcm_message, this);
        }
        ensure_recv_thread();
    }

private:
    void on_lcm_message(const lcm::ReceiveBuffer* rbuf, const std::string& channel) {
        std::shared_ptr<const std::vector<Dispatch>> handlers;
        {
            std::lock_guard<std::mutex> lock(routes_mu_);
            auto it = routes_.find(channel);
            if (it == routes_.end()) {
                return;
            }
            handlers = it->second;
        }
        const auto* data = static_cast<const uint8_t*>(rbuf->data);
        for (const auto& cb : *handlers) {
            cb(data, rbuf->data_size);
        }
    }

    void ensure_recv_thread() {
        bool expected = false;
        if (running_.compare_exchange_strong(expected, true)) {
            recv_thread_ = std::thread([this] {
                while (running_.load(std::memory_order_relaxed)) {
                    int rc = lcm_.handleTimeout(kHandleTimeoutMs);
                    if (rc < 0) {
                        DIMOS_ERROR_THROTTLED(log::from_secs(1), "lcm handleTimeout error",
                                              log::Field("rc", static_cast<std::int64_t>(rc)));
                    }
                }
            });
        }
    }

    static constexpr int kHandleTimeoutMs = 100;

    lcm::LCM lcm_;
    std::mutex routes_mu_;
    std::unordered_map<std::string, std::shared_ptr<const std::vector<Dispatch>>> routes_;
    std::atomic<bool> running_{false};
    std::thread recv_thread_;
};

/// Construct the transport named by `DIMOS_TRANSPORT`. Errors clearly for zenoh
/// (Rust-only) or any unknown/unset value, mirroring the Rust runtime.
inline std::unique_ptr<Transport> make_transport_from_env() {
    const char* env = std::getenv("DIMOS_TRANSPORT");
    std::string name = env != nullptr ? env : "";
    require_supported_transport(name);
    return std::make_unique<LcmTransport>();
}

}  // namespace dimos::native
