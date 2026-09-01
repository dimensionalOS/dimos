// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// LCM implementation of the Transport seam. One receive thread runs the LCM
// handle loop and demuxes to the callbacks registered for each channel.

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

    /// LCM has no per-topic publisher settings and no notion of a session-local
    /// publisher, so a baked host cannot hide an internal hop on this transport.
    void set_publisher_qos(const nlohmann::json& qos) override {
        if (!qos.is_object()) {
            return;
        }
        std::string suppressed;
        for (const auto& entry : qos.items()) {
            if (entry.value().is_object() && entry.value().contains("locality")) {
                suppressed += suppressed.empty() ? entry.key() : ", " + entry.key();
            }
        }
        if (!suppressed.empty()) {
            log::warn("LCM cannot suppress a topic; these stay visible on the multicast bus",
                      {log::Field("channels", suppressed)});
        }
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

}  // namespace dimos::native
