// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Zenoh implementation of the Transport seam over zenoh-c. The session settings
// and per-channel publisher QoS come from the coordinator's launch line, the
// same `session` and `qos` blocks the Rust native module reads.

#pragma once

#include <zenoh.h>

#ifndef Z_FEATURE_UNSTABLE_API
#error "dimos ZenohTransport needs zenoh-c built with Z_FEATURE_UNSTABLE_API (publisher reliability)"
#endif

#include <nlohmann/json.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "dimos/native/log.hpp"
#include "dimos/native/transport.hpp"

namespace dimos::native {

/// Zenoh keys can't start with '/'.
inline std::string zenoh_key(const std::string& channel) {
    return channel.rfind('/', 0) == 0 ? channel.substr(1) : channel;
}

/// The `zenoh config key -> JSON text` inserts for the launch line's `session`
/// block. Python owns every value, so each field is required and unknown ones
/// are rejected. Empty endpoint lists and a zero timeout keep zenoh's defaults.
inline std::vector<std::pair<std::string, std::string>> zenoh_config_inserts(
    const nlohmann::json& session) {
    static const std::set<std::string> known = {"mode",       "connect", "listen",
                                                "multicast",  "scout_addr", "gossip",
                                                "interface",  "connect_timeout_ms"};
    if (!session.is_object()) {
        throw std::runtime_error("zenoh session settings must be a JSON object");
    }
    for (const auto& item : session.items()) {
        if (known.count(item.key()) == 0) {
            throw std::runtime_error("unknown zenoh session setting: " + item.key());
        }
    }
    auto require = [&](const char* key) -> const nlohmann::json& {
        auto it = session.find(key);
        if (it == session.end()) {
            throw std::runtime_error(std::string("missing zenoh session setting: ") + key);
        }
        return *it;
    };
    std::vector<std::pair<std::string, std::string>> out = {
        {"mode", require("mode").dump()},
        {"scouting/multicast/enabled", require("multicast").dump()},
        {"scouting/multicast/interface", require("interface").dump()},
        {"scouting/gossip/enabled", require("gossip").dump()},
    };
    if (!require("scout_addr").get<std::string>().empty()) {
        out.emplace_back("scouting/multicast/address", session["scout_addr"].dump());
    }
    if (!require("connect").empty()) {
        out.emplace_back("connect/endpoints", session["connect"].dump());
    }
    if (!require("listen").empty()) {
        out.emplace_back("listen/endpoints", session["listen"].dump());
    }
    if (require("connect_timeout_ms").get<std::int64_t>() > 0) {
        out.emplace_back("connect/timeout_ms", session["connect_timeout_ms"].dump());
    }
    return out;
}

/// Publisher QoS for one channel. Unset fields keep zenoh's defaults.
struct ChannelQos {
    std::optional<z_reliability_t> reliability;
    std::optional<z_congestion_control_t> congestion_control;
    std::optional<z_locality_t> locality;
};

/// Parse the launch line's `qos` block (channel -> {reliability,
/// congestion_control, locality}). Unknown or absent fields keep defaults.
inline std::unordered_map<std::string, ChannelQos> parse_channel_qos(const nlohmann::json& qos) {
    std::unordered_map<std::string, ChannelQos> out;
    if (!qos.is_object()) {
        return out;
    }
    for (const auto& item : qos.items()) {
        const nlohmann::json& entry = item.value();
        if (!entry.is_object()) {
            continue;
        }
        auto field = [&](const char* key) {
            auto it = entry.find(key);
            return it != entry.end() && it->is_string() ? it->get<std::string>() : std::string();
        };
        ChannelQos q;
        const std::string reliability = field("reliability");
        if (reliability == "reliable") {
            q.reliability = Z_RELIABILITY_RELIABLE;
        } else if (reliability == "best_effort") {
            q.reliability = Z_RELIABILITY_BEST_EFFORT;
        }
        const std::string congestion = field("congestion_control");
        if (congestion == "drop") {
            q.congestion_control = Z_CONGESTION_CONTROL_DROP;
        } else if (congestion == "block") {
            q.congestion_control = Z_CONGESTION_CONTROL_BLOCK;
        }
        const std::string locality = field("locality");
        if (locality == "session_local") {
            q.locality = Z_LOCALITY_SESSION_LOCAL;
        } else if (locality == "remote") {
            q.locality = Z_LOCALITY_REMOTE;
        } else if (locality == "any") {
            q.locality = Z_LOCALITY_ANY;
        }
        out[item.key()] = q;
    }
    return out;
}

class ZenohTransport : public Transport {
public:
    /// `launch` is the coordinator's stdin blob. Its `session` block configures
    /// the session (absent keeps zenoh's defaults) and `qos` the publishers.
    explicit ZenohTransport(const nlohmann::json& launch = nlohmann::json::object()) {
        zc_init_log_from_env_or("warn");
        const nlohmann::json session = launch.value("session", nlohmann::json());
        z_owned_config_t config;
        if (z_config_default(&config) != Z_OK) {
            throw std::runtime_error("zenoh: default config failed");
        }
        if (session.is_null()) {
            log::warn(
                "no `session` block on the launch line, opening zenoh's defaults; "
                "add one to pin the mode, interface and endpoints");
        } else {
            for (const auto& [key, value] : zenoh_config_inserts(session)) {
                if (zc_config_insert_json5(z_loan_mut(config), key.c_str(), value.c_str()) !=
                    Z_OK) {
                    z_drop(z_move(config));
                    throw std::runtime_error("zenoh config rejected " + key + "=" + value);
                }
            }
        }
        if (z_open(&session_, z_move(config), nullptr) != Z_OK) {
            throw std::runtime_error("zenoh: session open failed");
        }
        qos_ = parse_channel_qos(launch.value("qos", nlohmann::json()));
        log::info("zenoh session opened",
                  {log::Field("zid", zid()), log::Field("mode", session.value("mode", "default")),
                   log::Field("connect", session.value("connect", nlohmann::json::array()).dump()),
                   log::Field("listen", session.value("listen", nlohmann::json::array()).dump())});
        if (!session.is_null()) {
            await_connect(session);
        }
    }

    ~ZenohTransport() override {
        for (auto& entry : publishers_) {
            z_drop(z_move(entry.second));
        }
        z_drop(z_move(session_));
    }

    ZenohTransport(const ZenohTransport&) = delete;
    ZenohTransport& operator=(const ZenohTransport&) = delete;

    void publish(const std::string& channel, std::vector<uint8_t> data) override {
        const z_loaned_publisher_t* publisher = publisher_for(channel);
        if (publisher == nullptr) {
            return;
        }
        z_owned_bytes_t payload;
        z_bytes_copy_from_buf(&payload, data.data(), data.size());
        if (z_publisher_put(publisher, z_move(payload), nullptr) != Z_OK) {
            DIMOS_ERROR_THROTTLED(log::from_secs(1), "zenoh publish failed",
                                  log::Field("channel", channel));
        }
    }

    void subscribe(const std::string& channel, Dispatch on_msg) override {
        const std::string key = zenoh_key(channel);
        z_view_keyexpr_t keyexpr;
        if (z_view_keyexpr_from_str(&keyexpr, key.c_str()) != Z_OK) {
            throw std::runtime_error("zenoh: invalid key expression: " + key);
        }
        // Owned by the closure; freed by drop_dispatch when the session closes.
        auto* dispatch = new Dispatch(std::move(on_msg));
        z_owned_closure_sample_t callback;
        z_closure(&callback, on_sample, drop_dispatch, dispatch);
        if (z_declare_background_subscriber(z_loan(session_), z_loan(keyexpr), z_move(callback),
                                            nullptr) != Z_OK) {
            throw std::runtime_error("zenoh: subscribe failed: " + key);
        }
    }

private:
    static void on_sample(z_loaned_sample_t* sample, void* context) {
        z_owned_slice_t slice;
        if (z_bytes_to_slice(z_sample_payload(sample), &slice) != Z_OK) {
            return;
        }
        (*static_cast<Dispatch*>(context))(z_slice_data(z_loan(slice)), z_slice_len(z_loan(slice)));
        z_drop(z_move(slice));
    }

    static void drop_dispatch(void* context) { delete static_cast<Dispatch*>(context); }

    static void count_zid(const z_id_t* /*zid*/, void* context) {
        ++*static_cast<std::size_t*>(context);
    }

    std::string zid() const {
        const z_id_t id = z_info_zid(z_loan(session_));
        z_owned_string_t text;
        z_id_to_string(&id, &text);
        std::string out(z_string_data(z_loan(text)), z_string_len(z_loan(text)));
        z_drop(z_move(text));
        return out;
    }

    /// Remote sessions this one currently has a link to.
    std::size_t linked_count() const {
        std::size_t count = 0;
        for (auto info : {z_info_peers_zid, z_info_routers_zid}) {
            z_owned_closure_zid_t callback;
            z_closure(&callback, count_zid, nullptr, &count);
            info(z_loan(session_), z_move(callback));
        }
        return count;
    }

    /// Block until the dialed endpoints have links, bounded by the timeout. A
    /// peer opens before its endpoints are dialed, so without this the first
    /// published messages have nowhere to go.
    void await_connect(const nlohmann::json& session) const {
        const auto& endpoints = session["connect"];
        const std::int64_t timeout_ms = session["connect_timeout_ms"].get<std::int64_t>();
        if (endpoints.empty() || timeout_ms <= 0) {
            return;
        }
        // A client holds one link; zenoh keeps the first endpoint that answers.
        // ponytail: counts any linked session, not the dialed ones specifically;
        // match link addresses against the endpoints if multicast peers fool it.
        const std::size_t needed = session["mode"] == "client" ? 1 : endpoints.size();
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
        while (linked_count() < needed) {
            if (std::chrono::steady_clock::now() >= deadline) {
                log::warn("zenoh endpoints not linked, published messages may be dropped",
                          {log::Field("endpoints", endpoints.dump()),
                           log::Field("timeout_ms", timeout_ms)});
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    /// The publisher for `channel`, declared on first use with its QoS. Null if
    /// the declaration failed (logged). Released before the put so a stalled
    /// channel can't block the others.
    const z_loaned_publisher_t* publisher_for(const std::string& channel) {
        std::lock_guard<std::mutex> lock(publishers_mu_);
        auto it = publishers_.find(channel);
        if (it != publishers_.end()) {
            return z_loan(it->second);
        }
        const std::string key = zenoh_key(channel);
        z_view_keyexpr_t keyexpr;
        z_owned_publisher_t publisher;
        z_publisher_options_t options;
        z_publisher_options_default(&options);
        if (auto qos = qos_.find(channel); qos != qos_.end()) {
            if (qos->second.reliability) {
                options.reliability = *qos->second.reliability;
            }
            if (qos->second.congestion_control) {
                options.congestion_control = *qos->second.congestion_control;
            }
            if (qos->second.locality) {
                options.allowed_destination = *qos->second.locality;
            }
        }
        if (z_view_keyexpr_from_str(&keyexpr, key.c_str()) != Z_OK ||
            z_declare_publisher(z_loan(session_), &publisher, z_loan(keyexpr), &options) != Z_OK) {
            DIMOS_ERROR_THROTTLED(log::from_secs(1), "zenoh declare publisher failed",
                                  log::Field("channel", channel));
            return nullptr;
        }
        // Node-based map: the stored publisher's address survives later inserts.
        return z_loan(publishers_.emplace(channel, publisher).first->second);
    }

    z_owned_session_t session_;
    std::unordered_map<std::string, ChannelQos> qos_;
    std::mutex publishers_mu_;
    std::unordered_map<std::string, z_owned_publisher_t> publishers_;
};

}  // namespace dimos::native
