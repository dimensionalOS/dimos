// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Zenoh transport for dimos C++ native modules. It is the peer of
// native/rust/dimos-module/src/zenoh.rs and reads the same launch line, so a
// C++ module and a rust module in one blueprint land on the same wire.

#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <netdb.h>
#include <sys/socket.h>

#include <nlohmann/json.hpp>
#include <zenoh.hxx>

#include "dimos/native/log.hpp"
#include "dimos/native/transport.hpp"

// Reliability QoS and the link list this transport waits on are both behind
// zenoh-c's unstable API, and the rust transport uses them unconditionally. A
// build without them would silently drop the messages it publishes first.
#if !defined(Z_FEATURE_UNSTABLE_API)
#error "the zenoh transport needs zenoh-c built with -DZENOHC_BUILD_WITH_UNSTABLE_API=ON"
#endif

namespace dimos::native {

/// The launch key carrying the session settings python resolved.
inline constexpr const char* kSessionKey = "session";

/// Zenoh keys can't start with '/'.
inline std::string zenoh_key(const std::string& channel) {
    return channel.rfind('/', 0) == 0 ? channel.substr(1) : channel;
}

namespace zenoh_detail {

/// Poll interval while waiting for the dialed endpoints to link.
constexpr std::chrono::milliseconds kConnectPoll{50};

/// The session settings python sends on the launch line. It owns every value
/// and resolves every derived one, so no field here has a default.
struct SessionSettings {
    std::string mode;
    std::vector<std::string> connect;
    std::vector<std::string> listen;
    bool multicast = false;
    std::string scout_addr;
    bool gossip = false;
    std::string interface;
    std::uint64_t connect_timeout_ms = 0;
};

/// Read `key` off `object`, requiring the JSON type `T` maps to.
template <class T>
T require_field(const nlohmann::json& object, const std::string& key) {
    auto it = object.find(key);
    if (it == object.end()) {
        throw std::runtime_error("zenoh session settings: missing field '" + key + "'");
    }
    // Without this a negative number is cast rather than rejected, turning a
    // bad timeout into a wait of a few hundred million years.
    if constexpr (std::is_unsigned_v<T> && !std::is_same_v<T, bool>) {
        if (!it->is_number_unsigned()) {
            throw std::runtime_error("zenoh session settings: field '" + key +
                                     "' must be a non-negative whole number");
        }
    }
    try {
        return it->get<T>();
    } catch (const std::exception& e) {
        throw std::runtime_error("zenoh session settings: field '" + key + "': " + e.what());
    }
}

/// The settings the launch carried, empty when it carried none. A module
/// started by hand keeps zenoh's own defaults.
inline std::optional<SessionSettings> settings_from_launch(const nlohmann::json& launch) {
    if (!launch.is_object()) {
        return std::nullopt;
    }
    auto it = launch.find(kSessionKey);
    if (it == launch.end() || it->is_null()) {
        return std::nullopt;
    }
    if (!it->is_object()) {
        throw std::runtime_error("zenoh session settings must be a JSON object");
    }
    const nlohmann::json& object = *it;
    SessionSettings settings;
    settings.mode = require_field<std::string>(object, "mode");
    settings.connect = require_field<std::vector<std::string>>(object, "connect");
    settings.listen = require_field<std::vector<std::string>>(object, "listen");
    settings.multicast = require_field<bool>(object, "multicast");
    settings.scout_addr = require_field<std::string>(object, "scout_addr");
    settings.gossip = require_field<bool>(object, "gossip");
    settings.interface = require_field<std::string>(object, "interface");
    settings.connect_timeout_ms = require_field<std::uint64_t>(object, "connect_timeout_ms");
    // The rust settings deny unknown fields. A key python sends that this SDK
    // never reads is version skew, and silently ignoring it loses the setting.
    static const std::set<std::string> known = {"mode",       "connect", "listen",
                                                "multicast",  "gossip",  "interface",
                                                "scout_addr", "connect_timeout_ms"};
    for (const auto& entry : object.items()) {
        if (known.count(entry.key()) == 0) {
            throw std::runtime_error("zenoh session settings: unknown field '" + entry.key() +
                                     "'");
        }
    }
    if (settings.mode != "peer" && settings.mode != "client" && settings.mode != "router") {
        throw std::runtime_error(
            "zenoh session mode must be 'peer', 'client' or 'router', got '" + settings.mode +
            "'");
    }
    return settings;
}

/// These settings as a zenoh session config.
inline ::zenoh::Config zenoh_config(const SessionSettings& settings) {
    ::zenoh::Config config = ::zenoh::Config::create_default();
    std::vector<std::pair<std::string, std::string>> inserts = {
        {"mode", nlohmann::json(settings.mode).dump()},
        {"scouting/multicast/enabled", nlohmann::json(settings.multicast).dump()},
        {"scouting/multicast/interface", nlohmann::json(settings.interface).dump()},
        {"scouting/gossip/enabled", nlohmann::json(settings.gossip).dump()},
    };
    // Empty means the stock multicast group. A moved group is a private
    // discovery bus, which is how parallel sessions on one host stay apart.
    if (!settings.scout_addr.empty()) {
        inserts.emplace_back("scouting/multicast/address",
                             nlohmann::json(settings.scout_addr).dump());
    }
    // An empty list means "whatever zenoh listens on by default", which for a
    // peer is an ephemeral port, not nothing at all.
    if (!settings.connect.empty()) {
        inserts.emplace_back("connect/endpoints", nlohmann::json(settings.connect).dump());
    }
    if (!settings.listen.empty()) {
        inserts.emplace_back("listen/endpoints", nlohmann::json(settings.listen).dump());
    }
    // Zero means "don't wait", which zenoh reads as "dial once and never
    // retry". Leaving the key unset keeps its own retry policy.
    if (settings.connect_timeout_ms > 0) {
        inserts.emplace_back("connect/timeout_ms",
                             nlohmann::json(settings.connect_timeout_ms).dump());
    }
    for (const auto& insert : inserts) {
        config.insert_json5(insert.first, insert.second);
    }
    return config;
}

/// The trailing `host:port` of a locator, which is the part a link reports.
inline std::string locator_address(const std::string& locator) {
    std::size_t slash = locator.rfind('/');
    return slash == std::string::npos ? locator : locator.substr(slash + 1);
}

/// The host:port forms a live link to this locator may report. A locator dialed
/// by name never matches the address the link reports.
inline std::unordered_set<std::string> endpoint_addresses(const std::string& endpoint) {
    const std::string address = locator_address(endpoint);
    std::unordered_set<std::string> out = {address};
    const std::size_t colon = address.rfind(':');
    if (colon == std::string::npos) {
        return out;
    }
    const std::string host = address.substr(0, colon);
    const std::string port = address.substr(colon + 1);
    ::addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    ::addrinfo* resolved = nullptr;
    if (::getaddrinfo(host.c_str(), port.c_str(), &hints, &resolved) != 0) {
        return out;
    }
    for (::addrinfo* entry = resolved; entry != nullptr; entry = entry->ai_next) {
        char numeric_host[NI_MAXHOST];
        char numeric_port[NI_MAXSERV];
        if (::getnameinfo(entry->ai_addr, entry->ai_addrlen, numeric_host, sizeof(numeric_host),
                          numeric_port, sizeof(numeric_port),
                          NI_NUMERICHOST | NI_NUMERICSERV) == 0) {
            out.insert(std::string(numeric_host) + ":" + numeric_port);
        }
    }
    ::freeaddrinfo(resolved);
    return out;
}

/// Block until the dialed endpoints have links, bounded by the timeout.
///
/// A peer opens before its endpoints are dialed, so without this the first
/// published messages have nowhere to go.
inline void await_connect(const ::zenoh::Session& session,
                          const std::vector<std::string>& endpoints, const std::string& mode,
                          std::chrono::milliseconds timeout) {
    if (endpoints.empty() || timeout.count() == 0) {
        return;
    }
    std::vector<std::pair<std::string, std::unordered_set<std::string>>> pending;
    pending.reserve(endpoints.size());
    for (const std::string& endpoint : endpoints) {
        pending.emplace_back(endpoint, endpoint_addresses(endpoint));
    }
    // A client session holds one link. Zenoh dials the endpoints as
    // alternatives and keeps the first that answers.
    const std::size_t needed = mode == "client" ? 1 : pending.size();
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (true) {
        std::unordered_set<std::string> linked;
        for (const ::zenoh::Link& link : session.get_links()) {
            linked.insert(locator_address(link.get_dst()));
        }
        std::erase_if(pending, [&linked](const auto& entry) {
            for (const std::string& address : entry.second) {
                if (linked.count(address) != 0) {
                    return true;
                }
            }
            return false;
        });
        if (endpoints.size() - pending.size() >= needed) {
            return;
        }
        if (std::chrono::steady_clock::now() >= deadline) {
            std::string unlinked;
            for (const auto& entry : pending) {
                unlinked += unlinked.empty() ? entry.first : ", " + entry.first;
            }
            log::warn("zenoh endpoints not linked, published messages may be dropped",
                      {log::Field("endpoints", unlinked),
                       log::Field("timeout_ms", static_cast<std::int64_t>(timeout.count()))});
            return;
        }
        std::this_thread::sleep_for(kConnectPoll);
    }
}

/// Publisher QoS for one channel. Unset fields keep zenoh's defaults.
struct ChannelQos {
    std::optional<::zenoh::CongestionControl> congestion_control;
    /// Where the publisher is allowed to deliver. Session-local keeps a topic
    /// inside the process that publishes it, which is how a baked host hides
    /// its internal hops without changing the modules.
    std::optional<::zenoh::Locality> locality;
    std::optional<::zenoh::Reliability> reliability;
};

/// Parse the coordinator's `qos` object (channel -> {reliability,
/// congestion_control, locality}) into a lookup. Unknown or absent fields keep
/// zenoh's defaults.
inline std::unordered_map<std::string, ChannelQos> parse_channel_qos(
    const nlohmann::json& value) {
    std::unordered_map<std::string, ChannelQos> map;
    if (!value.is_object()) {
        return map;
    }
    for (const auto& entry : value.items()) {
        const nlohmann::json& fields = entry.value();
        if (!fields.is_object()) {
            continue;
        }
        ChannelQos qos;
        const std::string congestion_control =
            fields.value("congestion_control", std::string());
        if (congestion_control == "drop") {
            qos.congestion_control = Z_CONGESTION_CONTROL_DROP;
        } else if (congestion_control == "block") {
            qos.congestion_control = Z_CONGESTION_CONTROL_BLOCK;
        }
        const std::string locality = fields.value("locality", std::string());
        if (locality == "session_local") {
            qos.locality = Z_LOCALITY_SESSION_LOCAL;
        } else if (locality == "remote") {
            qos.locality = Z_LOCALITY_REMOTE;
        } else if (locality == "any") {
            qos.locality = Z_LOCALITY_ANY;
        }
        const std::string reliability = fields.value("reliability", std::string());
        if (reliability == "reliable") {
            qos.reliability = Z_RELIABILITY_RELIABLE;
        } else if (reliability == "best_effort") {
            qos.reliability = Z_RELIABILITY_BEST_EFFORT;
        }
        map.emplace(entry.key(), qos);
    }
    return map;
}

}  // namespace zenoh_detail

/// Zenoh transport for a native module.
class ZenohTransport : public Transport {
public:
    /// Open the session the launch config describes. A launch with no session
    /// block keeps zenoh's own defaults, which is what a hand-started module
    /// gets.
    static std::unique_ptr<ZenohTransport> from_launch(const nlohmann::json& launch) {
        std::optional<zenoh_detail::SessionSettings> settings =
            zenoh_detail::settings_from_launch(launch);
        if (!settings.has_value()) {
            log::info("no session block on the launch line, opening zenoh's defaults");
            return std::unique_ptr<ZenohTransport>(
                new ZenohTransport(::zenoh::Session::open(::zenoh::Config::create_default())));
        }
        if (settings->mode == "client" && settings->connect.size() > 1) {
            log::warn(
                "zenoh client mode holds a single link, traffic flows only through the first "
                "endpoint that connects",
                {log::Field("connect", nlohmann::json(settings->connect).dump())});
        }
        ::zenoh::Session session = ::zenoh::Session::open(zenoh_detail::zenoh_config(*settings));
        log::info("zenoh session opened",
                  {log::Field("mode", settings->mode),
                   log::Field("connect", nlohmann::json(settings->connect).dump()),
                   log::Field("listen", nlohmann::json(settings->listen).dump())});
        zenoh_detail::await_connect(session, settings->connect, settings->mode,
                                    std::chrono::milliseconds(settings->connect_timeout_ms));
        return std::unique_ptr<ZenohTransport>(new ZenohTransport(std::move(session)));
    }

    ZenohTransport(const ZenohTransport&) = delete;
    ZenohTransport& operator=(const ZenohTransport&) = delete;

    void publish(const std::string& channel, std::vector<uint8_t> data) override {
        // This runs on a publish worker thread, and zenoh-cpp reports every
        // failure by throwing. An escaped exception would terminate the process,
        // so a failed publish is logged and dropped the way LCM's is.
        try {
            // Declaring a publisher talks to the network, so each channel
            // declares once and reuses it. The lock is released before the put
            // so a stalled channel can't block the others.
            std::shared_ptr<::zenoh::Publisher> publisher;
            {
                std::lock_guard<std::mutex> lock(publishers_mu_);
                auto it = publishers_.find(channel);
                if (it == publishers_.end()) {
                    it = publishers_.emplace(channel, declare_publisher(channel)).first;
                }
                publisher = it->second;
            }
            publisher->put(::zenoh::Bytes(std::move(data)));
        } catch (const std::exception& e) {
            DIMOS_ERROR_THROTTLED(log::from_secs(1), "zenoh publish failed",
                                  log::Field("channel", channel),
                                  log::Field("error", e.what()));
        }
    }

    void subscribe(const std::string& channel, Dispatch on_msg) override {
        // A background subscriber lives as long as the session, which outlives
        // every route the module declares.
        session_.declare_background_subscriber(
            ::zenoh::KeyExpr(zenoh_key(channel)),
            [on_msg = std::move(on_msg)](const ::zenoh::Sample& sample) {
                // Zenoh usually holds a payload as one contiguous slice, which
                // is handed straight to the callback. Only a fragmented one is
                // joined, and only that case costs a copy.
                const ::zenoh::Bytes& payload = sample.get_payload();
                ::zenoh::Bytes::SliceIterator slices = payload.slice_iter();
                const std::optional<::zenoh::Slice> first = slices.next();
                if (!first.has_value()) {
                    on_msg(nullptr, 0);
                } else if (!slices.next().has_value()) {
                    on_msg(first->data, first->len);
                } else {
                    const std::vector<uint8_t> joined = payload.as_vector();
                    on_msg(joined.data(), joined.size());
                }
            },
            ::zenoh::closures::none);
    }

    void set_publisher_qos(const nlohmann::json& qos) override {
        std::lock_guard<std::mutex> lock(publishers_mu_);
        qos_ = zenoh_detail::parse_channel_qos(qos);
    }

private:
    explicit ZenohTransport(::zenoh::Session session) : session_(std::move(session)) {}

    /// Caller holds `publishers_mu_`, which guards `qos_` too.
    std::shared_ptr<::zenoh::Publisher> declare_publisher(const std::string& channel) {
        ::zenoh::Session::PublisherOptions options =
            ::zenoh::Session::PublisherOptions::create_default();
        auto it = qos_.find(channel);
        if (it != qos_.end()) {
            const zenoh_detail::ChannelQos& qos = it->second;
            if (qos.congestion_control.has_value()) {
                options.congestion_control = *qos.congestion_control;
            }
            if (qos.locality.has_value()) {
                options.allowed_destination = *qos.locality;
            }
            if (qos.reliability.has_value()) {
                options.reliability = *qos.reliability;
            }
        }
        return std::make_shared<::zenoh::Publisher>(session_.declare_publisher(
            ::zenoh::KeyExpr(zenoh_key(channel)), std::move(options)));
    }

    ::zenoh::Session session_;
    std::mutex publishers_mu_;
    std::unordered_map<std::string, zenoh_detail::ChannelQos> qos_;
    std::unordered_map<std::string, std::shared_ptr<::zenoh::Publisher>> publishers_;
};

}  // namespace dimos::native
