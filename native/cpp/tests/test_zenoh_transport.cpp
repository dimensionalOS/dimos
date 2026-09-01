// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Covers the pure parts of the zenoh transport and typechecks its inline bodies
// against the real zenoh headers. Opens no session, so it needs no network.
// Compiled only when zenoh-cpp is available, see tests/CMakeLists.txt.

#include <doctest/doctest.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <unordered_set>
#include <vector>

#include <nlohmann/json.hpp>

#include "dimos/native/zenoh_transport.hpp"

using namespace dimos::native;
using zenoh_detail::parse_channel_qos;
using zenoh_detail::settings_from_launch;

// The launch line python sends for a client-mode session.
constexpr const char* kClientLaunch = R"({
  "session": {
    "mode": "client",
    "connect": ["tcp/192.0.2.10:7447"],
    "listen": [],
    "multicast": true,
    "scout_addr": "",
    "gossip": false,
    "interface": "lo",
    "connect_timeout_ms": 2000
  }
})";

TEST_CASE("a dimos topic becomes a zenoh key by losing its leading slash") {
    CHECK(zenoh_key("/robot/cmd_vel") == "robot/cmd_vel");
    CHECK(zenoh_key("robot/cmd_vel") == "robot/cmd_vel");
    CHECK(zenoh_key("/") == "");
    CHECK(zenoh_key("") == "");
    // Only the leading slash goes. An inner or trailing one is part of the key.
    CHECK(zenoh_key("//double") == "/double");
    CHECK(zenoh_key("/trailing/") == "trailing/");
}

TEST_CASE("the session settings are read off the launch line") {
    auto settings = settings_from_launch(nlohmann::json::parse(kClientLaunch));
    REQUIRE(settings.has_value());
    CHECK(settings->mode == "client");
    CHECK(settings->connect == std::vector<std::string>{"tcp/192.0.2.10:7447"});
    CHECK(settings->listen.empty());
    CHECK(settings->multicast);
    CHECK(settings->scout_addr.empty());
    CHECK_FALSE(settings->gossip);
    CHECK(settings->interface == "lo");
    CHECK(settings->connect_timeout_ms == 2000);
}

TEST_CASE("a launch with no session block keeps zenoh's defaults") {
    CHECK_FALSE(settings_from_launch(nlohmann::json::parse(R"({"topics":{}})")).has_value());
    CHECK_FALSE(settings_from_launch(nlohmann::json::parse(R"({"session":null})")).has_value());
    CHECK_FALSE(settings_from_launch(nlohmann::json::object()).has_value());
}

TEST_CASE("a session block missing a field is rejected and names it") {
    // Every field is required: python resolves them all, so an absent one is a
    // setting silently lost rather than a default taken.
    try {
        settings_from_launch(nlohmann::json::parse(R"({"session":{"mode":"peer"}})"));
        FAIL("expected a session block missing fields to throw");
    } catch (const std::runtime_error& e) {
        CHECK(std::string(e.what()).find("connect") != std::string::npos);
    }
}

TEST_CASE("a session block with an unknown field is rejected and names it") {
    nlohmann::json launch = nlohmann::json::parse(kClientLaunch);
    launch["session"]["bogus"] = 1;
    try {
        settings_from_launch(launch);
        FAIL("expected an unknown session field to throw");
    } catch (const std::runtime_error& e) {
        CHECK(std::string(e.what()).find("bogus") != std::string::npos);
    }
}

TEST_CASE("a negative connect timeout is rejected rather than wrapped") {
    // Cast instead of rejected it becomes a wait of a few hundred million
    // years, so opening the session would look like a hang.
    nlohmann::json launch = nlohmann::json::parse(kClientLaunch);
    launch["session"]["connect_timeout_ms"] = -1;
    try {
        settings_from_launch(launch);
        FAIL("expected a negative connect timeout to throw");
    } catch (const std::runtime_error& e) {
        CHECK(std::string(e.what()).find("connect_timeout_ms") != std::string::npos);
    }
}

TEST_CASE("an unknown session mode is rejected") {
    nlohmann::json launch = nlohmann::json::parse(kClientLaunch);
    launch["session"]["mode"] = "gateway";
    CHECK_THROWS_AS(settings_from_launch(launch), std::runtime_error);
}

TEST_CASE("the session settings become a zenoh config") {
    auto settings = settings_from_launch(nlohmann::json::parse(kClientLaunch));
    REQUIRE(settings.has_value());
    ::zenoh::Config config = zenoh_detail::zenoh_config(*settings);
    CHECK(config.get("mode") == R"("client")");
    CHECK(config.get("connect/endpoints") == R"(["tcp/192.0.2.10:7447"])");
    CHECK(config.get("connect/timeout_ms") == "2000");
    CHECK(config.get("scouting/multicast/enabled") == "true");
    CHECK(config.get("scouting/multicast/interface") == R"("lo")");
    CHECK(config.get("scouting/gossip/enabled") == "false");
}

TEST_CASE("publisher qos is read per channel, and unset fields keep defaults") {
    auto qos = parse_channel_qos(nlohmann::json::parse(R"({
      "/a": {"congestion_control": "block", "locality": "session_local",
             "reliability": "reliable"},
      "/b": {"congestion_control": "drop", "locality": "remote",
             "reliability": "best_effort"},
      "/c": {}
    })"));
    REQUIRE(qos.count("/a") == 1);
    CHECK(qos.at("/a").congestion_control == Z_CONGESTION_CONTROL_BLOCK);
    CHECK(qos.at("/a").locality == Z_LOCALITY_SESSION_LOCAL);
    CHECK(qos.at("/a").reliability == Z_RELIABILITY_RELIABLE);
    CHECK(qos.at("/b").congestion_control == Z_CONGESTION_CONTROL_DROP);
    CHECK(qos.at("/b").locality == Z_LOCALITY_REMOTE);
    CHECK(qos.at("/b").reliability == Z_RELIABILITY_BEST_EFFORT);
    CHECK_FALSE(qos.at("/c").congestion_control.has_value());
    CHECK_FALSE(qos.at("/c").locality.has_value());
    CHECK_FALSE(qos.at("/c").reliability.has_value());
}

TEST_CASE("an unrecognized qos value keeps zenoh's default rather than erroring") {
    auto qos = parse_channel_qos(
        nlohmann::json::parse(R"({"/a": {"congestion_control": "sometimes"}})"));
    REQUIRE(qos.count("/a") == 1);
    CHECK_FALSE(qos.at("/a").congestion_control.has_value());
}

TEST_CASE("an absent or non-object qos block yields no overrides") {
    CHECK(parse_channel_qos(nlohmann::json()).empty());
    CHECK(parse_channel_qos(nlohmann::json::parse("[1,2]")).empty());
}

TEST_CASE("a locator's address is the part a link reports") {
    CHECK(zenoh_detail::locator_address("tcp/127.0.0.1:7447") == "127.0.0.1:7447");
    CHECK(zenoh_detail::locator_address("127.0.0.1:7447") == "127.0.0.1:7447");
}

TEST_CASE("an endpoint dialed by name also matches the address a link reports") {
    // A link reports a numeric address, so a named endpoint would never be seen
    // as connected without resolving it first.
    auto addresses = zenoh_detail::endpoint_addresses("tcp/localhost:7447");
    CHECK(addresses.count("localhost:7447") == 1);
    CHECK(addresses.count("127.0.0.1:7447") == 1);
    // Nothing to resolve, so a portless endpoint is left as it is.
    CHECK(zenoh_detail::endpoint_addresses("tcp/localhost") ==
          std::unordered_set<std::string>{"localhost"});
}

// A session that neither scouts nor dials, so opening it touches no network.
constexpr const char* kIsolatedLaunch = R"({
  "session": {
    "mode": "peer",
    "connect": [],
    "listen": [],
    "multicast": false,
    "scout_addr": "",
    "gossip": false,
    "interface": "lo",
    "connect_timeout_ms": 0
  }
})";

TEST_CASE("an empty setting leaves zenoh's own default in place") {
    auto settings = settings_from_launch(nlohmann::json::parse(kIsolatedLaunch));
    REQUIRE(settings.has_value());
    ::zenoh::Config config = zenoh_detail::zenoh_config(*settings);
    // Unset, so zenoh falls back to its own default. Writing the empty string
    // would instead mean an empty multicast group and a never-retrying dial.
    CHECK(config.get("scouting/multicast/address") == "null");
    CHECK(config.get("connect/timeout_ms") == "null");
}

TEST_CASE("waiting on an endpoint that never links gives up at the timeout") {
    // Nothing listens on the endpoint, so the wait can only end at the deadline.
    // If it did not, opening a session against a dead peer would hang forever.
    ::zenoh::Session session =
        ::zenoh::Session::open(zenoh_detail::zenoh_config(*settings_from_launch(
            nlohmann::json::parse(kIsolatedLaunch))));
    const auto started = std::chrono::steady_clock::now();
    zenoh_detail::await_connect(session, {"tcp/127.0.0.1:1"}, "peer",
                                std::chrono::milliseconds(200));
    const auto waited = std::chrono::steady_clock::now() - started;
    CHECK(waited >= std::chrono::milliseconds(200));
    CHECK(waited < std::chrono::seconds(5));
}

TEST_CASE("a published payload reaches a subscriber unchanged") {
    std::unique_ptr<Transport> transport =
        ZenohTransport::from_launch(nlohmann::json::parse(kIsolatedLaunch));

    std::mutex received_mu;
    std::vector<uint8_t> received;
    transport->subscribe("/dimos_test/round_trip",
                         [&](const uint8_t* data, std::size_t len) {
                             std::lock_guard<std::mutex> lock(received_mu);
                             received.assign(data, data + len);
                         });

    // Republish until it lands, since declaring a subscriber is not immediate.
    const std::vector<uint8_t> payload = {0, 1, 2, 250, 251, 252};
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    std::vector<uint8_t> got;
    while (got.empty() && std::chrono::steady_clock::now() < deadline) {
        transport->publish("/dimos_test/round_trip", payload);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        std::lock_guard<std::mutex> lock(received_mu);
        got = received;
    }
    CHECK(got == payload);
}

TEST_CASE("a publish zenoh rejects is logged rather than thrown") {
    // Publishing happens on a worker thread with no catch of its own, so an
    // escaped zenoh exception would terminate the process.
    std::unique_ptr<Transport> transport =
        ZenohTransport::from_launch(nlohmann::json::parse(kIsolatedLaunch));
    // '?' cannot appear in a key expression, so declaring the publisher fails.
    CHECK_NOTHROW(transport->publish("/bad?key", std::vector<uint8_t>{1, 2, 3}));
}

// Compiled (so every inline body is typechecked against zenoh-cpp) but never
// called: a client-mode session dials an endpoint and scouts the network.
[[maybe_unused]] static void zenoh_transport_compile_check() {
    std::unique_ptr<Transport> transport =
        ZenohTransport::from_launch(nlohmann::json::parse(kClientLaunch));
    transport->set_publisher_qos(nlohmann::json::object());
    transport->publish("/c", std::vector<uint8_t>{1, 2, 3});
    transport->subscribe("/c", [](const uint8_t*, std::size_t) {});
}

TEST_CASE("ZenohTransport implements the Transport interface") {
    CHECK(std::is_base_of<Transport, ZenohTransport>::value);
}
