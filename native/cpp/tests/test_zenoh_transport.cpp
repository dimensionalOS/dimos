// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Covers the pure parts of the zenoh transport and typechecks its inline bodies
// against the real zenoh headers. Opens no session, so it needs no network.
// Compiled only when zenoh-cpp is available, see tests/CMakeLists.txt.

#include <doctest/doctest.h>

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <type_traits>
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

TEST_CASE("an unknown session mode is rejected") {
    nlohmann::json launch = nlohmann::json::parse(kClientLaunch);
    launch["session"]["mode"] = "gateway";
    CHECK_THROWS_AS(settings_from_launch(launch), std::runtime_error);
}

TEST_CASE("the session settings become a zenoh config") {
    auto settings = settings_from_launch(nlohmann::json::parse(kClientLaunch));
    REQUIRE(settings.has_value());
    zenoh_detail::zenoh_config(*settings);  // every insert_json5 key is accepted
    CHECK(true);
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
