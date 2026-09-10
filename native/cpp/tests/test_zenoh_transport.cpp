// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// ZenohTransport: session settings and QoS parsing against the goldens the
// Rust module and Python share, plus an in-process round trip on a session
// with discovery off, so it needs no network beyond loopback.

#include <doctest/doctest.h>

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <fstream>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "dimos/native/zenoh_transport.hpp"

using namespace dimos::native;
using nlohmann::json;

namespace {

json golden(const char* name) {
    std::ifstream file(std::string(DIMOS_RUST_FIXTURES_DIR) + "/" + name);
    REQUIRE(file.good());
    return json::parse(file);
}

std::map<std::string, std::string> inserts(const json& session) {
    std::map<std::string, std::string> out;
    for (const auto& [key, value] : zenoh_config_inserts(session)) {
        out[key] = value;
    }
    return out;
}

// A session only reachable from this process: discovery off, no endpoints.
json local_session() {
    return {{"mode", "peer"},        {"connect", json::array()}, {"listen", json::array()},
            {"multicast", false},    {"scout_addr", ""},         {"gossip", false},
            {"interface", "lo"},     {"connect_timeout_ms", 0}};
}

}  // namespace

TEST_CASE("zenoh_key strips the leading slash") {
    CHECK(zenoh_key("/cmd_vel/geometry_msgs.Twist") == "cmd_vel/geometry_msgs.Twist");
    CHECK(zenoh_key("dimos/cmd_vel/geometry_msgs.Twist") == "dimos/cmd_vel/geometry_msgs.Twist");
    CHECK(zenoh_key("") == "");
}

TEST_CASE("the client golden maps to the session config the Rust module builds") {
    auto out = inserts(golden("session_wire_client.json"));
    CHECK(out.at("mode") == R"("client")");
    CHECK(out.at("connect/endpoints") == R"(["tcp/192.0.2.10:7447"])");
    CHECK(out.count("listen/endpoints") == 0);
    CHECK(out.at("scouting/multicast/enabled") == "true");
    CHECK(out.count("scouting/multicast/address") == 0);
    CHECK(out.at("scouting/gossip/enabled") == "false");
    CHECK(out.at("scouting/multicast/interface") == R"("lo")");
    CHECK(out.at("connect/timeout_ms") == "2000");
}

TEST_CASE("the router golden keeps zenoh's defaults for empty lists and a zero timeout") {
    auto out = inserts(golden("session_wire_router.json"));
    CHECK(out.at("mode") == R"("router")");
    CHECK(out.at("listen/endpoints") == R"(["tcp/127.0.0.1:7447"])");
    CHECK(out.count("connect/endpoints") == 0);
    CHECK(out.count("connect/timeout_ms") == 0);
    CHECK(out.at("scouting/multicast/enabled") == "false");
    CHECK(out.at("scouting/gossip/enabled") == "true");
}

TEST_CASE("a moved scout group reaches the session config") {
    json session = local_session();
    session["scout_addr"] = "224.0.0.224:17700";
    CHECK(inserts(session).at("scouting/multicast/address") == R"("224.0.0.224:17700")");
}

TEST_CASE("a missing or unknown setting is an error, not a default") {
    CHECK_THROWS_AS(zenoh_config_inserts(json{{"mode", "peer"}}), std::runtime_error);
    json session = local_session();
    session["reliability"] = "reliable";
    CHECK_THROWS_AS(zenoh_config_inserts(session), std::runtime_error);
    CHECK_THROWS_AS(zenoh_config_inserts(json::array()), std::runtime_error);
}

TEST_CASE("parse_channel_qos reads the coordinator's qos block") {
    auto qos = parse_channel_qos(json::parse(
        R"({"a":{"reliability":"reliable","congestion_control":"block","locality":"session_local"},
            "b":{"reliability":"best_effort","congestion_control":"drop","locality":"remote"},
            "c":{"reliability":"bogus"},
            "d":7})"));
    CHECK(qos.at("a").reliability == Z_RELIABILITY_RELIABLE);
    CHECK(qos.at("a").congestion_control == Z_CONGESTION_CONTROL_BLOCK);
    CHECK(qos.at("a").locality == Z_LOCALITY_SESSION_LOCAL);
    CHECK(qos.at("b").reliability == Z_RELIABILITY_BEST_EFFORT);
    CHECK(qos.at("b").congestion_control == Z_CONGESTION_CONTROL_DROP);
    CHECK(qos.at("b").locality == Z_LOCALITY_REMOTE);
    CHECK_FALSE(qos.at("c").reliability.has_value());
    CHECK(qos.count("d") == 0);
    CHECK(parse_channel_qos(json()).empty());
}

TEST_CASE("ZenohTransport implements the Transport interface") {
    CHECK(std::is_base_of<Transport, ZenohTransport>::value);
}

TEST_CASE("a message published on a channel reaches that channel's subscriber") {
    json launch = {{"session", local_session()},
                   {"qos", {{"/t/x", {{"reliability", "reliable"}, {"congestion_control", "block"}}}}}};
    ZenohTransport transport(launch);

    std::mutex mu;
    std::condition_variable cv;
    std::vector<std::vector<uint8_t>> got;
    transport.subscribe("/t/x", [&](const uint8_t* data, std::size_t len) {
        std::lock_guard<std::mutex> lock(mu);
        got.emplace_back(data, data + len);
        cv.notify_all();
    });
    transport.subscribe("/t/other", [&](const uint8_t*, std::size_t) {
        std::lock_guard<std::mutex> lock(mu);
        got.emplace_back(std::vector<uint8_t>{0xFF});
        cv.notify_all();
    });

    transport.publish("/t/x", {1, 2, 3});
    transport.publish("/t/x", {4});

    std::unique_lock<std::mutex> lock(mu);
    REQUIRE(cv.wait_for(lock, std::chrono::seconds(5), [&] { return got.size() >= 2; }));
    CHECK(got[0] == std::vector<uint8_t>{1, 2, 3});
    CHECK(got[1] == std::vector<uint8_t>{4});
}
