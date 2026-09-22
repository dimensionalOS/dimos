// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

#include <doctest/doctest.h>
#include "dimos/native/cdr_codec.hpp"

using namespace dimos::native;

TEST_CASE("default CDR codecs preserve nested generated fields") {
    geometry_msgs::msg::PoseStamped message{};
    message.header.frame_id = "map";
    message.header.stamp.sec = 1700000000;
    message.header.stamp.nanosec = 123456789;
    message.pose.position.x = 1.5;
    message.pose.orientation.w = 1;
    const auto bytes = cdr_encode(message);
    REQUIRE(bytes.size() > 4);
    CHECK(bytes[0] == 0);
    CHECK(bytes[1] == 1);
    CHECK(cdr_decode<geometry_msgs::msg::PoseStamped>(bytes.data(), bytes.size()) == message);
    const auto big = dimos::cdr::encode(message, false);
    CHECK(cdr_decode<geometry_msgs::msg::PoseStamped>(big.data(), big.size()) == message);
}

TEST_CASE("default CDR decoder rejects malformed input") {
    std_msgs::msg::Int32 message{};
    message.data = 1234;
    auto bytes = cdr_encode(message);
    CHECK_THROWS_AS(cdr_decode<std_msgs::msg::Int32>(bytes.data(), bytes.size() - 1), std::invalid_argument);
    bytes.push_back(0);
    CHECK_THROWS_AS(cdr_decode<std_msgs::msg::Int32>(bytes.data(), bytes.size()), std::invalid_argument);
    bytes[0] = 255;
    CHECK_THROWS_AS(cdr_decode<std_msgs::msg::Int32>(bytes.data(), bytes.size()), std::invalid_argument);
}
