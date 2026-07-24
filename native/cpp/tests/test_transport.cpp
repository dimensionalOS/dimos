// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

#include <doctest/doctest.h>

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "dimos/native/transport.hpp"

using namespace dimos::native;

namespace {

// Records every interaction so tests can assert on the transport seam without
// a real wire.
struct MockTransport : Transport {
    std::vector<std::pair<std::string, std::vector<uint8_t>>> published;
    std::vector<std::pair<std::string, Dispatch>> subscriptions;
    std::string qos;

    void publish(const std::string& channel, std::vector<uint8_t> data) override {
        published.emplace_back(channel, std::move(data));
    }
    void subscribe(const std::string& channel, Dispatch on_msg) override {
        subscriptions.emplace_back(channel, std::move(on_msg));
    }
    void set_publisher_qos(const std::string& qos_json) override { qos = qos_json; }
};

}  // namespace

TEST_CASE("publish records channel and owned payload") {
    MockTransport t;
    t.publish("/data", {1, 2, 3});
    REQUIRE(t.published.size() == 1);
    CHECK(t.published[0].first == "/data");
    CHECK(t.published[0].second == std::vector<uint8_t>{1, 2, 3});
}

TEST_CASE("subscribed dispatch delivers raw bytes to the callback") {
    MockTransport t;
    std::vector<uint8_t> got;
    t.subscribe("/in", [&](const uint8_t* p, std::size_t n) { got.assign(p, p + n); });
    REQUIRE(t.subscriptions.size() == 1);

    const std::vector<uint8_t> payload{9, 8, 7};
    t.subscriptions[0].second(payload.data(), payload.size());
    CHECK(got == payload);
}

TEST_CASE("set_publisher_qos default is a no-op") {
    struct Bare : Transport {
        void publish(const std::string&, std::vector<uint8_t>) override {}
        void subscribe(const std::string&, Dispatch) override {}
    } bare;
    bare.set_publisher_qos(R"({"/x":{"reliability":"reliable"}})");
    CHECK(true);
}

TEST_CASE("transports are usable through the abstract base") {
    MockTransport concrete;
    Transport& t = concrete;
    t.set_publisher_qos("{}");
    t.publish("/c", {42});
    CHECK(concrete.qos == "{}");
    REQUIRE(concrete.published.size() == 1);
    CHECK(concrete.published[0].second == std::vector<uint8_t>{42});
}
