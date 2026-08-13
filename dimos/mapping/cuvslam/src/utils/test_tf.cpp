// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Mirror of the rust tf client's test suite (native/rust/dimos-module/src/tf.rs).

#include <doctest/doctest.h>

#include <array>
#include <chrono>
#include <cmath>
#include <string>
#include <thread>
#include <vector>

#include "tf.hpp"

using tf_client::DEFAULT_TF_WINDOW_SECS;
using tf_client::Rigid;
using tf_client::TBuffer;
using tf_client::Tf;
using tf_client::Transform;

namespace {

constexpr double PI = 3.14159265358979323846;

Rigid rigid_of(double x, double y, double z, double yaw) {
    return Rigid{{0.0, 0.0, std::sin(yaw / 2.0), std::cos(yaw / 2.0)}, {x, y, z}};
}

void add(Tf& tf, const std::string& parent, const std::string& child, double ts,
         double x, double y, double z, double yaw = 0.0) {
    const Rigid rigid = rigid_of(x, y, z, yaw);
    tf.receive(parent, child, ts, rigid.rotation, rigid.translation);
}

double yaw_of(const Transform& transform) {
    const std::array<double, 4>& q = transform.rotation();
    return std::atan2(2.0 * (q[3] * q[2] + q[0] * q[1]),
                      1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]));
}

}  // namespace

TEST_CASE("direct edge") {
    Tf tf;
    add(tf, "base_link", "arm", 1.0, 1.0, -1.0, 0.0);
    const auto t = tf.get_latest("base_link", "arm");
    REQUIRE(t.has_value());
    CHECK(std::abs(t->translation()[0] - 1.0) < 1e-9);
    CHECK(std::abs(t->translation()[1] + 1.0) < 1e-9);
    CHECK(t->parent == "base_link");
    CHECK(t->child == "arm");
}

TEST_CASE("reverse edge returns inverse") {
    Tf tf;
    add(tf, "base_link", "arm", 1.0, 1.0, 2.0, 3.0);
    const auto inv = tf.get_latest("arm", "base_link");
    REQUIRE(inv.has_value());
    CHECK(std::abs(inv->translation()[0] + 1.0) < 1e-9);
    CHECK(std::abs(inv->translation()[1] + 2.0) < 1e-9);
    CHECK(std::abs(inv->translation()[2] + 3.0) < 1e-9);
    CHECK(inv->parent == "arm");
    CHECK(inv->child == "base_link");
}

TEST_CASE("reverse edge inverts rotation and translation") {
    // Inverse of a rotated edge is t' = -R^T t, the classic sign/order trap.
    Tf tf;
    add(tf, "base_link", "arm", 1.0, 1.0, 2.0, 3.0, PI / 2.0);
    const auto inv = tf.get_latest("arm", "base_link");
    REQUIRE(inv.has_value());
    CHECK(std::abs(inv->translation()[0] + 2.0) < 1e-9);
    CHECK(std::abs(inv->translation()[1] - 1.0) < 1e-9);
    CHECK(std::abs(inv->translation()[2] + 3.0) < 1e-9);
    CHECK(std::abs(yaw_of(*inv) + PI / 2.0) < 1e-9);
}

TEST_CASE("composes the ros example chain") {
    Tf tf;
    add(tf, "base_link", "arm", 1.0, 1.0, -1.0, 0.0, PI / 6.0);
    add(tf, "arm", "end_effector", 1.0, 1.0, 1.0, 0.0);
    const auto t = tf.get_latest("base_link", "end_effector");
    REQUIRE(t.has_value());
    CHECK(std::abs(t->translation()[0] - 1.366) < 1e-3);
    CHECK(std::abs(t->translation()[1] - 0.366) < 1e-3);
    CHECK(t->parent == "base_link");
    CHECK(t->child == "end_effector");
}

TEST_CASE("composes a multi-hop chain") {
    Tf tf;
    add(tf, "world", "robot", 1.0, 1.0, 2.0, 3.0);
    add(tf, "robot", "sensor", 1.0, 0.5, 0.0, 0.2, PI / 2.0);
    const auto t = tf.get_latest("world", "sensor");
    REQUIRE(t.has_value());
    CHECK(std::abs(t->translation()[0] - 1.5) < 1e-3);
    CHECK(std::abs(t->translation()[1] - 2.0) < 1e-3);
    CHECK(std::abs(t->translation()[2] - 3.2) < 1e-3);
}

TEST_CASE("composed chain accumulates rotation") {
    Tf tf;
    add(tf, "a", "b", 1.0, 0.0, 0.0, 0.0, PI / 6.0);
    add(tf, "b", "c", 1.0, 0.0, 0.0, 0.0, PI / 6.0);
    const auto t = tf.get_latest("a", "c");
    REQUIRE(t.has_value());
    CHECK(std::abs(yaw_of(*t) - PI / 3.0) < 1e-9);
}

TEST_CASE("composed stamp is the stalest edge") {
    Tf tf;
    add(tf, "world", "robot", 700.0, 1.0, 0.0, 0.0);
    add(tf, "robot", "sensor", 1000.0, 0.5, 0.0, 0.0);
    REQUIRE(tf.get_latest("world", "sensor").has_value());
    CHECK(tf.get_latest("world", "sensor")->ts == 700.0);
    // Both directions, so the answer does not depend on which end is queried.
    CHECK(tf.get_latest("sensor", "world")->ts == 700.0);
}

TEST_CASE("missing path returns nullopt") {
    Tf tf;
    add(tf, "world", "robot", 1.0, 1.0, 0.0, 0.0);
    CHECK_FALSE(tf.get_latest("world", "unconnected").has_value());
}

TEST_CASE("identity for the same frame") {
    Tf tf;
    // No query time: identity is stamped now, not the epoch.
    const auto t = tf.get_latest("base_link", "base_link");
    REQUIRE(t.has_value());
    CHECK(std::abs(t->translation()[0]) < 1e-12);
    CHECK(std::abs(t->translation()[1]) < 1e-12);
    CHECK(std::abs(t->translation()[2]) < 1e-12);
    CHECK(t->ts > 1.0e9);
}

TEST_CASE("bfs takes the fewest hops on a branching graph") {
    // Two routes to d: three hops through b, c and two through x. BFS must
    // compose the two-hop route.
    Tf tf;
    add(tf, "a", "b", 1.0, 1.0, 0.0, 0.0);
    add(tf, "b", "c", 1.0, 1.0, 0.0, 0.0);
    add(tf, "c", "d", 1.0, 1.0, 0.0, 0.0);
    add(tf, "a", "x", 1.0, 10.0, 0.0, 0.0);
    add(tf, "x", "d", 1.0, 1.0, 0.0, 0.0);
    const auto t = tf.get_latest("a", "d");
    REQUIRE(t.has_value());
    CHECK(std::abs(t->translation()[0] - 11.0) < 1e-9);
}

TEST_CASE("time query picks the nearest sample") {
    Tf tf;
    add(tf, "a", "b", 10.0, 1.0, 0.0, 0.0);
    add(tf, "a", "b", 20.0, 2.0, 0.0, 0.0);
    const auto near_10 = tf.lookup("a", "b").at(11.0).get();
    REQUIRE(near_10.has_value());
    CHECK(std::abs(near_10->translation()[0] - 1.0) < 1e-9);
    const auto near_20 = tf.lookup("a", "b").at(18.0).get();
    REQUIRE(near_20.has_value());
    CHECK(std::abs(near_20->translation()[0] - 2.0) < 1e-9);
}

TEST_CASE("tie prefers the later sample") {
    Tf tf;
    add(tf, "a", "b", 10.0, 1.0, 0.0, 0.0);
    add(tf, "a", "b", 12.0, 2.0, 0.0, 0.0);
    const auto t = tf.lookup("a", "b").at(11.0).get();
    REQUIRE(t.has_value());
    CHECK(std::abs(t->translation()[0] - 2.0) < 1e-9);
}

TEST_CASE("time query outside tolerance returns nullopt") {
    Tf tf;
    add(tf, "a", "b", 10.0, 1.0, 0.0, 0.0);
    CHECK_FALSE(tf.lookup("a", "b").at(50.0).tolerance(1.0).get().has_value());
    CHECK(tf.lookup("a", "b").at(10.5).tolerance(1.0).get().has_value());
}

TEST_CASE("time query beyond the window returns nullopt without a tolerance") {
    Tf tf;
    add(tf, "a", "b", 100.0, 1.0, 0.0, 0.0);
    CHECK_FALSE(tf.lookup("a", "b").at(50.0).get().has_value());
}

TEST_CASE("time query inside the window resolves without a tolerance") {
    Tf tf;
    add(tf, "a", "b", 100.0, 1.0, 0.0, 0.0);
    const auto t = tf.lookup("a", "b").at(95.0).get();
    REQUIRE(t.has_value());
    CHECK(std::abs(t->translation()[0] - 1.0) < 1e-9);
}

TEST_CASE("an explicit tolerance reaches past the window") {
    // An explicit tolerance is the caller opting into staleness, so it widens
    // past the window rather than being clamped by it.
    Tf tf;
    add(tf, "a", "b", 100.0, 1.0, 0.0, 0.0);
    CHECK(tf.lookup("a", "b").at(50.0).tolerance(60.0).get().has_value());
}

TEST_CASE("latest is not bounded by the window") {
    // The window bounds queries against a stamp, not the latest sample. With no
    // requested time, the newest edge is returned however old it is.
    Tf tf;
    add(tf, "a", "b", 100.0, 1.0, 0.0, 0.0);
    CHECK(tf.get_latest("a", "b").has_value());
}

TEST_CASE("within returns without waiting when already buffered") {
    Tf tf;
    add(tf, "a", "b", 5.0, 1.0, 0.0, 0.0);
    const auto t = tf.lookup("a", "b").within(std::chrono::seconds(30));
    REQUIRE(t.has_value());
    CHECK(std::abs(t->translation()[0] - 1.0) < 1e-9);
}

TEST_CASE("within resolves when the transform arrives late") {
    // Returns as soon as the transform lands, not at the deadline.
    Tf tf;
    std::thread writer([&tf] {
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        add(tf, "a", "b", 5.0, 1.0, 0.0, 0.0);
    });
    const auto start = std::chrono::steady_clock::now();
    const auto t = tf.lookup("a", "b").within(std::chrono::seconds(10));
    const auto elapsed = std::chrono::steady_clock::now() - start;
    writer.join();
    REQUIRE(t.has_value());
    CHECK(elapsed < std::chrono::seconds(5));
}

TEST_CASE("within times out when nothing arrives") {
    Tf tf;
    const auto start = std::chrono::steady_clock::now();
    const auto t = tf.lookup("a", "b").within(std::chrono::milliseconds(50));
    const auto elapsed = std::chrono::steady_clock::now() - start;
    CHECK_FALSE(t.has_value());
    CHECK(elapsed >= std::chrono::milliseconds(50));
}

TEST_CASE("a zero rotation on the wire is dropped rather than stored as nan") {
    Tf tf;
    tf.receive("a", "b", 1.0, {0.0, 0.0, 0.0, 0.0}, {1.0, 0.0, 0.0});
    CHECK_FALSE(tf.get_latest("a", "b").has_value());
    tf.receive("a", "b", 2.0, {0.0, 0.0, 0.0, 1.0}, {1.0, 0.0, 0.0});
    const auto t = tf.get_latest("a", "b");
    REQUIRE(t.has_value());
    for (double component : t->rotation()) {
        CHECK(std::isfinite(component));
    }
}

TEST_CASE("a lookup that finds nothing warns with the frames") {
    Tf tf;
    std::vector<std::string> warnings;
    tf.set_warn_sink([&warnings](const std::string& message) { warnings.push_back(message); });
    add(tf, "world", "robot", 1.0, 1.0, 0.0, 0.0);
    CHECK_FALSE(tf.get_latest("world", "gripper").has_value());
    REQUIRE(warnings.size() == 1);
    CHECK(warnings[0].find("gripper") != std::string::npos);
}

TEST_CASE("repeated misses warn once per frame pair") {
    Tf tf;
    std::vector<std::string> warnings;
    tf.set_warn_sink([&warnings](const std::string& message) { warnings.push_back(message); });
    for (int i = 0; i < 5; ++i) {
        CHECK_FALSE(tf.get_latest("world", "gripper").has_value());
    }
    CHECK(warnings.size() == 1);
    // A pair that is throttled must not mute an unrelated one.
    CHECK_FALSE(tf.get_latest("world", "camera").has_value());
    REQUIRE(warnings.size() == 2);
    CHECK(warnings[1].find("camera") != std::string::npos);
}

TEST_CASE("a resolved lookup stays quiet") {
    Tf tf;
    std::vector<std::string> warnings;
    tf.set_warn_sink([&warnings](const std::string& message) { warnings.push_back(message); });
    add(tf, "world", "robot", 1.0, 1.0, 0.0, 0.0);
    CHECK(tf.get_latest("world", "robot").has_value());
    CHECK(warnings.empty());
}

TEST_CASE("publish feeds the local graph and the sink") {
    Tf tf;
    std::vector<Transform> seen;
    tf.set_publish_sink(
        [&seen](const std::vector<Transform>& transforms) { seen = transforms; });
    tf.publish({Transform{"odom", "base", 7.0, rigid_of(1.0, 2.0, 3.0, 0.0)}});
    REQUIRE(seen.size() == 1);
    CHECK(seen[0].parent == "odom");
    const auto t = tf.get_latest("odom", "base");
    REQUIRE(t.has_value());
    CHECK(std::abs(t->translation()[0] - 1.0) < 1e-9);
    CHECK(std::abs(t->translation()[1] - 2.0) < 1e-9);
    CHECK(std::abs(t->translation()[2] - 3.0) < 1e-9);
    CHECK(std::abs(t->ts - 7.0) < 1e-9);
}

TEST_CASE("tbuffer prunes samples outside the window") {
    TBuffer buffer(5.0);
    buffer.add(1.0, Rigid{});
    buffer.add(2.0, Rigid{});
    buffer.add(10.0, Rigid{});
    CHECK(buffer.samples().size() == 1);
    CHECK(std::abs(buffer.last()->ts - 10.0) < 1e-9);
}

TEST_CASE("tbuffer late sample does not spare ones the window has aged out") {
    TBuffer buffer(5.0);
    buffer.add(10.0, Rigid{});
    buffer.add(11.0, Rigid{});
    buffer.add(7.0, Rigid{});
    CHECK(std::abs(buffer.samples().front().ts - 7.0) < 1e-9);
    buffer.add(13.0, Rigid{});
    // 7.0 is now older than newest - window.
    CHECK(std::abs(buffer.samples().front().ts - 10.0) < 1e-9);
}

TEST_CASE("tbuffer clock reset drops the pre-jump samples") {
    TBuffer buffer(5.0);
    for (int i = 0; i < 5; ++i) {
        buffer.add(1000.0 + i, Rigid{});
    }
    for (int i = 0; i < 20; ++i) {
        buffer.add(100.0 + i, Rigid{});
    }
    CHECK(buffer.samples().size() <= 6);
    CHECK(std::abs(buffer.last()->ts - 119.0) < 1e-9);
}

TEST_CASE("tbuffer add out of order keeps samples sorted") {
    TBuffer buffer(DEFAULT_TF_WINDOW_SECS);
    buffer.add(3.0, Rigid{});
    buffer.add(1.0, Rigid{});
    buffer.add(2.0, Rigid{});
    CHECK(std::abs(buffer.last()->ts - 3.0) < 1e-9);
    const auto* closest = buffer.find_closest(1.9, std::nullopt);
    REQUIRE(closest != nullptr);
    CHECK(std::abs(closest->ts - 2.0) < 1e-9);
}
