// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

#include <doctest/doctest.h>

#include <cmath>
#include <cstdint>
#include <string>

#include "dimos/native/tf.hpp"

using dimos::native::TfTree;

namespace {

Eigen::Isometry3d translate(double x, double y, double z) {
    Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
    iso.translation() = Eigen::Vector3d(x, y, z);
    return iso;
}

Eigen::Isometry3d rotate_z(double radians) {
    Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
    iso.linear() = Eigen::AngleAxisd(radians, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return iso;
}

}  // namespace

TEST_CASE("identity for a frame against itself") {
    TfTree tree;
    tree.set("base", "camera", translate(1.0, 0.0, 0.0));
    const auto result = tree.get("camera", "camera");
    REQUIRE(result.has_value());
    CHECK(result->isApprox(Eigen::Isometry3d::Identity()));
}

TEST_CASE("chain lookup composes along the parent path") {
    TfTree tree;
    tree.set("base", "mount", translate(0.0, 0.0, 1.0));
    tree.set("mount", "camera", translate(0.5, 0.0, 0.0));
    const auto result = tree.get("base", "camera");
    REQUIRE(result.has_value());
    CHECK(result->translation().isApprox(Eigen::Vector3d(0.5, 0.0, 1.0)));
}

TEST_CASE("lookup through the nearest common ancestor") {
    TfTree tree;
    tree.set("base", "left", translate(0.0, 1.0, 0.0));
    tree.set("base", "right", translate(0.0, -1.0, 0.0));
    const auto result = tree.get("left", "right");
    REQUIRE(result.has_value());
    CHECK(result->translation().isApprox(Eigen::Vector3d(0.0, -2.0, 0.0)));
}

TEST_CASE("rotation composes and inverts correctly") {
    TfTree tree;
    tree.set("base", "turret", rotate_z(M_PI / 2.0));
    tree.set("turret", "camera", translate(1.0, 0.0, 0.0));
    const auto base_from_camera = tree.get("base", "camera");
    REQUIRE(base_from_camera.has_value());
    // x in the turret frame points along base +y after the 90 degree yaw.
    CHECK(base_from_camera->translation().isApprox(Eigen::Vector3d(0.0, 1.0, 0.0), 1e-12));
    const auto camera_from_base = tree.get("camera", "base");
    REQUIRE(camera_from_base.has_value());
    CHECK((*base_from_camera * *camera_from_base).isApprox(Eigen::Isometry3d::Identity()));
}

TEST_CASE("disconnected frames return nullopt") {
    TfTree tree;
    tree.set("base", "camera", translate(1.0, 0.0, 0.0));
    tree.set("world", "lidar", translate(0.0, 1.0, 0.0));
    CHECK_FALSE(tree.get("camera", "lidar").has_value());
    CHECK_FALSE(tree.get("base", "unknown").has_value());
}

TEST_CASE("a tf cycle terminates instead of hanging") {
    TfTree tree;
    tree.set("a", "b", translate(1.0, 0.0, 0.0));
    tree.set("b", "a", translate(-1.0, 0.0, 0.0));
    CHECK_FALSE(tree.get("a", "unrelated").has_value());
}

TEST_CASE("publish stores the edge and forwards it to the sink") {
    TfTree tree;
    std::string seen;
    std::int64_t seen_ns = 0;
    tree.set_publish_sink([&](const std::string& parent, const std::string& child,
                              const Eigen::Isometry3d&, std::int64_t timestamp_ns) {
        seen = parent + "->" + child;
        seen_ns = timestamp_ns;
    });
    tree.publish("odom", "base", translate(1.0, 2.0, 3.0), 42);
    CHECK(seen == "odom->base");
    CHECK(seen_ns == 42);
    const auto result = tree.get("odom", "base");
    REQUIRE(result.has_value());
    CHECK(result->translation().isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
}

TEST_CASE("re-setting an edge overwrites it") {
    TfTree tree;
    tree.set("base", "camera", translate(1.0, 0.0, 0.0));
    tree.set("base", "camera", translate(2.0, 0.0, 0.0));
    const auto result = tree.get("base", "camera");
    REQUIRE(result.has_value());
    CHECK(result->translation().isApprox(Eigen::Vector3d(2.0, 0.0, 0.0)));
}
