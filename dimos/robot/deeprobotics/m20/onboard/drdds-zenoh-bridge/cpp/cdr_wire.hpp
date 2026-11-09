// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

// DrIDL and ROSIDL define the same C++ namespaces with incompatible classes.
// Keep their headers in separate translation units; only bridge values cross.
namespace bridge_wire {
struct Time {
  int32_t sec{};
  uint32_t nanosec{};
};
struct Header {
  Time stamp;
  std::string frame_id;
};
struct Vector3 {
  double x{}, y{}, z{};
};
struct Quaternion {
  double x{}, y{}, z{}, w{};
};
struct Pose {
  Vector3 position;
  Quaternion orientation;
};
struct Twist {
  Vector3 linear, angular;
};
struct PoseWithCovariance {
  Pose pose;
  std::array<double, 36> covariance{};
};
struct TwistWithCovariance {
  Twist twist;
  std::array<double, 36> covariance{};
};
struct Odometry {
  Header header;
  std::string child_frame_id;
  PoseWithCovariance pose;
  TwistWithCovariance twist;
};
struct PointField {
  std::string name;
  uint32_t offset{};
  uint8_t datatype{};
  uint32_t count{};
};
struct PointCloud2 {
  Header header;
  uint32_t height{}, width{};
  std::vector<PointField> fields;
  bool is_bigendian{};
  uint32_t point_step{}, row_step{};
  std::vector<uint8_t> data;
  bool is_dense{};
};
struct Transform {
  Vector3 translation;
  Quaternion rotation;
};
struct TransformStamped {
  Header header;
  std::string child_frame_id;
  Transform transform;
};
struct TFMessage {
  std::vector<TransformStamped> transforms;
};
struct Bool {
  bool data{};
};
struct Int32 {
  int32_t data{};
};
struct UInt32 {
  uint32_t data{};
};
std::vector<uint8_t> encode(const PointCloud2 &);
std::vector<uint8_t> encode(const Odometry &);
std::vector<uint8_t> encode(const TFMessage &);
std::vector<uint8_t> encode(const Bool &);
std::vector<uint8_t> encode(const Int32 &);
std::vector<uint8_t> encode(const UInt32 &);
bool decode(const uint8_t *, size_t, Twist *);
bool decode(const uint8_t *, size_t, Int32 *);
bool decode(const uint8_t *, size_t, UInt32 *);
} // namespace bridge_wire
