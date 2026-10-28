// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
#include "cdr_wire.hpp"
#include <dimos_cdrgen/cdr.hpp>
#include <exception>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

namespace bridge_wire {
namespace {
std_msgs::msg::Header convert(const Header &value) {
  std_msgs::msg::Header out;
  out.stamp.sec = value.stamp.sec;
  out.stamp.nanosec = value.stamp.nanosec;
  out.frame_id = value.frame_id;
  return out;
}
template <class Output, class Input> Output xyz(const Input &value) {
  Output out;
  out.x = value.x;
  out.y = value.y;
  out.z = value.z;
  return out;
}
geometry_msgs::msg::Quaternion convert(const Quaternion &value) {
  auto out = xyz<geometry_msgs::msg::Quaternion>(value);
  out.w = value.w;
  return out;
}
template <class Generated, class Bridge>
bool scalar_decode(const uint8_t *data, size_t size, Bridge *out) {
  try {
    out->data = dimos::cdr::cdr_decode<Generated>(data, size).data;
    return true;
  } catch (const std::exception &) {
    return false;
  }
}
} // namespace
std::vector<uint8_t> encode(const PointCloud2 &value) {
  sensor_msgs::msg::PointCloud2 out;
  out.header = convert(value.header);
  out.height = value.height;
  out.width = value.width;
  for (const auto &field : value.fields) {
    sensor_msgs::msg::PointField generated;
    generated.name = field.name;
    generated.offset = field.offset;
    generated.datatype = field.datatype;
    generated.count = field.count;
    out.fields.push_back(generated);
  }
  out.is_bigendian = value.is_bigendian;
  out.point_step = value.point_step;
  out.row_step = value.row_step;
  out.data = value.data;
  out.is_dense = value.is_dense;
  return dimos::cdr::cdr_encode(out);
}
std::vector<uint8_t> encode(const Odometry &value) {
  nav_msgs::msg::Odometry out;
  out.header = convert(value.header);
  out.child_frame_id = value.child_frame_id;
  out.pose.pose.position =
      xyz<geometry_msgs::msg::Point>(value.pose.pose.position);
  out.pose.pose.orientation = convert(value.pose.pose.orientation);
  out.pose.covariance = value.pose.covariance;
  out.twist.twist.linear =
      xyz<geometry_msgs::msg::Vector3>(value.twist.twist.linear);
  out.twist.twist.angular =
      xyz<geometry_msgs::msg::Vector3>(value.twist.twist.angular);
  out.twist.covariance = value.twist.covariance;
  return dimos::cdr::cdr_encode(out);
}
std::vector<uint8_t> encode(const TFMessage &value) {
  tf2_msgs::msg::TFMessage out;
  for (const auto &edge : value.transforms) {
    geometry_msgs::msg::TransformStamped generated;
    generated.header = convert(edge.header);
    generated.child_frame_id = edge.child_frame_id;
    generated.transform.translation =
        xyz<geometry_msgs::msg::Vector3>(edge.transform.translation);
    generated.transform.rotation = convert(edge.transform.rotation);
    out.transforms.push_back(generated);
  }
  return dimos::cdr::cdr_encode(out);
}
std::vector<uint8_t> encode(const Bool &value) {
  std_msgs::msg::Bool out;
  out.data = value.data;
  return dimos::cdr::cdr_encode(out);
}
std::vector<uint8_t> encode(const Int32 &value) {
  std_msgs::msg::Int32 out;
  out.data = value.data;
  return dimos::cdr::cdr_encode(out);
}
std::vector<uint8_t> encode(const UInt32 &value) {
  std_msgs::msg::UInt32 out;
  out.data = value.data;
  return dimos::cdr::cdr_encode(out);
}
bool decode(const uint8_t *data, size_t size, Twist *out) {
  try {
    const auto value =
        dimos::cdr::cdr_decode<geometry_msgs::msg::Twist>(data, size);
    out->linear = xyz<Vector3>(value.linear);
    out->angular = xyz<Vector3>(value.angular);
    return true;
  } catch (const std::exception &) {
    return false;
  }
}
bool decode(const uint8_t *data, size_t size, Int32 *out) {
  return scalar_decode<std_msgs::msg::Int32>(data, size, out);
}
bool decode(const uint8_t *data, size_t size, UInt32 *out) {
  return scalar_decode<std_msgs::msg::UInt32>(data, size, out);
}
} // namespace bridge_wire
