// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
#include <telemetry_msgs/msg/device_reading.hpp>
#include <std_msgs/msg/header.hpp>
#include <dimos_cdrgen/cdr.hpp>
#include <fstream>
#include <iterator>
#include <type_traits>

using Reading = telemetry_msgs::msg::DeviceReading;
static_assert(std::is_same_v<decltype(Reading{}.header), std_msgs::msg::Header>);

int main(int argc, char** argv) {
  if (argc != 3) return 2;
  std::ifstream input(argv[1], std::ios::binary);
  const std::vector<uint8_t> bytes{std::istreambuf_iterator<char>(input), {}};
  auto value = dimos::cdr::cdr_decode<Reading>(bytes);
  const std_msgs::msg::Header shared = value.header;
  value.header = shared;
  value.value += 1.0;
  value.label += "/cpp";
  const auto encoded = dimos::cdr::cdr_encode(value);
  std::ofstream output(argv[2], std::ios::binary);
  output.write(reinterpret_cast<const char*>(encoded.data()), encoded.size());
  return output ? 0 : 1;
}
