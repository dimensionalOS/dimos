// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
#include <std_msgs/msg/header.hpp>
#include <dimos_cdrgen/cdr.hpp>
#include <fstream>
#include <iterator>

using Header = std_msgs::msg::Header;

int main(int argc, char** argv) {
  if (argc != 3) return 2;
  std::ifstream input(argv[1], std::ios::binary);
  const std::vector<uint8_t> bytes{std::istreambuf_iterator<char>(input), {}};
  auto value = dimos::cdr::cdr_decode<Header>(bytes);
  value.frame_id += "/cpp";
  const auto encoded = dimos::cdr::cdr_encode(value);
  std::ofstream output(argv[2], std::ios::binary);
  output.write(reinterpret_cast<const char*>(encoded.data()), encoded.size());
  return output ? 0 : 1;
}
