// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
#include "messages.hpp"
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>

int main(int argc, char** argv) {
    if (argc != 3) return 2;
    try {
        std::ifstream input(argv[1], std::ios::binary);
        if (!input) throw std::runtime_error("Cannot open input");
        std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(input)), {});
        auto message = dimos::cdr::decode<story_msgs::msg::DeviceReading>(bytes);
        ++message.sequence;
        message.value += 1;
        message.label += "/cpp";
        std::cout << "C++ consumer: value=" << message.value << " label=" << message.label << '\n';
        auto encoded = dimos::cdr::encode(message);
        std::ofstream output(argv[2], std::ios::binary);
        output.write(reinterpret_cast<const char*>(encoded.data()), encoded.size());
        if (!output) throw std::runtime_error("Cannot write output");
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
