// Copyright 2026 Dimensional Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include "messages.hpp"

int main(int argc, char** argv) {
    if (argc != 4) {
        std::cerr << "usage: relay <edit|echo|echo-be|defaults> <input.cdr> <output.cdr>\n";
        return 2;
    }
    try {
        std::ifstream input(argv[2], std::ios::binary);
        if (!input) throw std::runtime_error("Cannot open input");
        std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(input)), {});
        const std::string mode(argv[1]);
        auto value = mode == "defaults" ? demo_msgs::msg::Telemetry{} : dimos::cdr::decode<demo_msgs::msg::Telemetry>(bytes);
        std::cout << "C++ received: frame=" << value.header.frame_id
                  << " sequence=" << value.sequence << " label=" << value.label
                  << " temperature=" << value.reading.temperature
                  << " x=" << value.position.x << " axes=[" << value.axes[0]
                  << "," << value.axes[1] << "," << value.axes[2] << "]\n";
        if (mode == "edit") {
            ++value.sequence;
            value.hops.push_back(2);
            value.label += "/cpp";
        } else if (mode != "echo" && mode != "echo-be" && mode != "defaults") {
            throw std::runtime_error("Expected edit, echo, echo-be, or defaults mode");
        }
        const auto output = dimos::cdr::encode(value, mode != "echo-be");
        std::ofstream stream(argv[3], std::ios::binary);
        stream.write(reinterpret_cast<const char*>(output.data()), output.size());
        if (!stream) throw std::runtime_error("Cannot write output");
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
