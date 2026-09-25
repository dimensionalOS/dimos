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

#pragma once

#include <cstdint>
#include <stdexcept>
#include <vector>
#include <fastcdr/Cdr.h>
#include <fastcdr/CdrSizeCalculator.hpp>
#include <fastcdr/exceptions/Exception.h>

namespace dimos::cdr {

template<class T>
std::vector<uint8_t> encode(const T& value, bool little_endian = true) try {
    using namespace eprosima::fastcdr;
    value.validate();
    size_t alignment = 0;
    CdrSizeCalculator calculator(CdrVersion::XCDRv1);
    const auto body_size = calculator.calculate_serialized_size(value, alignment);
    std::vector<uint8_t> bytes(4 + body_size, 0);
    FastBuffer buffer(reinterpret_cast<char*>(bytes.data()), bytes.size());
    Cdr writer(buffer, little_endian ? Cdr::LITTLE_ENDIANNESS : Cdr::BIG_ENDIANNESS,
               CdrVersion::XCDRv1);
    writer.serialize_encapsulation();
    writer << value;
    bytes.resize(writer.get_serialized_data_length());
    return bytes;
} catch (const eprosima::fastcdr::exception::Exception& error) {
    throw std::invalid_argument(error.what());
}

template<class T>
T decode(const uint8_t* bytes, size_t size) try {
    using namespace eprosima::fastcdr;
    if (size < 4 || bytes[0] != 0 || bytes[1] > 1 || bytes[2] != 0 || bytes[3] != 0) {
        throw std::invalid_argument("Expected plain CDR/XCDR1 encapsulation");
    }
    // FastBuffer's API takes mutable storage; decoding only reads it.
    FastBuffer buffer(reinterpret_cast<char*>(const_cast<uint8_t*>(bytes)), size);
    Cdr reader(buffer, Cdr::DEFAULT_ENDIAN, CdrVersion::XCDRv1);
    reader.read_encapsulation();
    T value{};
    reader >> value;
    if (reader.get_serialized_data_length() != size) {
        throw std::invalid_argument("Trailing bytes after CDR message");
    }
    value.validate();
    return value;
} catch (const eprosima::fastcdr::exception::Exception& error) {
    throw std::invalid_argument(error.what());
}

template<class T>
T decode(const std::vector<uint8_t>& bytes) {
    return decode<T>(bytes.data(), bytes.size());
}

}  // namespace dimos::cdr
