// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <fastcdr/Cdr.h>
#include <fastcdr/FastBuffer.h>
#include <fastcdr/exceptions/Exception.h>
#include <rosidl_typesupport_fastrtps_cpp/message_type_support.h>
#include <rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace dimos::native {

// Upstream generates the specializations and callback table; no per-message
// DimOS traits, field walkers or serialization emitters are required.
template<class T>
const message_type_support_callbacks_t& cdr_callbacks() {
    const auto* support = rosidl_typesupport_fastrtps_cpp::get_message_type_support_handle<T>();
    return *static_cast<const message_type_support_callbacks_t*>(support->data);
}

template<class T>
std::vector<uint8_t> cdr_encode(const T& message, bool little_endian = true) {
    const auto& callbacks = cdr_callbacks<T>();
    std::vector<uint8_t> bytes(4 + callbacks.get_serialized_size(&message), 0);
    eprosima::fastcdr::FastBuffer buffer(reinterpret_cast<char*>(bytes.data()), bytes.size());
    eprosima::fastcdr::Cdr stream(buffer, little_endian ? eprosima::fastcdr::Cdr::LITTLE_ENDIANNESS : eprosima::fastcdr::Cdr::BIG_ENDIANNESS,
                                eprosima::fastcdr::CdrVersion::XCDRv1);
    stream.set_encoding_flag(eprosima::fastcdr::EncodingAlgorithmFlag::PLAIN_CDR);
    stream.serialize_encapsulation();
    if (!callbacks.cdr_serialize(&message, stream)) throw std::runtime_error("CDR serialization failed");
    bytes.resize(stream.get_serialized_data_length());
    return bytes;
}

template<class T>
T cdr_decode(const uint8_t* data, std::size_t size) {
    if (size < 4 || data[0] != 0 || data[1] > 1 || data[2] != 0 || data[3] != 0)
        throw std::invalid_argument("Expected plain CDR/XCDR1 encapsulation");
    try {
        eprosima::fastcdr::FastBuffer buffer(reinterpret_cast<char*>(const_cast<uint8_t*>(data)), size);
        eprosima::fastcdr::Cdr stream(buffer);
        stream.read_encapsulation();
        T message;
        if (!cdr_callbacks<T>().cdr_deserialize(stream, &message))
            throw std::invalid_argument("CDR deserialization failed");
        if (stream.get_serialized_data_length() != size)
            throw std::invalid_argument("Trailing bytes after CDR message");
        return message;
    } catch (const eprosima::fastcdr::exception::Exception& error) {
        throw std::invalid_argument(error.what());
    }
}
template<class T>
T cdr_decode(const std::vector<uint8_t>& bytes) {
    return cdr_decode<T>(bytes.data(), bytes.size());
}
} // namespace dimos::native
