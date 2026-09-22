// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <dimos_generated/messages.hpp>

namespace dimos::native {

/// Default output codec for generated ROS2-shaped message values.
template <class T>
std::vector<uint8_t> cdr_encode(const T& message) {
    return dimos::cdr::encode(message);
}

/// Default input codec; malformed and trailing data raise std::invalid_argument.
template <class T>
T cdr_decode(const uint8_t* data, std::size_t size) {
    return dimos::cdr::decode<T>(data, size);
}

}  // namespace dimos::native
