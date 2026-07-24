// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Generic adapters from dimos-lcm generated message types to the SDK's port
// encode/decode signatures. Every lcm-gen C++ type exposes getEncodedSize /
// encode / decode, so one template covers all of them.

#pragma once

#include <cstdint>
#include <stdexcept>
#include <vector>

namespace dimos::native {

/// Encode any lcm-gen message `T` to a byte buffer. Pass as an output's encoder:
/// `builder.output<Twist>("data", lcm_encode<Twist>)`.
template <class T>
std::vector<uint8_t> lcm_encode(const T& msg) {
    std::vector<uint8_t> buf(static_cast<std::size_t>(msg.getEncodedSize()));
    msg.encode(buf.data(), 0, static_cast<int>(buf.size()));
    return buf;
}

/// Decode a byte buffer into an lcm-gen message `T`, throwing on malformed input.
/// Pass as an input's decoder: `builder.input<Twist>("data", lcm_decode<Twist>, handler)`.
template <class T>
T lcm_decode(const uint8_t* data, std::size_t len) {
    T msg;
    if (msg.decode(data, 0, static_cast<int>(len)) < 0) {
        throw std::runtime_error("lcm_decode: message decode failed");
    }
    return msg;
}

}  // namespace dimos::native
