// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Adapters from dimos-lcm message types to the SDK's encode/decode signatures.

#pragma once

#include <cstdint>
#include <stdexcept>
#include <vector>

namespace dimos::native {

/// Encode any lcm-gen message `T` to a byte buffer. The default output encoder.
template <class T>
std::vector<uint8_t> lcm_encode(const T& msg) {
    std::vector<uint8_t> buf(static_cast<std::size_t>(msg.getEncodedSize()));
    msg.encode(buf.data(), 0, static_cast<int>(buf.size()));
    return buf;
}

/// Decode a byte buffer into an lcm-gen message `T`, throwing on malformed input.
/// The default input decoder.
template <class T>
T lcm_decode(const uint8_t* data, std::size_t len) {
    T msg;
    if (msg.decode(data, 0, static_cast<int>(len)) < 0) {
        throw std::runtime_error("lcm_decode: message decode failed");
    }
    return msg;
}

}  // namespace dimos::native
