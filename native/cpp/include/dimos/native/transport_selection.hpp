// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Transport selection. The coordinator sets DIMOS_TRANSPORT for every native
// process, and this SDK implements LCM only.

#pragma once

#include <stdexcept>
#include <string>

namespace dimos::native {

/// Throw unless `name` is a transport this SDK implements.
inline void require_supported_transport(const std::string& name) {
    if (name == "lcm") {
        return;
    }
    if (name == "zenoh") {
        throw std::runtime_error(
            "DIMOS_TRANSPORT=zenoh is not supported by the C++ native SDK (LCM only). "
            "Set DIMOS_TRANSPORT=lcm, or use a Rust native module for zenoh.");
    }
    throw std::runtime_error("DIMOS_TRANSPORT must be 'lcm', got '" + name + "'");
}

}  // namespace dimos::native
