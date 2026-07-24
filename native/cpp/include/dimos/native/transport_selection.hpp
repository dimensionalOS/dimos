// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Transport selection. The coordinator sets DIMOS_TRANSPORT for every native
// process. The C++ SDK ships only LCM, so this is where an unsupported value
// (notably zenoh, which is Rust/Python-only) turns into a clear error instead
// of a confusing failure deeper in the stack.

#pragma once

#include <stdexcept>
#include <string>

namespace dimos::native {

/// Throw unless `name` is a transport this SDK implements. Mirrors the Rust
/// runtime, which also treats an unset or unknown value as fatal.
inline void require_supported_transport(const std::string& name) {
    if (name == "lcm") {
        return;
    }
    // TODO: Add zenoh support here
    if (name == "zenoh") {
        throw std::runtime_error(
            "DIMOS_TRANSPORT=zenoh is not supported by the C++ native SDK (LCM only). "
            "Set DIMOS_TRANSPORT=lcm, or use a Rust native module for zenoh.");
    }
    throw std::runtime_error("DIMOS_TRANSPORT must be 'lcm', got '" + name + "'");
}

}  // namespace dimos::native
