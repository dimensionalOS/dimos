// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Transport selection. The coordinator sets DIMOS_TRANSPORT for every native
// process.

#pragma once

#include <stdexcept>
#include <string>

namespace dimos::native {

/// Throw unless `name` is a transport the SDK implements.
inline void require_supported_transport(const std::string& name) {
    if (name == "lcm" || name == "zenoh") {
        return;
    }
    throw std::runtime_error("DIMOS_TRANSPORT must be 'lcm' or 'zenoh', got '" + name + "'");
}

}  // namespace dimos::native
