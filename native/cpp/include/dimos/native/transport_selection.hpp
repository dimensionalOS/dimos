// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Transport selection. The coordinator sets DIMOS_TRANSPORT for every native
// process. Which transports a build can name depends on what it was compiled
// against, so this is a check on the build as much as on the value.

#pragma once

#include <stdexcept>
#include <string>

namespace dimos::native {

/// Whether this build has the zenoh transport compiled in. The SDK's CMake
/// defines DIMOS_NATIVE_ZENOH when it finds zenoh-cpp.
inline constexpr bool kZenohSupported =
#ifdef DIMOS_NATIVE_ZENOH
    true;
#else
    false;
#endif

/// Throw unless `name` is a transport this build implements.
inline void require_supported_transport(const std::string& name) {
    if (name == "lcm") {
        return;
    }
    if (name == "zenoh") {
        if (kZenohSupported) {
            return;
        }
        throw std::runtime_error(
            "DIMOS_TRANSPORT=zenoh, but this module was built without zenoh support. "
            "Build it against zenoh-cpp, or set DIMOS_TRANSPORT=lcm.");
    }
    throw std::runtime_error("DIMOS_TRANSPORT must be 'lcm' or 'zenoh', got '" + name + "'");
}

}  // namespace dimos::native
