// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Builds the transport DIMOS_TRANSPORT names. Kept apart from
// transport_selection.hpp so checking a transport name does not drag in every
// transport's library.

#pragma once

#include <cstdlib>
#include <memory>
#include <string>

#include <nlohmann/json.hpp>

#include "dimos/native/lcm_transport.hpp"
#include "dimos/native/transport.hpp"
#include "dimos/native/transport_selection.hpp"

#ifdef DIMOS_NATIVE_ZENOH
#include "dimos/native/zenoh_transport.hpp"
#endif

namespace dimos::native {

/// Construct the transport named by `DIMOS_TRANSPORT`, configured from the
/// coordinator's `launch` line. An unset, unknown, or not-compiled-in value
/// throws.
inline std::unique_ptr<Transport> make_transport_from_env(
    const nlohmann::json& launch = nlohmann::json::object()) {
    const char* env = std::getenv("DIMOS_TRANSPORT");
    const std::string name = env != nullptr ? env : "";
    require_supported_transport(name);
#ifdef DIMOS_NATIVE_ZENOH
    if (name == "zenoh") {
        return ZenohTransport::from_launch(launch);
    }
#else
    (void)launch;
#endif
    return std::make_unique<LcmTransport>();
}

}  // namespace dimos::native
