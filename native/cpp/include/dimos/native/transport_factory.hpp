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
#include "dimos/native/zenoh_transport.hpp"

namespace dimos::native {

/// Construct the transport named by `DIMOS_TRANSPORT`, configured from the
/// coordinator's `launch` line. An unset or unknown value throws.
inline std::unique_ptr<Transport> make_transport_from_env(
    const nlohmann::json& launch = nlohmann::json::object()) {
    const char* env = std::getenv("DIMOS_TRANSPORT");
    const std::string name = env != nullptr ? env : "";
    require_supported_transport(name);
    if (name == "zenoh") {
        return ZenohTransport::from_launch(launch);
    }
    return std::make_unique<LcmTransport>();
}

}  // namespace dimos::native
