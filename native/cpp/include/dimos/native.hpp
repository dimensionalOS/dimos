// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Umbrella header for the dimos C++ native module SDK.

#pragma once

#include <cstdlib>
#include <memory>
#include <string>
#include <utility>

#include "dimos/native/config.hpp"
#include "dimos/native/lcm_codec.hpp"
#include "dimos/native/lcm_transport.hpp"
#include "dimos/native/log.hpp"
#include "dimos/native/module.hpp"
#include "dimos/native/transport.hpp"
#include "dimos/native/transport_selection.hpp"
#include "dimos/native/zenoh_transport.hpp"

namespace dimos::native {

/// Construct the transport named by `DIMOS_TRANSPORT`. `launch` is the
/// coordinator's stdin blob; zenoh reads its `session` and `qos` blocks.
/// Errors clearly for an unknown or unset value.
inline std::unique_ptr<Transport> make_transport_from_env(const nlohmann::json& launch) {
    const char* env = std::getenv("DIMOS_TRANSPORT");
    std::string name = env != nullptr ? env : "";
    require_supported_transport(name);
    if (name == "zenoh") {
        return std::make_unique<ZenohTransport>(launch);
    }
    return std::make_unique<LcmTransport>();
}

/// Run module `M` over the transport named by DIMOS_TRANSPORT (LCM or zenoh).
/// The coordinator always sets it. An unset or unknown value is fatal.
template <class M>
void run_with_transport() {
    try {
        StdinConfig parsed = read_stdin_config();
        // Built before the call: the argument moves `parsed`.
        std::unique_ptr<Transport> transport = make_transport_from_env(parsed.launch);
        run_fallible<M>(std::move(transport), std::move(parsed));
    } catch (const std::exception& e) {
        log::error(e.what());
        std::exit(1);
    }
}

}  // namespace dimos::native
