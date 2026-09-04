// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Umbrella header for the dimos C++ native module SDK.

#pragma once

#include "dimos/native/config.hpp"
#include "dimos/native/lcm_codec.hpp"
#include "dimos/native/lcm_transport.hpp"
#include "dimos/native/log.hpp"
#include "dimos/native/module.hpp"
#include "dimos/native/transport.hpp"
#include "dimos/native/transport_factory.hpp"
#include "dimos/native/transport_selection.hpp"

namespace dimos::native {

/// Run module `M` over the transport named by DIMOS_TRANSPORT. The coordinator
/// always sets it. An unset or unknown value is fatal.
///
/// The launch line is read first because zenoh opens its session from it.
template <class M>
void run_with_transport() {
    try {
        StdinConfig parsed = read_stdin_config();
        std::unique_ptr<Transport> transport = make_transport_from_env(parsed.launch);
        run<M>(std::move(transport), std::move(parsed));
    } catch (const std::exception& e) {
        log::error(e.what());
        std::exit(1);
    }
}

}  // namespace dimos::native
