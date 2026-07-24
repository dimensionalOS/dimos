// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Umbrella header for the dimos C++ native module SDK. A module includes this,
// subclasses Module, and calls run_with_transport<M>() from main().

#pragma once

#include "dimos/native/config.hpp"
#include "dimos/native/lcm_codec.hpp"
#include "dimos/native/lcm_transport.hpp"
#include "dimos/native/log.hpp"
#include "dimos/native/module.hpp"
#include "dimos/native/transport.hpp"
#include "dimos/native/transport_selection.hpp"

namespace dimos::native {

/// Run module `M` over the transport named by DIMOS_TRANSPORT (LCM today).
/// The coordinator always sets it. An unset or unknown value is fatal.
template <class M>
void run_with_transport() {
    try {
        run<M>(make_transport_from_env());
    } catch (const std::exception& e) {
        log::error(e.what());
        std::exit(1);
    }
}

}  // namespace dimos::native
