// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Only compiled when liblcm is available (see tests/CMakeLists.txt). Its job is
// to typecheck LcmTransport's inline bodies against the real LCM headers and lib.
// It does not open an LCM endpoint, since unit tests must not assume a multicast
// route exists.

#include <doctest/doctest.h>

#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <vector>

#include "dimos/native/lcm_transport.hpp"

using namespace dimos::native;

// Compiled (so every inline body is typechecked against liblcm) but never
// called: constructing a real LCM endpoint needs a multicast route.
[[maybe_unused]] static void lcm_transport_compile_check() {
    LcmTransport t;
    Transport& base = t;
    base.publish("/c", std::vector<uint8_t>{1, 2, 3});
    base.subscribe("/c", [](const uint8_t*, std::size_t) {});
    base.set_publisher_qos("{}");
    auto owned = make_transport_from_env();
    (void)owned;
}

TEST_CASE("LcmTransport implements the Transport interface") {
    CHECK(std::is_base_of<Transport, LcmTransport>::value);
}
