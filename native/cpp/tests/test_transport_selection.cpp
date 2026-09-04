// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

#include <doctest/doctest.h>

#include <stdexcept>
#include <string>

#include "dimos/native/transport_selection.hpp"

using namespace dimos::native;

TEST_CASE("lcm is a supported transport") {
    require_supported_transport("lcm");  // must not throw
    CHECK(true);
}

TEST_CASE("zenoh is supported when the build has it, and says how to get it when not") {
    if (kZenohSupported) {
        require_supported_transport("zenoh");  // must not throw
        CHECK(true);
        return;
    }
    try {
        require_supported_transport("zenoh");
        FAIL("expected require_supported_transport(\"zenoh\") to throw");
    } catch (const std::runtime_error& e) {
        const std::string msg = e.what();
        CHECK(msg.find("zenoh") != std::string::npos);
        CHECK(msg.find("zenoh-cpp") != std::string::npos);
    }
}

TEST_CASE("an unknown transport is rejected and names the offending value") {
    CHECK_THROWS_AS(require_supported_transport("bogus"), std::runtime_error);
    try {
        require_supported_transport("bogus");
    } catch (const std::runtime_error& e) {
        CHECK(std::string(e.what()).find("bogus") != std::string::npos);
    }
}

TEST_CASE("an unset/empty transport is rejected") {
    CHECK_THROWS_AS(require_supported_transport(""), std::runtime_error);
}
