// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

#include <doctest/doctest.h>

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <vector>

#include "dimos/native/lcm_codec.hpp"

using namespace dimos::native;

namespace {

// Stand-in with the same method surface every lcm-gen C++ type exposes, so the
// generic codec can be exercised without pulling in real generated messages.
struct FakeMsg {
    std::int32_t value = 0;

    int getEncodedSize() const { return static_cast<int>(sizeof(std::int32_t)); }

    int encode(void* buf, int offset, int maxlen) const {
        if (maxlen - offset < static_cast<int>(sizeof(std::int32_t))) {
            return -1;
        }
        std::memcpy(static_cast<char*>(buf) + offset, &value, sizeof(std::int32_t));
        return static_cast<int>(sizeof(std::int32_t));
    }

    int decode(const void* buf, int offset, int maxlen) {
        if (maxlen - offset < static_cast<int>(sizeof(std::int32_t))) {
            return -1;
        }
        std::memcpy(&value, static_cast<const char*>(buf) + offset, sizeof(std::int32_t));
        return static_cast<int>(sizeof(std::int32_t));
    }
};

}  // namespace

TEST_CASE("lcm_encode / lcm_decode round-trip a message") {
    FakeMsg msg;
    msg.value = 1234;

    std::vector<uint8_t> bytes = lcm_encode(msg);
    CHECK(bytes.size() == sizeof(std::int32_t));

    FakeMsg decoded = lcm_decode<FakeMsg>(bytes.data(), bytes.size());
    CHECK(decoded.value == 1234);
}

TEST_CASE("lcm_decode throws on a short buffer") {
    std::vector<uint8_t> truncated{1, 2};
    CHECK_THROWS_AS(lcm_decode<FakeMsg>(truncated.data(), truncated.size()), std::runtime_error);
}
