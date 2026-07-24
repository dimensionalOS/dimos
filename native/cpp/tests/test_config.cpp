// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

#include <doctest/doctest.h>

#include <nlohmann/json.hpp>

#include <cstdint>
#include <cstdio>
#include <stdexcept>
#include <string>

#include "dimos/native/config.hpp"

using dimos::native::Config;
using nlohmann::json;

namespace {
struct RangedCfg {
    std::int64_t value;
    std::string name;

    void validate() const {
        if (value < 0 || value > 100) {
            throw std::runtime_error("value out of range [0, 100]");
        }
    }
};

// 70 fields: past the 64-field ceiling of nlohmann's field-list macros, which
// is why parse() reflects over the struct instead of using them.
struct WideCfg {
    std::int64_t f00; std::int64_t f01; std::int64_t f02; std::int64_t f03;
    std::int64_t f04; std::int64_t f05; std::int64_t f06; std::int64_t f07;
    std::int64_t f08; std::int64_t f09; std::int64_t f10; std::int64_t f11;
    std::int64_t f12; std::int64_t f13; std::int64_t f14; std::int64_t f15;
    std::int64_t f16; std::int64_t f17; std::int64_t f18; std::int64_t f19;
    std::int64_t f20; std::int64_t f21; std::int64_t f22; std::int64_t f23;
    std::int64_t f24; std::int64_t f25; std::int64_t f26; std::int64_t f27;
    std::int64_t f28; std::int64_t f29; std::int64_t f30; std::int64_t f31;
    std::int64_t f32; std::int64_t f33; std::int64_t f34; std::int64_t f35;
    std::int64_t f36; std::int64_t f37; std::int64_t f38; std::int64_t f39;
    std::int64_t f40; std::int64_t f41; std::int64_t f42; std::int64_t f43;
    std::int64_t f44; std::int64_t f45; std::int64_t f46; std::int64_t f47;
    std::int64_t f48; std::int64_t f49; std::int64_t f50; std::int64_t f51;
    std::int64_t f52; std::int64_t f53; std::int64_t f54; std::int64_t f55;
    std::int64_t f56; std::int64_t f57; std::int64_t f58; std::int64_t f59;
    std::int64_t f60; std::int64_t f61; std::int64_t f62; std::int64_t f63;
    std::int64_t f64; std::int64_t f65; std::int64_t f66; std::int64_t f67;
    std::int64_t f68; std::int64_t f69;
};

json wide_json() {
    json j = json::object();
    for (int i = 0; i < 70; ++i) {
        char key[4];
        std::snprintf(key, sizeof(key), "f%02d", i);
        j[key] = i;
    }
    return j;
}
}  // namespace

TEST_CASE("enforce_all_consumed passes when nothing was sent") {
    Config cfg(json::object());
    cfg.enforce_all_consumed();  // must not throw
    CHECK(true);
}

TEST_CASE("enforce_all_consumed rejects fields the module never read") {
    Config cfg(json{{"a", 1}, {"typo", 2}});
    try {
        cfg.enforce_all_consumed();
        FAIL("expected unconsumed fields to throw");
    } catch (const std::runtime_error& e) {
        const std::string msg = e.what();
        CHECK(msg.find("unexpected field") != std::string::npos);
        CHECK(msg.find("typo") != std::string::npos);
    }
}

TEST_CASE("a null config behaves as empty") {
    Config cfg(json(nullptr));
    cfg.enforce_all_consumed();  // nothing sent, nothing read: fine
}

TEST_CASE("a non-object config is rejected") {
    CHECK_THROWS_AS(Config(json(42)), std::runtime_error);
    CHECK_THROWS_AS(Config(json::array({1, 2})), std::runtime_error);
}

TEST_CASE("parse deserializes a typed config struct") {
    Config cfg(json{{"value", 5}, {"name", "lidar"}});
    RangedCfg c = cfg.parse<RangedCfg>();
    CHECK(c.value == 5);
    CHECK(c.name == "lidar");
}

TEST_CASE("parse rejects a missing field and names it") {
    Config cfg(json{{"value", 5}});
    try {
        cfg.parse<RangedCfg>();
        FAIL("expected a missing field to throw");
    } catch (const std::runtime_error& e) {
        const std::string msg = e.what();
        CHECK(msg.find("missing required field") != std::string::npos);
        CHECK(msg.find("name") != std::string::npos);
    }
}

TEST_CASE("parse rejects an unknown field (one-to-one)") {
    Config cfg(json{{"value", 5}, {"name", "x"}, {"extra", true}});
    CHECK_THROWS_AS(cfg.parse<RangedCfg>(), std::runtime_error);
}

TEST_CASE("parse rejects a wrong-typed field and names it") {
    Config cfg(json{{"value", "not_a_number"}, {"name", "x"}});
    try {
        cfg.parse<RangedCfg>();
        FAIL("expected a type mismatch to throw");
    } catch (const std::runtime_error& e) {
        const std::string msg = e.what();
        CHECK(msg.find("field 'value'") != std::string::npos);
    }
}

TEST_CASE("parse runs the config's validate()") {
    Config cfg(json{{"value", 999}, {"name", "x"}});
    CHECK_THROWS_AS(cfg.parse<RangedCfg>(), std::runtime_error);
}

TEST_CASE("parse handles a struct past the old 64-field macro limit") {
    Config cfg(wide_json());
    WideCfg c = cfg.parse<WideCfg>();
    CHECK(c.f00 == 0);
    CHECK(c.f42 == 42);
    CHECK(c.f69 == 69);
}
