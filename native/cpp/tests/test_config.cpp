// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

#include <doctest/doctest.h>

#include <nlohmann/json.hpp>

#include <cstdint>
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
DIMOS_NATIVE_CONFIG(RangedCfg, value, name)

struct ListCfg {
    std::int64_t value;
    std::string name;
};
#define LIST_CFG_FIELDS(F) \
    F(value)               \
    F(name)
DIMOS_NATIVE_CONFIG_LIST(ListCfg, LIST_CFG_FIELDS)
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

TEST_CASE("parse rejects a missing field") {
    Config cfg(json{{"value", 5}});
    CHECK_THROWS_AS(cfg.parse<RangedCfg>(), std::runtime_error);
}

TEST_CASE("parse rejects an unknown field (one-to-one)") {
    Config cfg(json{{"value", 5}, {"name", "x"}, {"extra", true}});
    CHECK_THROWS_AS(cfg.parse<RangedCfg>(), std::runtime_error);
}

TEST_CASE("parse runs the config's validate()") {
    Config cfg(json{{"value", 999}, {"name", "x"}});
    CHECK_THROWS_AS(cfg.parse<RangedCfg>(), std::runtime_error);
}

TEST_CASE("a list-form config parses like the plain form") {
    Config cfg(json{{"value", 5}, {"name", "lidar"}});
    ListCfg c = cfg.parse<ListCfg>();
    CHECK(c.value == 5);
    CHECK(c.name == "lidar");
}

TEST_CASE("a list-form config rejects a missing field") {
    Config cfg(json{{"value", 5}});
    CHECK_THROWS_AS(cfg.parse<ListCfg>(), std::runtime_error);
}

TEST_CASE("a list-form config rejects an unknown field (one-to-one)") {
    Config cfg(json{{"value", 5}, {"name", "x"}, {"extra", true}});
    CHECK_THROWS_AS(cfg.parse<ListCfg>(), std::runtime_error);
}
