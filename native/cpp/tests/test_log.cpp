// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

#include <doctest/doctest.h>

#include <atomic>
#include <limits>
#include <string>

#include "dimos/native/log.hpp"

using namespace dimos::native;

TEST_CASE("format_line emits level and message as JSON") {
    std::string line = log::format_line(log::Level::Info, "hello", {});
    CHECK(line == R"({"level":"info","message":"hello"})");
}

TEST_CASE("level strings match what the Python wrapper maps") {
    CHECK(std::string(log::level_str(log::Level::Trace)) == "trace");
    CHECK(std::string(log::level_str(log::Level::Debug)) == "debug");
    CHECK(std::string(log::level_str(log::Level::Info)) == "info");
    CHECK(std::string(log::level_str(log::Level::Warn)) == "warn");
    CHECK(std::string(log::level_str(log::Level::Error)) == "error");
}

TEST_CASE("structured fields render with correct JSON types") {
    std::string line = log::format_line(
        log::Level::Warn, "dropped",
        {log::Field("topic", "/data"), log::Field("count", std::int64_t{7}),
         log::Field("ratio", 0.5), log::Field("full", true)});
    CHECK(line ==
          R"({"level":"warn","message":"dropped","topic":"/data","count":7,"ratio":0.5,"full":true})");
}

TEST_CASE("non-finite double fields render as null, not invalid JSON") {
    const double inf = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK(log::format_line(log::Level::Info, "m", {log::Field("x", inf)}) ==
          R"({"level":"info","message":"m","x":null})");
    CHECK(log::format_line(log::Level::Info, "m", {log::Field("y", nan)}) ==
          R"({"level":"info","message":"m","y":null})");
}

TEST_CASE("message and string fields are escaped") {
    std::string line = log::format_line(log::Level::Error, "bad \"quote\"\nline", {});
    CHECK(line == R"({"level":"error","message":"bad \"quote\"\nline"})");
    // No raw control characters leak into the line.
    CHECK(line.find('\n') == std::string::npos);
}

TEST_CASE("check_and_record fires once then throttles") {
    std::atomic<std::uint64_t> last{0};
    // First call always fires and records the time.
    CHECK(log::check_and_record(last, log::from_secs(3600)));
    CHECK(last.load() != 0);
    // Immediately again, well within the interval: throttled.
    CHECK_FALSE(log::check_and_record(last, log::from_secs(3600)));
    // A zero interval always allows.
    CHECK(log::check_and_record(last, 0));
}
