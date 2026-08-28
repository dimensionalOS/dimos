// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Structured logging for dimos C++ native modules. One JSON object per line on
// stderr, in the shape the Python NativeModule wrapper parses. stdout is
// reserved, so logs go to stderr.

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <initializer_list>
#include <iostream>
#include <mutex>
#include <string>
#include <utility>

#include <nlohmann/json.hpp>

namespace dimos::native::log {

enum class Level { Info, Warn, Error };

inline const char* level_str(Level level) {
    switch (level) {
        case Level::Info: return "info";
        case Level::Warn: return "warn";
        case Level::Error: return "error";
    }
    return "info";
}

/// A structured key/value pair. A non-finite double serializes to null, since
/// JSON has no inf/nan literals.
class Field {
public:
    Field(std::string key, const char* value) : key_(std::move(key)), value_(value) {}
    Field(std::string key, std::string value)
        : key_(std::move(key)), value_(std::move(value)) {}
    Field(std::string key, bool value) : key_(std::move(key)), value_(value) {}
    Field(std::string key, std::int64_t value) : key_(std::move(key)), value_(value) {}
    Field(std::string key, double value) : key_(std::move(key)), value_(value) {}

    const std::string& key() const { return key_; }
    const nlohmann::ordered_json& value() const { return value_; }

private:
    std::string key_;
    nlohmann::ordered_json value_;
};

/// Render one JSON log line (no trailing newline). Exposed for testing.
inline std::string format_line(Level level, const std::string& message,
                               std::initializer_list<Field> fields) {
    // ordered_json, so fields keep the order the caller wrote them in.
    nlohmann::ordered_json line{{"level", level_str(level)}, {"message", message}};
    for (const Field& f : fields) {
        line[f.key()] = f.value();
    }
    return line.dump();
}

inline std::mutex& stderr_mutex() {
    static std::mutex mutex;
    return mutex;
}

inline void emit(Level level, const std::string& message,
                 std::initializer_list<Field> fields = {}) {
    std::string line = format_line(level, message, fields);
    std::lock_guard<std::mutex> lock(stderr_mutex());
    std::cerr << line << '\n';
    std::cerr.flush();
}

inline void info(const std::string& message, std::initializer_list<Field> fields = {}) {
    emit(Level::Info, message, fields);
}
inline void warn(const std::string& message, std::initializer_list<Field> fields = {}) {
    emit(Level::Warn, message, fields);
}
inline void error(const std::string& message, std::initializer_list<Field> fields = {}) {
    emit(Level::Error, message, fields);
}

/// Nanoseconds on a monotonic clock. Never 0 in practice, so 0 is the "never
/// logged" sentinel for the throttle state below.
inline std::uint64_t monotonic_ns() {
    return static_cast<std::uint64_t>(
        std::chrono::steady_clock::now().time_since_epoch().count());
}

inline constexpr std::uint64_t from_secs(std::uint64_t secs) {
    return secs * 1'000'000'000ull;
}

/// True at most once per `interval_ns`, recording the time. One thread wins
/// each window.
inline bool check_and_record(std::atomic<std::uint64_t>& last_ns, std::uint64_t interval_ns) {
    std::uint64_t now = monotonic_ns();
    std::uint64_t last = last_ns.load(std::memory_order_relaxed);
    if (last != 0 && now - last < interval_ns) {
        return false;
    }
    return last_ns.compare_exchange_strong(last, now, std::memory_order_relaxed);
}

}  // namespace dimos::native::log

// Emits at most once per interval_ns. Each expansion throttles independently.
#define DIMOS_LOG_THROTTLED(level, interval_ns, message, ...)                       \
    do {                                                                            \
        static std::atomic<std::uint64_t> _dimos_throttle_last_ns{0};               \
        if (::dimos::native::log::check_and_record(_dimos_throttle_last_ns,         \
                                                   (interval_ns))) {                \
            ::dimos::native::log::emit((level), (message), {__VA_ARGS__});          \
        }                                                                           \
    } while (0)

#define DIMOS_ERROR_THROTTLED(interval_ns, message, ...) \
    DIMOS_LOG_THROTTLED(::dimos::native::log::Level::Error, (interval_ns), (message), ##__VA_ARGS__)
