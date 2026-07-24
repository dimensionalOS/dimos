// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Structured logging for dimos C++ native modules. Mirrors the Rust SDK: one
// JSON object per line on stderr, in the shape the Python NativeModule wrapper
// parses (`level`, `message`, plus arbitrary structured fields). stdout is
// reserved, so logs always go to stderr.

#pragma once

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <initializer_list>
#include <iostream>
#include <locale>
#include <mutex>
#include <sstream>
#include <string>

namespace dimos::native::log {

enum class Level { Trace, Debug, Info, Warn, Error };

inline const char* level_str(Level level) {
    switch (level) {
        case Level::Trace: return "trace";
        case Level::Debug: return "debug";
        case Level::Info: return "info";
        case Level::Warn: return "warn";
        case Level::Error: return "error";
    }
    return "info";
}

/// A single structured key/value pair. The value is JSON-encoded eagerly so
/// callers can pass strings, integers, floats, or bools uniformly.
class Field {
public:
    Field(std::string key, const char* value) : key_(std::move(key)) {
        value_ = quote(value);
    }
    Field(std::string key, const std::string& value) : key_(std::move(key)) {
        value_ = quote(value);
    }
    Field(std::string key, bool value) : key_(std::move(key)) {
        value_ = value ? "true" : "false";
    }
    Field(std::string key, std::int64_t value)
        : key_(std::move(key)), value_(std::to_string(value)) {}
    Field(std::string key, double value) : key_(std::move(key)) {
        // JSON has no inf/nan literals, so a non-finite value would produce a
        // line the Python log reader cannot parse. Emit null instead.
        if (!std::isfinite(value)) {
            value_ = "null";
            return;
        }
        std::ostringstream ss;
        ss.imbue(std::locale::classic());
        ss << value;
        value_ = ss.str();
    }

    const std::string& key() const { return key_; }
    const std::string& json_value() const { return value_; }

    static std::string quote(const std::string& s) {
        std::string out;
        out.reserve(s.size() + 2);
        out.push_back('"');
        for (char c : s) {
            switch (c) {
                case '"': out += "\\\""; break;
                case '\\': out += "\\\\"; break;
                case '\n': out += "\\n"; break;
                case '\r': out += "\\r"; break;
                case '\t': out += "\\t"; break;
                case '\b': out += "\\b"; break;
                case '\f': out += "\\f"; break;
                default:
                    if (static_cast<unsigned char>(c) < 0x20) {
                        char buf[7];
                        std::snprintf(buf, sizeof(buf), "\\u%04x",
                                      static_cast<unsigned int>(static_cast<unsigned char>(c)));
                        out += buf;
                    } else {
                        out.push_back(c);
                    }
            }
        }
        out.push_back('"');
        return out;
    }

private:
    std::string key_;
    std::string value_;
};

/// Render one JSON log line (no trailing newline). Exposed for testing.
inline std::string format_line(Level level, const std::string& message,
                               std::initializer_list<Field> fields) {
    std::string out = "{\"level\":\"";
    out += level_str(level);
    out += "\",\"message\":";
    out += Field::quote(message);
    for (const Field& f : fields) {
        out += ",";
        out += Field::quote(f.key());
        out += ":";
        out += f.json_value();
    }
    out += "}";
    return out;
}

inline std::mutex& stderr_mutex() {
    static std::mutex m;
    return m;
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

/// Returns true (recording the current time) only on the first call or once at
/// least `interval_ns` has elapsed since the last true. One thread wins per
/// window via compare_exchange, matching the Rust `check_and_record`.
inline bool check_and_record(std::atomic<std::uint64_t>& last_ns, std::uint64_t interval_ns) {
    std::uint64_t now = monotonic_ns();
    std::uint64_t last = last_ns.load(std::memory_order_relaxed);
    if (last != 0 && now - last < interval_ns) {
        return false;
    }
    return last_ns.compare_exchange_strong(last, now, std::memory_order_relaxed);
}

}  // namespace dimos::native::log

// Throttled log at a single call site: emits at most once per `interval_ns`.
// The per-site state lives in a function-local static, so each expansion of
// this macro throttles independently, exactly like the Rust throttling macros.
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
