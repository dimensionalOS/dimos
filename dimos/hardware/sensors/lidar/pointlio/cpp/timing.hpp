// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Lightweight per-section timing for `run_main_iter`. Active only when the
// global `fastlio_debug` flag is set, so non-debug runs pay one branch per
// scope.
//
// Usage:
//   static timing::Section sec{"filter_cloud"};
//   { timing::Scope s(sec); /* work */ }
//   timing::maybe_flush(now);  // periodically

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <mutex>
#include <vector>

#include "fast_lio_debug.hpp"

namespace timing {

struct Section {
    const char* name;
    std::atomic<uint64_t> count{0};
    std::atomic<uint64_t> total_ns{0};
    std::atomic<uint64_t> max_ns{0};

    explicit Section(const char* n);

    void add(uint64_t ns) {
        count.fetch_add(1, std::memory_order_relaxed);
        total_ns.fetch_add(ns, std::memory_order_relaxed);
        uint64_t prev = max_ns.load(std::memory_order_relaxed);
        while (ns > prev &&
               !max_ns.compare_exchange_weak(prev, ns, std::memory_order_relaxed)) {
        }
    }
};

inline std::vector<Section*>& registry() {
    static std::vector<Section*> r;
    return r;
}

inline Section::Section(const char* n) : name(n) {
    registry().push_back(this);
}

struct Scope {
    Section& sec;
    std::chrono::steady_clock::time_point t0;
    bool active;

    explicit Scope(Section& s) : sec(s), active(fastlio_debug) {
        if (active) {
            t0 = std::chrono::steady_clock::now();
        }
    }

    ~Scope() {
        if (!active) {
            return;
        }
        auto dt = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - t0).count();
        sec.add(static_cast<uint64_t>(dt));
    }
};

// Print one line per section to stderr every FLUSH_INTERVAL, then reset.
// Mutex serialises flushes across threads (SDK callbacks vs main loop).
inline void maybe_flush(std::chrono::steady_clock::time_point now) {
    if (!fastlio_debug) {
        return;
    }
    constexpr auto FLUSH_INTERVAL = std::chrono::seconds(1);
    static std::mutex mtx;
    static std::chrono::steady_clock::time_point last;
    std::lock_guard<std::mutex> lock(mtx);
    if (last.time_since_epoch().count() == 0) {
        last = now;
        return;
    }
    if (now - last < FLUSH_INTERVAL) {
        return;
    }
    auto dt_ms = std::chrono::duration<double, std::milli>(now - last).count();
    last = now;

    for (Section* s : registry()) {
        uint64_t c = s->count.exchange(0, std::memory_order_relaxed);
        uint64_t tot = s->total_ns.exchange(0, std::memory_order_relaxed);
        uint64_t mx = s->max_ns.exchange(0, std::memory_order_relaxed);
        if (c == 0) {
            std::fprintf(stderr, "[timing] %-24s n=0\n", s->name);
            continue;
        }
        double mean_us = static_cast<double>(tot) / static_cast<double>(c) / 1e3;
        double max_us = static_cast<double>(mx) / 1e3;
        double total_ms = static_cast<double>(tot) / 1e6;
        double rate_hz = static_cast<double>(c) * 1000.0 / dt_ms;
        std::fprintf(stderr,
                     "[timing] %-24s n=%5lu rate=%7.1fHz mean=%8.3fus max=%9.3fus tot=%7.2fms\n",
                     s->name,
                     static_cast<unsigned long>(c),
                     rate_hz, mean_us, max_us, total_ms);
    }
}

}  // namespace timing
