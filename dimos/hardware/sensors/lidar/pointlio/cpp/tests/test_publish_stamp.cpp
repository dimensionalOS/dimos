// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// The rules in publish_stamp.hpp, one test per rule, plus the regression that
// cost a week of depth scores: a backlogged estimator must not stamp its
// output with the wall clock.

#include "publish_stamp.hpp"

#include <cmath>
#include <cstdio>
#include <string>

namespace {

int failures = 0;

void check(bool ok, const std::string& what) {
    if (!ok) {
        std::printf("FAIL  %s\n", what.c_str());
        ++failures;
    } else {
        std::printf("ok    %s\n", what.c_str());
    }
}

// The wall clock at the moment of the publish, and the estimator's own time
// for the state it is holding. On the R1 these drifted 22 s apart.
constexpr double kWallNow = 1789521303.0;
constexpr double kStateNow = 1789521281.0;
constexpr double kInterval = 0.1;
// What the Mid-360 actually reported on 2026-09-16: 45 hours of its own
// uptime, against a host at epoch.
constexpr double kDeviceNow = 162771.318961911;

}  // namespace

int main() {
    using pointlio::backlog_s;
    using pointlio::HostClockOffset;
    using pointlio::publish_decision;

    {
        // The Livox reports 45 hours of its own uptime while the host is at
        // epoch. Stamping with the sensor's number raw is worse than the
        // publish time it replaces: nothing downstream would resolve at all.
        HostClockOffset clock;
        check(!clock.ready(), "no packet seen yet: not ready");
        check(clock.to_host(kDeviceNow) == 0.0, "and it refuses to answer");

        // Packets arrive with a delay that is always positive, so the minimum
        // of (host - device) is the offset. Give it a lucky one and two late.
        const double offset = kWallNow - kDeviceNow;
        clock.observe(kDeviceNow, kWallNow + 0.004);
        clock.observe(kDeviceNow + 0.1, kWallNow + 0.1);
        clock.observe(kDeviceNow + 0.2, kWallNow + 0.2 + 0.050);
        check(clock.ready(), "one packet is enough to be ready");
        // A microsecond, not a nanosecond: these are epoch seconds, and a
        // double carries about 2e-7 s of resolution at 1.8e9. Fine for a
        // timestamp, worth knowing before writing a tighter assertion.
        check(std::abs(clock.offset() - offset) < 1e-6, "the offset is the minimum delay");
        check(std::abs(clock.to_host(kDeviceNow + 1.0) - (kWallNow + 1.0)) < 1e-6,
              "a sensor time comes back on the host's clock");
    }
    {
        // A backlog inside this process delays every packet equally and must
        // not drag the offset: the minimum is taken over arrivals, not over
        // anything the estimator touches.
        HostClockOffset clock;
        const double offset = kWallNow - kDeviceNow;
        for (int i = 0; i < 50; ++i) {
            clock.observe(kDeviceNow + i * 0.1, kWallNow + i * 0.1 + 20.0);
        }
        clock.observe(kDeviceNow + 5.0, kWallNow + 5.0 + 0.001);
        check(std::abs(clock.offset() - (offset + 0.001)) < 1e-6,
              "fifty late packets do not move it; one prompt one does");
    }
    {
        // Two buckets, so the estimate follows the clocks drifting apart
        // instead of being captured by the luckiest packet ever seen.
        HostClockOffset clock(10.0);
        clock.observe(kDeviceNow, kWallNow);
        const double drifted = kWallNow - kDeviceNow + 0.5;
        for (int i = 1; i <= 30; ++i) {
            clock.observe(kDeviceNow + i, kDeviceNow + i + drifted);
        }
        check(clock.offset() > kWallNow - kDeviceNow + 0.4,
              "the window expires and the old minimum is let go");
    }
    {
        HostClockOffset clock;
        clock.observe(0.0, kWallNow);
        check(!clock.ready(), "a packet with no device stamp is not an observation");
    }

    {
        // A fresh state past the rate limit publishes, and carries the STATE's
        // time. This is the whole fix: the stamp is 22 s behind the wall clock
        // here, and that is correct -- the world really was like this 22 s ago.
        const auto d = publish_decision(kStateNow, 0.0, 1.0, kInterval);
        check(d.publish, "a fresh state publishes");
        check(d.stamp_s == kStateNow, "the stamp is the state's time");
        check(d.stamp_s != kWallNow, "the stamp is NOT the wall clock");
        check(kWallNow - d.stamp_s == 22.0, "a 22 s backlog survives as a 22 s old stamp");
    }
    {
        // The regression. The module used to publish on its own rate limit
        // whatever the estimator had, stamped `now`, so a stalled Point-LIO
        // produced a stream of identical poses marching forward in time -- a
        // robot that reads as parked here, then parked there, then parked
        // somewhere else, 22 s after it actually moved.
        const auto d = publish_decision(kStateNow, kStateNow, 10.0, kInterval);
        check(!d.publish, "the same state is never published twice");
    }
    {
        const auto d = publish_decision(kStateNow + 0.1, kStateNow, 0.01, kInterval);
        check(!d.publish, "the rate limit still holds for a new state");
    }
    {
        const auto d = publish_decision(kStateNow + 0.1, kStateNow, 1.0, kInterval);
        check(d.publish, "a new state past the rate limit publishes");
        check(d.stamp_s == kStateNow + 0.1, "and carries its own time");
    }
    {
        // Before the first scan is processed. A zero must not become wall time.
        check(!publish_decision(0.0, 0.0, 1.0, kInterval).publish, "no estimate yet: silent");
        check(!publish_decision(-1.0, 0.0, 1.0, kInterval).publish, "a negative time: silent");
    }
    {
        // Time can only run one way: a state older than one already sent is a
        // reordering, not news.
        check(!publish_decision(kStateNow - 1.0, kStateNow, 1.0, kInterval).publish,
              "an older state is not news");
    }
    {
        check(backlog_s(kWallNow, kStateNow) == 22.0, "the backlog is reported");
        check(backlog_s(kWallNow, 0.0) == 0.0, "no estimate means no backlog");
        check(backlog_s(kStateNow, kWallNow) == 0.0, "a state ahead of the clock clamps to zero");
    }

    std::printf(failures ? "\n%d FAILED\n" : "\nall passed\n", failures);
    return failures == 0 ? 0 : 1;
}
