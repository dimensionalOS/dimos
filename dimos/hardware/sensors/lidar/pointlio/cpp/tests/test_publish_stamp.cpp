// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// The rules in publish_stamp.hpp, one test per rule, plus the regression that
// cost a week of depth scores: a backlogged estimator must not stamp its
// output with the wall clock.

#include "publish_stamp.hpp"

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

}  // namespace

int main() {
    using pointlio::backlog_s;
    using pointlio::publish_decision;

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
