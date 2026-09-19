// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// When a Point-LIO output may be published, and what time it carries.
//
// A timestamp answers "when was the world like this", not "when did this
// message leave". Point-LIO's own answer is `lidar_end_time`, the end of the
// scan the state was computed from, which the SDK already writes into
// `odomAftMapped.header.stamp` (laserMapping.hpp:458-467) and hands back
// through `get_odometry()`.
//
// The module used to overwrite that with `system_clock::now()`. While the
// estimator keeps up the two are within a frame of each other and nothing
// shows. When it falls behind -- a spin, a big map, a busy Orin -- the queue
// inside Point-LIO grows and the state being published is seconds old, while
// the stamp still says "now". On 20260916-085832-r1pro-kronknav that reached
// **22 seconds**: every camera frame paired with `tf` at its own timestamp got
// posed with where the robot had been 22 s earlier, and no lidar-side check
// could see it, because the cloud is deskewed into the same wrong estimate and
// the error cancels.
//
// Two rules fall out, and both are needed:
//
//   1. Stamp with the state's time, never the publish time.
//   2. Never publish a state twice. Re-emitting the same pose under a later
//      stamp is the same lie in a different shape -- it tells a consumer the
//      robot was still there when nothing new has been measured.
//
// Rule 2 means a stalled estimator goes quiet instead of inventing a present.
// That is deliberate: a gap is a fact a consumer can act on, a fresh stamp on
// a stale pose is not.

#ifndef DIMOS_POINTLIO_PUBLISH_STAMP_HPP
#define DIMOS_POINTLIO_PUBLISH_STAMP_HPP

#include <limits>

namespace pointlio {

/// The offset that carries the Livox's own clock onto the host's.
///
/// Point-LIO's `lidar_end_time` is in the **sensor's** time base, because that
/// is what the Livox packets carry: on this Mid-360 it reads 162,771 s, which
/// is 45 hours of lidar uptime, not an epoch. Publishing it raw is worse than
/// the publish-time stamp it replaces -- every consumer that looks a pose up by
/// timestamp would find nothing at all.
///
/// So the sensor's time has to be carried onto the host's, and the only handle
/// on that is when packets arrive: `host_arrival - device_stamp` is the offset
/// plus a transport and scheduling delay that is always positive and sometimes
/// enormous. The **minimum** of that difference is therefore the estimate --
/// the least-delayed packet is the closest look at the true offset, and a
/// backlog inside this process cannot bias it downwards.
///
/// A plain running minimum would be captured by the single luckiest packet and
/// then never follow the clocks drifting apart. Two buckets fix that: the
/// offset in use is the minimum over the previous window, while a candidate
/// accumulates the minimum over the current one, and they swap when the window
/// ends. The estimate is then never older than two windows.
class HostClockOffset {
public:
    explicit HostClockOffset(double window_s = 30.0) : window_s_(window_s) {}

    /// One packet: `device_s` is its own stamp, `host_s` the wall clock now.
    void observe(double device_s, double host_s) {
        if (!(device_s > 0.0)) return;
        const double difference = host_s - device_s;
        if (difference < candidate_) candidate_ = difference;
        if (difference < offset_) offset_ = difference;
        if (window_started_ <= 0.0) {
            window_started_ = host_s;
        } else if (host_s - window_started_ >= window_s_) {
            offset_ = candidate_;
            candidate_ = std::numeric_limits<double>::infinity();
            window_started_ = host_s;
        }
    }

    bool ready() const { return offset_ < std::numeric_limits<double>::infinity(); }

    /// A sensor time on the host's clock, or 0 if no packet has been seen.
    double to_host(double device_s) const {
        if (!ready() || !(device_s > 0.0)) return 0.0;
        return device_s + offset_;
    }

    double offset() const { return ready() ? offset_ : 0.0; }

private:
    double window_s_;
    double offset_ = std::numeric_limits<double>::infinity();
    double candidate_ = std::numeric_limits<double>::infinity();
    double window_started_ = 0.0;
};

/// Whether an output port may publish now, and the stamp it must carry.
struct PublishDecision {
    bool publish;
    /// Seconds, the estimator's own time for this state. Only meaningful when
    /// `publish` is true.
    double stamp_s;
};

/// Decide one port's publish.
///
/// `state_s`         the estimator's time for the state it is holding
///                   (`get_odometry().header.stamp.toSec()`); <= 0 before the
///                   first scan has been processed.
/// `last_state_s`    the state time this port published last; 0 if never.
/// `since_publish_s` wall-clock seconds since this port last published.
/// `interval_s`      this port's output period.
inline PublishDecision publish_decision(double state_s, double last_state_s,
                                        double since_publish_s, double interval_s) {
    // No estimate yet. `has_estimate` on the pose catches most of this; a
    // zero stamp catches the rest, and a zero must never become wall time.
    if (!(state_s > 0.0)) return {false, 0.0};
    // Nothing new has been measured since the last publish on this port.
    if (!(state_s > last_state_s)) return {false, 0.0};
    // The port's rate limit, still on the wall clock: it governs how often we
    // are willing to send, which is a question about the link, not the world.
    if (since_publish_s < interval_s) return {false, 0.0};
    return {true, state_s};
}

/// How far the estimator's state trails the wall clock, in seconds.
///
/// Both arguments must already be on the host's clock -- see HostClockOffset.
///
/// This is the quantity that silently became a 22 s pose error. It is worth
/// logging: it is the difference between "Point-LIO is slow" (visible in the
/// output rate) and "Point-LIO is behind" (invisible until something compares
/// it against a camera).
inline double backlog_s(double wall_now_s, double state_s) {
    if (!(state_s > 0.0)) return 0.0;
    const double lag = wall_now_s - state_s;
    return lag > 0.0 ? lag : 0.0;
}

}  // namespace pointlio

#endif  // DIMOS_POINTLIO_PUBLISH_STAMP_HPP
