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

namespace pointlio {

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
