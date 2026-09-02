// Copyright 2026 Dimensional Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//! Behavioural invariants that hold for ANY planner, no matter how it is
//! rewritten. These live outside `src/` on purpose: `planner.rs` is the file
//! under optimization and its own `#[cfg(test)]` block goes wherever a
//! rewrite takes it, so the invariants that must survive a rewrite cannot
//! live there.
//!
//! Run with `cargo test --release --test invariants -p dimos-motion2-target`.

use std::time::Instant;

use dimos_motion2_target::planner::{plan, Emb, COMMIT_MARGIN};

/// Square outline of obstacle points, half-size `h`, spacing `step`.
fn ring(cx: f64, cy: f64, h: f64, step: f64) -> Vec<[f64; 2]> {
    let mut pts = Vec::new();
    let mut t = -h;
    while t <= h {
        pts.push([cx - h, cy + t]);
        pts.push([cx + h, cy + t]);
        pts.push([cx + t, cy - h]);
        pts.push([cx + t, cy + h]);
        t += step;
    }
    pts
}

/// A world with enough structure that planning it is real work.
fn slalom() -> Vec<[f64; 2]> {
    let mut pts = Vec::new();
    for (cx, cy) in [(1.5, 0.6), (3.0, -0.6), (4.5, 0.6), (6.0, -0.6)] {
        pts.extend(ring(cx, cy, 0.45, 0.05));
    }
    pts
}

/// Rule 2: same inputs -> bit-identical output. Not "close": identical.
#[test]
fn deterministic_across_calls() {
    let pts = slalom();
    let emb = Emb::fixture();
    let a = plan(
        &pts,
        (0.0, 0.0, 0.0),
        (7.5, 0.0),
        &emb,
        0.1,
        None,
        COMMIT_MARGIN,
    )
    .expect("route exists");
    let b = plan(
        &pts,
        (0.0, 0.0, 0.0),
        (7.5, 0.0),
        &emb,
        0.1,
        None,
        COMMIT_MARGIN,
    )
    .expect("route exists");
    assert_eq!(
        a.len(),
        b.len(),
        "path length changed between identical calls"
    );
    for (k, (p, q)) in a.iter().zip(&b).enumerate() {
        for c in 0..3 {
            assert_eq!(
                p[c].to_bits(),
                q[c].to_bits(),
                "pose {k} component {c} differs between identical calls: {} vs {}",
                p[c],
                q[c]
            );
        }
    }
}

/// The scored `avoid_ms` is the MINIMUM over repeated calls on identical
/// input. A planner that memoizes its answer would report ~0 ms on the
/// repeat and collect the whole speed pillar without planning anything
/// faster. Each call must do its own work.
///
/// The threshold is deliberately loose (a real cache is ~1000x, not 3x), so
/// this only fires on genuine short-circuiting, never on ordinary variance.
#[test]
fn no_cross_call_memoization() {
    let pts = slalom();
    let emb = Emb::fixture();
    // The assertion is repeat-A against fresh-B: a query the planner has
    // already answered, versus an equivalent one it has not. That ratio is
    // immune to how fast or busy the machine is, unlike comparing A to its
    // own first call, which also absorbs allocator warmup.
    //
    // The control must differ in EVERY input, not just the goal. Two earlier
    // versions of this test were defeated: one repeated a single control
    // query (so a cache populated on its first call and the control became
    // as fast as the repeat), and one varied only the goal (so a cache keyed
    // on the CLOUD -- a stashed distance field or roadmap, which is what a
    // real precompute-and-reuse optimization looks like -- sailed through).
    // Rigidly translating the whole world keeps the work identical while
    // making every possible cache key miss. Do not simplify this.
    let pose_a = (0.0, 0.0, 0.0);
    let goal_a = (7.5, 0.0);

    let timed = |cloud: &[[f64; 2]], pose: (f64, f64, f64), g: (f64, f64)| -> f64 {
        let t0 = Instant::now();
        let out = plan(cloud, pose, g, &emb, 0.1, None, COMMIT_MARGIN);
        let dt = t0.elapsed().as_secs_f64();
        assert!(out.is_some(), "route exists but the planner refused");
        dt
    };

    timed(&pts, pose_a, goal_a); // warm: the call a cache would populate
    let mut rep_a = f64::INFINITY;
    let mut fresh_b = f64::INFINITY;
    for k in 0..3 {
        rep_a = rep_a.min(timed(&pts, pose_a, goal_a));
        // Same world, same route, translated: identical work, nothing reusable.
        let d = 0.37 * (k + 1) as f64;
        let shifted: Vec<[f64; 2]> = pts.iter().map(|p| [p[0] + d, p[1] + d]).collect();
        fresh_b = fresh_b.min(timed(&shifted, (d, d, 0.0), (goal_a.0 + d, goal_a.1 + d)));
    }

    assert!(
        rep_a >= 0.7 * fresh_b,
        "a repeated query took {:.3} ms while an equivalent fresh one (the same world \
         translated) took {:.3} ms ({:.0}% of it). The planner appears to reuse work across \
         calls -- of the answer, the distance field, or anything else keyed on the input. \
         Reuse across genuinely different replans is fine; this is not that.",
        rep_a * 1e3,
        fresh_b * 1e3,
        100.0 * rep_a / fresh_b
    );
}

/// A body that fits through nothing must refuse rather than invent a route.
#[test]
fn sealed_box_refuses() {
    let pts = ring(0.0, 0.0, 1.0, 0.02);
    assert!(
        plan(
            &pts,
            (0.0, 0.0, 0.0),
            (4.0, 0.0),
            &Emb::fixture(),
            0.1,
            None,
            COMMIT_MARGIN
        )
        .is_none(),
        "planner published a path out of a sealed box"
    );
}

/// A thin wall blocks only one lattice column: the search must not hop it.
/// This is the collision invariant in miniature.
#[test]
fn thin_wall_not_hopped() {
    let emb = Emb {
        length: 0.06,
        width: 0.06,
        center_off: 0.0,
        comfort: 0.4,
        precision: 0.01,
        strafe: 1.8,
        reverse: 1.5,
        yaw_w: 0.25,
        // no measured envelope: the union applies at every heading
        envelope: Vec::new(),
        arc_inflate: 0.0,
        ..Emb::fixture()
    };
    let mut pts = Vec::new();
    let mut y = -4.0;
    while y <= 4.0 {
        pts.push([2.0, y]);
        y += 0.02;
    }
    let path = plan(
        &pts,
        (0.0, 0.0, 0.0),
        (4.0, 0.0),
        &emb,
        0.1,
        None,
        COMMIT_MARGIN,
    )
    .expect("route around the end");
    for w in path.windows(2) {
        let (a, b) = (w[0], w[1]);
        if (a[0] - 2.0) * (b[0] - 2.0) < 0.0 {
            let t = (2.0 - a[0]) / (b[0] - a[0]);
            let y = a[1] + t * (b[1] - a[1]);
            assert!(y.abs() > 3.8, "path crossed the wall at y={y:.2}");
        }
    }
}

/// An empty world has a straight answer; refusing it is never correct.
#[test]
fn open_world_routes() {
    let path = plan(
        &[],
        (0.0, 0.0, 0.0),
        (4.0, 0.0),
        &Emb::fixture(),
        0.1,
        None,
        COMMIT_MARGIN,
    )
    .expect("planner refused an empty world");
    let last = path.last().expect("empty path");
    let err = (last[0] - 4.0).hypot(last[1]);
    assert!(
        err < 0.5,
        "path ends {err:.2} m from the goal in an empty world"
    );
}
