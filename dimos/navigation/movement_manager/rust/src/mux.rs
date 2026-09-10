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

//! The mux decision, free of transport and of the clock.

use std::time::{Duration, Instant};

use dimos_module::native_config;
use lcm_msgs::geometry_msgs::{Twist, Vector3};

#[native_config]
pub struct Config {
    /// How long a teleop command keeps nav preempted.
    #[validate(range(min = 0.0))]
    pub tele_cooldown_sec: f64,
    /// Per-axis multipliers applied to teleop twists only. Nav is forwarded raw.
    pub tele_scale_linear: [f64; 3],
    pub tele_scale_angular: [f64; 3],
    /// The deadman: `nav_cmd_vel` unheard for this long zeros `cmd_vel`.
    #[validate(range(exclusive_min = 0.0))]
    pub nav_stale_s: f64,
}

impl Config {
    pub fn cooldown(&self) -> Duration {
        Duration::from_secs_f64(self.tele_cooldown_sec.max(0.0))
    }

    pub fn nav_stale(&self) -> Duration {
        Duration::from_secs_f64(self.nav_stale_s.max(0.0))
    }
}

/// Arrival times only. Both are `Instant`s taken when the message reached this
/// process: the producers' clocks are not ours, and what these decide is how
/// long since a producer was last heard from.
#[derive(Debug, Default)]
pub struct MuxState {
    last_teleop: Option<Instant>,
    last_nav: Option<Instant>,
}

impl MuxState {
    /// True while teleop owns `cmd_vel`.
    pub fn teleop_holds(&self, now: Instant, cooldown: Duration) -> bool {
        self.last_teleop
            .is_some_and(|t| now.saturating_duration_since(t) < cooldown)
    }

    pub fn on_teleop(&mut self, now: Instant) {
        self.last_teleop = Some(now);
    }

    /// Record a nav arrival and say whether it reaches `cmd_vel`. The arrival is
    /// recorded even when dropped — the watchdog guards the producer's liveness,
    /// not what got forwarded.
    pub fn on_nav(&mut self, now: Instant, cooldown: Duration) -> bool {
        self.last_nav = Some(now);
        !self.teleop_holds(now, cooldown)
    }

    /// True when the watchdog should publish a zero twist this tick: nav has
    /// been heard from once, has since gone quiet, and teleop is not driving.
    pub fn nav_is_stale(&self, now: Instant, cooldown: Duration, stale: Duration) -> bool {
        self.last_nav
            .is_some_and(|t| now.saturating_duration_since(t) > stale)
            && !self.teleop_holds(now, cooldown)
    }
}

/// Teleop twist times the per-axis config scaling.
pub fn scale_twist(msg: &Twist, linear: &[f64; 3], angular: &[f64; 3]) -> Twist {
    Twist {
        linear: scale_vec3(&msg.linear, linear),
        angular: scale_vec3(&msg.angular, angular),
    }
}

fn scale_vec3(v: &Vector3, s: &[f64; 3]) -> Vector3 {
    Vector3 {
        x: v.x * s[0],
        y: v.y * s[1],
        z: v.z * s[2],
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const COOLDOWN: Duration = Duration::from_secs(1);
    const STALE: Duration = Duration::from_millis(500);

    fn twist(lx: f64, az: f64) -> Twist {
        Twist {
            linear: Vector3 {
                x: lx,
                y: 0.0,
                z: 0.0,
            },
            angular: Vector3 {
                x: 0.0,
                y: 0.0,
                z: az,
            },
        }
    }

    #[test]
    fn teleop_preempts_nav_until_the_cooldown_expires() {
        let t0 = Instant::now();
        let mut state = MuxState::default();

        assert!(state.on_nav(t0, COOLDOWN), "nav flows before any teleop");

        state.on_teleop(t0 + Duration::from_millis(100));
        assert!(!state.on_nav(t0 + Duration::from_millis(200), COOLDOWN));
        assert!(!state.on_nav(t0 + Duration::from_millis(1099), COOLDOWN));
        assert!(state.on_nav(t0 + Duration::from_millis(1101), COOLDOWN));
    }

    #[test]
    fn a_second_teleop_restarts_the_cooldown() {
        let t0 = Instant::now();
        let mut state = MuxState::default();
        state.on_teleop(t0);
        state.on_teleop(t0 + Duration::from_millis(900));
        assert!(!state.on_nav(t0 + Duration::from_millis(1500), COOLDOWN));
        assert!(state.on_nav(t0 + Duration::from_millis(2000), COOLDOWN));
    }

    #[test]
    fn zero_cooldown_never_preempts() {
        let t0 = Instant::now();
        let mut state = MuxState::default();
        state.on_teleop(t0);
        assert!(state.on_nav(t0, Duration::ZERO));
    }

    #[test]
    fn watchdog_is_disarmed_until_the_first_nav() {
        let t0 = Instant::now();
        let state = MuxState::default();
        assert!(!state.nav_is_stale(t0 + Duration::from_secs(10), COOLDOWN, STALE));
    }

    #[test]
    fn watchdog_fires_once_nav_goes_quiet_and_keeps_firing() {
        let t0 = Instant::now();
        let mut state = MuxState::default();
        state.on_nav(t0, COOLDOWN);

        assert!(!state.nav_is_stale(t0 + Duration::from_millis(400), COOLDOWN, STALE));
        // Continuous, not edge-triggered: a single zero can be lost, a stream of
        // zeros is what a deadman is.
        assert!(state.nav_is_stale(t0 + Duration::from_millis(600), COOLDOWN, STALE));
        assert!(state.nav_is_stale(t0 + Duration::from_millis(5000), COOLDOWN, STALE));
    }

    #[test]
    fn watchdog_is_suppressed_while_teleop_drives() {
        let t0 = Instant::now();
        let mut state = MuxState::default();
        state.on_nav(t0, COOLDOWN);
        state.on_teleop(t0 + Duration::from_millis(600));

        // Nav is stale, but teleop owns cmd_vel, so no zeros over its commands.
        assert!(!state.nav_is_stale(t0 + Duration::from_millis(1000), COOLDOWN, STALE));
        // Once the teleop cooldown lapses the deadman takes over again.
        assert!(state.nav_is_stale(t0 + Duration::from_millis(1700), COOLDOWN, STALE));
    }

    #[test]
    fn a_dropped_nav_still_counts_as_an_arrival() {
        let t0 = Instant::now();
        let mut state = MuxState::default();
        state.on_teleop(t0);
        assert!(!state.on_nav(t0 + Duration::from_millis(100), COOLDOWN));
        assert!(!state.on_nav(t0 + Duration::from_millis(900), COOLDOWN));
        // Nav kept arriving through the cooldown, so the deadman does not trip
        // the instant teleop releases, even though none of it was forwarded.
        assert!(!state.nav_is_stale(t0 + Duration::from_millis(1200), COOLDOWN, STALE));
    }

    #[test]
    fn scaling_multiplies_each_axis_independently() {
        let scaled = scale_twist(
            &Twist {
                linear: Vector3 {
                    x: 1.0,
                    y: 1.0,
                    z: 1.0,
                },
                angular: Vector3 {
                    x: 1.0,
                    y: 1.0,
                    z: 1.0,
                },
            },
            &[0.5, 2.0, 0.0],
            &[1.0, 1.0, 0.25],
        );
        assert_eq!(scaled.linear.x, 0.5);
        assert_eq!(scaled.linear.y, 2.0);
        assert_eq!(scaled.linear.z, 0.0);
        assert_eq!(scaled.angular.z, 0.25);
    }

    #[test]
    fn identity_scaling_is_a_passthrough() {
        let msg = twist(0.3, -0.2);
        assert_eq!(scale_twist(&msg, &[1.0; 3], &[1.0; 3]), msg);
    }
}
