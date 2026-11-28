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
//! When a velocity reaches rl_deploy, as the vendor drdds-zenoh-bridge's m20 mode decides it: every command bounded
//! (each axis clamped, a non-finite one zeroed), forwarded only while rl_deploy's MOTION_INFO is fresh, in RL control
//! and HES_STATUS is clear (or not yet heard), zero otherwise, and a final zero once commands stop.
//!
//! Event driven: a command is written when it arrives, and the latest is written again on every MOTION_INFO, which
//! keeps rl_deploy's 150 ms REAL_STEER timeout fed for as long as rl_deploy itself is alive.

use std::time::{Duration, Instant};

/// A velocity as REAL_STEER carries it: m/s forward and left, rad/s counterclockwise.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Velocity {
    pub x: f32,
    pub y: f32,
    pub yaw: f32,
}

impl Velocity {
    pub const ZERO: Velocity = Velocity {
        x: 0.0,
        y: 0.0,
        yaw: 0.0,
    };
}

#[derive(Clone, Debug)]
pub struct Limits {
    pub max_linear_x: f64,
    pub max_linear_y: f64,
    pub max_angular_z: f64,
}

#[derive(Clone, Debug)]
pub struct GateConfig {
    pub limits: Limits,
    /// A command older than this is released (final zero).
    pub command_timeout: Duration,
    /// MOTION_INFO older than this means rl_deploy is not ready.
    pub state_timeout: Duration,
    /// rl_deploy's MOTION_INFO state that takes velocity commands (17, RL control).
    pub rl_control_state: i32,
}

/// What to write to REAL_STEER.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Write {
    Nothing,
    Velocity(Velocity),
    /// The last command stopped: zero, written twice, as the vendor bridge does.
    FinalZero,
}

pub fn bound(x: f64, y: f64, yaw: f64, limits: &Limits) -> Velocity {
    if !(x.is_finite() && y.is_finite() && yaw.is_finite()) {
        return Velocity::ZERO;
    }
    let clamp = |v: f64, max: f64| v.clamp(-max, max) as f32;
    Velocity {
        x: clamp(x, limits.max_linear_x),
        y: clamp(y, limits.max_linear_y),
        yaw: clamp(yaw, limits.max_angular_z),
    }
}

pub struct Gate {
    config: GateConfig,
    latest: Velocity,
    command_at: Option<Instant>,
    motion_state: Option<(i32, Instant)>,
    hes_status: Option<i32>,
    /// rl_deploy's REAL_STEER reader is matched.
    matched: bool,
    /// Another REAL_STEER writer (basic_server) is up: write nothing, so the two never interleave.
    blocked: bool,
}

impl Gate {
    pub fn new(config: GateConfig) -> Self {
        Self {
            config,
            latest: Velocity::ZERO,
            command_at: None,
            motion_state: None,
            hes_status: None,
            matched: false,
            blocked: false,
        }
    }

    pub fn ready(&self, now: Instant) -> bool {
        !self.blocked
            && self.matched
            && self.motion_state.is_some_and(|(state, at)| {
                state == self.config.rl_control_state
                    && now.saturating_duration_since(at) <= self.config.state_timeout
            })
            && self.hes_status.is_none_or(|status| status == 0)
    }

    fn active(&self) -> bool {
        self.command_at.is_some()
    }

    fn gated(&self, now: Instant) -> Write {
        if self.blocked {
            Write::Nothing
        } else if self.ready(now) {
            Write::Velocity(self.latest)
        } else {
            Write::Velocity(Velocity::ZERO)
        }
    }

    pub fn on_command(&mut self, x: f64, y: f64, yaw: f64, now: Instant) -> Write {
        self.latest = bound(x, y, yaw, &self.config.limits);
        self.command_at = Some(now);
        self.gated(now)
    }

    /// rl_deploy's heartbeat: resend the latest command, or release it once it is older than the command timeout.
    pub fn on_motion_info(&mut self, motion_state: i32, now: Instant) -> Write {
        self.motion_state = Some((motion_state, now));
        let Some(at) = self.command_at else {
            return Write::Nothing;
        };
        if now.saturating_duration_since(at) > self.config.command_timeout {
            return self.release();
        }
        self.gated(now)
    }

    pub fn on_hes_status(&mut self, status: i32) {
        self.hes_status = Some(status);
    }

    pub fn set_matched(&mut self, matched: bool) {
        self.matched = matched;
    }

    pub fn set_blocked(&mut self, blocked: bool) {
        self.blocked = blocked;
    }

    pub fn blocked(&self) -> bool {
        self.blocked
    }

    /// Stop commanding: a final zero if a command was live (and nobody else owns the topic).
    pub fn release(&mut self) -> Write {
        let was_active = self.active();
        self.command_at = None;
        self.latest = Velocity::ZERO;
        if was_active && !self.blocked {
            Write::FinalZero
        } else {
            Write::Nothing
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const RL: i32 = 17;

    fn gate() -> Gate {
        let mut gate = Gate::new(GateConfig {
            limits: Limits {
                max_linear_x: 2.0,
                max_linear_y: 1.0,
                max_angular_z: 2.0,
            },
            command_timeout: Duration::from_millis(400),
            state_timeout: Duration::from_millis(2500),
            rl_control_state: RL,
        });
        gate.set_matched(true);
        gate
    }

    fn v(x: f32, y: f32, yaw: f32) -> Write {
        Write::Velocity(Velocity { x, y, yaw })
    }

    #[test]
    fn clamps_each_axis_and_zeroes_non_finite() {
        let mut g = gate();
        let t = Instant::now();
        g.on_motion_info(RL, t);
        assert_eq!(g.on_command(3.0, -5.0, 9.0, t), v(2.0, -1.0, 2.0));
        assert_eq!(g.on_command(0.5, f64::NAN, 0.1, t), v(0.0, 0.0, 0.0));
        assert_eq!(g.on_command(0.5, 0.2, f64::INFINITY, t), v(0.0, 0.0, 0.0));
    }

    #[test]
    fn zero_unless_rl_control_fresh_clear_and_matched() {
        let mut g = gate();
        let t = Instant::now();
        assert_eq!(g.on_command(1.0, 0.0, 0.0, t), v(0.0, 0.0, 0.0));
        g.on_motion_info(0, t);
        assert_eq!(g.on_command(1.0, 0.0, 0.0, t), v(0.0, 0.0, 0.0));
        g.on_motion_info(RL, t);
        assert_eq!(g.on_command(1.0, 0.0, 0.0, t), v(1.0, 0.0, 0.0));
        g.on_hes_status(1);
        assert_eq!(g.on_command(1.0, 0.0, 0.0, t), v(0.0, 0.0, 0.0));
        g.on_hes_status(0);
        let stale = t + Duration::from_millis(2600);
        assert_eq!(g.on_command(1.0, 0.0, 0.0, stale), v(0.0, 0.0, 0.0));
        g.set_matched(false);
        assert_eq!(g.on_command(1.0, 0.0, 0.0, t), v(0.0, 0.0, 0.0));
    }

    #[test]
    fn heartbeat_resends_then_releases_after_the_command_timeout() {
        let mut g = gate();
        let t = Instant::now();
        assert_eq!(g.on_motion_info(RL, t), Write::Nothing);
        g.on_command(0.3, 0.0, 0.1, t);
        assert_eq!(
            g.on_motion_info(RL, t + Duration::from_millis(50)),
            v(0.3, 0.0, 0.1)
        );
        assert_eq!(
            g.on_motion_info(RL, t + Duration::from_millis(450)),
            Write::FinalZero
        );
        assert_eq!(
            g.on_motion_info(RL, t + Duration::from_millis(500)),
            Write::Nothing
        );
    }

    #[test]
    fn another_writer_blocks_everything() {
        let mut g = gate();
        let t = Instant::now();
        g.on_motion_info(RL, t);
        g.set_blocked(true);
        assert_eq!(g.on_command(1.0, 0.0, 0.0, t), Write::Nothing);
        assert_eq!(g.on_motion_info(RL, t), Write::Nothing);
        assert_eq!(g.release(), Write::Nothing);
    }

    #[test]
    fn release_sends_a_final_zero_only_after_a_command() {
        let mut g = gate();
        let t = Instant::now();
        assert_eq!(g.release(), Write::Nothing);
        g.on_command(0.1, 0.0, 0.0, t);
        assert_eq!(g.release(), Write::FinalZero);
    }
}
