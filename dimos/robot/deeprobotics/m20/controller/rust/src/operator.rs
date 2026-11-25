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

//! The operator's command, changed by the module's inputs and the app server alike, and the slot the loop polls.

use std::sync::Mutex;

use tracing::info;

use crate::types::{Target, UserCommand};

#[derive(Default)]
pub struct Operator {
    /// what the operator asks for now
    current: Mutex<UserCommand>,
    /// the newest change, until the loop takes it
    pub(crate) slot: Mutex<Option<UserCommand>>,
}

impl Operator {
    /// The gait walking starts on.
    pub fn set_start_gait(&self, gait: u32) {
        self.change(|c| c.gait = gait, false);
    }

    fn change(&self, f: impl FnOnce(&mut UserCommand), send: bool) {
        let mut current = self.current.lock().unwrap_or_else(|p| p.into_inner());
        f(&mut current);
        if send {
            *self.slot.lock().unwrap_or_else(|p| p.into_inner()) = Some(current.clone());
        }
    }

    /// A metric twist (m/s, m/s, rad/s): non-finite is zero (the vendor bridge's guard), each axis clamped to `limits`.
    pub fn velocity(&self, v: [f64; 3], limits: [f64; 3]) {
        let v = if v.iter().all(|x| x.is_finite()) {
            std::array::from_fn(|i| v[i].clamp(-limits[i], limits[i]))
        } else {
            [0.0; 3]
        };
        self.change(
            |c| {
                c.velocity = Some(v);
                c.steer = false;
            },
            true,
        );
    }

    /// basic_server's /STEER (the app's joystick): normalized axes, the same NaN guard, each clamped to [-1, 1].
    pub fn steer(&self, axes: [f64; 3]) {
        let a = if axes.iter().all(|x| x.is_finite()) {
            axes.map(|x| x.clamp(-1.0, 1.0))
        } else {
            [0.0; 3]
        };
        self.change(
            |c| {
                c.velocity = Some(a);
                c.steer = true;
            },
            true,
        );
    }

    /// A motion command: a state change starts from standing still.
    pub fn motion(&self, target: Target) {
        info!(?target, "motion command");
        self.change(
            |c| {
                c.target = target;
                c.motion_seq += 1;
                c.velocity = None;
            },
            true,
        );
    }

    /// A gait command (vendor gait code).
    pub fn gait(&self, gait: u32) {
        info!(gait = format!("{gait:#x}"), "gait command");
        self.change(
            |c| {
                c.gait = gait;
                c.motion_seq += 1;
                c.gait_seq += 1;
                c.velocity = None;
            },
            true,
        );
    }

    /// The operator's command now.
    pub fn current(&self) -> UserCommand {
        self.current
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone()
    }

    /// The newest change since the last take; a change still being written stays for the next tick.
    pub fn take(&self) -> Option<UserCommand> {
        crate::rt::try_read(&self.slot, &crate::rt::LOCK_SKIPS, |c| c.take()).flatten()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_twist_is_guarded_and_clamped_and_a_steer_is_normalized() {
        let o = Operator::default();
        o.velocity([3.0, -0.2, f64::NAN], [2.0, 1.0, 2.0]);
        assert_eq!(o.take().unwrap().velocity, Some([0.0; 3]));
        o.velocity([3.0, -1.5, 0.5], [2.0, 1.0, 2.0]);
        let c = o.take().unwrap();
        assert_eq!(c.velocity, Some([2.0, -1.0, 0.5]));
        assert!(!c.steer);
        assert!(o.take().is_none());
        o.steer([1.5, -0.5, 0.0]);
        let c = o.take().unwrap();
        assert_eq!(c.velocity, Some([1.0, -0.5, 0.0]));
        assert!(c.steer);
    }

    #[test]
    fn motion_and_gait_commands_count_and_drop_the_velocity() {
        let o = Operator::default();
        o.set_start_gait(0x1001);
        assert!(o.take().is_none());
        o.velocity([0.5, 0.0, 0.0], [2.0, 1.0, 2.0]);
        o.motion(Target::Stand);
        let c = o.take().unwrap();
        assert_eq!(
            (c.target, c.motion_seq, c.velocity, c.gait),
            (Target::Stand, 1, None, 0x1001)
        );
        o.gait(0x3002);
        let c = o.take().unwrap();
        assert_eq!((c.gait, c.motion_seq, c.gait_seq), (0x3002, 2, 1));
    }
}
