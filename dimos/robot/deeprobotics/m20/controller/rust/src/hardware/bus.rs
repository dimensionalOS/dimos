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

//! The controller on m20-ecan's bus owner (the one the dimos controller module runs): rl_deploy's startup
//! in `read_state`, then one frame per tick. Gains go out only with `allow_gains`; without it every command that
//! holds a joint becomes rl_deploy's sitting command, so the controller can run but not move the robot. Every frame
//! sent and received is handed to `sink` (joint state / battery publishing), and the charge rule runs on them.

#[cfg(target_os = "linux")]
use std::time::Duration;

#[cfg(target_os = "linux")]
use anyhow::{bail, Result};
#[cfg(target_os = "linux")]
use m20_battery::charge::BusCharge;
#[cfg(target_os = "linux")]
use m20_ecan::decode_ethernet;
#[cfg(any(target_os = "linux", test))]
use m20_ecan::owner::OwnerConfig;
#[cfg(any(target_os = "linux", test))]
use m20_ecan::Message;
use m20_ecan::{startup, Calibration, Direction, JointId};

#[cfg(target_os = "linux")]
use super::ecan::ImuSource;
#[cfg(target_os = "linux")]
use super::Hardware;
use crate::config::JOINTS;
use crate::types::{JointCommand, RobotState};

/// A frame seen on the bus, for whoever publishes: CLOCK_REALTIME ns, direction, Ethernet frame.
pub type Sink = Box<dyn FnMut(u64, Direction, &[u8]) + Send>;

/// The motor commands (motor units) for one controller command.
pub fn motor_commands(
    cmd: &JointCommand,
    calibration: &Calibration,
) -> Vec<m20_ecan::JointCommand> {
    if cmd.enable {
        return startup::zero_commands(2);
    }
    JointId::all()
        .map(|joint| {
            let i = joint.index();
            m20_ecan::JointCommand {
                // protocol constant: the M20's one motor board (m20-ecan, rl_deploy's frames)
                board_id: 0x79,
                joint,
                // rl_deploy's f32 conversion (Calibration::m20's offsets, as rl_deploy rounds them)
                position: crate::frames::vendor_motor_position(i, cmd.q[i] as f32),
                velocity: calibration.urdf_rate(joint, cmd.dq[i] as f32),
                torque: calibration.urdf_rate(joint, cmd.tau[i] as f32),
                kp: cmd.kp[i] as f32,
                kd: cmd.kd[i] as f32,
                control_word: m20_ecan::joint::CONTROL_MOTOR,
                data_id: 0,
            }
        })
        .collect()
}

/// The controller's view of a joint-state reply on top of the one before: a reply carries a record per joint it has,
/// and a joint missing from it keeps its last position, rates, temperatures and status word (rl_deploy's joint data
/// are persistent arrays, overwritten per joint, 0x167720); none before it and not every joint: no state yet.
pub fn robot_state(
    joints: &[m20_ecan::JointState],
    calibration: &Calibration,
    previous: Option<&RobotState>,
) -> Option<RobotState> {
    let mut s = previous.cloned().unwrap_or_default();
    let mut drivers = s.drivers.unwrap_or_default();
    let mut seen = 0u32;
    for j in joints {
        let i = j.joint.index();
        if i < JOINTS {
            seen |= 1 << i;
            drivers.status_word[i] = j.status_word;
            drivers.motor_temperature_c[i] = j.motor_temperature_c;
            drivers.driver_temperature_c[i] = j.driver_temperature_c;
            let q = crate::frames::vendor_joint_position(i, j.position);
            // single-turn hip / knee encoders into (-pi, pi] (only ever off when the encoder wraps)
            s.q[i] = if i % 4 == 3 || (-std::f32::consts::PI < q && q <= std::f32::consts::PI) {
                q as f64
            } else {
                calibration.urdf_position(j.joint, j.position) as f64
            };
            s.dq[i] = calibration.urdf_rate(j.joint, j.velocity) as f64;
            s.tau[i] = calibration.urdf_rate(j.joint, j.torque) as f64;
        }
    }
    if previous.is_none() && seen != (1 << JOINTS) - 1 {
        return None;
    }
    s.drivers = Some(drivers);
    Some(s)
}

#[cfg(target_os = "linux")]
pub struct BusHardware {
    bus: m20_ecan::bus::Bus,
    calibration: Calibration,
    imu: Box<dyn ImuSource>,
    sink: Sink,
    charge: Option<BusCharge>,
    /// m20-battery's rule decides requests (off: only request_charge_enable does)
    charge_rule: bool,
    /// ticks still to send a requested charge-enable frame in place of the command
    charge_ticks: usize,
    reply_timeout: Duration,
    started: std::time::Instant,
    last: Option<RobotState>,
}

#[cfg(target_os = "linux")]
impl BusHardware {
    /// `charge_enable`: run the charge rule and send its requests.
    pub fn open(
        interface: &str,
        config: OwnerConfig,
        charge_enable: bool,
        reply_timeout_s: f64,
        imu: Box<dyn ImuSource>,
        sink: Sink,
    ) -> Result<Self> {
        Ok(BusHardware {
            bus: m20_ecan::bus::Bus::open(interface, config)?,
            calibration: Calibration::m20(),
            imu,
            sink,
            charge: charge_enable.then(BusCharge::new),
            charge_rule: true,
            charge_ticks: 0,
            reply_timeout: Duration::from_secs_f64(reply_timeout_s),
            started: std::time::Instant::now(),
            last: None,
        })
    }

    /// A charge-enable request from the dock sequence (charge_manager's /BATTERY_CHARGE_ENABLE 1), as rl_deploy sends
    /// it: 3 frames on the next 3 ticks in place of whatever command they had (stand, lie-down or enable frames).
    /// Nothing unless built with charge_enable.
    pub fn request_charge_enable(&mut self) {
        if self.charge.is_some() {
            self.bus.owner.queue_charge_enable();
            self.charge_ticks = m20_battery::charge::FRAMES_PER_REQUEST;
        }
    }

    /// m20-battery's own lie-down / battery-report rule on (the default) or off (the dock sequence sends requests).
    pub fn set_charge_rule(&mut self, on: bool) {
        self.charge_rule = on;
    }

    /// Still sending rl_deploy's startup (status words are not yet meaningful).
    pub fn in_startup(&self) -> bool {
        self.bus.owner.in_startup()
    }

    pub fn refused(&self) -> u64 {
        self.bus.owner.refused
    }

    fn see(&mut self, t: u64, direction: Direction, frame: &[u8]) -> Option<RobotState> {
        (self.sink)(t, direction, frame);
        if let Some(charge) = self.charge.as_mut() {
            if charge.on_frame(direction, frame) && self.charge_rule {
                self.bus.owner.queue_charge_enable();
            }
        }
        match decode_ethernet(frame, direction) {
            Ok((_, Message::JointStates { joints, .. })) => {
                let s = robot_state(&joints, &self.calibration, self.last.as_ref())?;
                self.last = Some(s.clone());
                Some(s)
            }
            _ => None,
        }
    }

    /// Send one tick and take what came back; the joint state of this tick's reply if it arrived in time.
    /// A charge-enable frame went out (the board answers none: no reply is waited for).
    fn step(
        &mut self,
        commands: Option<&[m20_ecan::JointCommand]>,
    ) -> Result<(Option<RobotState>, bool)> {
        let mut wait_reply = true;
        if let Some(sent) = self.bus.tick(commands)? {
            wait_reply = !matches!(
                decode_ethernet(&sent.frame, sent.direction),
                Ok((_, Message::ChargeEnable))
            );
            self.see(sent.timestamp_ns, sent.direction, &sent.frame);
        }
        let mut state = None;
        for f in self.bus.drain() {
            state = self.see(f.timestamp_ns, f.direction, &f.frame).or(state);
        }
        if state.is_none() && wait_reply {
            if let Some(f) = self.bus.recv_timeout(self.reply_timeout) {
                state = self.see(f.timestamp_ns, f.direction, &f.frame);
            }
        }
        Ok((state, !wait_reply))
    }

    fn with_imu(&mut self, mut s: RobotState) -> Result<RobotState> {
        s.stamp = self.started.elapsed().as_secs_f64();
        let Some(imu) = self.imu.latest() else {
            bail!("no IMU sample")
        };
        s.quat = imu.quat;
        s.gyro = imu.gyro;
        s.accel = imu.accel;
        Ok(s)
    }
}

#[cfg(target_os = "linux")]
impl Hardware for BusHardware {
    fn read_state(&mut self) -> Result<RobotState> {
        // rl_deploy's startup, then sitting commands until a reply lands
        loop {
            if !self.bus.owner.in_startup() {
                if let Some(s) = self.last.clone() {
                    return self.with_imu(s);
                }
            }
            if let (Some(s), _) = self.step(None)? {
                self.last = Some(s);
            }
        }
    }

    fn exchange(&mut self, cmd: &JointCommand) -> Result<RobotState> {
        // a requested charge-enable frame goes out in place of this tick's command (the owner swaps it in only for a
        // command that holds nothing)
        let cmd = if self.charge_ticks > 0 {
            self.charge_ticks -= 1;
            &JointCommand::default()
        } else {
            cmd
        };
        let commands = motor_commands(cmd, &self.calibration);
        match self.step(Some(&commands))? {
            (Some(s), _) => {
                self.last = Some(s.clone());
                self.with_imu(s)
            }
            // the board does not answer a charge-enable frame: the tick runs on the last state, as rl_deploy's does
            (None, true) if self.last.is_some() => {
                let s = self.last.clone().unwrap_or_default();
                self.with_imu(s)
            }
            (None, _) => bail!("no joint-state reply within {:?}", self.reply_timeout),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn conversions_round_trip_through_the_vendor_calibration() {
        let calibration = Calibration::m20();
        let mut cmd = JointCommand {
            q: [
                0.1, -0.7, 1.4, 3.0, -0.1, -0.7, 1.4, 3.0, 0.1, 0.7, -1.4, 3.0, -0.1, 0.7, -1.4,
                3.0,
            ],
            ..Default::default()
        };
        cmd.kp = [60.0; JOINTS];
        let motor = motor_commands(&cmd, &calibration);
        let states: Vec<m20_ecan::JointState> = motor
            .iter()
            .map(|c| m20_ecan::JointState {
                board_id: 0x79,
                joint: c.joint,
                position: c.position,
                velocity: c.velocity,
                torque: c.torque,
                motor_temperature_c: 0.0,
                driver_temperature_c: 0.0,
                status_word: 1,
                data_id: 0,
            })
            .collect();
        let back = robot_state(&states, &calibration, None).unwrap();
        // a reply without some joints: they keep their last values and status word (no fault from a missing record)
        let mut first = back.clone();
        first.drivers.as_mut().unwrap().motor_temperature_c[0] = 40.0;
        let partial = robot_state(&states[4..], &calibration, Some(&first)).unwrap();
        assert_eq!(partial.drivers.as_ref().unwrap().status_word, [1; JOINTS]);
        assert_eq!(
            partial.drivers.as_ref().unwrap().motor_temperature_c[0],
            40.0
        );
        assert_eq!(partial.q[0], first.q[0]);
        let none = robot_state(&[], &calibration, Some(&first)).unwrap();
        assert_eq!(none.drivers.unwrap().status_word, [1; JOINTS]);
        // nothing before and not every joint: no state yet
        assert!(robot_state(&states[4..], &calibration, None).is_none());
        for i in 0..JOINTS {
            assert!((back.q[i] - cmd.q[i]).abs() < 1e-5, "{i}");
        }
        assert!(motor.iter().all(|c| c.kp == 60.0));
        let enable = motor_commands(
            &JointCommand {
                enable: true,
                ..Default::default()
            },
            &calibration,
        );
        assert!(enable
            .iter()
            .all(|c| c.control_word == 2 && c.kp == 0.0 && c.kd == 0.0));
    }

    #[test]
    fn owner_refuses_the_controllers_gains_by_default() {
        let calibration = Calibration::m20();
        let cmd = JointCommand {
            kp: [80.0; JOINTS],
            kd: [3.0; JOINTS],
            ..Default::default()
        };
        let mut owner = m20_ecan::owner::Owner::new(OwnerConfig::default());
        let commands = motor_commands(&cmd, &calibration);
        for _ in 0..(startup::sequence().len() + 100) {
            if let Some(body) = owner.next(Some(&commands)).body {
                let frame = m20_ecan::Frame::parse(&body).unwrap();
                if let Ok(Message::JointCommands(sent)) =
                    Message::decode(&frame, Direction::ToBoard)
                {
                    assert!(!startup::has_gain(&sent));
                }
            }
        }
        assert_eq!(owner.refused, 100);
    }
}
