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

//! Motor-bus records to dimos messages: URDF-named JointStates and per-pack / combined BatteryStates.

use lcm_msgs::sensor_msgs::{BatteryState, JointState};
use lcm_msgs::std_msgs::{Header, Time};
use m20_battery::BatteryPack;
use m20_ecan::{Calibration, JointCommand, JointId};

pub fn stamp(timestamp_ns: u64) -> Time {
    Time {
        sec: (timestamp_ns / 1_000_000_000) as i32,
        nsec: (timestamp_ns % 1_000_000_000) as i32,
    }
}

fn header(timestamp_ns: u64, frame_id: &str) -> Header {
    Header {
        seq: 0,
        stamp: stamp(timestamp_ns),
        frame_id: frame_id.to_string(),
    }
}

/// URDF-side (joint, position, velocity, effort) rows as a JointState in rl_deploy's joint order.
fn joint_state(
    rows: impl Iterator<Item = (JointId, f32, f32, f32)>,
    timestamp_ns: u64,
    frame_id: &str,
) -> JointState {
    let mut message = JointState {
        header: header(timestamp_ns, frame_id),
        ..Default::default()
    };
    for (joint, position, velocity, effort) in rows {
        message.name.push(joint.urdf_name());
        message.position.push(position as f64);
        message.velocity.push(velocity as f64);
        message.effort.push(effort as f64);
    }
    message
}

/// The measured joint state: URDF position (rad), velocity (rad/s), torque (N m).
pub fn measured(
    states: &[m20_ecan::JointState],
    calibration: &Calibration,
    timestamp_ns: u64,
    frame_id: &str,
) -> JointState {
    joint_state(
        states.iter().map(|s| {
            (
                s.joint,
                calibration.urdf_position(s.joint, s.position),
                calibration.urdf_rate(s.joint, s.velocity),
                calibration.urdf_rate(s.joint, s.torque),
            )
        }),
        timestamp_ns,
        frame_id,
    )
}

/// rl_deploy's command: URDF target position, target velocity, feed-forward torque (gains are not in a JointState).
pub fn commanded(
    commands: &[JointCommand],
    calibration: &Calibration,
    timestamp_ns: u64,
    frame_id: &str,
) -> JointState {
    joint_state(
        commands.iter().map(|c| {
            (
                c.joint,
                calibration.urdf_position(c.joint, c.position),
                calibration.urdf_rate(c.joint, c.velocity),
                calibration.urdf_rate(c.joint, c.torque),
            )
        }),
        timestamp_ns,
        frame_id,
    )
}

const TEMPERATURE_BITS: [(usize, i8); 4] = [
    (4, BatteryState::POWER_SUPPLY_HEALTH_OVERHEAT),
    (6, BatteryState::POWER_SUPPLY_HEALTH_OVERHEAT),
    (5, BatteryState::POWER_SUPPLY_HEALTH_COLD),
    (7, BatteryState::POWER_SUPPLY_HEALTH_COLD),
];

fn health(pack: &BatteryPack) -> i8 {
    let bits = pack.protection_state;
    if bits == 0 {
        return BatteryState::POWER_SUPPLY_HEALTH_GOOD;
    }
    for (bit, health) in TEMPERATURE_BITS {
        if bits & (1 << bit) != 0 {
            return health;
        }
    }
    if bits & 0b101 != 0 {
        return BatteryState::POWER_SUPPLY_HEALTH_OVERVOLTAGE;
    }
    BatteryState::POWER_SUPPLY_HEALTH_UNSPEC_FAILURE
}

fn status(pack: &BatteryPack) -> i8 {
    if pack.current_a > 0.0 {
        BatteryState::POWER_SUPPLY_STATUS_CHARGING
    } else if pack.level_percent >= 100 {
        BatteryState::POWER_SUPPLY_STATUS_FULL
    } else if pack.current_a < 0.0 {
        BatteryState::POWER_SUPPLY_STATUS_DISCHARGING
    } else {
        BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING
    }
}

/// One pack. Positive current = charging (as the BMS reports it). Technology: Li-ion, from the 20 cells at 4.13-4.15 V
/// when full.
pub fn pack_state(
    pack: &BatteryPack,
    location: &str,
    timestamp_ns: u64,
    frame_id: &str,
) -> BatteryState {
    let temperatures = pack.temperatures();
    BatteryState {
        header: header(timestamp_ns, frame_id),
        voltage: pack.voltage_v,
        temperature: temperatures.iter().copied().fold(f32::NAN, f32::max),
        current: pack.current_a,
        charge: pack.remaining_capacity_ah,
        capacity: f32::NAN,
        design_capacity: pack.nominal_capacity_ah,
        percentage: pack.level_percent as f32 / 100.0,
        power_supply_status: status(pack) as u8,
        power_supply_health: health(pack) as u8,
        power_supply_technology: BatteryState::POWER_SUPPLY_TECHNOLOGY_LION as u8,
        present: pack.voltage_v > 0.0,
        cell_voltage: pack
            .cells_mv()
            .iter()
            .map(|&mv| mv as f32 / 1000.0)
            .collect(),
        cell_temperature: temperatures.to_vec(),
        location: location.to_string(),
        serial_number: pack.serial_number.clone(),
    }
}

/// Both packs as one battery, the way M20Connection combines basic_server's report: the mean voltage and level of the
/// packs present, the hottest temperature, summed current and charge; charging if either pack charges.
pub fn combined_state(packs: &[&BatteryPack], timestamp_ns: u64, frame_id: &str) -> BatteryState {
    let present: Vec<BatteryState> = packs
        .iter()
        .zip(["left", "right"])
        .map(|(pack, side)| pack_state(pack, side, timestamp_ns, frame_id))
        .filter(|state| state.present)
        .collect();
    let count = present.len().max(1) as f32;
    let any = |status: i8| {
        present
            .iter()
            .any(|s| s.power_supply_status == status as u8)
    };
    let all = |status: i8| {
        !present.is_empty()
            && present
                .iter()
                .all(|s| s.power_supply_status == status as u8)
    };
    let power_supply_status = if any(BatteryState::POWER_SUPPLY_STATUS_CHARGING) {
        BatteryState::POWER_SUPPLY_STATUS_CHARGING
    } else if all(BatteryState::POWER_SUPPLY_STATUS_FULL) {
        BatteryState::POWER_SUPPLY_STATUS_FULL
    } else if any(BatteryState::POWER_SUPPLY_STATUS_DISCHARGING) {
        BatteryState::POWER_SUPPLY_STATUS_DISCHARGING
    } else if present.is_empty() {
        BatteryState::POWER_SUPPLY_STATUS_UNKNOWN
    } else {
        BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING
    };
    let good = BatteryState::POWER_SUPPLY_HEALTH_GOOD as u8;
    BatteryState {
        header: header(timestamp_ns, frame_id),
        voltage: present.iter().map(|s| s.voltage).sum::<f32>() / count,
        temperature: present
            .iter()
            .map(|s| s.temperature)
            .fold(f32::NAN, f32::max),
        current: present.iter().map(|s| s.current).sum(),
        charge: present.iter().map(|s| s.charge).sum(),
        capacity: f32::NAN,
        design_capacity: present.iter().map(|s| s.design_capacity).sum(),
        percentage: present.iter().map(|s| s.percentage).sum::<f32>() / count,
        power_supply_status: power_supply_status as u8,
        power_supply_health: present
            .iter()
            .map(|s| s.power_supply_health)
            .find(|&h| h != good)
            .unwrap_or(if present.is_empty() {
                BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN as u8
            } else {
                good
            }),
        power_supply_technology: BatteryState::POWER_SUPPLY_TECHNOLOGY_LION as u8,
        present: !present.is_empty(),
        cell_voltage: present
            .iter()
            .flat_map(|s| s.cell_voltage.iter().copied())
            .collect(),
        cell_temperature: present
            .iter()
            .flat_map(|s| s.cell_temperature.iter().copied())
            .collect(),
        location: present
            .iter()
            .map(|s| s.location.as_str())
            .collect::<Vec<_>>()
            .join("+"),
        serial_number: String::new(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use m20_battery::BatteryReport;
    use m20_ecan::{decode_ethernet, tap, Direction, Message};

    const CAPTURE: &[u8] = include_bytes!("../tests/sitting_docked_20261120_125557.pcapng");

    #[test]
    fn joint_states_from_a_real_capture() {
        let calibration = Calibration::m20();
        let (timestamp_ns, joints) = tap::read_capture(CAPTURE)
            .unwrap()
            .into_iter()
            .find_map(
                |(t, direction, frame)| match decode_ethernet(frame, direction) {
                    Ok((_, Message::JointStates { joints, .. })) => Some((t, joints)),
                    _ => None,
                },
            )
            .unwrap();
        let message = measured(&joints, &calibration, timestamp_ns, "base_link");
        assert_eq!(message.name.len(), 16);
        assert_eq!(message.name[2], "fl_knee_joint");
        // folded on the dock: front-left knee about 2.63 rad, hip pitch about -0.9 rad
        assert!((message.position[2] - 2.63).abs() < 0.05);
        assert!((message.position[1] + 0.90).abs() < 0.05);
        assert_eq!(message.header.stamp, stamp(timestamp_ns));
        let decoded = JointState::decode(&message.encode()).unwrap();
        assert_eq!(decoded, message);
    }

    #[test]
    fn damping_command_is_the_urdf_zero_pose() {
        let calibration = Calibration::m20();
        let commands = tap::read_capture(CAPTURE)
            .unwrap()
            .into_iter()
            .find_map(
                |(_, direction, frame)| match decode_ethernet(frame, direction) {
                    Ok((_, Message::JointCommands(commands))) => Some(commands),
                    _ => None,
                },
            )
            .unwrap();
        let message = commanded(&commands, &calibration, 0, "");
        for (name, position) in message.name.iter().zip(&message.position) {
            if !name.contains("wheel") {
                assert!(position.abs() < 1e-3, "{name} {position}");
            }
        }
    }

    #[test]
    fn batteries_from_a_real_capture() {
        let (timestamp_ns, frame) = tap::read_capture(CAPTURE)
            .unwrap()
            .into_iter()
            .find(|(_, direction, frame)| *direction == Direction::FromBoard && frame[19] == 0x87)
            .map(|(t, _, frame)| (t, frame))
            .unwrap();
        let report = BatteryReport::from_ethernet(frame).unwrap();
        let left = pack_state(&report.left, "left", timestamp_ns, "");
        assert!((left.voltage - 82.68).abs() < 0.01);
        assert!((left.percentage - 0.90).abs() < 1e-6);
        assert_eq!(
            left.power_supply_status,
            BatteryState::POWER_SUPPLY_STATUS_CHARGING as u8
        );
        assert_eq!(
            left.power_supply_health,
            BatteryState::POWER_SUPPLY_HEALTH_GOOD as u8
        );
        assert_eq!(left.cell_voltage.len(), 20);
        assert!((left.temperature - 42.3).abs() < 0.05);
        let both = combined_state(&[&report.left, &report.right], timestamp_ns, "");
        assert!((both.percentage - 0.945).abs() < 1e-6);
        assert!((both.voltage - (82.68 + 82.95) / 2.0).abs() < 0.01);
        assert_eq!(both.location, "left+right");
        assert_eq!(both.cell_voltage.len(), 40);
        assert_eq!(
            BatteryState::decode(&both.encode()).unwrap().location,
            "left+right"
        );
    }

    #[test]
    fn protection_bits_to_health() {
        let mut record = [0u8; 258];
        record[0] = 100;
        record[16] = 1 << 4;
        let pack = BatteryPack::parse(&record).unwrap();
        assert_eq!(health(&pack), BatteryState::POWER_SUPPLY_HEALTH_OVERHEAT);
        record[16] = 1 << 1;
        assert_eq!(
            health(&BatteryPack::parse(&record).unwrap()),
            BatteryState::POWER_SUPPLY_HEALTH_UNSPEC_FAILURE
        );
        assert!(!combined_state(&[], 0, "").present);
    }
}
