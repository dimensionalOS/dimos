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

//! The robot as the 1 kHz loop hands it to the app server (Snapshots), and basic_server's pushed reports built from
//! it: T1002 C4 motion (UpdateMotionStatus 0x17fa88, UpdateMotorStatus 0x17ff38), C5 device (UpdateBatteryStatus
//! 0x181db0), C6 basic (UpdateBasicStatus 0x1812f0) and C3 errors (ExceptionHandle::updateErrorResponseJSON 0x2483f8).
//! Fields our stack has no source for (GPS, CPU, the power rails, payload, mileage) are sent as zeros.

use std::sync::Mutex;

use serde::ser::{SerializeMap, Serializer};
use serde::Serialize;

use crate::battery::BatteryLevel;
use crate::config::JOINTS;
use crate::state_machine::State;

/// The controller's state at one tick, as the reports read it.
#[derive(Debug, Clone, Default)]
pub struct RobotView {
    pub state: Option<State>,
    /// the gait rl_control runs (the commanded one outside it)
    pub gait: u32,
    /// roll, pitch, yaw (rad)
    pub rpy: [f64; 3],
    /// body angular velocity (rad/s)
    pub gyro: [f64; 3],
    /// the commanded velocity (m/s, m/s, rad/s)
    pub velocity: [f64; 3],
    /// rl_deploy's MotionInfo height (m)
    pub height_m: f64,
    pub q: [f64; JOINTS],
    pub dq: [f64; JOINTS],
    pub motor_temperature_c: [f64; JOINTS],
    pub driver_temperature_c: [f64; JOINTS],
    pub usage_mode: i32,
    pub direction: i32,
    /// charge_manager's state for docking (0 idle, 1 docking, 2 charging, 3 undocking, 4 error, 5 on the dock)
    pub charger_state: i32,
    /// the ECAN board's hard-stop byte (none before the first)
    pub hard_stop: Option<u8>,
    /// the vendor error codes for the controller's fault and battery level, with their component
    pub errors: Vec<ErrorEntry>,
}

/// One battery pack's reading for the device report.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct PackView {
    pub voltage_v: f64,
    pub level_percent: u8,
    pub temperature_c: f64,
    pub charging: bool,
    pub serial: String,
}

/// What the 1 kHz loop hands over: written with try_lock (a slot the server holds is skipped this time).
#[derive(Default)]
pub struct Snapshots {
    pub robot: Mutex<Option<RobotView>>,
    /// left, right
    pub packs: Mutex<Option<[PackView; 2]>>,
}

impl Snapshots {
    /// From the loop: never waits.
    pub fn offer_robot(&self, view: RobotView) {
        if let Ok(mut slot) = self.robot.try_lock() {
            *slot = Some(view);
        }
    }

    pub fn offer_packs(&self, packs: [PackView; 2]) {
        if let Ok(mut slot) = self.packs.try_lock() {
            *slot = Some(packs);
        }
    }

    pub fn robot(&self) -> Option<RobotView> {
        self.robot.lock().unwrap_or_else(|p| p.into_inner()).clone()
    }

    pub fn packs(&self) -> Option<[PackView; 2]> {
        self.packs.lock().unwrap_or_else(|p| p.into_inner()).clone()
    }
}

/// The vendor's error codes (M20 software development guide 1.3.2) for our controller's faults, by their text.
const FAULT_CODES: [(&str, u32); 8] = [
    ("driver status word", 0x8028),
    ("motor overheated", 0x8002),
    ("driver overheated", 0x8007),
    ("frozen", 0x8025),
    ("NaN", 0x8024),
    ("beyond its range", 0x8022),
    ("tilt", 0x8027),
    ("replies missed", 0x8012),
];

/// One ErrorList entry: the code and its component (a joint fault's bit in the vendor's joint order).
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct ErrorEntry {
    pub error_code: u32,
    pub component: u32,
}

/// The ErrorList entries for a controller fault (its joint by name in `joint_names`) and the battery level.
pub fn errors_of(
    fault: Option<&str>,
    joint_names: &[String],
    battery: Option<BatteryLevel>,
) -> Vec<ErrorEntry> {
    let mut out = Vec::new();
    if let Some(f) = fault {
        if let Some(&(_, code)) = FAULT_CODES.iter().find(|(key, _)| f.contains(key)) {
            let joint = joint_names.iter().position(|n| f.contains(n.as_str()));
            out.push(ErrorEntry {
                error_code: code,
                component: joint.map_or(0, |i| 1 << i),
            });
        }
    }
    match battery {
        Some(BatteryLevel::Warning) => out.push(ErrorEntry {
            error_code: 0x8102,
            component: 0,
        }),
        Some(BatteryLevel::Error) => out.push(ErrorEntry {
            error_code: 0x8103,
            component: 0,
        }),
        _ => {}
    }
    out
}

impl RobotView {
    /// The loop's tick as the reports read it.
    pub fn of(
        cfg: &crate::config::Config,
        tick: &crate::rt::Tick,
        usage_mode: i32,
        direction: i32,
        dock: Option<&crate::dock::docking::Status>,
        hard_stop: Option<u8>,
    ) -> RobotView {
        let c = tick.controller;
        let s = tick.state;
        let (roll, pitch) = crate::types::roll_pitch(&s.quat);
        let [w, x, y, z] = s.quat;
        let yaw = (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z));
        let gait = if c.state == State::RlControl {
            c.gait()
        } else {
            tick.user.gait
        };
        // MotionInfo's vel_x / vel_y: the command in m/s (normalized axes times the gait's REAL_STEER divisor)
        let velocity = tick.user.velocity.unwrap_or_else(|| {
            let d = cfg.real_steer.divisor(gait);
            std::array::from_fn(|i| tick.user.axes[i] * d[i])
        });
        let temperatures = |f: fn(&crate::types::Drivers) -> [f32; JOINTS]| {
            s.drivers
                .as_ref()
                .map_or([0.0; JOINTS], |d| f(d).map(f64::from))
        };
        RobotView {
            state: Some(c.state),
            gait,
            rpy: [roll, pitch, yaw],
            gyro: s.gyro,
            velocity,
            height_m: c.motion_info_height() as f64,
            q: s.q,
            dq: s.dq,
            motor_temperature_c: temperatures(|d| d.motor_temperature_c),
            driver_temperature_c: temperatures(|d| d.driver_temperature_c),
            usage_mode,
            direction,
            charger_state: dock.map_or(0, crate::basic_server::charger_state_of),
            hard_stop,
            errors: errors_of(c.fault.as_deref(), &cfg.robot.joint_names, s.battery),
        }
    }
}

/// /MOTION_INFO's motion state as basic_server reports it (m20tool's MOTION table): 0 idle, 1 standing, 2 soft
/// e-stop, 3 power-on damping, 4 sitting, 17 RL control; -1 before the controller runs (1.2.32's default).
pub fn motion_state(state: Option<State>) -> i32 {
    match state {
        None => -1,
        Some(State::Passive) => 0,
        Some(State::StandUp) => 1,
        Some(State::JointDamping) => 2,
        Some(State::StartDamping) => 3,
        Some(State::LieDown) => 4,
        Some(State::RlControl) => 17,
    }
}

/// The vendor's joint names in robot order (fl, fr, hl, hr x hipx, hipy, knee, wheel).
pub const JOINT_NAMES: [&str; JOINTS] = [
    "LeftFrontHipX",
    "LeftFrontHipY",
    "LeftFrontKnee",
    "LeftFrontWheel",
    "RightFrontHipX",
    "RightFrontHipY",
    "RightFrontKnee",
    "RightFrontWheel",
    "LeftBackHipX",
    "LeftBackHipY",
    "LeftBackKnee",
    "LeftBackWheel",
    "RightBackHipX",
    "RightBackHipY",
    "RightBackKnee",
    "RightBackWheel",
];

/// An object of one number per joint, keyed `<joint><suffix>` in the vendor's order.
struct PerJoint<'a> {
    values: [f64; JOINTS],
    suffix: &'a str,
}

impl Serialize for PerJoint<'_> {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        let mut map = serializer.serialize_map(Some(JOINTS))?;
        for (name, v) in JOINT_NAMES.iter().zip(self.values) {
            map.serialize_entry(&format!("{name}{}", self.suffix), &finite(v))?;
        }
        map.end()
    }
}

/// basic_server sends a NaN as 0.
fn finite(v: f64) -> f64 {
    if v.is_finite() {
        v
    } else {
        0.0
    }
}

#[derive(Serialize)]
#[serde(rename_all = "PascalCase")]
struct MotionStatus {
    roll: f64,
    pitch: f64,
    yaw: f64,
    omega_z: f64,
    linear_x: f64,
    linear_y: f64,
    height: f64,
    payload: f64,
    remain_mile: f64,
}

#[derive(Serialize)]
pub struct MotionItems<'a> {
    #[serde(rename = "MotionStatus")]
    motion: MotionStatus,
    #[serde(rename = "MotorStatus")]
    motors: PerJoint<'a>,
}

/// T1002/C4's Items: the hips and knees report their position (rad), the wheels their velocity (rad/s).
pub fn motion_items(r: &RobotView) -> MotionItems<'static> {
    MotionItems {
        motion: MotionStatus {
            roll: finite(r.rpy[0]),
            pitch: finite(r.rpy[1]),
            yaw: finite(r.rpy[2]),
            omega_z: finite(r.gyro[2]),
            linear_x: finite(r.velocity[0]),
            linear_y: finite(r.velocity[1]),
            height: finite(r.height_m),
            payload: 0.0,
            remain_mile: 0.0,
        },
        motors: PerJoint {
            values: std::array::from_fn(|i| if i % 4 == 3 { r.dq[i] } else { r.q[i] }),
            suffix: "",
        },
    }
}

#[derive(Serialize)]
#[serde(rename_all = "PascalCase")]
struct BasicStatus {
    motion_state: i32,
    gait: u32,
    charge: i32,
    #[serde(rename = "HES")]
    hes: i32,
    control_usage_mode: i32,
    direction: i32,
    #[serde(rename = "OOA")]
    ooa: i32,
    power_management: i32,
    sleep: i32,
    version: String,
    status_code: i32,
}

#[derive(Serialize)]
pub struct BasicItems {
    #[serde(rename = "BasicStatus")]
    basic: BasicStatus,
}

/// T1002/C6's Items.
pub fn basic_items(r: &RobotView, version: &str) -> BasicItems {
    BasicItems {
        basic: BasicStatus {
            motion_state: motion_state(r.state),
            gait: r.gait,
            charge: r.charger_state,
            hes: r.hard_stop.map_or(0, i32::from),
            control_usage_mode: r.usage_mode,
            direction: r.direction,
            ooa: 0,
            power_management: 0,
            sleep: 0,
            version: version.into(),
            status_code: 0,
        },
    }
}

#[derive(Serialize)]
struct BatteryStatus {
    #[serde(rename = "VoltageLeft")]
    voltage_left: i64,
    #[serde(rename = "VoltageRight")]
    voltage_right: i64,
    #[serde(rename = "BatteryLevelLeft")]
    level_left: i64,
    #[serde(rename = "BatteryLevelRight")]
    level_right: i64,
    #[serde(rename = "battery_temperatureLeft")]
    temperature_left: f64,
    #[serde(rename = "battery_temperatureRight")]
    temperature_right: f64,
    #[serde(rename = "chargeLeft")]
    charge_left: bool,
    #[serde(rename = "chargeRight")]
    charge_right: bool,
    #[serde(rename = "serialLeft")]
    serial_left: String,
    #[serde(rename = "serialRight")]
    serial_right: String,
}

#[derive(Serialize)]
#[serde(rename_all = "PascalCase")]
struct Sides {
    front: i32,
    back: i32,
}

#[derive(Serialize)]
#[serde(rename_all = "PascalCase")]
struct Led {
    fill: Sides,
}

#[derive(Serialize)]
#[serde(rename_all = "PascalCase")]
struct Gps {
    latitude: f64,
    longitude: f64,
    speed: f64,
    course: f64,
    altitude: f64,
    #[serde(rename = "HDOP")]
    hdop: f64,
    #[serde(rename = "VDOP")]
    vdop: f64,
    #[serde(rename = "PDOP")]
    pdop: f64,
    fix_quality: i32,
    num_satellites: i32,
    visible_satellites: i32,
}

#[derive(Serialize)]
#[serde(rename_all = "PascalCase")]
struct DevEnable {
    fan_speed: i32,
    load_power: i32,
    led_host: i32,
    led_ext: i32,
    #[serde(rename = "FP")]
    fp: i32,
    #[serde(rename = "GPS")]
    gps: i32,
    lidar: Sides,
    video: Sides,
}

#[derive(Serialize)]
#[serde(rename_all = "PascalCase")]
struct Cpu {
    temperature: f64,
    frequency_int: f64,
    frequency_app: f64,
}

#[derive(Serialize)]
struct Cpus {
    #[serde(rename = "AOS")]
    aos: Cpu,
    #[serde(rename = "GOS")]
    gos: Cpu,
    #[serde(rename = "NOS")]
    nos: Cpu,
}

/// The joint temperatures, `<joint>Motor` then `<joint>Driver` per joint.
struct DeviceTemperature {
    motor: [f64; JOINTS],
    driver: [f64; JOINTS],
}

impl Serialize for DeviceTemperature {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        let mut map = serializer.serialize_map(Some(2 * JOINTS))?;
        for (i, name) in JOINT_NAMES.iter().enumerate() {
            map.serialize_entry(&format!("{name}Motor"), &finite(self.motor[i]))?;
            map.serialize_entry(&format!("{name}Driver"), &finite(self.driver[i]))?;
        }
        map.end()
    }
}

#[derive(Serialize)]
#[serde(rename_all = "PascalCase")]
pub struct DeviceItems {
    battery_status: BatteryStatus,
    device_temperature: DeviceTemperature,
    #[serde(rename = "CPU")]
    cpu: Cpus,
    dev_enable: DevEnable,
    #[serde(rename = "GPS")]
    gps: Gps,
    led: Led,
}

/// T1002/C5's Items: whole volts and the level as integers, the pack's hottest sensor; zeros before a battery frame.
pub fn device_items(r: &RobotView, packs: Option<&[PackView; 2]>) -> DeviceItems {
    let empty = PackView::default();
    let (l, rt) = packs.map_or((&empty, &empty), |p| (&p[0], &p[1]));
    let cpu = || Cpu {
        temperature: 0.0,
        frequency_int: 0.0,
        frequency_app: 0.0,
    };
    let sides = || Sides { front: 0, back: 0 };
    DeviceItems {
        battery_status: BatteryStatus {
            voltage_left: l.voltage_v as i64,
            voltage_right: rt.voltage_v as i64,
            level_left: l.level_percent as i64,
            level_right: rt.level_percent as i64,
            temperature_left: finite(l.temperature_c),
            temperature_right: finite(rt.temperature_c),
            charge_left: l.charging,
            charge_right: rt.charging,
            serial_left: l.serial.clone(),
            serial_right: rt.serial.clone(),
        },
        device_temperature: DeviceTemperature {
            motor: r.motor_temperature_c,
            driver: r.driver_temperature_c,
        },
        cpu: Cpus {
            aos: cpu(),
            gos: cpu(),
            nos: cpu(),
        },
        dev_enable: DevEnable {
            fan_speed: 0,
            load_power: 0,
            led_host: 0,
            led_ext: 0,
            fp: 0,
            gps: 0,
            lidar: sides(),
            video: sides(),
        },
        gps: Gps {
            latitude: 0.0,
            longitude: 0.0,
            speed: 0.0,
            course: 0.0,
            altitude: 0.0,
            hdop: 0.0,
            vdop: 0.0,
            pdop: 0.0,
            fix_quality: 0,
            num_satellites: 0,
            visible_satellites: 0,
        },
        led: Led { fill: sides() },
    }
}

#[derive(Serialize)]
pub struct ErrorItems<'a> {
    #[serde(rename = "ErrorList")]
    list: &'a [ErrorEntry],
}

/// T1002/C3's Items (pushed every error_report_period_s, and GetError's reply).
pub fn error_items(r: &RobotView) -> ErrorItems<'_> {
    ErrorItems { list: &r.errors }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn json<T: Serialize>(t: &T) -> String {
        serde_json::to_string(t).unwrap()
    }

    #[test]
    fn the_motion_report_is_the_vendor_s_shape_with_floats_and_wheel_velocities() {
        let mut r = RobotView::default();
        r.q[0] = 0.25;
        r.dq[3] = 4.5;
        r.q[3] = 99.0;
        r.rpy[0] = f64::NAN;
        let text = json(&motion_items(&r));
        assert!(
            text.starts_with(r#"{"MotionStatus":{"Roll":0.0,"Pitch":0.0,"Yaw":0.0,"OmegaZ":0.0,"#),
            "{text}"
        );
        assert!(text.contains(r#""MotorStatus":{"LeftFrontHipX":0.25,"LeftFrontHipY":0.0,"LeftFrontKnee":0.0,"LeftFrontWheel":4.5,"#));
        let v: serde_json::Value = serde_json::from_str(&text).unwrap();
        assert_eq!(v["MotorStatus"].as_object().unwrap().len(), JOINTS);
        assert!(v["MotionStatus"]["RemainMile"].is_f64());
    }

    #[test]
    fn the_basic_and_device_reports_carry_the_vendor_s_fields_and_types() {
        let r = RobotView {
            state: Some(State::RlControl),
            gait: 0x3002,
            charger_state: 2,
            usage_mode: 1,
            direction: 1,
            hard_stop: Some(1),
            ..Default::default()
        };
        let basic = json(&basic_items(&r, "1.2.43"));
        assert_eq!(
            basic,
            r#"{"BasicStatus":{"MotionState":17,"Gait":12290,"Charge":2,"HES":1,"ControlUsageMode":1,"Direction":1,"OOA":0,"PowerManagement":0,"Sleep":0,"Version":"1.2.43","StatusCode":0}}"#
        );
        let packs = [
            PackView {
                voltage_v: 83.67,
                level_percent: 90,
                temperature_c: 31.5,
                charging: true,
                serial: "L1".into(),
            },
            PackView::default(),
        ];
        let v: serde_json::Value = serde_json::to_value(device_items(&r, Some(&packs))).unwrap();
        let b = &v["BatteryStatus"];
        assert_eq!(
            (b["VoltageLeft"].as_i64(), b["BatteryLevelLeft"].as_i64()),
            (Some(83), Some(90))
        );
        assert_eq!(
            (
                b["battery_temperatureLeft"].as_f64(),
                b["chargeLeft"].as_bool()
            ),
            (Some(31.5), Some(true))
        );
        assert_eq!(b["serialLeft"], "L1");
        assert!(v["DeviceTemperature"]["RightBackWheelDriver"].is_f64());
        assert_eq!(
            v["DeviceTemperature"].as_object().unwrap().len(),
            2 * JOINTS
        );
        for key in ["CPU", "DevEnable", "GPS", "Led"] {
            assert!(v[key].is_object(), "{key}");
        }
        assert_eq!(v["Led"]["Fill"]["Front"], 0);
    }

    #[test]
    fn faults_and_battery_levels_are_the_vendor_s_error_codes() {
        let names: Vec<String> = ["fl_hipx", "fl_hipy", "fl_knee"].map(String::from).into();
        let e = errors_of(
            Some("joint fl_knee NaN (31 readings)"),
            &names,
            Some(BatteryLevel::Warning),
        );
        assert_eq!(
            e,
            [
                ErrorEntry {
                    error_code: 0x8024,
                    component: 1 << 2
                },
                ErrorEntry {
                    error_code: 0x8102,
                    component: 0
                }
            ]
        );
        assert!(errors_of(Some("policy stalled"), &names, None).is_empty());
        let r = RobotView {
            errors: e,
            ..Default::default()
        };
        assert_eq!(
            json(&error_items(&r)),
            r#"{"ErrorList":[{"errorCode":32804,"component":4},{"errorCode":33026,"component":0}]}"#
        );
    }
}
