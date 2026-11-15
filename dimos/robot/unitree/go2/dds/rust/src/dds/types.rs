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

//! The Go2's DDS types we read and write, field order 1:1 with the unitree_sdk2 IDL
//! (plain FINAL CDR). The on-wire type name uses `::` separators, set per type below;
//! the topic name is given at topic creation.

use cyclonedds_rs::TopicType;
use serde::{Deserialize, Serialize};
use serde_big_array::BigArray;

/// Impl `TopicType` for a keyless ROS2 type with an explicit DDS type name.
macro_rules! ros2_topic_type {
    ($t:ty, $typename:literal) => {
        impl TopicType for $t {
            fn has_key() -> bool {
                false
            }
            fn key_cdr(&self) -> Vec<u8> {
                Vec::new()
            }
            fn force_md5_keyhash() -> bool {
                false
            }
            fn typename() -> std::ffi::CString {
                std::ffi::CString::new($typename).expect("valid type name")
            }
        }
    };
}

#[derive(Serialize, Deserialize, Clone, Debug, Default, PartialEq)]
pub struct Time {
    pub sec: i32,
    pub nanosec: u32,
}

#[derive(Serialize, Deserialize, Clone, Debug, Default, PartialEq)]
pub struct Header {
    pub stamp: Time,
    pub frame_id: String,
}

#[derive(Serialize, Deserialize, Clone, Debug, Default, PartialEq)]
pub struct StdString {
    pub data: String,
}
ros2_topic_type!(StdString, "std_msgs::msg::dds_::String_");

#[derive(Serialize, Deserialize, Clone, Debug, Default, PartialEq)]
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[derive(Serialize, Deserialize, Clone, Debug, Default, PartialEq)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

#[derive(Serialize, Deserialize, Clone, Debug, Default, PartialEq)]
pub struct Pose {
    pub position: Vector3,
    pub orientation: Quaternion,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct PoseWithCovariance {
    pub pose: Pose,
    #[serde(with = "BigArray")]
    pub covariance: [f64; 36],
}

impl Default for PoseWithCovariance {
    fn default() -> Self {
        Self {
            pose: Pose::default(),
            covariance: [0.0; 36],
        }
    }
}

#[derive(Serialize, Deserialize, Clone, Debug, Default, PartialEq)]
pub struct Twist {
    pub linear: Vector3,
    pub angular: Vector3,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct TwistWithCovariance {
    pub twist: Twist,
    #[serde(with = "BigArray")]
    pub covariance: [f64; 36],
}

impl Default for TwistWithCovariance {
    fn default() -> Self {
        Self {
            twist: Twist::default(),
            covariance: [0.0; 36],
        }
    }
}

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct Odometry {
    pub header: Header,
    pub child_frame_id: String,
    pub pose: PoseWithCovariance,
    pub twist: TwistWithCovariance,
}
ros2_topic_type!(Odometry, "nav_msgs::msg::dds_::Odometry_");

#[derive(Serialize, Deserialize, Clone, Debug, Default, PartialEq)]
pub struct PointField {
    pub name: String,
    pub offset: u32,
    pub datatype: u8,
    pub count: u32,
}

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct PointCloud2 {
    pub header: Header,
    pub height: u32,
    pub width: u32,
    pub fields: Vec<PointField>,
    pub is_bigendian: bool,
    pub point_step: u32,
    pub row_step: u32,
    pub data: Vec<u8>,
    pub is_dense: bool,
}
ros2_topic_type!(PointCloud2, "sensor_msgs::msg::dds_::PointCloud2_");

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct Imu {
    pub header: Header,
    pub orientation: Quaternion,
    pub orientation_covariance: [f64; 9],
    pub angular_velocity: Vector3,
    pub angular_velocity_covariance: [f64; 9],
    pub linear_acceleration: Vector3,
    pub linear_acceleration_covariance: [f64; 9],
}
ros2_topic_type!(Imu, "sensor_msgs::msg::dds_::Imu_");

/// Quaternion is `[w, x, y, z]`, rpy in radians.
#[derive(Serialize, Deserialize, Clone, Debug, Default, PartialEq)]
pub struct IMUState {
    pub quaternion: [f32; 4],
    pub gyroscope: [f32; 3],
    pub accelerometer: [f32; 3],
    pub rpy: [f32; 3],
    pub temperature: u8,
}

/// `current` in mA, `cell_vol` in mV, ntc temperatures in C.
#[derive(Serialize, Deserialize, Clone, Debug, Default, PartialEq)]
pub struct BmsState {
    pub version_high: u8,
    pub version_low: u8,
    pub status: u8,
    pub soc: u8,
    pub current: i32,
    pub cycle: u16,
    pub bq_ntc: [u8; 2],
    pub mcu_ntc: [u8; 2],
    pub cell_vol: [u16; 15],
}

#[derive(Serialize, Deserialize, Clone, Debug, Default, PartialEq)]
pub struct MotorState {
    pub mode: u8,
    pub q: f32,
    pub dq: f32,
    pub ddq: f32,
    pub tau_est: f32,
    pub q_raw: f32,
    pub dq_raw: f32,
    pub ddq_raw: f32,
    pub temperature: u8,
    pub lost: u32,
    pub reserve: [u32; 2],
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct LowState {
    pub head: [u8; 2],
    pub level_flag: u8,
    pub frame_reserve: u8,
    pub sn: [u32; 2],
    pub version: [u32; 2],
    pub bandwidth: u16,
    pub imu_state: IMUState,
    pub motor_state: [MotorState; 20],
    pub bms_state: BmsState,
    pub foot_force: [i16; 4],
    pub foot_force_est: [i16; 4],
    pub tick: u32,
    #[serde(with = "BigArray")]
    pub wireless_remote: [u8; 40],
    pub bit_flag: u8,
    pub adc_reel: f32,
    pub temperature_ntc1: u8,
    pub temperature_ntc2: u8,
    pub power_v: f32,
    pub power_a: f32,
    pub fan_frequency: [u16; 4],
    pub reserve: u32,
    pub crc: u32,
}
ros2_topic_type!(LowState, "unitree_go::msg::dds_::LowState_");

impl Default for LowState {
    fn default() -> Self {
        Self {
            head: [0; 2],
            level_flag: 0,
            frame_reserve: 0,
            sn: [0; 2],
            version: [0; 2],
            bandwidth: 0,
            imu_state: IMUState::default(),
            motor_state: Default::default(),
            bms_state: BmsState::default(),
            foot_force: [0; 4],
            foot_force_est: [0; 4],
            tick: 0,
            wireless_remote: [0; 40],
            bit_flag: 0,
            adc_reel: 0.0,
            temperature_ntc1: 0,
            temperature_ntc2: 0,
            power_v: 0.0,
            power_a: 0.0,
            fan_frequency: [0; 4],
            reserve: 0,
            crc: 0,
        }
    }
}

/// One joint's target: `q` rad, `dq` rad/s, feedforward `tau` Nm, PD gains `kp`/`kd`.
#[derive(Serialize, Deserialize, Clone, Debug, Default, PartialEq)]
pub struct MotorCmd {
    pub mode: u8,
    pub q: f32,
    pub dq: f32,
    pub tau: f32,
    pub kp: f32,
    pub kd: f32,
    pub reserve: [u32; 3],
}

#[derive(Serialize, Deserialize, Clone, Debug, Default, PartialEq)]
pub struct BmsCmd {
    pub off: u8,
    pub reserve: [u8; 3],
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct LowCmd {
    pub head: [u8; 2],
    pub level_flag: u8,
    pub frame_reserve: u8,
    pub sn: [u32; 2],
    pub version: [u32; 2],
    pub bandwidth: u16,
    pub motor_cmd: [MotorCmd; 20],
    pub bms_cmd: BmsCmd,
    #[serde(with = "BigArray")]
    pub wireless_remote: [u8; 40],
    pub led: [u8; 12],
    pub fan: [u8; 2],
    pub gpio: u8,
    pub reserve: u32,
    pub crc: u32,
}
ros2_topic_type!(LowCmd, "unitree_go::msg::dds_::LowCmd_");

impl Default for LowCmd {
    fn default() -> Self {
        Self {
            head: [0; 2],
            level_flag: 0,
            frame_reserve: 0,
            sn: [0; 2],
            version: [0; 2],
            bandwidth: 0,
            motor_cmd: Default::default(),
            bms_cmd: BmsCmd::default(),
            wireless_remote: [0; 40],
            led: [0; 12],
            fan: [0; 2],
            gpio: 0,
            reserve: 0,
            crc: 0,
        }
    }
}

/// The L1's local elevation grid: `data[row * width + col]` is the height in metres of
/// the cell at `origin + (col, row) * resolution` in `frame_id`; ~1e9 marks no data.
#[derive(Serialize, Deserialize, Clone, Debug, Default, PartialEq)]
pub struct HeightMap {
    pub stamp: f64,
    pub frame_id: String,
    pub resolution: f32,
    pub width: u32,
    pub height: u32,
    pub origin: [f32; 2],
    pub data: Vec<f32>,
}
ros2_topic_type!(HeightMap, "unitree_go::msg::dds_::HeightMap_");

/// The remote's sticks in [-1, 1] and a 16-bit button mask.
#[derive(Serialize, Deserialize, Clone, Debug, Default, PartialEq)]
pub struct WirelessController {
    pub lx: f32,
    pub ly: f32,
    pub rx: f32,
    pub ry: f32,
    pub keys: u16,
}
ros2_topic_type!(
    WirelessController,
    "unitree_go::msg::dds_::WirelessController_"
);

#[derive(Serialize, Deserialize, Clone, Debug, Default, PartialEq)]
pub struct RequestIdentity {
    pub id: i64,
    pub api_id: i64,
}

#[derive(Serialize, Deserialize, Clone, Debug, Default, PartialEq)]
pub struct RequestLease {
    pub id: i64,
}

#[derive(Serialize, Deserialize, Clone, Debug, Default, PartialEq)]
pub struct RequestPolicy {
    pub priority: i32,
    pub noreply: bool,
}

#[derive(Serialize, Deserialize, Clone, Debug, Default, PartialEq)]
pub struct RequestHeader {
    pub identity: RequestIdentity,
    pub lease: RequestLease,
    pub policy: RequestPolicy,
}

/// `unitree_api::Request_`, the sport RPC envelope.
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct Request {
    pub header: RequestHeader,
    pub parameter: String,
    pub binary: Vec<u8>,
}
ros2_topic_type!(Request, "unitree_api::msg::dds_::Request_");

impl Request {
    pub fn new(api_id: i64, parameter: impl Into<String>) -> Self {
        Request {
            header: RequestHeader {
                identity: RequestIdentity { id: 0, api_id },
                ..Default::default()
            },
            parameter: parameter.into(),
            binary: Vec::new(),
        }
    }
}

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct ResponseStatus {
    pub code: i32,
}

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct ResponseHeader {
    pub identity: RequestIdentity,
    pub status: ResponseStatus,
}

/// `unitree_api::Response_`, the RPC reply.
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct Response {
    pub header: ResponseHeader,
    pub data: String,
    pub binary: Vec<u8>,
}
ros2_topic_type!(Response, "unitree_api::msg::dds_::Response_");

#[cfg(test)]
mod tests {
    use super::*;
    use cdr::{CdrLe, Infinite};

    fn roundtrip<T: Serialize + serde::de::DeserializeOwned>(v: &T) {
        let bytes = cdr::serialize::<_, _, CdrLe>(v, Infinite).expect("serialize");
        let _back: T = cdr::deserialize(&bytes).expect("deserialize");
    }

    #[test]
    fn cdr_roundtrips() {
        roundtrip(&Request::new(1004, "{}"));
        roundtrip(&StdString { data: "ON".into() });
        roundtrip(&Response::default());
        roundtrip(&LowState::default());
        roundtrip(&WirelessController::default());
        roundtrip(&Imu::default());
        roundtrip(&Odometry::default());
        roundtrip(&PointCloud2 {
            point_step: 16,
            width: 2,
            height: 1,
            fields: vec![PointField {
                name: "x".into(),
                offset: 0,
                datatype: 7,
                count: 1,
            }],
            data: vec![0u8; 32],
            is_dense: true,
            ..Default::default()
        });
        roundtrip(&LowCmd::default());
        roundtrip(&HeightMap::default());
    }

    /// A real sample re-encodes byte for byte: every field and pad lands where the robot
    /// put it (a struct padded to its widest member would shift the u8-first records).
    fn decode_exact<T: Serialize + serde::de::DeserializeOwned>(bytes: &[u8]) -> T {
        let v: T = cdr::deserialize(bytes).expect("deserialize");
        let again = cdr::serialize::<_, _, CdrLe>(&v, Infinite).expect("serialize");
        assert_eq!(again, bytes, "re-encoded sample differs");
        v
    }

    /// rt/lowstate off athens_outdoors_4.mcap, the dog standing under the policy.
    #[test]
    fn real_lowstate_decodes() {
        let s: LowState = decode_exact(include_bytes!("../../tests/lowstate.cdr"));
        let legs = &s.motor_state[..12];
        assert!(legs.iter().all(|m| m.mode == 1), "{legs:?}");
        assert!((20.0..35.0).contains(&s.power_v), "{}", s.power_v);
        for leg in legs.chunks(3) {
            assert!((0.1..2.4).contains(&leg[1].q), "thigh {}", leg[1].q);
            assert!((-2.7..-0.8).contains(&leg[2].q), "calf {}", leg[2].q);
        }
        let q = s.imu_state.quaternion;
        assert!((q.iter().map(|v| v * v).sum::<f32>() - 1.0).abs() < 1e-2);
        assert_ne!(s.crc, 0);
    }

    /// rt/lowcmd from the same recording: the policy's PD targets.
    #[test]
    fn real_lowcmd_decodes() {
        let c: LowCmd = decode_exact(include_bytes!("../../tests/lowcmd.cdr"));
        for m in &c.motor_cmd[..12] {
            assert_eq!((m.mode, m.kp, m.kd), (1, 40.0, 1.0), "{m:?}");
        }
        assert!((-2.7..-0.8).contains(&c.motor_cmd[2].q));
        assert_ne!(c.crc, 0);
    }
}
