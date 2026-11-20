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

//! Just enough RTPS to pull one participant's sensor_msgs/Imu samples out of captured UDP payloads: DATA
//! submessages, the SPDP participant name, and the CDR Imu layout.

use std::collections::HashMap;

const SUBMESSAGE_INFO_SRC: u8 = 0x0c;
const SUBMESSAGE_DATA: u8 = 0x15;
/// SPDP's built-in participant writer.
const SPDP_WRITER: [u8; 4] = [0x00, 0x01, 0x00, 0xc2];
const PID_SENTINEL: u16 = 0x0001;
const PID_ENTITY_NAME: u16 = 0x0062;

/// One DATA submessage: its writer and sequence number, and the serialized sample (encapsulation header included).
#[derive(Debug, Clone, PartialEq)]
pub struct Data<'a> {
    pub prefix: [u8; 12],
    pub writer: [u8; 4],
    pub sequence: i64,
    pub payload: &'a [u8],
}

fn u16_at(bytes: &[u8], at: usize, little: bool) -> Option<u16> {
    let b: [u8; 2] = bytes.get(at..at + 2)?.try_into().ok()?;
    Some(if little {
        u16::from_le_bytes(b)
    } else {
        u16::from_be_bytes(b)
    })
}

fn u32_at(bytes: &[u8], at: usize, little: bool) -> Option<u32> {
    let b: [u8; 4] = bytes.get(at..at + 4)?.try_into().ok()?;
    Some(if little {
        u32::from_le_bytes(b)
    } else {
        u32::from_be_bytes(b)
    })
}

fn f64_at(bytes: &[u8], at: usize, little: bool) -> Option<f64> {
    let b: [u8; 8] = bytes.get(at..at + 8)?.try_into().ok()?;
    Some(if little {
        f64::from_le_bytes(b)
    } else {
        f64::from_be_bytes(b)
    })
}

/// Every DATA submessage in one RTPS message (an empty list for anything that is not RTPS).
pub fn data_submessages(message: &[u8]) -> Vec<Data<'_>> {
    let mut found = Vec::new();
    if message.len() < 20 || &message[..4] != b"RTPS" {
        return found;
    }
    let mut prefix: [u8; 12] = message[8..20].try_into().unwrap();
    let mut at = 20;
    while at + 4 <= message.len() {
        let (id, flags) = (message[at], message[at + 1]);
        let little = flags & 1 == 1;
        let Some(length) = u16_at(message, at + 2, little) else {
            break;
        };
        let body_end = if length == 0 {
            message.len()
        } else {
            (at + 4 + length as usize).min(message.len())
        };
        let body = &message[at + 4..body_end];
        match id {
            SUBMESSAGE_INFO_SRC if body.len() >= 20 => prefix = body[8..20].try_into().unwrap(),
            SUBMESSAGE_DATA if body.len() >= 20 => {
                let inline_qos = flags & 0x02 != 0;
                let has_payload = flags & 0x0c != 0;
                if let (false, true, Some(to_qos)) =
                    (inline_qos, has_payload, u16_at(body, 2, little))
                {
                    let high = u32_at(body, 12, little).unwrap_or(0) as i32 as i64;
                    let low = u32_at(body, 16, little).unwrap_or(0) as i64;
                    if let Some(payload) = body.get(4 + to_qos as usize..) {
                        found.push(Data {
                            prefix,
                            writer: body[8..12].try_into().unwrap(),
                            sequence: (high << 32) | low,
                            payload,
                        });
                    }
                }
            }
            _ => {}
        }
        if length == 0 {
            break;
        }
        at = body_end;
    }
    found
}

/// The participant name (PID_ENTITY_NAME) an SPDP DATA carries.
pub fn participant_name(data: &Data) -> Option<String> {
    if data.writer != SPDP_WRITER || data.payload.len() < 4 {
        return None;
    }
    // PL_CDR_BE = 0x0002, PL_CDR_LE = 0x0003
    let little = data.payload[1] == 0x03;
    let list = &data.payload[4..];
    let mut at = 0;
    while at + 4 <= list.len() {
        let id = u16_at(list, at, little)?;
        let length = u16_at(list, at + 2, little)? as usize;
        if id == PID_SENTINEL {
            return None;
        }
        if id == PID_ENTITY_NAME {
            let size = u32_at(list, at + 4, little)? as usize;
            let text = list.get(at + 8..at + 8 + size)?;
            return Some(
                String::from_utf8_lossy(text)
                    .trim_end_matches('\0')
                    .to_string(),
            );
        }
        at += 4 + length;
    }
    None
}

/// A sensor_msgs/Imu sample as the vendor serialized it.
#[derive(Debug, Clone, PartialEq)]
pub struct ImuSample {
    pub stamp_sec: i32,
    pub stamp_nanosec: u32,
    pub frame_id: String,
    pub orientation_xyzw: [f64; 4],
    pub orientation_covariance: [f64; 9],
    pub angular_velocity: [f64; 3],
    pub angular_velocity_covariance: [f64; 9],
    pub linear_acceleration: [f64; 3],
    pub linear_acceleration_covariance: [f64; 9],
}

/// Decode a CDR sensor_msgs/Imu (with its encapsulation header); None unless the payload is exactly one.
pub fn decode_imu(payload: &[u8]) -> Option<ImuSample> {
    // CDR_BE = 0x0000, CDR_LE = 0x0001
    let little = match payload.get(..2)? {
        [0x00, 0x01] => true,
        [0x00, 0x00] => false,
        _ => return None,
    };
    let cdr = &payload[4..];
    let stamp_sec = u32_at(cdr, 0, little)? as i32;
    let stamp_nanosec = u32_at(cdr, 4, little)?;
    let name_size = u32_at(cdr, 8, little)? as usize;
    let name = cdr.get(12..12 + name_size)?;
    let frame_id = String::from_utf8_lossy(name)
        .trim_end_matches('\0')
        .to_string();
    let mut at = (12 + name_size + 7) & !7;
    let mut doubles = |n: usize| -> Option<Vec<f64>> {
        let values = (0..n)
            .map(|i| f64_at(cdr, at + 8 * i, little))
            .collect::<Option<Vec<_>>>()?;
        at += 8 * n;
        Some(values)
    };
    let orientation = doubles(4)?;
    let orientation_covariance = doubles(9)?;
    let angular_velocity = doubles(3)?;
    let angular_velocity_covariance = doubles(9)?;
    let linear_acceleration = doubles(3)?;
    let linear_acceleration_covariance = doubles(9)?;
    // CDR pads the end to 4 bytes at most, so anything longer is another type
    if cdr.len() < at || cdr.len() - at > 3 {
        return None;
    }
    Some(ImuSample {
        stamp_sec,
        stamp_nanosec,
        frame_id,
        orientation_xyzw: orientation.try_into().ok()?,
        orientation_covariance: orientation_covariance.try_into().ok()?,
        angular_velocity: angular_velocity.try_into().ok()?,
        angular_velocity_covariance: angular_velocity_covariance.try_into().ok()?,
        linear_acceleration: linear_acceleration.try_into().ok()?,
        linear_acceleration_covariance: linear_acceleration_covariance.try_into().ok()?,
    })
}

/// Learns the named participant's GUID prefix from its SPDP announcements, then yields each of its Imu samples once
/// (a sample sent to several readers arrives once per reader).
#[derive(Debug)]
pub struct Sniffer {
    participant: String,
    prefix: Option<[u8; 12]>,
    last_sequence: HashMap<[u8; 4], i64>,
    /// Sequence numbers skipped by the participant's Imu writers since it was found (samples never captured).
    pub missed: i64,
}

impl Sniffer {
    pub fn new(participant: &str) -> Self {
        Self {
            participant: participant.to_string(),
            prefix: None,
            last_sequence: HashMap::new(),
            missed: 0,
        }
    }

    pub fn prefix(&self) -> Option<[u8; 12]> {
        self.prefix
    }

    /// The new Imu samples in one captured UDP payload.
    pub fn feed(&mut self, message: &[u8]) -> Vec<ImuSample> {
        let mut samples = Vec::new();
        for data in data_submessages(message) {
            if participant_name(&data).as_deref() == Some(self.participant.as_str())
                && self.prefix != Some(data.prefix)
            {
                // a restarted participant has a new prefix and restarts its sequence numbers
                self.prefix = Some(data.prefix);
                self.last_sequence.clear();
            }
            if Some(data.prefix) != self.prefix {
                continue;
            }
            let last = self.last_sequence.get(&data.writer).copied().unwrap_or(0);
            if data.sequence <= last {
                continue;
            }
            if let Some(sample) = decode_imu(data.payload) {
                if last > 0 {
                    self.missed += data.sequence - last - 1;
                }
                self.last_sequence.insert(data.writer, data.sequence);
                samples.push(sample);
            }
        }
        samples
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const IMU: &[u8] = include_bytes!("../tests/fixtures/imu.bin");
    const SPDP: &[u8] = include_bytes!("../tests/fixtures/spdp.bin");

    #[test]
    fn spdp_names_the_yesense_participant() {
        let spdp = data_submessages(SPDP);
        assert_eq!(
            spdp.iter().filter_map(participant_name).collect::<Vec<_>>(),
            vec!["yesense".to_string()]
        );
    }

    #[test]
    fn decodes_the_vendor_imu_sample() {
        let data = data_submessages(IMU);
        let sample = data
            .iter()
            .find_map(|d| decode_imu(d.payload))
            .expect("an Imu sample");
        assert_eq!(sample.frame_id, "");
        assert!(sample.stamp_sec > 1_795_000_000);
        let norm = sample
            .orientation_xyzw
            .iter()
            .map(|v| v * v)
            .sum::<f64>()
            .sqrt();
        assert!((norm - 1.0).abs() < 1e-6, "{norm}");
        let g = sample
            .linear_acceleration
            .iter()
            .map(|v| v * v)
            .sum::<f64>()
            .sqrt();
        assert!((g - 9.8).abs() < 0.2, "{g}");
    }

    #[test]
    fn only_after_spdp_and_each_sample_once() {
        let mut sniffer = Sniffer::new("yesense");
        assert!(
            sniffer.feed(IMU).is_empty(),
            "nothing before the participant is known"
        );
        assert!(sniffer.feed(SPDP).is_empty());
        assert_eq!(sniffer.feed(IMU).len(), 1);
        assert!(
            sniffer.feed(IMU).is_empty(),
            "the copy sent to a second reader is dropped"
        );
        assert!(Sniffer::new("someone_else").feed(SPDP).is_empty());
    }

    #[test]
    fn other_payloads_are_not_imu() {
        assert!(decode_imu(&[0, 1, 0, 0, 1, 2, 3]).is_none());
        assert!(data_submessages(b"not rtps").is_empty());
    }
}
