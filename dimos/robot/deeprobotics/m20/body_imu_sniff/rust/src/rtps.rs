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

fn array_at<const N: usize>(bytes: &[u8], at: usize) -> Option<[u8; N]> {
    bytes.get(at..at.checked_add(N)?)?.try_into().ok()
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
    // RTPS header: "RTPS", version, vendor, GUID prefix (bytes 8..20)
    let (Some(b"RTPS"), Some(mut prefix)) = (message.get(..4), array_at::<12>(message, 8)) else {
        return found;
    };
    let mut at = 20;
    while let Some([id, flags]) = array_at::<2>(message, at) {
        // E flag: this submessage's own endianness
        let little = flags & 1 == 1;
        let Some(length) = u16_at(message, at + 2, little) else {
            break;
        };
        // length 0: the submessage runs to the end of the message (RTPS 9.4.5.1.3)
        let body_end = if length == 0 {
            message.len()
        } else {
            (at + 4 + usize::from(length)).min(message.len())
        };
        let body = message.get(at + 4..body_end).unwrap_or_default();
        match id {
            // INFO_SRC: unused (4), version (2), vendor (2), GUID prefix (12)
            SUBMESSAGE_INFO_SRC => {
                if let Some(source) = array_at::<12>(body, 8) {
                    prefix = source;
                }
            }
            // DATA: extraFlags (2), octetsToInlineQos (2), readerId (4), writerId (4), writerSN high (4) low (4)
            SUBMESSAGE_DATA if body.len() >= 20 => {
                let inline_qos = flags & 0x02 != 0;
                let has_payload = flags & 0x0c != 0;
                if let (false, true, Some(to_qos), Some(writer)) = (
                    inline_qos,
                    has_payload,
                    u16_at(body, 2, little),
                    array_at::<4>(body, 8),
                ) {
                    let high = i64::from(u32_at(body, 12, little).unwrap_or(0) as i32);
                    let low = i64::from(u32_at(body, 16, little).unwrap_or(0));
                    if let Some(payload) = body.get(4 + usize::from(to_qos)..) {
                        found.push(Data {
                            prefix,
                            writer,
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
    if data.writer != SPDP_WRITER {
        return None;
    }
    // PL_CDR_BE = 0x0002, PL_CDR_LE = 0x0003
    let little = *data.payload.get(1)? == 0x03;
    let list = data.payload.get(4..)?;
    let mut at = 0;
    while at + 4 <= list.len() {
        let id = u16_at(list, at, little)?;
        let length = usize::from(u16_at(list, at + 2, little)?);
        if id == PID_SENTINEL {
            return None;
        }
        if id == PID_ENTITY_NAME {
            // a CDR string (u32 size, bytes) that must stay inside its own parameter
            let parameter = list.get(at + 4..at + 4 + length)?;
            let size = usize::try_from(u32_at(parameter, 0, little)?).ok()?;
            let text = parameter.get(4..4usize.checked_add(size)?)?;
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
    let cdr = payload.get(4..)?;
    let stamp_sec = u32_at(cdr, 0, little)? as i32;
    let stamp_nanosec = u32_at(cdr, 4, little)?;
    let name_size = usize::try_from(u32_at(cdr, 8, little)?).ok()?;
    let name_end = 12usize.checked_add(name_size)?;
    let name = cdr.get(12..name_end)?;
    let frame_id = String::from_utf8_lossy(name)
        .trim_end_matches('\0')
        .to_string();
    // CDR aligns the doubles to 8 bytes from the start of the body
    let mut at = name_end.checked_add(7)? & !7;
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

/// rt/IMU_DATA's roll, pitch, yaw (deg) from one 56-byte CDR drdds ImuData (MetaType, then nine f32), else None.
pub fn decode_imu_data(payload: &[u8]) -> Option<[f32; 3]> {
    if payload.len() != 56 {
        return None;
    }
    let little = match payload.get(..2)? {
        [0x00, 0x01] => true,
        [0x00, 0x00] => false,
        _ => return None,
    };
    let cdr = payload.get(4..)?;
    let f = |at: usize| u32_at(cdr, at, little).map(f32::from_bits);
    Some([f(16)?, f(20)?, f(24)?])
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
    /// rt/IMU_DATA angles seen by `feed` since the last `take_angles`
    angles: Vec<[f32; 3]>,
}

impl Sniffer {
    pub fn new(participant: &str) -> Self {
        Self {
            participant: participant.to_string(),
            prefix: None,
            last_sequence: HashMap::new(),
            angles: Vec::new(),
            missed: 0,
        }
    }

    pub fn prefix(&self) -> Option<[u8; 12]> {
        self.prefix
    }

    /// The rt/IMU_DATA roll, pitch, yaw (deg) the participant sent in what was fed since the last call.
    pub fn take_angles(&mut self) -> Vec<[f32; 3]> {
        std::mem::take(&mut self.angles)
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
            if let Some(rpy) = decode_imu_data(data.payload) {
                self.last_sequence.insert(data.writer, data.sequence);
                self.angles.push(rpy);
            } else if let Some(sample) = decode_imu(data.payload) {
                if last > 0 {
                    self.missed = self
                        .missed
                        .saturating_add(data.sequence.saturating_sub(last).saturating_sub(1));
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
    fn decodes_the_vendor_imu_data_angles() {
        // a yesense rt/IMU_DATA sample as it went over the wire (2026-11-20 AOS capture, UDP 7441)
        let hex = "000100000000000000000000a3e5ff6ab787e019c05cd9bf27a213bf114a0fc11d73043b1e6aa53940d1973b753cc63dfa7a96be9e5a1d41";
        let bytes: Vec<u8> = (0..hex.len())
            .step_by(2)
            .map(|i| u8::from_str_radix(&hex[i..i + 2], 16).unwrap())
            .collect();
        let [roll, _, _] = decode_imu_data(&bytes).unwrap();
        assert_eq!(roll, -1.6981430053710938);
        assert!(decode_imu_data(&bytes[..55]).is_none());
        assert!(decode_imu_data(IMU).is_none());
    }

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

    #[test]
    fn truncated_or_bogus_input_never_panics() {
        // payloads shorter than the encapsulation header used to slice past their end
        for short in [&[0u8, 1][..], &[0, 0, 0], &[0, 1, 0, 0]] {
            assert!(decode_imu(short).is_none());
        }
        // every prefix of the fixtures
        for message in [IMU, SPDP] {
            for cut in 0..message.len() {
                for data in data_submessages(&message[..cut]) {
                    let _ = participant_name(&data);
                    let _ = decode_imu(data.payload);
                }
            }
        }
        // a huge frame_id length and an entity name longer than its parameter
        assert!(
            decode_imu(&[0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xff, 0xff, 0xff, 0xff]).is_none()
        );
        let data = Data {
            prefix: [0; 12],
            writer: SPDP_WRITER,
            sequence: 1,
            payload: &[0, 3, 0, 0, 0x62, 0, 4, 0, 0xff, 0, 0, 0, b'x'],
        };
        assert_eq!(participant_name(&data), None);
    }

    #[test]
    fn a_wild_sequence_jump_saturates_the_miss_count() {
        let mut sniffer = Sniffer::new("yesense");
        sniffer.feed(SPDP);
        sniffer.missed = i64::MAX - 1;
        let data = data_submessages(IMU);
        let imu = data
            .iter()
            .find(|d| decode_imu(d.payload).is_some())
            .unwrap();
        assert!(imu.sequence > 3, "{}", imu.sequence);
        sniffer.last_sequence.insert(imu.writer, 1);
        assert_eq!(sniffer.feed(IMU).len(), 1);
        assert_eq!(sniffer.missed, i64::MAX);
    }
}
