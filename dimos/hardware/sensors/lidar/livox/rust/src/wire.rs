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

//! Livox SDK2 wire protocol for the Mid-360.
//!
//! Layouts follow `livox_lidar_def.h` (SDK2 1.2.5). All structs on the wire are
//! packed little-endian. Both directions are implemented so the host driver and
//! the virtual device share one definition of every frame.

use std::fmt;
use std::net::Ipv4Addr;

// ---- UDP ports (lidar side sends from, host side listens on) ----
pub const DISCOVERY_PORT: u16 = 56000;
pub const LIDAR_CMD_PORT: u16 = 56100;
pub const LIDAR_PUSH_MSG_PORT: u16 = 56200;
pub const LIDAR_POINT_PORT: u16 = 56300;
pub const LIDAR_IMU_PORT: u16 = 56400;
pub const HOST_CMD_PORT: u16 = 56101;
pub const HOST_PUSH_MSG_PORT: u16 = 56201;
pub const HOST_POINT_PORT: u16 = 56301;
pub const HOST_IMU_PORT: u16 = 56401;

// ---- control plane (LivoxLidarCmdPacket) ----
pub const SOF: u8 = 0xAA;
/// Bytes before `data[]`: sof, version, length, seq, cmd_id, cmd_type,
/// sender_type, rsvd[6], crc16, crc32.
pub const CONTROL_HEADER_LEN: usize = 24;
/// The crc16 covers header bytes [0..CONTROL_CRC16_SPAN).
pub const CONTROL_CRC16_SPAN: usize = 18;

pub const CMD_TYPE_REQUEST: u8 = 0;
pub const CMD_TYPE_ACK: u8 = 1;
pub const SENDER_HOST: u8 = 0;
pub const SENDER_LIDAR: u8 = 1;

/// Command IDs (SDK2 `general_command_handler`).
pub mod cmd_id {
    /// Device discovery / search.
    pub const SEARCH: u16 = 0x0000;
    /// Parameter set (work mode, host net info, IMU enable, ...).
    pub const PARAM_SET: u16 = 0x0100;
    /// Parameter query (`GetInternalInfo`, `QueryFwType`).
    pub const GET_INTERNAL_INFO: u16 = 0x0101;
}

/// Parameter keys (SDK2 `ParamKeyName`).
pub mod param_key {
    pub const POINT_DATA_HOST_IP_CFG: u16 = 0x0006;
    pub const IMU_HOST_IP_CFG: u16 = 0x0007;
    pub const WORK_MODE: u16 = 0x001A;
    pub const IMU_DATA_EN: u16 = 0x001C;
    pub const FW_TYPE: u16 = 0x8010;
}

/// `LivoxLidarWorkMode::kLivoxLidarNormal`.
pub const WORK_MODE_NORMAL: u8 = 0x01;
/// `LivoxLidarDeviceType::kLivoxLidarTypeMid360`.
pub const DEVICE_TYPE_MID360: u8 = 9;
/// Firmware type reported by app firmware (vs loader/upgrade mode).
pub const FW_TYPE_APP: u8 = 1;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WireError {
    TooShort,
    BadSof,
    BadLength,
    BadCrc16,
    BadCrc32,
    BadDataType(u8),
    BadPayload,
}

impl fmt::Display for WireError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            WireError::TooShort => write!(f, "frame too short"),
            WireError::BadSof => write!(f, "bad start-of-frame byte"),
            WireError::BadLength => write!(f, "length field disagrees with frame size"),
            WireError::BadCrc16 => write!(f, "header crc16 mismatch"),
            WireError::BadCrc32 => write!(f, "data crc32 mismatch"),
            WireError::BadDataType(t) => write!(f, "unknown data_type {t}"),
            WireError::BadPayload => write!(f, "payload layout invalid"),
        }
    }
}

impl std::error::Error for WireError {}

// ---- CRCs (CRC-16/IBM-3740 aka CCITT-FALSE over the header, CRC-32/ISO-HDLC over data) ----
const CRC16: crc::Crc<u16> = crc::Crc::<u16>::new(&crc::CRC_16_IBM_3740);
const CRC32: crc::Crc<u32> = crc::Crc::<u32>::new(&crc::CRC_32_ISO_HDLC);

pub fn crc16(data: &[u8]) -> u16 {
    CRC16.checksum(data)
}

pub fn crc32(data: &[u8]) -> u32 {
    CRC32.checksum(data)
}

/// One control-plane frame, borrowed from the receive buffer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ControlFrame<'a> {
    pub seq: u32,
    pub cmd_id: u16,
    pub cmd_type: u8,
    pub sender_type: u8,
    pub data: &'a [u8],
}

impl<'a> ControlFrame<'a> {
    /// Parse and verify a control frame (SOF, length, both CRCs).
    pub fn parse(frame: &'a [u8]) -> Result<Self, WireError> {
        if frame.len() < CONTROL_HEADER_LEN {
            return Err(WireError::TooShort);
        }
        if frame[0] != SOF {
            return Err(WireError::BadSof);
        }
        let length = u16::from_le_bytes([frame[2], frame[3]]) as usize;
        if length != frame.len() {
            return Err(WireError::BadLength);
        }
        let crc16_stated = u16::from_le_bytes([frame[18], frame[19]]);
        if crc16(&frame[..CONTROL_CRC16_SPAN]) != crc16_stated {
            return Err(WireError::BadCrc16);
        }
        let data = &frame[CONTROL_HEADER_LEN..];
        let crc32_stated = u32::from_le_bytes([frame[20], frame[21], frame[22], frame[23]]);
        if crc32(data) != crc32_stated {
            return Err(WireError::BadCrc32);
        }
        Ok(ControlFrame {
            seq: u32::from_le_bytes([frame[4], frame[5], frame[6], frame[7]]),
            cmd_id: u16::from_le_bytes([frame[8], frame[9]]),
            cmd_type: frame[10],
            sender_type: frame[11],
            data,
        })
    }

    pub fn build(&self) -> Vec<u8> {
        build_control(
            self.seq,
            self.cmd_id,
            self.cmd_type,
            self.sender_type,
            self.data,
        )
    }
}

/// Serialize a control frame with both CRCs filled in.
pub fn build_control(seq: u32, cmd_id: u16, cmd_type: u8, sender_type: u8, data: &[u8]) -> Vec<u8> {
    let length = CONTROL_HEADER_LEN + data.len();
    let mut frame = vec![0u8; length];
    frame[0] = SOF;
    frame[1] = 0; // protocol version
    frame[2..4].copy_from_slice(&(length as u16).to_le_bytes());
    frame[4..8].copy_from_slice(&seq.to_le_bytes());
    frame[8..10].copy_from_slice(&cmd_id.to_le_bytes());
    frame[10] = cmd_type;
    frame[11] = sender_type;
    let header_crc = crc16(&frame[..CONTROL_CRC16_SPAN]);
    frame[18..20].copy_from_slice(&header_crc.to_le_bytes());
    frame[20..24].copy_from_slice(&crc32(data).to_le_bytes());
    frame[CONTROL_HEADER_LEN..].copy_from_slice(data);
    frame
}

// ---- control payloads ----

/// Search (0x0000) ACK body: `DetectionData`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DetectionAck {
    pub ret_code: u8,
    pub dev_type: u8,
    /// Treated as a C string by the SDK. Must be NUL-terminated within 16 bytes.
    pub sn: [u8; 16],
    pub lidar_ip: Ipv4Addr,
    pub cmd_port: u16,
}

impl DetectionAck {
    pub const LEN: usize = 24;

    pub fn build(&self) -> Vec<u8> {
        let mut payload = Vec::with_capacity(Self::LEN);
        payload.push(self.ret_code);
        payload.push(self.dev_type);
        payload.extend_from_slice(&self.sn);
        payload.extend_from_slice(&self.lidar_ip.octets());
        payload.extend_from_slice(&self.cmd_port.to_le_bytes());
        payload
    }

    pub fn parse(data: &[u8]) -> Result<Self, WireError> {
        if data.len() < Self::LEN {
            return Err(WireError::BadPayload);
        }
        let mut sn = [0u8; 16];
        sn.copy_from_slice(&data[2..18]);
        Ok(DetectionAck {
            ret_code: data[0],
            dev_type: data[1],
            sn,
            lidar_ip: Ipv4Addr::new(data[18], data[19], data[20], data[21]),
            cmd_port: u16::from_le_bytes([data[22], data[23]]),
        })
    }
}

/// Generic param-set (0x0100) ACK body: `LivoxLidarAsyncControlResponse`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AsyncControlAck {
    pub ret_code: u8,
    pub error_key: u16,
}

impl AsyncControlAck {
    pub const LEN: usize = 3;

    pub fn build(&self) -> Vec<u8> {
        let mut payload = vec![self.ret_code];
        payload.extend_from_slice(&self.error_key.to_le_bytes());
        payload
    }

    pub fn parse(data: &[u8]) -> Result<Self, WireError> {
        if data.len() < Self::LEN {
            return Err(WireError::BadPayload);
        }
        Ok(AsyncControlAck {
            ret_code: data[0],
            error_key: u16::from_le_bytes([data[1], data[2]]),
        })
    }
}

/// One `LivoxLidarKeyValueParam`: key, value length, value bytes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct KeyValue<'a> {
    pub key: u16,
    pub value: &'a [u8],
}

/// Serialize key-value params back to back (key u16, len u16, value).
pub fn build_kv_list(params: &[KeyValue<'_>]) -> Vec<u8> {
    let mut out = Vec::new();
    for param in params {
        out.extend_from_slice(&param.key.to_le_bytes());
        out.extend_from_slice(&(param.value.len() as u16).to_le_bytes());
        out.extend_from_slice(param.value);
    }
    out
}

/// Parse a back-to-back key-value list, e.g. an internal-info response body.
pub fn parse_kv_list(mut data: &[u8]) -> Result<Vec<KeyValue<'_>>, WireError> {
    let mut out = Vec::new();
    while !data.is_empty() {
        if data.len() < 4 {
            return Err(WireError::BadPayload);
        }
        let key = u16::from_le_bytes([data[0], data[1]]);
        let len = u16::from_le_bytes([data[2], data[3]]) as usize;
        if data.len() < 4 + len {
            return Err(WireError::BadPayload);
        }
        out.push(KeyValue {
            key,
            value: &data[4..4 + len],
        });
        data = &data[4 + len..];
    }
    Ok(out)
}

/// Param-set (0x0100) request body: key_num u16, rsvd u16, key-value list
/// (SDK2 `BuildRequest` / `CommandImpl`).
pub fn build_param_set_body(params: &[KeyValue<'_>]) -> Vec<u8> {
    let mut body = Vec::with_capacity(4);
    body.extend_from_slice(&(params.len() as u16).to_le_bytes());
    body.extend_from_slice(&0u16.to_le_bytes());
    body.extend_from_slice(&build_kv_list(params));
    body
}

/// Parse a param-set (0x0100) request body.
pub fn parse_param_set_body(data: &[u8]) -> Result<Vec<KeyValue<'_>>, WireError> {
    if data.len() < 4 {
        return Err(WireError::BadPayload);
    }
    let key_num = u16::from_le_bytes([data[0], data[1]]) as usize;
    let params = parse_kv_list(&data[4..])?;
    if params.len() != key_num {
        return Err(WireError::BadPayload);
    }
    Ok(params)
}

/// Value for the host IP config keys (0x0005-0x0007): `HostIpInfoValue`,
/// ip[4] then host port and lidar port.
pub fn host_ip_config_value(ip: Ipv4Addr, host_port: u16, lidar_port: u16) -> [u8; 8] {
    let mut value = [0u8; 8];
    value[..4].copy_from_slice(&ip.octets());
    value[4..6].copy_from_slice(&host_port.to_le_bytes());
    value[6..8].copy_from_slice(&lidar_port.to_le_bytes());
    value
}

/// GetInternalInfo (0x0101) ACK body: `LivoxLidarDiagInternalInfoResponse`
/// (ret_code u8, param_num u16, key-value list).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct InternalInfoAck<'a> {
    pub ret_code: u8,
    pub params: Vec<KeyValue<'a>>,
}

impl<'a> InternalInfoAck<'a> {
    pub fn build(&self) -> Vec<u8> {
        let mut payload = vec![self.ret_code];
        payload.extend_from_slice(&(self.params.len() as u16).to_le_bytes());
        payload.extend_from_slice(&build_kv_list(&self.params));
        payload
    }

    pub fn parse(data: &'a [u8]) -> Result<Self, WireError> {
        if data.len() < 3 {
            return Err(WireError::BadPayload);
        }
        let param_num = u16::from_le_bytes([data[1], data[2]]) as usize;
        let params = parse_kv_list(&data[3..])?;
        if params.len() != param_num {
            return Err(WireError::BadPayload);
        }
        Ok(InternalInfoAck {
            ret_code: data[0],
            params,
        })
    }
}

// ---- data plane (LivoxLidarEthernetPacket) ----

/// Header bytes before `data[]`: version, length, time_interval, dot_num,
/// udp_cnt, frame_cnt, data_type, time_type, rsvd[12], crc32, timestamp[8].
pub const DATA_HEADER_LEN: usize = 36;
/// Byte offset of the u64 nanosecond timestamp within the packet.
pub const DATA_TIMESTAMP_OFFSET: usize = 28;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum DataType {
    Imu,
    CartesianHigh,
    CartesianLow,
}

impl TryFrom<u8> for DataType {
    type Error = WireError;

    fn try_from(value: u8) -> Result<Self, WireError> {
        match value {
            0x00 => Ok(DataType::Imu),
            0x01 => Ok(DataType::CartesianHigh),
            0x02 => Ok(DataType::CartesianLow),
            other => Err(WireError::BadDataType(other)),
        }
    }
}

/// High-precision cartesian point, 14 bytes on the wire.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PointHigh {
    pub x_mm: i32,
    pub y_mm: i32,
    pub z_mm: i32,
    pub reflectivity: u8,
    pub tag: u8,
}

pub const POINT_HIGH_LEN: usize = 14;

/// Low-precision cartesian point, 8 bytes on the wire.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PointLow {
    pub x_cm: i16,
    pub y_cm: i16,
    pub z_cm: i16,
    pub reflectivity: u8,
    pub tag: u8,
}

pub const POINT_LOW_LEN: usize = 8;

/// One IMU sample: gyro in rad/s, accel in g. 24 bytes on the wire.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuSample {
    pub gyro: [f32; 3],
    pub acc_g: [f32; 3],
}

pub const IMU_SAMPLE_LEN: usize = 24;

/// One data-plane packet, borrowed from the receive buffer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DataPacket<'a> {
    /// Time between points within the packet, in 0.1 microsecond units.
    pub time_interval: u16,
    pub dot_num: u16,
    pub udp_cnt: u16,
    pub frame_cnt: u8,
    pub data_type: DataType,
    pub time_type: u8,
    pub timestamp_ns: u64,
    pub payload: &'a [u8],
}

impl<'a> DataPacket<'a> {
    /// Parse a data packet. The header crc32 field is not verified: real
    /// devices stream at rates where the SDK itself skips it, and captures
    /// from hardware do not reliably fill it.
    pub fn parse(packet: &'a [u8]) -> Result<Self, WireError> {
        if packet.len() < DATA_HEADER_LEN {
            return Err(WireError::TooShort);
        }
        let dot_num = u16::from_le_bytes([packet[5], packet[6]]);
        let data_type = DataType::try_from(packet[10])?;
        let payload = &packet[DATA_HEADER_LEN..];
        let expected = dot_num as usize
            * match data_type {
                DataType::Imu => IMU_SAMPLE_LEN,
                DataType::CartesianHigh => POINT_HIGH_LEN,
                DataType::CartesianLow => POINT_LOW_LEN,
            };
        if payload.len() < expected {
            return Err(WireError::BadLength);
        }
        Ok(DataPacket {
            time_interval: u16::from_le_bytes([packet[3], packet[4]]),
            dot_num,
            udp_cnt: u16::from_le_bytes([packet[7], packet[8]]),
            frame_cnt: packet[9],
            data_type,
            time_type: packet[11],
            timestamp_ns: read_timestamp_ns(packet).unwrap_or(0),
            payload,
        })
    }

    /// Nanoseconds between consecutive points in this packet.
    pub fn point_interval_ns(&self) -> u64 {
        if self.dot_num == 0 {
            return 0;
        }
        u64::from(self.time_interval) * 100 / u64::from(self.dot_num)
    }

    pub fn points_high(&self) -> impl Iterator<Item = PointHigh> + 'a {
        let count = if self.data_type == DataType::CartesianHigh {
            self.dot_num as usize
        } else {
            0
        };
        let payload = self.payload;
        (0..count).map(move |i| {
            let p = &payload[i * POINT_HIGH_LEN..];
            PointHigh {
                x_mm: i32::from_le_bytes([p[0], p[1], p[2], p[3]]),
                y_mm: i32::from_le_bytes([p[4], p[5], p[6], p[7]]),
                z_mm: i32::from_le_bytes([p[8], p[9], p[10], p[11]]),
                reflectivity: p[12],
                tag: p[13],
            }
        })
    }

    pub fn points_low(&self) -> impl Iterator<Item = PointLow> + 'a {
        let count = if self.data_type == DataType::CartesianLow {
            self.dot_num as usize
        } else {
            0
        };
        let payload = self.payload;
        (0..count).map(move |i| {
            let p = &payload[i * POINT_LOW_LEN..];
            PointLow {
                x_cm: i16::from_le_bytes([p[0], p[1]]),
                y_cm: i16::from_le_bytes([p[2], p[3]]),
                z_cm: i16::from_le_bytes([p[4], p[5]]),
                reflectivity: p[6],
                tag: p[7],
            }
        })
    }

    pub fn imu_samples(&self) -> impl Iterator<Item = ImuSample> + 'a {
        let count = if self.data_type == DataType::Imu {
            self.dot_num as usize
        } else {
            0
        };
        let payload = self.payload;
        (0..count).map(move |i| {
            let p = &payload[i * IMU_SAMPLE_LEN..];
            let f = |o: usize| f32::from_le_bytes([p[o], p[o + 1], p[o + 2], p[o + 3]]);
            ImuSample {
                gyro: [f(0), f(4), f(8)],
                acc_g: [f(12), f(16), f(20)],
            }
        })
    }

    pub fn build(&self) -> Vec<u8> {
        let length = DATA_HEADER_LEN + self.payload.len();
        let mut packet = vec![0u8; length];
        packet[0] = 0; // packet version
        packet[1..3].copy_from_slice(&(length as u16).to_le_bytes());
        packet[3..5].copy_from_slice(&self.time_interval.to_le_bytes());
        packet[5..7].copy_from_slice(&self.dot_num.to_le_bytes());
        packet[7..9].copy_from_slice(&self.udp_cnt.to_le_bytes());
        packet[9] = self.frame_cnt;
        packet[10] = match self.data_type {
            DataType::Imu => 0x00,
            DataType::CartesianHigh => 0x01,
            DataType::CartesianLow => 0x02,
        };
        packet[11] = self.time_type;
        packet[24..28].copy_from_slice(&crc32(self.payload).to_le_bytes());
        packet[DATA_TIMESTAMP_OFFSET..DATA_TIMESTAMP_OFFSET + 8]
            .copy_from_slice(&self.timestamp_ns.to_le_bytes());
        packet[DATA_HEADER_LEN..].copy_from_slice(self.payload);
        packet
    }
}

/// Read the u64 timestamp from a raw data-plane packet.
pub fn read_timestamp_ns(packet: &[u8]) -> Option<u64> {
    let bytes = packet.get(DATA_TIMESTAMP_OFFSET..DATA_TIMESTAMP_OFFSET + 8)?;
    Some(u64::from_le_bytes(bytes.try_into().unwrap()))
}

/// Shift the u64 timestamp of a raw data-plane packet in place.
pub fn shift_timestamp_ns(packet: &mut [u8], shift: u64) {
    if let Some(orig) = read_timestamp_ns(packet) {
        packet[DATA_TIMESTAMP_OFFSET..DATA_TIMESTAMP_OFFSET + 8]
            .copy_from_slice(&orig.wrapping_add(shift).to_le_bytes());
    }
}

/// Serialize high-precision points into a data payload for `DataPacket::build`.
pub fn build_points_high(points: &[PointHigh]) -> Vec<u8> {
    let mut out = Vec::with_capacity(points.len() * POINT_HIGH_LEN);
    for p in points {
        out.extend_from_slice(&p.x_mm.to_le_bytes());
        out.extend_from_slice(&p.y_mm.to_le_bytes());
        out.extend_from_slice(&p.z_mm.to_le_bytes());
        out.push(p.reflectivity);
        out.push(p.tag);
    }
    out
}

/// Serialize low-precision points into a data payload for `DataPacket::build`.
pub fn build_points_low(points: &[PointLow]) -> Vec<u8> {
    let mut out = Vec::with_capacity(points.len() * POINT_LOW_LEN);
    for p in points {
        out.extend_from_slice(&p.x_cm.to_le_bytes());
        out.extend_from_slice(&p.y_cm.to_le_bytes());
        out.extend_from_slice(&p.z_cm.to_le_bytes());
        out.push(p.reflectivity);
        out.push(p.tag);
    }
    out
}

/// Serialize IMU samples into a data payload for `DataPacket::build`.
pub fn build_imu_samples(samples: &[ImuSample]) -> Vec<u8> {
    let mut out = Vec::with_capacity(samples.len() * IMU_SAMPLE_LEN);
    for s in samples {
        for v in s.gyro.iter().chain(s.acc_g.iter()) {
            out.extend_from_slice(&v.to_le_bytes());
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc_check_values() {
        // Catalog check values for CRC-16/IBM-3740 and CRC-32/ISO-HDLC.
        assert_eq!(crc16(b"123456789"), 0x29B1);
        assert_eq!(crc32(b"123456789"), 0xCBF43926);
    }

    #[test]
    fn control_frame_round_trip() {
        let data = [1u8, 2, 3, 4, 5];
        let frame = build_control(42, cmd_id::PARAM_SET, CMD_TYPE_REQUEST, SENDER_HOST, &data);
        let parsed = ControlFrame::parse(&frame).unwrap();
        assert_eq!(parsed.seq, 42);
        assert_eq!(parsed.cmd_id, cmd_id::PARAM_SET);
        assert_eq!(parsed.cmd_type, CMD_TYPE_REQUEST);
        assert_eq!(parsed.sender_type, SENDER_HOST);
        assert_eq!(parsed.data, &data);
        assert_eq!(parsed.build(), frame);
    }

    #[test]
    fn control_frame_rejects_corruption() {
        let frame = build_control(1, cmd_id::SEARCH, CMD_TYPE_ACK, SENDER_LIDAR, &[9, 9]);
        assert_eq!(ControlFrame::parse(&frame[..10]), Err(WireError::TooShort));

        let mut bad_sof = frame.clone();
        bad_sof[0] = 0xAB;
        assert_eq!(ControlFrame::parse(&bad_sof), Err(WireError::BadSof));

        let mut bad_header = frame.clone();
        bad_header[4] ^= 0xFF; // seq is inside the crc16 span
        assert_eq!(ControlFrame::parse(&bad_header), Err(WireError::BadCrc16));

        let mut bad_data = frame.clone();
        *bad_data.last_mut().unwrap() ^= 0xFF;
        assert_eq!(ControlFrame::parse(&bad_data), Err(WireError::BadCrc32));

        let mut bad_length = frame.clone();
        bad_length.push(0);
        assert_eq!(ControlFrame::parse(&bad_length), Err(WireError::BadLength));
    }

    #[test]
    fn detection_ack_round_trip() {
        let mut sn = [0u8; 16];
        sn[..10].copy_from_slice(b"FAKEMID360");
        let ack = DetectionAck {
            ret_code: 0,
            dev_type: DEVICE_TYPE_MID360,
            sn,
            lidar_ip: Ipv4Addr::new(192, 168, 1, 171),
            cmd_port: LIDAR_CMD_PORT,
        };
        let bytes = ack.build();
        assert_eq!(bytes.len(), DetectionAck::LEN);
        assert_eq!(DetectionAck::parse(&bytes).unwrap(), ack);
    }

    #[test]
    fn async_control_ack_round_trip() {
        let ack = AsyncControlAck {
            ret_code: 0,
            error_key: 0x001A,
        };
        assert_eq!(AsyncControlAck::parse(&ack.build()).unwrap(), ack);
    }

    #[test]
    fn internal_info_round_trip() {
        let ack = InternalInfoAck {
            ret_code: 0,
            params: vec![KeyValue {
                key: param_key::FW_TYPE,
                value: &[FW_TYPE_APP],
            }],
        };
        let bytes = ack.build();
        let parsed = InternalInfoAck::parse(&bytes).unwrap();
        assert_eq!(parsed, ack);
    }

    #[test]
    fn param_set_body_round_trip() {
        let value = host_ip_config_value(Ipv4Addr::new(192, 168, 1, 5), HOST_POINT_PORT, 56300);
        let params = [
            KeyValue {
                key: param_key::POINT_DATA_HOST_IP_CFG,
                value: &value,
            },
            KeyValue {
                key: param_key::WORK_MODE,
                value: &[WORK_MODE_NORMAL],
            },
        ];
        let body = build_param_set_body(&params);
        assert_eq!(&body[..4], &[2, 0, 0, 0]); // key_num = 2, rsvd = 0
        let parsed = parse_param_set_body(&body).unwrap();
        assert_eq!(parsed, params);
        assert_eq!(parsed[0].value[..4], [192, 168, 1, 5]);
        assert_eq!(
            parse_param_set_body(&body[..body.len() - 1]),
            Err(WireError::BadPayload)
        );
    }

    #[test]
    fn kv_list_rejects_truncation() {
        let bytes = build_kv_list(&[KeyValue {
            key: param_key::WORK_MODE,
            value: &[WORK_MODE_NORMAL],
        }]);
        assert!(parse_kv_list(&bytes).is_ok());
        assert_eq!(
            parse_kv_list(&bytes[..bytes.len() - 1]),
            Err(WireError::BadPayload)
        );
    }

    #[test]
    fn point_packet_round_trip() {
        let points = [
            PointHigh {
                x_mm: 1500,
                y_mm: -2500,
                z_mm: 300,
                reflectivity: 200,
                tag: 0x10,
            },
            PointHigh {
                x_mm: -1,
                y_mm: 2,
                z_mm: 3,
                reflectivity: 0,
                tag: 0,
            },
        ];
        let payload = build_points_high(&points);
        let packet = DataPacket {
            time_interval: 1000,
            dot_num: points.len() as u16,
            udp_cnt: 7,
            frame_cnt: 3,
            data_type: DataType::CartesianHigh,
            time_type: 0,
            timestamp_ns: 1_700_000_000_123_456_789,
            payload: &payload,
        };
        let bytes = packet.build();
        let parsed = DataPacket::parse(&bytes).unwrap();
        assert_eq!(parsed.dot_num, 2);
        assert_eq!(parsed.timestamp_ns, packet.timestamp_ns);
        assert_eq!(parsed.points_high().collect::<Vec<_>>(), points);
        assert_eq!(parsed.points_low().count(), 0);
        assert_eq!(parsed.imu_samples().count(), 0);
        // 100 us across the packet, 2 points -> 50 us between points.
        assert_eq!(parsed.point_interval_ns(), 50_000);
    }

    #[test]
    fn low_precision_packet_round_trip() {
        let points = [PointLow {
            x_cm: 150,
            y_cm: -250,
            z_cm: 30,
            reflectivity: 128,
            tag: 1,
        }];
        let payload = build_points_low(&points);
        let packet = DataPacket {
            time_interval: 1000,
            dot_num: 1,
            udp_cnt: 0,
            frame_cnt: 0,
            data_type: DataType::CartesianLow,
            time_type: 0,
            timestamp_ns: 5,
            payload: &payload,
        };
        let parsed_bytes = packet.build();
        let parsed = DataPacket::parse(&parsed_bytes).unwrap();
        assert_eq!(parsed.points_low().collect::<Vec<_>>(), points);
    }

    #[test]
    fn imu_packet_round_trip() {
        let samples = [ImuSample {
            gyro: [0.01, -0.02, 0.03],
            acc_g: [0.0, 0.0, 1.0],
        }];
        let payload = build_imu_samples(&samples);
        let packet = DataPacket {
            time_interval: 0,
            dot_num: 1,
            udp_cnt: 0,
            frame_cnt: 0,
            data_type: DataType::Imu,
            time_type: 0,
            timestamp_ns: 99,
            payload: &payload,
        };
        let bytes = packet.build();
        let parsed = DataPacket::parse(&bytes).unwrap();
        assert_eq!(parsed.imu_samples().collect::<Vec<_>>(), samples);
    }

    #[test]
    fn data_packet_rejects_short_payload() {
        let payload = build_points_high(&[PointHigh {
            x_mm: 0,
            y_mm: 0,
            z_mm: 0,
            reflectivity: 0,
            tag: 0,
        }]);
        let packet = DataPacket {
            time_interval: 0,
            dot_num: 2, // claims two points, payload has one
            udp_cnt: 0,
            frame_cnt: 0,
            data_type: DataType::CartesianHigh,
            time_type: 0,
            timestamp_ns: 0,
            payload: &payload,
        };
        assert_eq!(
            DataPacket::parse(&packet.build()),
            Err(WireError::BadLength)
        );
    }

    #[test]
    fn timestamp_shift_in_place() {
        let packet = DataPacket {
            time_interval: 0,
            dot_num: 0,
            udp_cnt: 0,
            frame_cnt: 0,
            data_type: DataType::CartesianHigh,
            time_type: 0,
            timestamp_ns: 100,
            payload: &[],
        };
        let mut bytes = packet.build();
        shift_timestamp_ns(&mut bytes, 50);
        assert_eq!(read_timestamp_ns(&bytes), Some(150));
        let reparsed = DataPacket::parse(&bytes).unwrap();
        assert_eq!(reparsed.timestamp_ns, 150);
    }
}
