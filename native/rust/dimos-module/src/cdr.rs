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

// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

//! XCDR1 transport framing around the upstream Serde CDR codec.
use re_cdr::{BigEndian, LittleEndian};
use serde::{de::DeserializeOwned, Serialize};
use std::io;

pub fn encode<T: Serialize>(message: &T) -> io::Result<Vec<u8>> {
    encode_endian(message, true)
}

pub fn encode_endian<T: Serialize>(message: &T, little_endian: bool) -> io::Result<Vec<u8>> {
    let body = if little_endian {
        re_cdr::to_vec::<_, LittleEndian>(message)
    } else {
        re_cdr::to_vec::<_, BigEndian>(message)
    }
    .map_err(|error| io::Error::new(io::ErrorKind::InvalidInput, error.to_string()))?;
    let mut bytes = Vec::with_capacity(4 + body.len());
    bytes.extend_from_slice(&[0, u8::from(little_endian), 0, 0]);
    bytes.extend(body);
    Ok(bytes)
}

pub fn decode<T: DeserializeOwned>(bytes: &[u8]) -> io::Result<T> {
    if bytes.len() < 4 || bytes[0] != 0 || bytes[1] > 1 || bytes[2] != 0 || bytes[3] != 0 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "Expected plain CDR/XCDR1 encapsulation",
        ));
    }
    let (value, consumed) = if bytes[1] == 1 {
        re_cdr::from_bytes::<T, LittleEndian>(&bytes[4..])
    } else {
        re_cdr::from_bytes::<T, BigEndian>(&bytes[4..])
    }
    .map_err(|error| io::Error::new(io::ErrorKind::InvalidData, error.to_string()))?;
    if consumed != bytes.len() - 4 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "Trailing bytes after CDR message",
        ));
    }
    Ok(value)
}

#[cfg(test)]
mod tests {
    use super::*;
    use dimos_generated_messages::{
        builtin_interfaces::msg::time::Time,
        geometry_msgs::msg::{
            point::Point, pose::Pose, pose_stamped::PoseStamped, quaternion::Quaternion,
        },
        std_msgs::msg::{header::Header, int32::Int32},
    };

    #[test]
    fn nested_fields_and_both_byte_orders() {
        let message = PoseStamped {
            header: Header {
                frame_id: "map".into(),
                stamp: Time {
                    sec: 1700000000,
                    nanosec: 123456789,
                },
            },
            pose: Pose {
                position: Point {
                    x: 1.5,
                    y: 0.0,
                    z: 0.0,
                },
                orientation: Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
            },
        };
        assert_eq!(
            decode::<PoseStamped>(&encode(&message).unwrap()).unwrap(),
            message
        );
        assert_eq!(
            decode::<PoseStamped>(&encode_endian(&message, false).unwrap()).unwrap(),
            message
        );
    }

    #[test]
    fn malformed_payload_is_a_decode_error() {
        let mut bytes = encode(&Int32 { data: 1234 }).unwrap();
        assert_eq!(
            decode::<Int32>(&bytes[..bytes.len() - 1])
                .unwrap_err()
                .kind(),
            io::ErrorKind::InvalidData
        );
        bytes.push(0);
        assert_eq!(
            decode::<Int32>(&bytes).unwrap_err().kind(),
            io::ErrorKind::InvalidData
        );
        bytes[0] = 255;
        assert_eq!(
            decode::<Int32>(&bytes).unwrap_err().kind(),
            io::ErrorKind::InvalidData
        );
    }
}
