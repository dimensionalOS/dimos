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

//! Vendored `sensor_msgs.ImuInfo` until dimos-lcm ships the type.

#[cfg(test)]
use std::io::Write;
use std::io::{self, Cursor, Read};

use lcm_msgs::std_msgs::Header;

#[derive(Debug, Clone, Default, PartialEq)]
pub struct ImuInfo {
    pub header: Header,
    pub gyro_noise_density: f64,
    pub gyro_random_walk: f64,
    pub accel_noise_density: f64,
    pub accel_random_walk: f64,
    pub frequency: f64,
}

impl ImuInfo {
    #[cfg(test)]
    pub const HASH: i64 = 0x437a196cbd6b4e49u64 as i64;
    /// rot1(HASH + rot1(Header::HASH + rot1(Time::HASH))), the lcm recursive hash.
    const PACKED_FINGERPRINT: u64 = 0xE6AA5563F2A33280;

    #[cfg(test)]
    pub fn encode(&self) -> Vec<u8> {
        let mut buf = Vec::new();
        buf.extend_from_slice(&Self::PACKED_FINGERPRINT.to_be_bytes());
        self.encode_one(&mut buf).unwrap();
        buf
    }

    pub fn decode(data: &[u8]) -> io::Result<Self> {
        let mut cursor = Cursor::new(data);
        let mut hash = [0u8; 8];
        cursor.read_exact(&mut hash)?;
        let hash = u64::from_be_bytes(hash);
        if hash != Self::PACKED_FINGERPRINT {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!(
                    "Hash mismatch: expected {:016x}, got {:016x}",
                    Self::PACKED_FINGERPRINT,
                    hash
                ),
            ));
        }
        Self::decode_one(&mut cursor)
    }

    #[cfg(test)]
    fn encode_one<W: Write>(&self, buf: &mut W) -> io::Result<()> {
        self.header.encode_one(buf)?;
        for value in [
            self.gyro_noise_density,
            self.gyro_random_walk,
            self.accel_noise_density,
            self.accel_random_walk,
            self.frequency,
        ] {
            buf.write_all(&value.to_be_bytes())?;
        }
        Ok(())
    }

    fn decode_one<R: Read>(buf: &mut R) -> io::Result<Self> {
        let header = Header::decode_one(buf)?;
        let mut fields = [0f64; 5];
        for value in &mut fields {
            let mut bytes = [0u8; 8];
            buf.read_exact(&mut bytes)?;
            *value = f64::from_be_bytes(bytes);
        }
        let [gyro_noise_density, gyro_random_walk, accel_noise_density, accel_random_walk, frequency] =
            fields;
        Ok(Self {
            header,
            gyro_noise_density,
            gyro_random_walk,
            accel_noise_density,
            accel_random_walk,
            frequency,
        })
    }
}

#[cfg(test)]
mod tests {
    use lcm_msgs::std_msgs::Time;

    use super::*;

    fn rot1(hash: u64) -> u64 {
        hash.rotate_left(1)
    }

    #[test]
    fn fingerprint_matches_the_hash_chain() {
        let header = rot1((Header::HASH as u64).wrapping_add(rot1(Time::HASH as u64)));
        let expected = rot1((ImuInfo::HASH as u64).wrapping_add(header));
        assert_eq!(ImuInfo::PACKED_FINGERPRINT, expected);
    }

    #[test]
    fn roundtrips() {
        let message = ImuInfo {
            header: Header {
                seq: 7,
                stamp: Time { sec: 12, nsec: 34 },
                frame_id: "imu_link".to_string(),
            },
            gyro_noise_density: 1e-3,
            gyro_random_walk: 2e-5,
            accel_noise_density: 3e-3,
            accel_random_walk: 4e-4,
            frequency: 400.0,
        };
        assert_eq!(ImuInfo::decode(&message.encode()).unwrap(), message);
    }
}
