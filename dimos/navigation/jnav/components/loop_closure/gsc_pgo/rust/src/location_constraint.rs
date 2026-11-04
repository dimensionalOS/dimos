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

//! `jnav.LocationConstraint` (dimos/navigation/jnav/msgs/LocationConstraint.py), the landmark sighting GSC takes: its hand-packed big-endian wire format, decoded.

use std::io;

/// A sighting of location `to_id` from `frame_id`; a later one with the same non-empty `constraint_instance_id` replaces it. `covariance` is 6x6 row-major in (rotation, position) order; a non-positive variance is unknown.
#[derive(Clone, Debug, PartialEq)]
pub struct LocationConstraint {
    pub ts: f64,
    pub to_id: String,
    pub frame_id: String,
    pub constraint_instance_id: String,
    /// Position xyz, then the orientation quaternion xyzw (all zero = unknown).
    pub position: [f64; 3],
    pub orientation_xyzw: [f64; 4],
    pub covariance: [f64; 36],
}

/// Reads big-endian fields off the front of a buffer.
struct Reader<'a>(&'a [u8]);

impl Reader<'_> {
    fn take(&mut self, n: usize) -> io::Result<&[u8]> {
        if self.0.len() < n {
            return Err(io::Error::new(
                io::ErrorKind::UnexpectedEof,
                "LocationConstraint too short",
            ));
        }
        let (head, rest) = self.0.split_at(n);
        self.0 = rest;
        Ok(head)
    }
    fn f64(&mut self) -> io::Result<f64> {
        Ok(f64::from_be_bytes(
            self.take(8)?.try_into().expect("8 bytes"),
        ))
    }
    fn text(&mut self) -> io::Result<String> {
        let length = u32::from_be_bytes(self.take(4)?.try_into().expect("4 bytes")) as usize;
        String::from_utf8(self.take(length)?.to_vec())
            .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))
    }
}

impl LocationConstraint {
    /// ts, to_id, frame_id, constraint_instance_id, pose (xyz, xyzw), covariance; trailing fields (map_id, kind) ignored.
    pub fn decode(bytes: &[u8]) -> io::Result<Self> {
        let mut reader = Reader(bytes);
        let ts = reader.f64()?;
        let (to_id, frame_id, constraint_instance_id) =
            (reader.text()?, reader.text()?, reader.text()?);
        let mut pose = [0.0; 7];
        for value in pose.iter_mut() {
            *value = reader.f64()?;
        }
        let mut covariance = [0.0; 36];
        for value in covariance.iter_mut() {
            *value = reader.f64()?;
        }
        Ok(Self {
            ts,
            to_id,
            frame_id,
            constraint_instance_id,
            position: [pose[0], pose[1], pose[2]],
            orientation_xyzw: [pose[3], pose[4], pose[5], pose[6]],
            covariance,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::LocationConstraint;

    /// The bytes LocationConstraint.lcm_encode packs (with its map_id / kind tail) decode back.
    #[test]
    fn decodes_the_python_encoding() {
        let mut bytes = 12.5f64.to_be_bytes().to_vec();
        for text in ["apriltag/13", "base_link", "apriltag/13#4"] {
            bytes.extend((text.len() as u32).to_be_bytes());
            bytes.extend(text.as_bytes());
        }
        for value in (0..43).map(|i| i as f64) {
            bytes.extend(value.to_be_bytes());
        }
        bytes.extend(3u32.to_be_bytes());
        bytes.extend(b"map");
        let decoded = LocationConstraint::decode(&bytes).expect("decodes");
        assert_eq!(decoded.ts, 12.5);
        assert_eq!(decoded.to_id, "apriltag/13");
        assert_eq!(decoded.frame_id, "base_link");
        assert_eq!(decoded.constraint_instance_id, "apriltag/13#4");
        assert_eq!(decoded.position, [0.0, 1.0, 2.0]);
        assert_eq!(decoded.orientation_xyzw, [3.0, 4.0, 5.0, 6.0]);
        assert_eq!(decoded.covariance[35], 42.0);
        assert!(LocationConstraint::decode(&bytes[..40]).is_err());
    }
}
