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

//! Reading a `PointCloud2` off the wire. Every wire count is signed and untrusted, so each
//! is range-checked before it indexes; a malformed cloud is an `ExtractError`, never a panic.

use lcm_msgs::sensor_msgs::{PointCloud2, PointField};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExtractError(pub &'static str);

impl std::fmt::Display for ExtractError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.0)
    }
}

impl std::error::Error for ExtractError {}

fn count(v: i32, what: &'static str) -> Result<usize, ExtractError> {
    usize::try_from(v).map_err(|_| ExtractError(what))
}

/// The finite xyz points of a little-endian float32 cloud, in order.
pub fn extract_xyz(msg: &PointCloud2) -> Result<Vec<[f32; 3]>, ExtractError> {
    let mut offsets: [Option<usize>; 3] = [None; 3];
    for f in &msg.fields {
        if f.datatype != PointField::FLOAT32 as u8 {
            continue;
        }
        let slot = match f.name.as_str() {
            "x" => 0,
            "y" => 1,
            "z" => 2,
            _ => continue,
        };
        offsets[slot] = Some(count(f.offset, "negative field offset")?);
    }
    let [Some(xo), Some(yo), Some(zo)] = offsets else {
        return Err(ExtractError("missing a float32 x/y/z field"));
    };
    if msg.is_bigendian {
        return Err(ExtractError("big-endian point data not supported"));
    }

    let step = count(msg.point_step, "negative point_step")?;
    if step == 0 {
        return Err(ExtractError("point_step is 0"));
    }
    let n = count(msg.width, "negative width")?
        .checked_mul(count(msg.height, "negative height")?)
        .ok_or(ExtractError("width*height overflows"))?;
    let needed = n
        .checked_mul(step)
        .ok_or(ExtractError("width*height*point_step overflows"))?;
    if msg.data.len() < needed {
        return Err(ExtractError(
            "data buffer shorter than width*height*point_step",
        ));
    }
    for off in [xo, yo, zo] {
        if off.checked_add(4).is_none_or(|end| end > step) {
            return Err(ExtractError(
                "xyz field offsets do not fit within point_step",
            ));
        }
    }

    let mut out = Vec::with_capacity(n);
    for base in (0..needed).step_by(step) {
        let x = read_f32_le(&msg.data, base + xo);
        let y = read_f32_le(&msg.data, base + yo);
        let z = read_f32_le(&msg.data, base + zo);
        if x.is_finite() && y.is_finite() && z.is_finite() {
            out.push([x, y, z]);
        }
    }
    Ok(out)
}

#[inline]
fn read_f32_le(buf: &[u8], off: usize) -> f32 {
    let bytes: [u8; 4] = buf[off..off + 4]
        .try_into()
        .expect("bounds checked by caller");
    f32::from_le_bytes(bytes)
}

#[cfg(test)]
mod tests {
    use super::*;
    use lcm_msgs::std_msgs::Header;

    /// An xyz float32 cloud, the shape the raytracer publishes.
    fn cloud_of(points: &[[f32; 3]]) -> PointCloud2 {
        let mut data = Vec::with_capacity(points.len() * 12);
        for p in points {
            for v in p {
                data.extend_from_slice(&v.to_le_bytes());
            }
        }
        let field = |name: &str, off: i32| PointField {
            name: name.into(),
            offset: off,
            datatype: PointField::FLOAT32 as u8,
            count: 1,
        };
        PointCloud2 {
            header: Header::default(),
            height: 1,
            width: points.len() as i32,
            fields: vec![field("x", 0), field("y", 4), field("z", 8)],
            is_bigendian: false,
            point_step: 12,
            row_step: 12 * points.len() as i32,
            data,
            is_dense: true,
        }
    }

    #[test]
    fn drops_non_finite_points_and_moves_nothing() {
        let cloud = cloud_of(&[[1.0, 2.0, 0.3], [f32::NAN, 0.0, 0.0], [0.0, 0.0, 0.0]]);
        assert_eq!(
            extract_xyz(&cloud).unwrap(),
            vec![[1.0, 2.0, 0.3], [0.0, 0.0, 0.0]]
        );
    }

    #[test]
    fn refuses_a_cloud_it_cannot_read() {
        let mut cloud = cloud_of(&[[0.0; 3]]);
        cloud.fields.remove(2); // no z
        assert!(extract_xyz(&cloud).is_err());

        let mut cloud = cloud_of(&[[0.0; 3]]);
        cloud.is_bigendian = true;
        assert!(extract_xyz(&cloud).is_err());

        let mut cloud = cloud_of(&[[0.0; 3]]);
        cloud.width = 99; // claims more points than the buffer holds
        assert!(extract_xyz(&cloud).is_err());
    }

    #[test]
    fn malformed_metadata_is_an_error_not_a_panic() {
        // every wire count is signed; none of these may reach an index
        for (what, tweak) in [
            (
                "negative width",
                (|c: &mut PointCloud2| c.width = -1) as fn(&mut PointCloud2),
            ),
            ("negative height", |c| c.height = -1),
            ("negative step", |c| c.point_step = -12),
            ("negative offset", |c| c.fields[0].offset = -4),
            ("offset past step", |c| c.fields[2].offset = i32::MAX),
            ("width*height overflows", |c| {
                c.width = i32::MAX;
                c.height = i32::MAX;
            }),
            ("width*height*step overflows", |c| {
                c.width = i32::MAX;
                c.height = i32::MAX;
                c.point_step = i32::MAX;
            }),
        ] {
            let mut cloud = cloud_of(&[[0.0; 3]]);
            tweak(&mut cloud);
            assert!(extract_xyz(&cloud).is_err(), "{what}");
        }
    }
}
