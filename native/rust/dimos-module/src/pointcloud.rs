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
    let width = count(msg.width, "negative width")?;
    let height = count(msg.height, "negative height")?;
    let row_step = width
        .checked_mul(step)
        .ok_or(ExtractError("row size overflows"))?;
    extract_xyz_rows(
        &msg.data,
        [width, height],
        [step, row_step],
        [xo, yo, zo],
        false,
    )
}

/// Read finite float32 xyz values from validated field offsets, respecting row padding.
/// Message adapters retain their own schema types and pass only the wire layout here.
pub fn extract_xyz_rows(
    data: &[u8],
    dimensions: [usize; 2],
    steps: [usize; 2],
    offsets: [usize; 3],
    big_endian: bool,
) -> Result<Vec<[f32; 3]>, ExtractError> {
    let [width, height] = dimensions;
    let [step, row_step] = steps;
    if step == 0 {
        return Err(ExtractError("point_step is 0"));
    }
    let packed = width
        .checked_mul(step)
        .ok_or(ExtractError("row size overflows"))?;
    if row_step < packed {
        return Err(ExtractError("row_step is shorter than a row"));
    }
    let needed = height
        .checked_mul(row_step)
        .ok_or(ExtractError("row buffer size overflows"))?;
    if data.len() < needed {
        return Err(ExtractError("data buffer shorter than rows"));
    }
    for offset in offsets {
        if offset.checked_add(4).is_none_or(|end| end > step) {
            return Err(ExtractError(
                "xyz field offsets do not fit within point_step",
            ));
        }
    }
    let mut out = Vec::new();
    if width == 0 || height == 0 {
        return Ok(out);
    }
    for row in 0..height {
        for column in 0..width {
            let base = row * row_step + column * step;
            let point = offsets.map(|offset| {
                let bytes: [u8; 4] = data[base + offset..base + offset + 4]
                    .try_into()
                    .expect("bounds checked above");
                if big_endian {
                    f32::from_be_bytes(bytes)
                } else {
                    f32::from_le_bytes(bytes)
                }
            });
            if point.iter().all(|value| value.is_finite()) {
                out.push(point);
            }
        }
    }
    Ok(out)
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
    fn padded_rows_and_both_byte_orders_preserve_finite_points() {
        for big_endian in [false, true] {
            let mut bytes = Vec::new();
            for xyz in [[1.0f32, 2.0, 3.0], [4.0, 5.0, 6.0]] {
                for value in xyz {
                    bytes.extend(if big_endian {
                        value.to_be_bytes()
                    } else {
                        value.to_le_bytes()
                    });
                }
                bytes.extend([255u8; 4]);
            }
            assert_eq!(
                extract_xyz_rows(&bytes, [1, 2], [12, 16], [0, 4, 8], big_endian).unwrap(),
                vec![[1., 2., 3.], [4., 5., 6.]]
            );
        }
    }

    #[test]
    fn malformed_row_layouts_are_rejected_before_allocation_or_indexing() {
        for (dimensions, steps, offsets) in [
            ([1, 1], [12, 11], [0, 4, 8]),
            ([1, 2], [12, 16], [0, 4, 8]),
            ([1, 1], [12, 12], [0, 4, usize::MAX]),
            ([usize::MAX, 1], [12, 12], [0, 4, 8]),
            ([1, usize::MAX], [12, 16], [0, 4, 8]),
        ] {
            assert!(extract_xyz_rows(&[0; 16], dimensions, steps, offsets, false).is_err());
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
