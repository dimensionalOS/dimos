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

//! Geometry extraction kept separate from generated message values and codecs.
use std::io;

use dimos_generated_messages::sensor_msgs::msg::{PointCloud2, PointField};

fn invalid(reason: &str) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, reason)
}

/// Read finite XYZ points in row order, respecting field offsets, row padding,
/// float32/float64 coordinates and byte order. Other fields are left untouched.
pub fn xyz_points(message: &PointCloud2) -> io::Result<Vec<(f32, f32, f32)>> {
    let mut coordinates = Vec::with_capacity(3);
    let step = message.point_step as usize;
    for name in ["x", "y", "z"] {
        let mut matching = message.fields.iter().filter(|field| field.name == name);
        let field = matching
            .next()
            .ok_or_else(|| invalid("missing XYZ field"))?;
        if matching.next().is_some() || field.count != 1 {
            return Err(invalid("XYZ fields must be unique and scalar"));
        }
        let size = match field.datatype {
            PointField::FLOAT32 => 4,
            PointField::FLOAT64 => 8,
            _ => return Err(invalid("XYZ fields must be float32 or float64")),
        };
        let offset = field.offset as usize;
        if offset.checked_add(size).is_none_or(|end| end > step) {
            return Err(invalid("XYZ field exceeds point_step"));
        }
        coordinates.push((offset, size));
    }
    let width = message.width as usize;
    let height = message.height as usize;
    let row_step = message.row_step as usize;
    let row_bytes = width
        .checked_mul(step)
        .ok_or_else(|| invalid("row size overflow"))?;
    let total = height
        .checked_mul(row_step)
        .ok_or_else(|| invalid("data size overflow"))?;
    if step == 0 || row_step < row_bytes || total != message.data.len() {
        return Err(invalid("point cloud dimensions/strides do not match data"));
    }
    let count = width
        .checked_mul(height)
        .ok_or_else(|| invalid("point count overflow"))?;
    let read = |base: usize, (offset, size): (usize, usize)| -> f32 {
        let bytes = &message.data[base + offset..base + offset + size];
        match (size, message.is_bigendian) {
            (4, false) => f32::from_le_bytes(bytes.try_into().unwrap()),
            (4, true) => f32::from_be_bytes(bytes.try_into().unwrap()),
            (8, false) => f64::from_le_bytes(bytes.try_into().unwrap()) as f32,
            (8, true) => f64::from_be_bytes(bytes.try_into().unwrap()) as f32,
            _ => unreachable!("field size checked above"),
        }
    };
    let mut points = Vec::with_capacity(count);
    for row in 0..height {
        for column in 0..width {
            let base = row * row_step + column * step;
            let (x, y, z) = (
                read(base, coordinates[0]),
                read(base, coordinates[1]),
                read(base, coordinates[2]),
            );
            if x.is_finite() && y.is_finite() && z.is_finite() {
                points.push((x, y, z));
            }
        }
    }
    Ok(points)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cloud(big: bool) -> PointCloud2 {
        let mut data = vec![0; 64];
        for row in 0..2 {
            let x = row as f32 + 1.0;
            let z = row as f64 + 3.0;
            data[row * 32..row * 32 + 4].copy_from_slice(&if big {
                x.to_be_bytes()
            } else {
                x.to_le_bytes()
            });
            data[row * 32 + 8..row * 32 + 16].copy_from_slice(&if big {
                z.to_be_bytes()
            } else {
                z.to_le_bytes()
            });
            data[row * 32 + 16..row * 32 + 20].copy_from_slice(&if big {
                2f32.to_be_bytes()
            } else {
                2f32.to_le_bytes()
            });
        }
        PointCloud2 {
            width: 1,
            height: 2,
            point_step: 24,
            row_step: 32,
            data,
            is_bigendian: big,
            fields: vec![
                PointField {
                    name: "z".into(),
                    offset: 8,
                    datatype: PointField::FLOAT64,
                    count: 1,
                },
                PointField {
                    name: "x".into(),
                    offset: 0,
                    datatype: PointField::FLOAT32,
                    count: 1,
                },
                PointField {
                    name: "y".into(),
                    offset: 16,
                    datatype: PointField::FLOAT32,
                    count: 1,
                },
            ],
            ..Default::default()
        }
    }

    #[test]
    fn organized_padded_mixed_precision_both_byte_orders() {
        for big in [false, true] {
            assert_eq!(
                xyz_points(&cloud(big)).unwrap(),
                vec![(1., 2., 3.), (2., 2., 4.)]
            );
        }
    }

    #[test]
    fn invalid_layouts_fail_before_reading() {
        let mut bad = Vec::new();
        let mut message = cloud(false);
        message.row_step = 23;
        bad.push(message);
        let mut message = cloud(false);
        message.data.pop();
        bad.push(message);
        let mut message = cloud(false);
        message.fields[0].offset = 20;
        bad.push(message);
        let mut message = cloud(false);
        message.fields[0].count = 2;
        bad.push(message);
        let mut message = cloud(false);
        message.fields[0].datatype = PointField::UINT32;
        bad.push(message);
        let mut message = cloud(false);
        message.fields.push(message.fields[0].clone());
        bad.push(message);
        let mut message = cloud(false);
        message.fields.remove(0);
        bad.push(message);
        for message in bad {
            assert_eq!(
                xyz_points(&message).unwrap_err().kind(),
                io::ErrorKind::InvalidData
            );
        }
    }

    #[test]
    fn nonfinite_coordinates_are_skipped() {
        let mut message = cloud(false);
        message.data[..4].copy_from_slice(&f32::NAN.to_le_bytes());
        assert_eq!(xyz_points(&message).unwrap(), vec![(2., 2., 4.)]);
    }
}
