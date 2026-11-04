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

//! The Go2's binary data-channel messages and its compressed voxel map (`rt/utlidar/voxel_map_compressed`),
//! decoded.

use serde_json::Value;

/// A binary message: its JSON header and the raw bytes after it. Two framings: `02 00 00 00` + u32 JSON length
/// + 4 reserved bytes (the lidar's), else a u16 JSON length + 2 reserved bytes.
pub fn parse_binary(buf: &[u8]) -> Option<(Value, &[u8])> {
    if buf.len() < 4 {
        return None;
    }
    let h1 = u16::from_le_bytes([buf[0], buf[1]]);
    let h2 = u16::from_le_bytes([buf[2], buf[3]]);
    let (json, binary) = if h1 == 2 && h2 == 0 {
        let rest = &buf[4..];
        let len = u32::from_le_bytes(rest.get(0..4)?.try_into().ok()?) as usize;
        (rest.get(8..8 + len)?, rest.get(8 + len..)?)
    } else {
        let len = h1 as usize;
        (buf.get(4..4 + len)?, buf.get(4 + len..)?)
    };
    Some((serde_json::from_slice(json).ok()?, binary))
}

fn array3(value: &Value) -> Option<[f64; 3]> {
    let a = value.as_array()?;
    Some([
        a.first()?.as_f64()?,
        a.get(1)?.as_f64()?,
        a.get(2)?.as_f64()?,
    ])
}

/// The voxel bitfield -> points: an LZ4 block (`src_size` bytes out), then 2048 bytes per z slice of a 128x128
/// grid, one bit per voxel, most significant bit first; a point = (x, y, z) * resolution + origin.
pub fn decode_voxels(meta: &Value, compressed: &[u8]) -> Result<Vec<[f32; 3]>, String> {
    let size = meta["src_size"]
        .as_u64()
        .ok_or("the lidar message has no src_size")? as usize;
    if size > 64 * 1024 * 1024 {
        return Err("the lidar message's src_size is implausible".into());
    }
    let raw =
        lz4_flex::block::decompress(compressed, size).map_err(|e| format!("lidar LZ4: {e}"))?;
    let resolution = meta["resolution"].as_f64().unwrap_or(0.05);
    let origin = array3(&meta["origin"]).unwrap_or_default();
    let mut points = Vec::new();
    for (index, &byte) in raw.iter().enumerate() {
        if byte == 0 {
            continue;
        }
        let z = index / 0x800;
        let slice = index % 0x800;
        let y = slice / 0x10;
        let x_base = (slice % 0x10) * 8;
        for bit in 0..8 {
            if byte & (0x80 >> bit) != 0 {
                points.push([
                    ((x_base + bit) as f64 * resolution + origin[0]) as f32,
                    (y as f64 * resolution + origin[1]) as f32,
                    (z as f64 * resolution + origin[2]) as f32,
                ]);
            }
        }
    }
    Ok(points)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn lidar_message(voxels: &[u8], origin: [f64; 3]) -> Vec<u8> {
        let header = serde_json::json!({ "type": "msg", "topic": "rt/utlidar/voxel_map_compressed", "data": {
            "origin": origin, "resolution": 0.05, "src_size": voxels.len(), "stamp": 1.0
        }})
        .to_string();
        let mut buf = vec![2, 0, 0, 0];
        buf.extend_from_slice(&(header.len() as u32).to_le_bytes());
        buf.extend_from_slice(&[0, 0, 0, 0]);
        buf.extend_from_slice(header.as_bytes());
        buf.extend_from_slice(&lz4_flex::block::compress(voxels));
        buf
    }

    #[test]
    fn a_lidar_frame_decodes_to_points() {
        let mut voxels = vec![0u8; 0x800 * 2];
        voxels[0] = 0b1000_0001; // x 0 and 7, y 0, z 0
        voxels[0x800 + 0x10 + 1] = 0b0100_0000; // z 1, y 1, x 8 + 1
        let buf = lidar_message(&voxels, [1.0, 2.0, 3.0]);
        let (json, binary) = parse_binary(&buf).unwrap();
        assert_eq!(json["topic"], "rt/utlidar/voxel_map_compressed");
        let points = decode_voxels(&json["data"], binary).unwrap();
        assert_eq!(points.len(), 3);
        assert_eq!(points[0], [1.0, 2.0, 3.0]);
        assert!((points[1][0] - 1.35).abs() < 1e-6);
        assert!(
            (points[2][0] - 1.45).abs() < 1e-6
                && (points[2][1] - 2.05).abs() < 1e-6
                && (points[2][2] - 3.05).abs() < 1e-6
        );
    }
}
