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

//! LCM messages in, hyperspace types out (and a voxel heat map back out as a cloud).

use hyperspace::{CameraIntrinsics, DepthImage, ImageFrame, Transform, VoxelHeatmap};
use lcm_msgs::sensor_msgs::{CameraInfo, Image, PointCloud2, PointField};
use lcm_msgs::std_msgs::{Header, Time};
use lcm_msgs::tf2_msgs::TFMessage;

pub fn time_secs(stamp: &Time) -> f64 {
    stamp.sec as f64 + stamp.nsec as f64 * 1e-9
}

pub fn secs_to_time(seconds: f64) -> Time {
    Time {
        sec: seconds.trunc() as i32,
        nsec: (seconds.fract() * 1e9).round() as i32,
    }
}

/// Every transform in a TFMessage, ready for `Hyperspace::update`.
///
/// Fed straight into hyperspace's own tf graph rather than the module's `#[tf]`
/// helper: queries resolve poses at *query* time against the full history, so a
/// later loop closure that rewrites old transforms also moves old answers.
pub fn transforms(msg: &TFMessage) -> Vec<Transform> {
    msg.transforms
        .iter()
        .map(|stamped| Transform {
            parent_frame: stamped.header.frame_id.clone(),
            child_frame: stamped.child_frame_id.clone(),
            timestamp: time_secs(&stamped.header.stamp),
            translation: [
                stamped.transform.translation.x,
                stamped.transform.translation.y,
                stamped.transform.translation.z,
            ],
            rotation: [
                stamped.transform.rotation.x,
                stamped.transform.rotation.y,
                stamped.transform.rotation.z,
                stamped.transform.rotation.w,
            ],
        })
        .collect()
}

pub fn intrinsics(msg: &CameraInfo) -> CameraIntrinsics {
    CameraIntrinsics {
        camera_frame: msg.header.frame_id.clone(),
        width: msg.width as u32,
        height: msg.height as u32,
        fx: msg.K[0],
        fy: msg.K[4],
        cx: msg.K[2],
        cy: msg.K[5],
        distortion_model: msg.distortion_model.clone(),
        distortion: msg.D.clone(),
    }
}

/// Colour image to an rgb8 frame. Accepts rgb8, bgr8, rgba8, mono8 and the
/// compressed encodings dimos cameras publish (jpeg/png).
pub fn color_frame(msg: &Image) -> Result<ImageFrame, String> {
    let (width, height) = (msg.width as u32, msg.height as u32);
    let pixels = (width as usize) * (height as usize);
    let data = match msg.encoding.as_str() {
        "rgb8" => {
            expect_len(&msg.data, pixels * 3, &msg.encoding)?;
            msg.data.clone()
        }
        "bgr8" => {
            expect_len(&msg.data, pixels * 3, &msg.encoding)?;
            let mut out = msg.data.clone();
            for pixel in out.as_chunks_mut::<3>().0 {
                pixel.swap(0, 2);
            }
            out
        }
        "rgba8" | "bgra8" => {
            expect_len(&msg.data, pixels * 4, &msg.encoding)?;
            let swap = msg.encoding.starts_with("bgr");
            let mut out = Vec::with_capacity(pixels * 3);
            for pixel in msg.data.as_chunks::<4>().0 {
                if swap {
                    out.extend_from_slice(&[pixel[2], pixel[1], pixel[0]]);
                } else {
                    out.extend_from_slice(&[pixel[0], pixel[1], pixel[2]]);
                }
            }
            out
        }
        "mono8" => {
            expect_len(&msg.data, pixels, &msg.encoding)?;
            msg.data
                .iter()
                .flat_map(|value| [*value, *value, *value])
                .collect()
        }
        "jpeg" | "jpg" | "png" => {
            let decoded = image::load_from_memory(&msg.data)
                .map_err(|e| format!("cannot decode {} image: {e}", msg.encoding))?
                .to_rgb8();
            return Ok(ImageFrame {
                camera_frame: msg.header.frame_id.clone(),
                timestamp: time_secs(&msg.header.stamp),
                width: decoded.width(),
                height: decoded.height(),
                encoding: "rgb8".into(),
                data: decoded.into_raw(),
            });
        }
        other => return Err(format!("unsupported colour encoding {other:?}")),
    };
    Ok(ImageFrame {
        camera_frame: msg.header.frame_id.clone(),
        timestamp: time_secs(&msg.header.stamp),
        width,
        height,
        encoding: "rgb8".into(),
        data,
    })
}

/// Depth image to metres. Accepts 16UC1 (millimetres) and 32FC1 (metres).
///
/// Readings beyond `max_depth_m` become holes: RealSense frames carry 65535 mm
/// sentinels and the odd 20-40 m glitch, and one of those in a patch's median
/// would put its pyramid tens of metres outside the map.
pub fn depth_frame(msg: &Image, max_depth_m: f32) -> Result<DepthImage, String> {
    let (width, height) = (msg.width as u32, msg.height as u32);
    let pixels = (width as usize) * (height as usize);
    let depth_m = match msg.encoding.as_str() {
        "16UC1" | "mono16" => {
            expect_len(&msg.data, pixels * 2, &msg.encoding)?;
            msg.data
                .as_chunks::<2>()
                .0
                .iter()
                .map(|pair| u16::from_le_bytes(*pair) as f32 * 0.001)
                .map(|metres| if metres > max_depth_m { 0.0 } else { metres })
                .collect()
        }
        "32FC1" => {
            expect_len(&msg.data, pixels * 4, &msg.encoding)?;
            msg.data
                .as_chunks::<4>()
                .0
                .iter()
                .map(|quad| f32::from_le_bytes(*quad))
                .map(|metres| {
                    if metres > max_depth_m || !metres.is_finite() {
                        0.0
                    } else {
                        metres
                    }
                })
                .collect()
        }
        other => return Err(format!("unsupported depth encoding {other:?}")),
    };
    Ok(DepthImage {
        camera_frame: msg.header.frame_id.clone(),
        timestamp: time_secs(&msg.header.stamp),
        width,
        height,
        depth_m,
    })
}

fn expect_len(data: &[u8], expected: usize, encoding: &str) -> Result<(), String> {
    if data.len() < expected {
        return Err(format!(
            "{encoding} image has {} bytes, expected {expected}",
            data.len()
        ));
    }
    Ok(())
}

/// Voxel centres as an xyz + intensity cloud (intensity = score, the field name
/// every viewer and dimos's PointCloud2 wrapper already understand). `seq`
/// carries the id of the query that
/// produced it, so a subscriber can pair an answer with its request without an RPC.
pub fn heatmap_cloud(heatmap: &VoxelHeatmap, seq: i32, stamp: Time) -> PointCloud2 {
    let points = heatmap.voxels.iter().map(|(index, score)| {
        let centre = |axis: usize| ((index[axis] as f64 + 0.5) * heatmap.voxel_size) as f32;
        ([centre(0), centre(1), centre(2)], *score)
    });
    scored_cloud(points, &heatmap.frame, seq, stamp)
}

/// Compact JSON answer for callers that cannot read a PointCloud2's header
/// (dimos's Python wrapper drops `seq`): the id, the query, how many voxels
/// answered, and the `top` best voxel centres with their scores.
pub fn answer_json(heatmap: &VoxelHeatmap, id: i32, text: &str, top: usize) -> String {
    let best: Vec<serde_json::Value> = heatmap
        .voxels
        .iter()
        .take(top)
        .map(|(index, score)| {
            let centre = |axis: usize| (index[axis] as f64 + 0.5) * heatmap.voxel_size;
            serde_json::json!({"xyz": [centre(0), centre(1), centre(2)], "score": score})
        })
        .collect();
    serde_json::json!({
        "id": id,
        "text": text,
        "frame": heatmap.frame,
        "voxel_size": heatmap.voxel_size,
        "voxels": heatmap.voxels.len(),
        "best": best,
        "stats": heatmap.stats,
    })
    .to_string()
}

/// Occupied scene voxels as an xyz + score cloud, score = normalized sample count.
pub fn scene_cloud(
    voxels: &[([i32; 3], u32)],
    voxel_size: f64,
    frame: &str,
    stamp: Time,
) -> PointCloud2 {
    let most = voxels.iter().map(|(_, n)| *n).max().unwrap_or(1).max(1) as f32;
    let points = voxels.iter().map(|(index, count)| {
        let centre = |axis: usize| ((index[axis] as f64 + 0.5) * voxel_size) as f32;
        ([centre(0), centre(1), centre(2)], *count as f32 / most)
    });
    scored_cloud(points, frame, 0, stamp)
}

fn scored_cloud(
    points: impl Iterator<Item = ([f32; 3], f32)>,
    frame_id: &str,
    seq: i32,
    stamp: Time,
) -> PointCloud2 {
    const POINT_STEP: i32 = 16;
    let mut data = Vec::new();
    let mut count = 0i32;
    for (position, score) in points {
        for value in position {
            data.extend_from_slice(&value.to_le_bytes());
        }
        data.extend_from_slice(&score.to_le_bytes());
        count += 1;
    }
    let field = |name: &str, offset: i32| PointField {
        name: name.to_string(),
        offset,
        datatype: 7, // FLOAT32
        count: 1,
    };
    PointCloud2 {
        header: Header {
            seq,
            stamp,
            frame_id: frame_id.to_string(),
        },
        height: 1,
        width: count,
        fields: vec![
            field("x", 0),
            field("y", 4),
            field("z", 8),
            field("intensity", 12),
        ],
        is_bigendian: false,
        point_step: POINT_STEP,
        row_step: POINT_STEP * count,
        data,
        is_dense: true,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use hyperspace::query::QueryStats;

    fn image(encoding: &str, width: i32, height: i32, data: Vec<u8>) -> Image {
        Image {
            header: Header {
                seq: 0,
                stamp: secs_to_time(1.5),
                frame_id: "camera".into(),
            },
            height,
            width,
            encoding: encoding.into(),
            is_bigendian: 0,
            step: 0,
            data,
        }
    }

    #[test]
    fn bgr8_is_swapped_to_rgb8() {
        let frame = color_frame(&image("bgr8", 1, 1, vec![10, 20, 30])).unwrap();
        assert_eq!(frame.data, vec![30, 20, 10]);
        assert_eq!(frame.encoding, "rgb8");
        assert_eq!(frame.timestamp, 1.5);
    }

    #[test]
    fn mono8_is_replicated() {
        let frame = color_frame(&image("mono8", 2, 1, vec![7, 9])).unwrap();
        assert_eq!(frame.data, vec![7, 7, 7, 9, 9, 9]);
    }

    #[test]
    fn short_buffers_and_unknown_encodings_are_errors() {
        assert!(color_frame(&image("rgb8", 4, 4, vec![0; 3])).is_err());
        assert!(color_frame(&image("yuv422", 1, 1, vec![0; 3])).is_err());
        assert!(depth_frame(&image("rgb8", 1, 1, vec![0; 3]), 10.0).is_err());
    }

    #[test]
    fn depth_16uc1_is_millimetres() {
        let depth = depth_frame(&image("16UC1", 2, 1, vec![0xE8, 0x03, 0x00, 0x00]), 10.0).unwrap();
        assert_eq!(depth.depth_m, vec![1.0, 0.0]);
    }

    #[test]
    fn depth_beyond_max_range_becomes_a_hole() {
        // 65535 mm is the RealSense "no reading" sentinel; 12 m is past a 10 m range.
        let data = vec![0xFF, 0xFF, 0xE0, 0x2E, 0xE8, 0x03];
        let depth = depth_frame(&image("16UC1", 3, 1, data), 10.0).unwrap();
        assert_eq!(depth.depth_m, vec![0.0, 0.0, 1.0]);
    }

    #[test]
    fn answer_json_carries_id_and_best_voxels() {
        let heatmap = VoxelHeatmap {
            frame: "odom".into(),
            voxel_size: 0.1,
            voxels: vec![([1, 2, 3], 1.0), ([4, 5, 6], 0.25)],
            stats: QueryStats::default(),
        };
        let parsed: serde_json::Value =
            serde_json::from_str(&answer_json(&heatmap, 9, "a cone", 1)).unwrap();
        assert_eq!(parsed["id"], 9);
        assert_eq!(parsed["text"], "a cone");
        assert_eq!(parsed["voxels"], 2);
        assert_eq!(parsed["best"].as_array().unwrap().len(), 1);
        assert!((parsed["best"][0]["xyz"][0].as_f64().unwrap() - 0.15).abs() < 1e-9);
        assert_eq!(parsed["best"][0]["score"], 1.0);
    }

    #[test]
    fn heatmap_cloud_carries_the_query_id() {
        let heatmap = VoxelHeatmap {
            frame: "odom".into(),
            voxel_size: 0.1,
            voxels: vec![([1, 2, 3], 1.0), ([4, 5, 6], 0.25)],
            stats: QueryStats::default(),
        };
        let cloud = heatmap_cloud(&heatmap, 42, secs_to_time(2.0));
        assert_eq!(cloud.header.seq, 42);
        assert_eq!(cloud.header.frame_id, "odom");
        assert_eq!(cloud.width, 2);
        assert_eq!(cloud.data.len(), 32);
        assert_eq!(
            cloud
                .fields
                .iter()
                .map(|f| f.name.as_str())
                .collect::<Vec<_>>(),
            ["x", "y", "z", "intensity"]
        );
        let first_x = f32::from_le_bytes(cloud.data[0..4].try_into().unwrap());
        assert!((first_x - 0.15).abs() < 1e-6);
        let first_score = f32::from_le_bytes(cloud.data[12..16].try_into().unwrap());
        assert_eq!(first_score, 1.0);
    }
}
