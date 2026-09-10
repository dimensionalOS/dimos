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

//! Offline driver behind `python -m dimos.mapping.hyperspace.cli`.
//!
//! Reads the frame directory the CLI exports from a recording, runs the same
//! pipeline the module runs, and prints scene voxels plus one heat map per query
//! as JSON on stdout. The CLI turns that into an rrd; nothing here draws.
//!
//! ```text
//! hyperspace_offline --export DIR [--model-dir DIR] [--depth-weights DIR] [--cuda]
//!                    [--frame odom] [--voxel-size 0.1] [--max-frames N]
//!                    --query "a chair" [--query "a door" ...]
//! ```

use std::collections::HashMap;
use std::path::{Path, PathBuf};

use dimos_hyperspace::config::{build_backends, Config};
use hyperspace::{CameraIntrinsics, DepthImage, ImageFrame, Transform};
use serde::Deserialize;

#[derive(Deserialize)]
struct ExportIntrinsics {
    frame_id: String,
    width: u32,
    height: u32,
    fx: f64,
    fy: f64,
    cx: f64,
    cy: f64,
    distortion_model: String,
    distortion: Vec<f64>,
}

#[derive(Deserialize)]
struct ExportFrame {
    name: String,
    ts: f64,
    depth_ts: f64,
    color_frame: String,
    depth_frame: String,
    width: u32,
    height: u32,
}

#[derive(Deserialize)]
struct ExportTransform {
    parent: String,
    child: String,
    ts: f64,
    t: [f64; 3],
    q: [f64; 4],
}

struct Args {
    export: PathBuf,
    queries: Vec<String>,
    frame: String,
    voxel_size: f64,
    model_dir: String,
    depth_weights: String,
    cuda: bool,
    max_frames: usize,
    scene_min_samples: u32,
    max_depth_m: f32,
}

fn parse_args() -> Result<Args, String> {
    let mut args = Args {
        export: PathBuf::new(),
        queries: Vec::new(),
        frame: "odom".into(),
        voxel_size: 0.1,
        model_dir: String::new(),
        depth_weights: String::new(),
        cuda: false,
        max_frames: usize::MAX,
        scene_min_samples: 3,
        max_depth_m: 10.0,
    };
    let mut raw = std::env::args().skip(1);
    while let Some(flag) = raw.next() {
        let mut value = || raw.next().ok_or_else(|| format!("{flag} needs a value"));
        match flag.as_str() {
            "--export" => args.export = PathBuf::from(value()?),
            "--query" => args.queries.push(value()?),
            "--frame" => args.frame = value()?,
            "--voxel-size" => {
                args.voxel_size = value()?.parse().map_err(|e| format!("--voxel-size: {e}"))?
            }
            "--model-dir" => args.model_dir = value()?,
            "--depth-weights" => args.depth_weights = value()?,
            "--max-frames" => {
                args.max_frames = value()?.parse().map_err(|e| format!("--max-frames: {e}"))?
            }
            "--scene-min-samples" => {
                args.scene_min_samples = value()?
                    .parse()
                    .map_err(|e| format!("--scene-min-samples: {e}"))?
            }
            "--max-depth" => {
                args.max_depth_m = value()?.parse().map_err(|e| format!("--max-depth: {e}"))?
            }
            "--cuda" => args.cuda = true,
            "--help" | "-h" => {
                eprintln!("{}", USAGE);
                std::process::exit(0);
            }
            other => return Err(format!("unknown flag {other}")),
        }
    }
    if args.export.as_os_str().is_empty() {
        return Err("--export is required".into());
    }
    Ok(args)
}

const USAGE: &str = "hyperspace_offline --export DIR --query TEXT [--query TEXT ...] \
[--frame odom] [--voxel-size 0.1] [--model-dir DIR] [--depth-weights DIR] [--cuda] \
[--max-frames N] [--scene-min-samples N] [--max-depth 10.0]";

fn intrinsics(export: &ExportIntrinsics) -> CameraIntrinsics {
    CameraIntrinsics {
        camera_frame: export.frame_id.clone(),
        width: export.width,
        height: export.height,
        fx: export.fx,
        fy: export.fy,
        cx: export.cx,
        cy: export.cy,
        distortion_model: export.distortion_model.clone(),
        distortion: export.distortion.clone(),
    }
}

fn read_json<T: serde::de::DeserializeOwned>(path: &Path) -> Result<T, String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
    serde_json::from_str(&text).map_err(|e| format!("{}: {e}", path.display()))
}

fn load_color(path: &Path, camera_frame: &str, timestamp: f64) -> Result<ImageFrame, String> {
    let image = image::open(path)
        .map_err(|e| format!("{}: {e}", path.display()))?
        .to_rgb8();
    Ok(ImageFrame {
        camera_frame: camera_frame.into(),
        timestamp,
        width: image.width(),
        height: image.height(),
        encoding: "rgb8".into(),
        data: image.into_raw(),
    })
}

fn load_depth(
    path: &Path,
    camera_frame: &str,
    timestamp: f64,
    width: u32,
    height: u32,
    max_depth_m: f32,
) -> Result<DepthImage, String> {
    let bytes = std::fs::read(path).map_err(|e| format!("{}: {e}", path.display()))?;
    let expected = (width * height * 2) as usize;
    if bytes.len() != expected {
        return Err(format!(
            "{}: {} bytes, expected {expected}",
            path.display(),
            bytes.len()
        ));
    }
    let millimetres: Vec<u16> = bytes
        .as_chunks::<2>()
        .0
        .iter()
        .map(|pair| u16::from_le_bytes(*pair))
        // Past the sensor's range (or the 65535 "no reading" sentinel) = a hole.
        .map(|mm| {
            if mm as f32 * 0.001 > max_depth_m {
                0
            } else {
                mm
            }
        })
        .collect();
    Ok(DepthImage::from_millimeters(
        camera_frame,
        timestamp,
        width,
        height,
        &millimetres,
    ))
}

fn run() -> Result<(), String> {
    let args = parse_args()?;
    let intrinsics_json: serde_json::Value = read_json(&args.export.join("intrinsics.json"))?;
    let color: ExportIntrinsics = serde_json::from_value(intrinsics_json["color"].clone())
        .map_err(|e| format!("intrinsics.color: {e}"))?;
    let depth: ExportIntrinsics = serde_json::from_value(intrinsics_json["depth"].clone())
        .map_err(|e| format!("intrinsics.depth: {e}"))?;
    let frames: Vec<ExportFrame> = read_json(&args.export.join("index.json"))?;

    let config = Config {
        voxel_size: args.voxel_size,
        world_frame: args.frame.clone(),
        motion_reference_frame: args.frame.clone(),
        model_dir: args.model_dir.clone(),
        depth_weights_dir: args.depth_weights.clone(),
        cuda: args.cuda,
        buffer_len: 11,
        novelty_threshold: 0.05,
        patch_novelty_threshold: 0.5,
        max_angular_velocity: -1.0,
        max_linear_velocity: -1.0,
        max_dark_fraction: 0.6,
        max_bright_fraction: -1.0,
        min_keyframe_interval: -1.0,
        max_depth_m: args.max_depth_m,
        depth_max_dt: 0.05,
        depth_history: 64,
        depth_thumbnail_stride: 4,
        hot_threshold: 0.02,
        max_hot_patches: 6000,
        cap_near: 0.9,
        cap_far: 1.1,
        background_prompts: String::new(),
        scene_emit_every: 0,
        scene_min_samples: args.scene_min_samples,
    };
    let (embedder, text_embedder, depth_fuser) = build_backends(&config)?;
    let mut state = hyperspace::Hyperspace::new(
        config.hyperspace_config(),
        embedder,
        text_embedder,
        depth_fuser,
    );
    state.set_camera_intrinsics(intrinsics(&color));
    state.set_camera_intrinsics(intrinsics(&depth));

    let tf_text = std::fs::read_to_string(args.export.join("tf.jsonl"))
        .map_err(|e| format!("tf.jsonl: {e}"))?;
    for line in tf_text.lines().filter(|line| !line.trim().is_empty()) {
        let transform: ExportTransform =
            serde_json::from_str(line).map_err(|e| format!("tf.jsonl: {e}"))?;
        state.update(&Transform {
            parent_frame: transform.parent,
            child_frame: transform.child,
            timestamp: transform.ts,
            translation: transform.t,
            rotation: transform.q,
        });
    }

    let total = args.max_frames.min(frames.len());
    let started = std::time::Instant::now();
    for (index, frame) in frames.iter().take(total).enumerate() {
        let depth_path = args
            .export
            .join("depth")
            .join(format!("{}.u16", frame.name));
        state.add_depth(load_depth(
            &depth_path,
            &frame.depth_frame,
            frame.depth_ts,
            frame.width,
            frame.height,
            args.max_depth_m,
        )?);
        let color_path = args
            .export
            .join("color")
            .join(format!("{}.jpg", frame.name));
        state.add_image(load_color(&color_path, &frame.color_frame, frame.ts)?)?;
        if (index + 1) % 100 == 0 || index + 1 == total {
            eprintln!(
                "{}/{total} frames, {} keyframes, {:.0}s",
                index + 1,
                state.stats.kept,
                started.elapsed().as_secs_f64()
            );
        }
    }
    state.flush()?;

    let scene: Vec<[i64; 4]> = state
        .scene_voxels(&args.frame, args.scene_min_samples)
        .into_iter()
        .map(|(index, count)| {
            [
                index[0] as i64,
                index[1] as i64,
                index[2] as i64,
                count as i64,
            ]
        })
        .collect();
    let poses: Vec<serde_json::Value> = state
        .keyframes()
        .iter()
        .filter_map(|keyframe| {
            state
                .tf
                .get(&args.frame, &keyframe.camera_frame, keyframe.timestamp)
                .map(|pose| {
                    let position = pose.translation.vector;
                    let rotation = pose.rotation.quaternion();
                    serde_json::json!({
                        "t": [position.x, position.y, position.z],
                        "q": [rotation.i, rotation.j, rotation.k, rotation.w],
                        "ts": keyframe.timestamp,
                    })
                })
        })
        .collect();

    let mut answers = Vec::new();
    for (id, text) in args.queries.iter().enumerate() {
        let started = std::time::Instant::now();
        let heatmap = state.query(text, &args.frame)?;
        let voxels: Vec<serde_json::Value> = heatmap
            .voxels
            .iter()
            .map(|(index, score)| serde_json::json!([index[0], index[1], index[2], score]))
            .collect();
        eprintln!(
            "query {text:?}: {} voxels in {:.0} ms",
            voxels.len(),
            started.elapsed().as_secs_f64() * 1000.0
        );
        answers.push(serde_json::json!({
            "id": id as i32,
            "text": text,
            "voxels": voxels,
            "stats": serde_json::to_value(&heatmap.stats).map_err(|e| e.to_string())?,
        }));
    }

    let mut ingest_stats: HashMap<String, serde_json::Value> = HashMap::new();
    ingest_stats.insert("frames_in".into(), serde_json::json!(total));
    ingest_stats.insert(
        "stats".into(),
        serde_json::to_value(&state.stats).map_err(|e| e.to_string())?,
    );
    let output = serde_json::json!({
        "frame": args.frame,
        "voxel_size": args.voxel_size,
        "keyframes": poses,
        "scene": scene,
        "queries": answers,
        "ingest": ingest_stats,
    });
    println!(
        "{}",
        serde_json::to_string(&output).map_err(|e| e.to_string())?
    );
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("error: {error}");
        eprintln!("{USAGE}");
        std::process::exit(1);
    }
}
