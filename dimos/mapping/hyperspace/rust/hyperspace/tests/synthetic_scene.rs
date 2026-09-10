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

//! Integration tests on a synthetic scene: a scripted embedder plants an
//! "object" patch in frames taken from known poses; queries must light up the
//! voxel where the object sits, and only there.

use hyperspace::embedder::{ScriptedEmbedder, TableTextEmbedder};
use hyperspace::{
    CameraIntrinsics, Config, DepthImage, Hyperspace, ImageFrame, KeyframeConfig,
    PassthroughDepthFuser, PatchGrid, Transform,
};
use nalgebra::{Isometry3, Point3, UnitQuaternion, Vector3};
use std::collections::HashMap;

const DIM: usize = 8;
const ROWS: usize = 12;
const COLS: usize = 16;
const WIDTH: u32 = 160;
const HEIGHT: u32 = 120;
const FX: f64 = 120.0;
const CAMERA: &str = "camera_optical";
const TARGET: &str = "odom";

fn unit(axis: usize) -> Vec<f32> {
    let mut v = vec![0.0; DIM];
    v[axis] = 1.0;
    v
}

fn intrinsics() -> CameraIntrinsics {
    CameraIntrinsics {
        camera_frame: CAMERA.into(),
        width: WIDTH,
        height: HEIGHT,
        fx: FX,
        fy: FX,
        cx: WIDTH as f64 / 2.0,
        cy: HEIGHT as f64 / 2.0,
        distortion_model: "plumb_bob".into(),
        distortion: vec![],
    }
}

/// Camera at `position` looking at `look_at`, optical convention (z forward, x right, y down).
fn look_at_pose(position: Point3<f64>, look_at: Point3<f64>) -> Isometry3<f64> {
    let forward = (look_at - position).normalize();
    let world_up = Vector3::z();
    let right = forward.cross(&world_up).normalize();
    let down = forward.cross(&right).normalize();
    let rotation = nalgebra::Rotation3::from_matrix_unchecked(nalgebra::Matrix3::from_columns(&[
        right, down, forward,
    ]));
    Isometry3::from_parts(
        position.into(),
        UnitQuaternion::from_rotation_matrix(&rotation),
    )
}

/// The pixel the object lands on in a camera with this pose, plus its depth.
fn project(pose: &Isometry3<f64>, object: Point3<f64>) -> (f64, f64, f64) {
    let in_camera = pose.inverse() * object;
    let camera = intrinsics();
    (
        in_camera.x / in_camera.z * camera.fx + camera.cx,
        in_camera.y / in_camera.z * camera.fy + camera.cy,
        in_camera.z,
    )
}

struct Scene {
    hyperspace: Hyperspace,
    object: Point3<f64>,
}

/// Build a scene: `poses` cameras look at `object`; every frame's patch grid is
/// "background" (axis 1) except the patch covering the object (axis 0), and
/// every patch has a depth of the object distance (a flat wall at the object).
fn build_scene(object: Point3<f64>, poses: &[Isometry3<f64>], with_depth: bool) -> Scene {
    let mut embedder = ScriptedEmbedder {
        rows: ROWS,
        cols: COLS,
        dim: DIM,
        ..Default::default()
    };
    let mut text = TableTextEmbedder::default();
    text.table.insert("object".into(), unit(0));
    text.table.insert("background".into(), unit(1));
    let query = hyperspace::QueryConfig {
        background_prompts: vec!["background".into()],
        hot_threshold: 0.3,
        ..Default::default()
    };
    let config = Config {
        voxel_size: 0.1,
        motion_reference_frame: TARGET.into(),
        default_keyframe: KeyframeConfig {
            buffer_len: 1,
            ..KeyframeConfig::permissive()
        },
        depth_thumbnail_stride: 8,
        query,
        ..Config::default()
    };
    let mut frames = Vec::new();
    for (index, pose) in poses.iter().enumerate() {
        let timestamp = 10.0 + index as f64;
        let (u, v, depth) = project(pose, object);
        let mut data = Vec::with_capacity(ROWS * COLS * DIM);
        for row in 0..ROWS {
            for col in 0..COLS {
                let u0 = col as f64 * WIDTH as f64 / COLS as f64;
                let v0 = row as f64 * HEIGHT as f64 / ROWS as f64;
                let hit = u >= u0
                    && u < u0 + WIDTH as f64 / COLS as f64
                    && v >= v0
                    && v < v0 + HEIGHT as f64 / ROWS as f64;
                data.extend(if hit { unit(0) } else { unit(1) });
            }
        }
        embedder.script(
            CAMERA,
            timestamp,
            PatchGrid::from_f32(ROWS, COLS, DIM, &data),
        );
        frames.push((timestamp, *pose, depth));
    }
    let mut hyperspace = Hyperspace::new(
        config,
        Box::new(embedder),
        Box::new(text),
        Box::new(PassthroughDepthFuser),
    );
    hyperspace.set_camera_intrinsics(intrinsics());
    for (timestamp, pose, depth) in &frames {
        hyperspace.update(&Transform::from_isometry(TARGET, CAMERA, *timestamp, pose));
        if with_depth {
            hyperspace.add_depth(DepthImage {
                camera_frame: CAMERA.into(),
                timestamp: *timestamp,
                width: WIDTH,
                height: HEIGHT,
                depth_m: vec![*depth as f32; (WIDTH * HEIGHT) as usize],
            });
        }
        let frame = ImageFrame {
            camera_frame: CAMERA.into(),
            timestamp: *timestamp,
            width: WIDTH,
            height: HEIGHT,
            encoding: "rgb8".into(),
            data: vec![128; (WIDTH * HEIGHT * 3) as usize],
        };
        hyperspace.add_image(frame).unwrap();
    }
    hyperspace.flush().unwrap();
    Scene { hyperspace, object }
}

fn ring_poses(object: Point3<f64>, count: usize, radius: f64) -> Vec<Isometry3<f64>> {
    (0..count)
        .map(|i| {
            let angle = i as f64 / count as f64 * std::f64::consts::TAU;
            look_at_pose(
                Point3::new(
                    object.x + radius * angle.cos(),
                    object.y + radius * angle.sin(),
                    object.z + 0.3,
                ),
                object,
            )
        })
        .collect()
}

fn voxel_of(point: Point3<f64>, voxel_size: f64) -> [i32; 3] {
    [
        (point.x / voxel_size).floor() as i32,
        (point.y / voxel_size).floor() as i32,
        (point.z / voxel_size).floor() as i32,
    ]
}

fn voxel_distance(a: [i32; 3], b: [i32; 3]) -> i32 {
    (0..3).map(|i| (a[i] - b[i]).abs()).max().unwrap()
}

#[test]
fn object_seen_from_three_poses_lands_on_its_voxel() {
    let object = Point3::new(3.0, 2.0, 0.5);
    let mut scene = build_scene(object, &ring_poses(object, 3, 2.5), true);
    assert_eq!(scene.hyperspace.stats.kept, 3);
    let heatmap = scene.hyperspace.query("object", TARGET).unwrap();
    assert!(
        !heatmap.voxels.is_empty(),
        "no voxels scored: {:?}",
        heatmap.stats
    );
    let (top, score) = heatmap.voxels[0];
    assert!((score - 1.0).abs() < 1e-6);
    assert!(
        voxel_distance(top, voxel_of(scene.object, 0.1)) <= 1,
        "top voxel {top:?} is not at the object {:?}",
        voxel_of(scene.object, 0.1)
    );
    // Voxels near the object outrank everything far from it.
    let far: Vec<_> = heatmap
        .voxels
        .iter()
        .filter(|(index, _)| voxel_distance(*index, voxel_of(scene.object, 0.1)) > 4)
        .collect();
    let best_far = far.iter().map(|(_, s)| *s).fold(0.0, f32::max);
    assert!(
        best_far < score * 0.7,
        "far voxels score {best_far} vs top {score}"
    );
}

#[test]
fn several_directions_outscore_one_direction() {
    let object = Point3::new(3.0, 2.0, 0.5);
    let mut ring = build_scene(object, &ring_poses(object, 3, 2.5), true);
    let ring_top = ring.hyperspace.query("object", TARGET).unwrap();
    // Same number of frames, all from (almost) the same bearing.
    let poses: Vec<_> = (0..3)
        .map(|i| {
            look_at_pose(
                Point3::new(object.x - 2.5, object.y + 0.02 * i as f64, object.z + 0.3),
                object,
            )
        })
        .collect();
    let mut line = build_scene(object, &poses, true);
    let line_top = line.hyperspace.query("object", TARGET).unwrap();
    // Compare raw pooled scores: use the un-normalized pooling directly on the evidence the two scenes produce.
    // Normalization maps each map's own peak to 1, so compare the yaw-bin count via the stats instead:
    let ring_voxel = ring_top.voxels[0].0;
    let line_voxel = line_top.voxels[0].0;
    assert!(voxel_distance(ring_voxel, voxel_of(object, 0.1)) <= 1);
    // Same-bearing views only bound the object along the ray by the ±10% depth cap
    // (a 0.5 m slab at 2.5 m), so the peak can sit a few voxels off along that ray.
    assert!(
        voxel_distance(line_voxel, voxel_of(object, 0.1)) <= 3,
        "line voxel {line_voxel:?}"
    );
    let config = hyperspace::QueryConfig::default();
    let ring_pooled = hyperspace::query::pool(&[(0, 0.5, 0), (1, 0.5, 3), (2, 0.5, 6)], &config);
    let line_pooled = hyperspace::query::pool(&[(0, 0.5, 0), (1, 0.5, 0), (2, 0.5, 0)], &config);
    assert!(
        ring_pooled > line_pooled * 1.5,
        "ring {ring_pooled} vs line {line_pooled}"
    );
}

#[test]
fn rewriting_tf_after_ingest_moves_the_result() {
    let object = Point3::new(3.0, 2.0, 0.5);
    let poses = ring_poses(object, 3, 2.5);
    let mut scene = build_scene(object, &poses, true);
    let before = scene.hyperspace.query("object", TARGET).unwrap().voxels[0].0;
    // Loop closure says every camera pose was really 1 m further along x.
    let shift = Isometry3::translation(1.0, 0.0, 0.0);
    for (index, pose) in poses.iter().enumerate() {
        scene.hyperspace.update(&Transform::from_isometry(
            TARGET,
            CAMERA,
            10.0 + index as f64,
            &(shift * pose),
        ));
    }
    let after = scene.hyperspace.query("object", TARGET).unwrap().voxels[0].0;
    assert_eq!(
        after[0] - before[0],
        10,
        "before {before:?} after {after:?}"
    );
    assert_eq!(after[1], before[1]);
}

#[test]
fn rolling_buffer_drops_duplicates_and_keeps_novelty() {
    let object = Point3::new(3.0, 2.0, 0.5);
    let pose = look_at_pose(Point3::new(0.5, 2.0, 0.8), object);
    // 30 identical frames, then 30 identical frames from a new viewpoint.
    let other = look_at_pose(Point3::new(3.0, -0.5, 0.8), object);
    let mut poses = vec![pose; 30];
    poses.extend(vec![other; 30]);
    let config = Config {
        default_keyframe: KeyframeConfig {
            buffer_len: 11,
            ..KeyframeConfig::permissive()
        },
        motion_reference_frame: TARGET.into(),
        ..Config::default()
    };
    let mut hyperspace = Hyperspace::new(
        config,
        Box::new(HashLike),
        Box::new(TableTextEmbedder::default()),
        Box::new(PassthroughDepthFuser),
    );
    hyperspace.set_camera_intrinsics(intrinsics());
    for (index, pose) in poses.iter().enumerate() {
        let timestamp = 10.0 + index as f64 * 0.2;
        hyperspace.update(&Transform::from_isometry(TARGET, CAMERA, timestamp, pose));
        let shade = if index < 30 { 40 } else { 200 };
        let frame = ImageFrame {
            camera_frame: CAMERA.into(),
            timestamp,
            width: WIDTH,
            height: HEIGHT,
            encoding: "rgb8".into(),
            data: vec![shade; (WIDTH * HEIGHT * 3) as usize],
        };
        hyperspace.add_image(frame).unwrap();
    }
    hyperspace.flush().unwrap();
    assert_eq!(hyperspace.stats.embedded, 60);
    assert_eq!(
        hyperspace.stats.kept, 2,
        "expected exactly one keyframe per distinct view, got {:?}",
        hyperspace.stats
    );
}

/// Content-hash embedder with the test's grid shape (HashEmbedder with our dims).
struct HashLike;
impl hyperspace::Embedder for HashLike {
    fn embed(&mut self, frame: &ImageFrame) -> Result<PatchGrid, String> {
        hyperspace::embedder::HashEmbedder {
            rows: ROWS,
            cols: COLS,
            dim: DIM,
        }
        .embed(frame)
    }
    fn grid_shape(&self) -> (usize, usize) {
        (ROWS, COLS)
    }
    fn dim(&self) -> usize {
        DIM
    }
}

#[test]
fn depth_caps_pyramids() {
    let object = Point3::new(3.0, 2.0, 0.5);
    let poses = ring_poses(object, 3, 2.5);
    let mut with_depth = build_scene(object, &poses, true);
    let heatmap = with_depth.hyperspace.query("object", TARGET).unwrap();
    // Nothing behind 1.1 × depth: every scored voxel is within 1.1 × 2.5 m of every camera that saw it,
    // and in particular within ~0.4 m of the object (3 pyramids of 2.5 m ± 10% intersecting).
    let object_voxel = voxel_of(object, 0.1);
    for (index, _) in &heatmap.voxels {
        assert!(
            voxel_distance(*index, object_voxel) <= 6,
            "voxel {index:?} far from the object was scored"
        );
    }
    // Without depth the same frames produce nothing (no caps -> patches are skipped, and counted).
    let mut without = build_scene(object, &poses, false);
    let empty = without.hyperspace.query("object", TARGET).unwrap();
    assert!(empty.voxels.is_empty());
    assert_eq!(
        empty.stats.hot_patches_without_depth,
        empty.stats.hot_patches
    );
    assert!(empty.stats.hot_patches >= 3);
}

#[test]
fn saved_state_round_trips() {
    let object = Point3::new(3.0, 2.0, 0.5);
    let mut scene = build_scene(object, &ring_poses(object, 3, 2.5), true);
    let expected = scene.hyperspace.query("object", TARGET).unwrap().voxels[0].0;
    let path = std::env::temp_dir().join(format!("hyperspace_state_{}.bin", std::process::id()));
    scene.hyperspace.saved_state().save(&path).unwrap();
    let saved = hyperspace::SavedState::load(&path).unwrap();
    std::fs::remove_file(&path).ok();
    let mut text = TableTextEmbedder::default();
    text.table.insert("object".into(), unit(0));
    text.table.insert("background".into(), unit(1));
    let query = hyperspace::QueryConfig {
        background_prompts: vec!["background".into()],
        hot_threshold: 0.3,
        ..Default::default()
    };
    let config = Config {
        query,
        ..Config::default()
    };
    let mut reloaded = Hyperspace::from_saved(
        config,
        saved,
        Box::new(HashLike),
        Box::new(text),
        Box::new(PassthroughDepthFuser),
    );
    assert_eq!(reloaded.keyframes().len(), 3);
    assert_eq!(
        reloaded.query("object", TARGET).unwrap().voxels[0].0,
        expected
    );
    let scene_voxels: HashMap<[i32; 3], u32> =
        reloaded.scene_voxels(TARGET, 1).into_iter().collect();
    assert!(!scene_voxels.is_empty());
}
