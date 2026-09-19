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

//! Run the stereo matcher over a saved pair, outside the module harness.
//!
//! The module itself only ever runs under a transport, which makes it awkward
//! to look at what it produces on one specific pair of frames. This drives the
//! same `stereo` code directly:
//!
//!     stereo_offline left.jpg right.jpg calib.txt out_prefix [downscale] [name=value ...]
//!
//! `ground_plane=1` turns on the floor-flattening path (see `ground_plane`),
//! which is off by default so that the raw matcher stays exactly what it was.
//!
//! Writes `<prefix>_depth.bin` (f32 metres, row-major) and `<prefix>_cloud.bin`
//! (x,y,z f32 triples), plus a one-line summary on stdout.
//!
//! `calib.txt` carries **both** eyes, one per line, each as
//!
//!     width height fx fy cx cy k1 k2 p1 p2 k3 k4 k5 k6
//!
//! at the resolution the calibration was taken at (see
//! [`dimos_depth_cloud::calibration`], which also reads a `depth_eval`
//! `manifest.json`). Both eyes are required, with their own distortion,
//! because that is what rectification needs: an earlier version of this tool
//! took one focal length and no distortion at all, used them for both eyes,
//! and so measured a pipeline the robot does not run. On the R1's head that
//! alone read the floor about a third too far away, which is a
//! fifty-centimetre error at four metres -- larger than anything it was being
//! used to investigate.

use std::env;
use std::fs;
use std::path::Path;

use dimos_depth_cloud::calibration::read_pair;
use dimos_depth_cloud::ground_plane::{fit_ground, flatten_ground, GroundParams};
use dimos_depth_cloud::stereo::{
    disparity_to_depth, downsample, horizontal_fov, match_stereo, rectify_pair_rotated, to_gray,
    MatchParams, Rotation,
};

fn decode(path: &str) -> (Vec<u8>, usize, usize) {
    let bytes = fs::read(path).unwrap_or_else(|e| panic!("cannot read {path}: {e}"));
    let image = turbojpeg::decompress(&bytes, turbojpeg::PixelFormat::GRAY)
        .unwrap_or_else(|e| panic!("cannot decode {path}: {e}"));
    (image.pixels, image.width, image.height)
}

fn main() {
    let args: Vec<String> = env::args().collect();
    assert!(
        args.len() >= 5,
        "usage: stereo_offline L R calib.txt prefix [downscale] [name=value ...]\n\
         parameters: uniqueness, max_lr_difference, min_region, speckle_max_step,\n\
         disparity_range, p1, p2, baseline_m, diagonal_paths,\n\
         right_roll_rad, right_pitch_rad, right_yaw_rad,\n\
         max_depth_m, ground_plane, ground_tolerance_px, ground_horizon,\n\
         ground_max_depth_change"
    );
    let prefix = &args[4];
    let factor: usize = args.get(5).and_then(|v| v.parse().ok()).unwrap_or(4);
    let (left_calibration, right_calibration) =
        read_pair(Path::new(&args[3])).unwrap_or_else(|error| panic!("{error}"));

    let (lp, lw, lh) = decode(&args[1]);
    let (rp, rw, rh) = decode(&args[2]);
    assert_eq!((lw, lh), (rw, rh), "the pair must agree in size");

    let left = to_gray(&lp, lw, lh, 1).expect("gray");
    let right = to_gray(&rp, rw, rh, 1).expect("gray");
    let left_small = downsample(&left, factor);
    let right_small = downsample(&right, factor);
    let (width, height) = (left_small.width, left_small.height);

    // The confidence gates, overridable from the command line so they can be
    // swept against a score rather than guessed. Named rather than positional,
    // because a sweep that silently shifts one argument onto another is the one
    // mistake that would invalidate every number it produced.
    let mut params = MatchParams::default();
    let mut baseline_m: f32 = 0.120195;
    let mut right_rotation = Rotation::IDENTITY;
    // Far clip for the cloud. Not a quality gate -- it is only here to keep a
    // divide-by-almost-zero disparity from writing a point a kilometre away --
    // so it must sit well beyond anything being measured. It used to be 6 m,
    // which was exactly the top of the range the floor is scored over, and that
    // censored the 5-6 m band: a direction whose true range was 5.8 m produced
    // a reading only when the noise happened to fall short, so the band's
    // error came back systematically negative and the depth looked biased
    // towards the camera when it was the tool doing the biasing.
    let mut max_depth_m: f32 = 20.0;
    let mut ground_plane = false;
    let mut ground = GroundParams::default();
    for argument in args.iter().skip(5) {
        let Some((name, value)) = argument.split_once('=') else {
            continue;
        };
        match name {
            "uniqueness" => params.uniqueness = value.parse().expect("uniqueness"),
            "max_lr_difference" => {
                params.max_lr_difference = value.parse().expect("max_lr_difference")
            }
            "min_region" => params.min_region = value.parse().expect("min_region"),
            "speckle_max_step" => {
                params.speckle_max_step = value.parse().expect("speckle_max_step")
            }
            "disparity_range" => params.disparity_range = value.parse().expect("disparity_range"),
            "p1" => params.p1 = value.parse().expect("p1"),
            "p2" => params.p2 = value.parse().expect("p2"),
            "baseline_m" => baseline_m = value.parse().expect("baseline_m"),
            "diagonal_paths" => {
                params.diagonal_paths = value != "0" && !value.eq_ignore_ascii_case("false")
            }
            "right_roll_rad" => right_rotation.roll_rad = value.parse().expect("right_roll_rad"),
            "right_pitch_rad" => right_rotation.pitch_rad = value.parse().expect("right_pitch_rad"),
            "right_yaw_rad" => right_rotation.yaw_rad = value.parse().expect("right_yaw_rad"),
            "max_depth_m" => max_depth_m = value.parse().expect("max_depth_m"),
            "ground_plane" => ground_plane = value != "0" && !value.eq_ignore_ascii_case("false"),
            "ground_tolerance_px" => {
                ground.tolerance_px = value.parse().expect("ground_tolerance_px")
            }
            "ground_horizon" => ground.horizon = value.parse().expect("ground_horizon"),
            "ground_max_depth_change" => {
                ground.max_depth_change = value.parse().expect("ground_max_depth_change")
            }
            other => panic!("unknown parameter {other:?}"),
        }
    }
    // The exact path the live module takes, from the same function, so that a
    // number measured here is a number about the robot.
    let rectification = rectify_pair_rotated(
        &left_calibration.scaled(lw, lh, factor),
        &right_calibration.scaled(lw, lh, factor),
        width,
        height,
        right_rotation,
    );
    let lr = rectification.left.apply(&left_small);
    let rr = rectification.right.apply(&right_small);

    let mut disparity = match_stereo(&lr, &rr, &params);
    let mut ground_note = String::new();
    if ground_plane {
        match fit_ground(&disparity, width, height, &ground) {
            Some(plane) => {
                let moved = flatten_ground(&mut disparity, width, height, &plane, &ground);
                ground_note = format!(
                    " ground={moved}px alpha={:.5} beta={:.5} gamma={:.3}",
                    plane.alpha, plane.beta, plane.gamma
                );
            }
            None => ground_note = " ground=none".to_string(),
        }
    }
    let rfx = rectification.fx;
    let depth = disparity_to_depth(&disparity, rfx, baseline_m, 1.0);

    let mut depth_bytes = Vec::with_capacity(depth.len() * 4);
    for d in &depth {
        depth_bytes.extend_from_slice(&d.to_le_bytes());
    }
    fs::write(format!("{prefix}_depth.bin"), &depth_bytes).unwrap();

    let mut cloud = Vec::new();
    let mut valid = 0usize;
    let (mut near, mut far) = (f32::INFINITY, 0.0f32);
    for row in 0..height {
        for column in 0..width {
            let z = depth[row * width + column];
            if !(z.is_finite() && z > 0.3 && z < max_depth_m) {
                continue;
            }
            valid += 1;
            near = near.min(z);
            far = far.max(z);
            let x = (column as f32 - rectification.cx) * z / rfx;
            let y = (row as f32 - rectification.cy) * z / rectification.fy;
            cloud.extend_from_slice(&x.to_le_bytes());
            cloud.extend_from_slice(&y.to_le_bytes());
            cloud.extend_from_slice(&z.to_le_bytes());
        }
    }
    fs::write(format!("{prefix}_cloud.bin"), &cloud).unwrap();

    println!(
        "{width}x{height} rectified fx={rfx:.2} fov={:.1}deg baseline={baseline_m} \
         valid={valid}/{} near={near:.2} far={far:.2}{ground_note}",
        horizontal_fov(rfx as f64, width),
        depth.len(),
    );
}
