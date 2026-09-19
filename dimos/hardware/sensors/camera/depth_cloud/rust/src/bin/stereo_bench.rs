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

//! Time the module's whole per-frame path over a directory of recorded pairs.
//!
//!     stereo_bench <dir> [--passes N] [--calibration FILE] [options]
//!
//! `<dir>` holds `<n>.left.jpg` / `<n>.right.jpg` pairs (or a `frames/`
//! subdirectory that does), which is how `depth_eval`'s frame store is laid
//! out. The calibration is `--calibration`, else the first of `calib.txt`,
//! `manifest.json` beside the pairs or one directory up; both shapes are read
//! by [`dimos_depth_cloud::calibration`].
//!
//! Every stage is the function the live module calls, in the order it calls
//! them -- decode from the JPEG bytes, downsample and rectify, match, denoise,
//! unproject -- so a millisecond measured here is a millisecond about the
//! module and not about a copy of it. File reads happen before the clock
//! starts: the robot's frames arrive over a transport, not off a disk.
//!
//! The defaults are the R1 Pro's `r1pro-kronknav` blueprint, so a bare run
//! measures what the robot runs.

use std::env;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::exit;

use lcm_msgs::sensor_msgs::CompressedImage;
use lcm_msgs::std_msgs::Header;

use dimos_depth_cloud::calibration::{read_pair, Calibration};
use dimos_depth_cloud::denoise::Chain;
use dimos_depth_cloud::stereo::{
    disparity_to_depth, downsample, match_stereo, MatchParams, Rotation, StereoRectification,
};
use dimos_depth_cloud::stereo_module::{
    build_rectification, decode_pair_scaled, depth_to_image, rectified_camera_info,
};
use dimos_depth_cloud::timing::{median, StageMs, Stopwatch, STAGES};
use dimos_depth_cloud::unproject::{unproject, HeightGate, Params};

const USAGE: &str = "usage: stereo_bench <dir> [--passes N] [--calibration FILE] [--limit N]
  matcher:  --downscale 8  --disparity-range 32  --p1 8  --p2 120  --uniqueness 0.10
            --max-lr-difference 1.5  --min-region 350  --speckle-max-step 1.5
            --diagonal-paths  --min-disparity-px 1.0  --baseline-m 0.120195
            --right-roll-rad -0.002  --right-pitch-rad 0.0035  --right-yaw-rad -0.01275
  cloud:    --denoise \"median:8+plane:16:1+fill:8\"  --decimation 2
            --min-range 0.3  --max-range 6
            --min-height M  --max-height M  (base-frame height gate, off by default)
            --base-xyz X,Y,Z  --base-rpy R,P,Y  (camera optical frame in base)";

/// Everything the module's config would carry, with the R1's values.
struct Options {
    dir: PathBuf,
    passes: usize,
    calibration: Option<PathBuf>,
    limit: Option<usize>,
    downscale: usize,
    matcher: MatchParams,
    min_disparity_px: f32,
    baseline_m: f32,
    right_rotation: Rotation,
    denoise: String,
    decimation: usize,
    min_range_m: f32,
    max_range_m: f32,
    min_height_m: Option<f64>,
    max_height_m: Option<f64>,
    base_xyz_m: [f64; 3],
    base_rpy_rad: [f64; 3],
}

impl Options {
    fn r1_defaults(dir: PathBuf) -> Self {
        Self {
            dir,
            passes: 1,
            calibration: None,
            limit: None,
            downscale: 8,
            matcher: MatchParams {
                disparity_range: 32,
                ..MatchParams::default()
            },
            min_disparity_px: 1.0,
            baseline_m: 0.120195,
            right_rotation: Rotation {
                roll_rad: -0.002,
                pitch_rad: 0.0035,
                yaw_rad: -0.01275,
            },
            denoise: "median:8+plane:16:1+fill:8".to_string(),
            decimation: 2,
            min_range_m: 0.3,
            max_range_m: 6.0,
            min_height_m: None,
            max_height_m: None,
            base_xyz_m: [0.0; 3],
            base_rpy_rad: [0.0; 3],
        }
    }
}

fn fail(message: impl std::fmt::Display) -> ! {
    eprintln!("stereo_bench: {message}");
    exit(2)
}

fn parse_options(args: &[String]) -> Options {
    // The directory is the one positional argument, wherever it sits among the
    // flags.
    let dir = args
        .iter()
        .enumerate()
        .find(|(i, arg)| {
            !arg.starts_with("--")
                && !(*i > 0 && args[i - 1].starts_with("--") && args[i - 1] != "--diagonal-paths")
        })
        .map(|(_, arg)| PathBuf::from(arg))
        .unwrap_or_else(|| fail(USAGE));
    let mut options = Options::r1_defaults(dir);
    let mut rest = args.iter();
    while let Some(flag) = rest.next() {
        if !flag.starts_with("--") {
            // The positional, already taken.
            continue;
        }
        // Flags with no value first, so the value-taking arm cannot eat the
        // next flag as this one's argument.
        if flag == "--diagonal-paths" {
            options.matcher.diagonal_paths = true;
            continue;
        }
        let Some(value) = rest.next() else {
            fail(format!("{flag} needs a value\n{USAGE}"));
        };
        let number = |what: &str| -> f64 {
            value
                .parse()
                .unwrap_or_else(|_| fail(format!("{flag}: {value:?} is not a {what}")))
        };
        let triple = || -> [f64; 3] {
            let parts: Vec<f64> = value
                .split(',')
                .map(|part| {
                    part.trim()
                        .parse()
                        .unwrap_or_else(|_| fail(format!("{flag}: {value:?} is not X,Y,Z")))
                })
                .collect();
            parts
                .try_into()
                .unwrap_or_else(|_| fail(format!("{flag}: {value:?} needs three numbers")))
        };
        match flag.as_str() {
            "--passes" => options.passes = number("count") as usize,
            "--limit" => options.limit = Some(number("count") as usize),
            "--calibration" => options.calibration = Some(PathBuf::from(value)),
            "--downscale" => options.downscale = number("factor") as usize,
            "--disparity-range" => options.matcher.disparity_range = number("count") as usize,
            "--p1" => options.matcher.p1 = number("penalty") as u16,
            "--p2" => options.matcher.p2 = number("penalty") as u16,
            "--uniqueness" => options.matcher.uniqueness = number("fraction") as f32,
            "--max-lr-difference" => options.matcher.max_lr_difference = number("px") as f32,
            "--min-region" => options.matcher.min_region = number("count") as usize,
            "--speckle-max-step" => options.matcher.speckle_max_step = number("px") as f32,
            "--min-disparity-px" => options.min_disparity_px = number("px") as f32,
            "--baseline-m" => options.baseline_m = number("length") as f32,
            "--right-roll-rad" => options.right_rotation.roll_rad = number("angle"),
            "--right-pitch-rad" => options.right_rotation.pitch_rad = number("angle"),
            "--right-yaw-rad" => options.right_rotation.yaw_rad = number("angle"),
            "--denoise" => options.denoise = value.clone(),
            "--decimation" => options.decimation = number("stride") as usize,
            "--min-range" => options.min_range_m = number("length") as f32,
            "--max-range" => options.max_range_m = number("length") as f32,
            "--min-height" => options.min_height_m = Some(number("length")),
            "--max-height" => options.max_height_m = Some(number("length")),
            "--base-xyz" => options.base_xyz_m = triple(),
            "--base-rpy" => options.base_rpy_rad = triple(),
            other => fail(format!("unknown flag {other}\n{USAGE}")),
        }
    }
    if options.passes == 0 {
        fail("--passes must be at least 1");
    }
    options
}

/// The `<n>.left.jpg` / `<n>.right.jpg` pairs in `dir`, by `n`.
fn find_pairs(dir: &Path) -> Vec<(usize, PathBuf, PathBuf)> {
    let Ok(entries) = fs::read_dir(dir) else {
        return Vec::new();
    };
    let mut pairs = Vec::new();
    for entry in entries.flatten() {
        let path = entry.path();
        let Some(name) = path.file_name().and_then(|n| n.to_str()) else {
            continue;
        };
        let Some(stem) = name.strip_suffix(".left.jpg") else {
            continue;
        };
        let Ok(index) = stem.parse::<usize>() else {
            continue;
        };
        let right = dir.join(format!("{stem}.right.jpg"));
        if right.is_file() {
            pairs.push((index, path.clone(), right));
        }
    }
    pairs.sort_by_key(|(index, _, _)| *index);
    pairs
}

/// The calibration file: the one asked for, else the first that exists.
fn find_calibration(options: &Options, frames_dir: &Path) -> PathBuf {
    if let Some(path) = &options.calibration {
        return path.clone();
    }
    let candidates = [
        frames_dir.join("calib.txt"),
        frames_dir.join("manifest.json"),
        frames_dir.join("..").join("calib.txt"),
        frames_dir.join("..").join("manifest.json"),
    ];
    candidates
        .into_iter()
        .find(|path| path.is_file())
        .unwrap_or_else(|| {
            fail(format!(
                "no calib.txt or manifest.json beside {}; pass --calibration",
                frames_dir.display()
            ))
        })
}

fn jpeg(path: &Path) -> CompressedImage {
    CompressedImage {
        header: Header::default(),
        format: "jpeg".into(),
        data: fs::read(path)
            .unwrap_or_else(|e| fail(format!("cannot read {}: {e}", path.display()))),
    }
}

/// What one frame produced, beyond its timing.
struct Outcome {
    stages: StageMs,
    answered: usize,
    points: i32,
}

/// The module's `process`, minus the transport.
struct Pipeline {
    options: Options,
    chain: Chain,
    left: Calibration,
    right: Calibration,
    rectification: Option<StereoRectification>,
}

impl Pipeline {
    fn run(&mut self, left: &CompressedImage, right: &CompressedImage) -> Outcome {
        let mut stopwatch = Stopwatch::start();
        let factor = self.options.downscale.max(1);
        let (left_decoded, right_decoded) = decode_pair_scaled(left, right, factor);
        let left_decoded = left_decoded.unwrap_or_else(|| fail("left decode"));
        let right_decoded = right_decoded.unwrap_or_else(|| fail("right decode"));
        let decode_ms = stopwatch.lap_ms();

        let left_small = downsample(&left_decoded.gray, left_decoded.remaining);
        let right_small = downsample(&right_decoded.gray, right_decoded.remaining);
        if self.rectification.is_none() {
            self.rectification = build_rectification(
                &self.left.camera_info(),
                &self.right.camera_info(),
                left_decoded.full_width,
                left_decoded.full_height,
                factor,
                self.options.right_rotation,
            );
        }
        let rectification = self
            .rectification
            .as_ref()
            .unwrap_or_else(|| fail("the calibration has no usable intrinsics"));
        let left_rect = rectification.left.apply(&left_small);
        let right_rect = rectification.right.apply(&right_small);
        let rectify_ms = stopwatch.lap_ms();

        let disparity = match_stereo(&left_rect, &right_rect, &self.options.matcher);
        let depth = disparity_to_depth(
            &disparity,
            rectification.fx,
            self.options.baseline_m,
            self.options.min_disparity_px,
        );
        let match_ms = stopwatch.lap_ms();

        let depth = self.chain.apply(
            &depth,
            rectification.width,
            rectification.height,
            rectification.fx,
        );
        let denoise_ms = stopwatch.lap_ms();

        let image = depth_to_image(
            &depth,
            rectification.width,
            rectification.height,
            &left.header,
            "camera_head_left_link",
        );
        let info = rectified_camera_info(rectification, &self.left.camera_info());
        let params = Params {
            decimation: self.options.decimation,
            min_range_m: self.options.min_range_m,
            max_range_m: self.options.max_range_m,
            depth_scale: 1.0,
            height_gate: HeightGate::new(
                self.options.base_xyz_m,
                self.options.base_rpy_rad,
                self.options.min_height_m,
                self.options.max_height_m,
            ),
        };
        let (_, points) = unproject(&image, &info, &params).unwrap_or_else(|e| fail(e));
        let unproject_ms = stopwatch.lap_ms();

        Outcome {
            stages: [decode_ms, rectify_ms, match_ms, denoise_ms, unproject_ms],
            answered: depth.iter().filter(|z| z.is_finite()).count(),
            points,
        }
    }
}

/// Median, min and max of each stage and of the whole frame, over `outcomes`.
fn report(label: &str, outcomes: &[Outcome]) {
    println!("{label}: {} frames", outcomes.len());
    println!(
        "  {:<10} {:>9} {:>9} {:>9}",
        "stage", "median", "min", "max"
    );
    let row = |name: &str, values: &mut Vec<f32>| {
        let (min, max) = values
            .iter()
            .fold((f32::INFINITY, f32::NEG_INFINITY), |(lo, hi), v| {
                (lo.min(*v), hi.max(*v))
            });
        println!(
            "  {name:<10} {:>9.2} {:>9.2} {:>9.2}",
            median(values),
            min,
            max
        );
    };
    for (stage, name) in STAGES.iter().enumerate() {
        let mut values: Vec<f32> = outcomes.iter().map(|o| o.stages[stage]).collect();
        row(name, &mut values);
    }
    let mut frame: Vec<f32> = outcomes
        .iter()
        .map(|o| o.stages.iter().sum::<f32>())
        .collect();
    row("frame", &mut frame);
    let frame_median = median(&mut frame);
    let mut answered: Vec<f32> = outcomes.iter().map(|o| o.answered as f32).collect();
    let mut points: Vec<f32> = outcomes.iter().map(|o| o.points as f32).collect();
    println!(
        "  {:.2} fps at the median frame; median {} depth pixels answered, {} cloud points",
        1000.0 / frame_median,
        median(&mut answered) as usize,
        median(&mut points) as usize,
    );
}

fn main() {
    let args: Vec<String> = env::args().skip(1).collect();
    let options = parse_options(&args);

    let mut frames_dir = options.dir.clone();
    let mut pairs = find_pairs(&frames_dir);
    if pairs.is_empty() {
        frames_dir = options.dir.join("frames");
        pairs = find_pairs(&frames_dir);
    }
    if pairs.is_empty() {
        fail(format!(
            "no <n>.left.jpg / <n>.right.jpg pairs in {} or its frames/",
            options.dir.display()
        ));
    }
    if let Some(limit) = options.limit {
        pairs.truncate(limit);
    }
    let calibration_path = find_calibration(&options, &frames_dir);
    let (left, right) = read_pair(&calibration_path).unwrap_or_else(|error| fail(error));
    let chain = Chain::parse(&options.denoise).unwrap_or_else(|error| fail(error));

    println!(
        "{} pairs from {}, calibration {}, downscale {}, disparity_range {}, denoise {}, \
         decimation {}, {} pass(es)",
        pairs.len(),
        frames_dir.display(),
        calibration_path.display(),
        options.downscale,
        options.matcher.disparity_range,
        chain.name(),
        options.decimation,
        options.passes,
    );

    // Read before the clock: the module receives bytes, it does not open files.
    let frames: Vec<(CompressedImage, CompressedImage)> = pairs
        .iter()
        .map(|(_, left, right)| (jpeg(left), jpeg(right)))
        .collect();

    let passes = options.passes;
    let mut pipeline = Pipeline {
        options,
        chain,
        left,
        right,
        rectification: None,
    };
    let mut all = Vec::with_capacity(frames.len() * passes);
    for pass in 1..=passes {
        let outcomes: Vec<Outcome> = frames
            .iter()
            .map(|(left, right)| pipeline.run(left, right))
            .collect();
        report(&format!("pass {pass}"), &outcomes);
        all.extend(outcomes);
    }
    if passes > 1 {
        report("overall", &all);
    }
}
