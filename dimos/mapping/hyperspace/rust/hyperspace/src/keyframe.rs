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

//! Keyframing: pre-model quality gate, embedding-diff novelty gate, and the
//! 11-frame rolling buffer that judges its middle frame against 5 before / 5 after.

use crate::patch::PatchGrid;
use crate::ImageFrame;
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;

/// Optional image-sharpness metric plugged in per camera (Laplacian variance
/// was unreliable, so nothing is built in). Higher = sharper.
pub trait SharpnessMetric: Send {
    fn sharpness(&self, frame: &ImageFrame) -> f32;
}

/// Every knob is per camera and individually disable-able (`None`).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct KeyframeConfig {
    /// Rolling buffer length; odd. The middle frame is the candidate.
    pub buffer_len: usize,
    /// Drop frames whose camera turns faster than this (rad/s), from tf.
    pub max_angular_velocity: Option<f64>,
    /// Drop frames whose camera moves faster than this (m/s), from tf.
    pub max_linear_velocity: Option<f64>,
    /// Half-window (s) for the finite-difference velocity estimate.
    pub velocity_half_window: f64,
    /// Drop frames with more than this fraction of pixels at/below `dark_level`.
    pub max_dark_fraction: Option<f32>,
    pub dark_level: u8,
    /// Drop frames with more than this fraction of pixels at/above `bright_level`.
    pub max_bright_fraction: Option<f32>,
    pub bright_level: u8,
    /// Drop frames whose mean luma (0..255) falls outside this range.
    pub brightness_range: Option<(f32, f32)>,
    /// Never keep two keyframes closer in time than this (s).
    pub min_interval: Option<f64>,
    /// Mean per-patch (1 - cosine) vs the last kept keyframe needed to count as novel.
    pub novelty_threshold: f32,
    /// Also novel if any single patch changed by more than this (new object in a static view).
    pub patch_novelty_threshold: Option<f32>,
}

impl Default for KeyframeConfig {
    fn default() -> Self {
        KeyframeConfig {
            buffer_len: 11,
            max_angular_velocity: Some(1.5),
            max_linear_velocity: None,
            velocity_half_window: 0.05,
            max_dark_fraction: Some(0.6),
            dark_level: 8,
            max_bright_fraction: Some(0.3),
            bright_level: 247,
            brightness_range: None,
            min_interval: Some(0.1),
            novelty_threshold: 0.05,
            patch_novelty_threshold: Some(0.5),
        }
    }
}

impl KeyframeConfig {
    /// Everything off except novelty: for cameras with no blur and any exposure.
    pub fn permissive() -> Self {
        KeyframeConfig {
            max_angular_velocity: None,
            max_linear_velocity: None,
            max_dark_fraction: None,
            max_bright_fraction: None,
            brightness_range: None,
            min_interval: None,
            ..Default::default()
        }
    }
}

/// Pixel statistics used by the exposure checks.
#[derive(Debug, Clone, Copy, Default)]
pub struct ExposureStats {
    pub mean_luma: f32,
    pub dark_fraction: f32,
    pub bright_fraction: f32,
}

pub fn exposure_stats(frame: &ImageFrame, dark_level: u8, bright_level: u8) -> ExposureStats {
    let pixels = (frame.width as usize) * (frame.height as usize);
    if pixels == 0 || frame.data.len() < pixels * 3 {
        return ExposureStats::default();
    }
    let (mut sum, mut dark, mut bright) = (0u64, 0u64, 0u64);
    let stride = 4; // every 4th pixel is plenty for a gate
    let mut counted = 0u64;
    for index in (0..pixels).step_by(stride) {
        let p = &frame.data[index * 3..index * 3 + 3];
        let luma = (p[0] as u32 * 299 + p[1] as u32 * 587 + p[2] as u32 * 114) / 1000;
        sum += luma as u64;
        if luma as u8 <= dark_level {
            dark += 1;
        }
        if luma as u8 >= bright_level {
            bright += 1;
        }
        counted += 1;
    }
    ExposureStats {
        mean_luma: sum as f32 / counted as f32,
        dark_fraction: dark as f32 / counted as f32,
        bright_fraction: bright as f32 / counted as f32,
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum GateVerdict {
    Pass,
    TooFast { angular: f64, linear: f64 },
    TooDark(f32),
    TooBright(f32),
    BadBrightness(f32),
}

/// Pre-model quality gate. `speeds` is (angular rad/s, linear m/s) from tf when available.
pub fn quality_gate(
    config: &KeyframeConfig,
    frame: &ImageFrame,
    speeds: Option<(f64, f64)>,
) -> GateVerdict {
    if let Some((angular, linear)) = speeds {
        if config.max_angular_velocity.is_some_and(|max| angular > max)
            || config.max_linear_velocity.is_some_and(|max| linear > max)
        {
            return GateVerdict::TooFast { angular, linear };
        }
    }
    if config.max_dark_fraction.is_some()
        || config.max_bright_fraction.is_some()
        || config.brightness_range.is_some()
    {
        let stats = exposure_stats(frame, config.dark_level, config.bright_level);
        if config
            .max_dark_fraction
            .is_some_and(|max| stats.dark_fraction > max)
        {
            return GateVerdict::TooDark(stats.dark_fraction);
        }
        if config
            .max_bright_fraction
            .is_some_and(|max| stats.bright_fraction > max)
        {
            return GateVerdict::TooBright(stats.bright_fraction);
        }
        if config
            .brightness_range
            .is_some_and(|(low, high)| stats.mean_luma < low || stats.mean_luma > high)
        {
            return GateVerdict::BadBrightness(stats.mean_luma);
        }
    }
    GateVerdict::Pass
}

/// One embedded frame waiting in the rolling buffer.
pub struct BufferedFrame<T> {
    pub timestamp: f64,
    pub grid: PatchGrid,
    /// Higher is better; used to pick the cleanest of the novel frames in a window.
    pub quality: f32,
    /// Whatever the caller needs to finish a kept frame (pixels, depth pairing).
    pub payload: T,
}

/// Rolling buffer per camera. Push embedded frames; when the buffer is full the
/// middle frame is judged against the frames before and after it.
pub struct RollingBuffer<T> {
    pub config: KeyframeConfig,
    frames: VecDeque<BufferedFrame<T>>,
    /// Grid of the last kept keyframe (novelty is measured against it).
    last_kept: Option<PatchGrid>,
    last_kept_time: Option<f64>,
}

impl<T> RollingBuffer<T> {
    pub fn new(config: KeyframeConfig) -> Self {
        RollingBuffer {
            config,
            frames: VecDeque::new(),
            last_kept: None,
            last_kept_time: None,
        }
    }

    pub fn len(&self) -> usize {
        self.frames.len()
    }

    pub fn is_empty(&self) -> bool {
        self.frames.is_empty()
    }

    fn novelty(&self, grid: &PatchGrid) -> f32 {
        match &self.last_kept {
            None => f32::INFINITY,
            Some(last) => {
                let mean = grid.distance(last);
                match self.config.patch_novelty_threshold {
                    Some(threshold) if grid.max_patch_distance(last) > threshold => {
                        mean.max(self.config.novelty_threshold + 1e-6)
                    }
                    _ => mean,
                }
            }
        }
    }

    fn is_novel(&self, grid: &PatchGrid) -> bool {
        self.novelty(grid) > self.config.novelty_threshold
    }

    /// Push one frame. Returns a kept keyframe when the window's middle frame
    /// wins: novel vs the last kept keyframe, and the best-quality frame among
    /// the window's novel frames.
    pub fn push(&mut self, frame: BufferedFrame<T>) -> Option<BufferedFrame<T>> {
        self.frames.push_back(frame);
        if self.frames.len() < self.config.buffer_len.max(1) {
            return None;
        }
        let middle = self.frames.len() / 2;
        let kept = self.judge(middle);
        if kept {
            let winner = self.frames.remove(middle).expect("middle exists");
            self.last_kept = Some(winner.grid.clone());
            self.last_kept_time = Some(winner.timestamp);
            // Everything before the winner has been judged already (or lost to it).
            self.frames.drain(..middle);
            Some(winner)
        } else {
            self.frames.pop_front();
            None
        }
    }

    /// Judge the remaining frames at end of stream, oldest first, with whatever
    /// window is left. Returns every frame kept.
    pub fn flush(&mut self) -> Vec<BufferedFrame<T>> {
        let mut kept = Vec::new();
        while !self.frames.is_empty() {
            let middle = self.frames.len() / 2;
            if self.judge(middle) {
                let winner = self.frames.remove(middle).expect("middle exists");
                self.last_kept = Some(winner.grid.clone());
                self.last_kept_time = Some(winner.timestamp);
                self.frames.drain(..middle);
                kept.push(winner);
            } else {
                self.frames.pop_front();
            }
        }
        kept
    }

    fn judge(&self, middle: usize) -> bool {
        let candidate = &self.frames[middle];
        if let (Some(min_interval), Some(last_time)) =
            (self.config.min_interval, self.last_kept_time)
        {
            if candidate.timestamp - last_time < min_interval {
                return false;
            }
        }
        if !self.is_novel(&candidate.grid) {
            return false;
        }
        // Best quality among the window's novel frames wins; ties go to the candidate.
        self.frames
            .iter()
            .enumerate()
            .filter(|(index, frame)| *index != middle && self.is_novel(&frame.grid))
            .all(|(_, frame)| frame.quality <= candidate.quality)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn grid(value: f32) -> PatchGrid {
        // Two 2-d patches pointing along `value`'s direction on the unit circle.
        let (s, c) = value.sin_cos();
        PatchGrid::from_f32(1, 2, 2, &[c, s, c, s])
    }

    fn frame(t: f64, angle: f32, quality: f32) -> BufferedFrame<()> {
        BufferedFrame {
            timestamp: t,
            grid: grid(angle),
            quality,
            payload: (),
        }
    }

    fn config(len: usize) -> KeyframeConfig {
        KeyframeConfig {
            buffer_len: len,
            min_interval: None,
            patch_novelty_threshold: None,
            ..Default::default()
        }
    }

    #[test]
    fn static_view_keeps_one_frame() {
        let mut buffer = RollingBuffer::new(config(3));
        let mut kept = 0;
        for i in 0..20 {
            if buffer.push(frame(i as f64, 0.0, 1.0)).is_some() {
                kept += 1;
            }
        }
        kept += buffer.flush().len();
        assert_eq!(kept, 1);
    }

    #[test]
    fn novel_frames_are_kept_and_best_quality_wins() {
        let mut buffer = RollingBuffer::new(config(3));
        // window [a, b, c]: all novel, b is the middle but c is sharper -> b loses, then c wins.
        assert!(buffer.push(frame(0.0, 0.0, 1.0)).is_none());
        assert!(buffer.push(frame(1.0, 1.0, 0.5)).is_none());
        let first = buffer.push(frame(2.0, 2.0, 0.9)); // judges frame(1.0): first ever -> novel; but c has higher quality
        assert!(first.is_none());
        let second = buffer.push(frame(3.0, 2.0, 0.1)); // judges frame(2.0): novel, best quality in window -> kept
        assert_eq!(second.map(|f| f.timestamp), Some(2.0));
    }

    #[test]
    fn exposure_gate_flags_dark_frames() {
        let dark = ImageFrame {
            camera_frame: "c".into(),
            timestamp: 0.0,
            width: 4,
            height: 4,
            encoding: "rgb8".into(),
            data: vec![0; 48],
        };
        assert!(matches!(
            quality_gate(&KeyframeConfig::default(), &dark, None),
            GateVerdict::TooDark(_)
        ));
        assert_eq!(
            quality_gate(&KeyframeConfig::permissive(), &dark, None),
            GateVerdict::Pass
        );
        assert!(matches!(
            quality_gate(&KeyframeConfig::default(), &dark, Some((9.0, 0.0))),
            GateVerdict::TooFast { .. }
        ));
    }
}
