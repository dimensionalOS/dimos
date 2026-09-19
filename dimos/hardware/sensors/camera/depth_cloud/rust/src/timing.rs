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

//! Per-stage timing of the stereo path, for the live module and the bench.
//!
//! The question this answers is "which stage is the robot's Orin actually
//! spending its frame on", and the answer changes with every config knob
//! (downscale, disparity range, the denoise chain), so it is measured rather
//! than remembered. A rolling window of the last frames, summarised as
//! medians: a median ignores the one frame that hit a page fault, which is
//! the frame a mean would be dominated by.

use std::time::{Duration, Instant};

/// The stages of one frame, in the order they run.
pub const STAGES: [&str; 5] = ["decode", "rectify", "match", "denoise", "unproject"];

/// Milliseconds spent in each of [`STAGES`] for one frame.
pub type StageMs = [f32; STAGES.len()];

/// Lap timer: each `lap()` is the time since the previous one.
pub struct Stopwatch {
    last: Instant,
}

impl Stopwatch {
    pub fn start() -> Self {
        Self {
            last: Instant::now(),
        }
    }

    pub fn lap_ms(&mut self) -> f32 {
        let now = Instant::now();
        let elapsed = now.duration_since(self.last);
        self.last = now;
        elapsed.as_secs_f32() * 1000.0
    }
}

/// The last `capacity` frames' stage times and arrival instants.
///
/// A ring rather than a queue so a frame costs one write and no allocation;
/// the summary is what does the sorting, and it runs a few times a minute.
pub struct Window {
    frames: Vec<(Instant, StageMs)>,
    next: usize,
    capacity: usize,
    last_report: Instant,
}

impl Default for Window {
    /// 150 frames is about half a minute at the R1's 4-5 Hz: long enough
    /// that a 5 s report is not dominated by the frames since the last one,
    /// short enough to see a config change take effect.
    fn default() -> Self {
        Self::with_capacity(150)
    }
}

/// What a report says about the window.
#[derive(Debug, Clone, PartialEq)]
pub struct Summary {
    pub frames: usize,
    /// Frames per second over the window's span; 0 for a window of one.
    pub fps: f32,
    pub median_ms: StageMs,
}

impl Window {
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            frames: Vec::with_capacity(capacity),
            next: 0,
            capacity: capacity.max(2),
            last_report: Instant::now(),
        }
    }

    pub fn push(&mut self, stages: StageMs) {
        self.push_at(Instant::now(), stages);
    }

    fn push_at(&mut self, at: Instant, stages: StageMs) {
        if self.frames.len() < self.capacity {
            self.frames.push((at, stages));
        } else {
            self.frames[self.next] = (at, stages);
        }
        self.next = (self.next + 1) % self.capacity;
    }

    pub fn summary(&self) -> Option<Summary> {
        if self.frames.is_empty() {
            return None;
        }
        let (oldest, newest) =
            self.frames
                .iter()
                .fold((None, None), |(oldest, newest), (at, _)| {
                    (
                        Some(oldest.map_or(*at, |o: Instant| o.min(*at))),
                        Some(newest.map_or(*at, |n: Instant| n.max(*at))),
                    )
                });
        let span = newest?.duration_since(oldest?).as_secs_f32();
        let fps = if self.frames.len() > 1 && span > 0.0 {
            (self.frames.len() - 1) as f32 / span
        } else {
            0.0
        };
        let mut median_ms = [0.0f32; STAGES.len()];
        let mut held = Vec::with_capacity(self.frames.len());
        for (stage, slot) in median_ms.iter_mut().enumerate() {
            held.clear();
            held.extend(self.frames.iter().map(|(_, ms)| ms[stage]));
            *slot = median(&mut held);
        }
        Some(Summary {
            frames: self.frames.len(),
            fps,
            median_ms,
        })
    }

    /// `Some` once per `every`, so the caller can log on a clock rather than
    /// on a frame count that would drift with the frame rate.
    pub fn report_due(&mut self, every: Duration) -> Option<Summary> {
        if self.last_report.elapsed() < every {
            return None;
        }
        self.last_report = Instant::now();
        self.summary()
    }
}

/// The median of `values`, reordering them in place. NaN for an empty slice.
pub fn median(values: &mut [f32]) -> f32 {
    if values.is_empty() {
        return f32::NAN;
    }
    let middle = values.len() / 2;
    values.select_nth_unstable_by(middle, |a, b| a.total_cmp(b));
    values[middle]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn median_of_an_odd_count_is_the_middle_value() {
        assert_eq!(median(&mut [5.0, 1.0, 3.0]), 3.0);
    }

    #[test]
    fn median_ignores_the_one_slow_frame_a_mean_would_not() {
        let mut ms = vec![10.0f32; 9];
        ms.push(1000.0);
        assert_eq!(median(&mut ms), 10.0);
    }

    #[test]
    fn the_window_keeps_only_the_last_frames() {
        let mut window = Window::with_capacity(3);
        for value in [1.0f32, 2.0, 3.0, 4.0, 5.0] {
            window.push([value; STAGES.len()]);
        }
        let summary = window.summary().expect("has frames");
        assert_eq!(summary.frames, 3);
        // The survivors are 3, 4, 5.
        assert_eq!(summary.median_ms[0], 4.0);
    }

    #[test]
    fn fps_is_frames_over_the_windows_span() {
        let mut window = Window::with_capacity(10);
        let start = Instant::now();
        for frame in 0..5 {
            window.push_at(
                start + Duration::from_millis(100 * frame),
                [1.0; STAGES.len()],
            );
        }
        // Four intervals of 100 ms between five frames.
        let summary = window.summary().expect("has frames");
        assert!((summary.fps - 10.0).abs() < 1e-3, "{}", summary.fps);
    }

    #[test]
    fn a_window_of_one_reports_no_rate_rather_than_infinity() {
        let mut window = Window::default();
        window.push([1.0; STAGES.len()]);
        assert_eq!(window.summary().expect("has a frame").fps, 0.0);
    }

    #[test]
    fn an_empty_window_has_nothing_to_report() {
        assert!(Window::default().summary().is_none());
    }

    #[test]
    fn a_report_is_due_on_the_clock_not_on_the_frame_count() {
        let mut window = Window::default();
        window.push([1.0; STAGES.len()]);
        assert!(window.report_due(Duration::from_secs(3600)).is_none());
        assert!(window.report_due(Duration::ZERO).is_some());
    }
}
