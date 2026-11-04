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

//! The Go2's clock on the host's. A stream's own stamps (a data-channel message's) are moved onto the host clock by the smallest
//! arrival - stamp seen in a sliding window: the message that crossed fastest, so neither a wrong
//! robot clock nor queueing moves it.

use std::cmp::Reverse;
use std::collections::{BinaryHeap, VecDeque};
use std::time::{SystemTime, UNIX_EPOCH};

const NANOS_PER_SEC: i64 = 1_000_000_000;

/// Nanoseconds on a clock that never steps (unlike now_ns under NTP), never 0; for deadlines, not stamps.
pub fn monotonic_ns() -> u64 {
    static START: std::sync::OnceLock<std::time::Instant> = std::sync::OnceLock::new();
    START
        .get_or_init(std::time::Instant::now)
        .elapsed()
        .as_nanos() as u64
        + 1
}

pub fn now_ns() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(0)
}

#[derive(Debug)]
pub struct HostClock {
    window_secs: i64,
    /// an offset this far from the estimate is a clock reset (robot reboot, host step), not jitter
    reset_ns: i64,
    /// (host second, smallest arrival - device stamp in it)
    minima: VecDeque<(i64, i64)>,
    offset_ns: Option<i64>,
}

impl HostClock {
    pub fn new(window_secs: f64, reset_secs: f64) -> Self {
        Self {
            window_secs: window_secs.ceil() as i64,
            reset_ns: (reset_secs * NANOS_PER_SEC as f64) as i64,
            minima: VecDeque::new(),
            offset_ns: None,
        }
    }

    /// Fold in one message's device stamp and host arrival time.
    pub fn observe(&mut self, device_ns: i64, arrival_ns: u64) {
        let arrival_ns = arrival_ns as i64;
        let offset = arrival_ns - device_ns;
        if self
            .offset_ns
            .is_some_and(|estimate| (offset - estimate).abs() >= self.reset_ns)
        {
            tracing::warn!(
                estimate_ns = self.offset_ns,
                offset_ns = offset,
                "Go2 clock offset jumped, restarting its estimate"
            );
            self.minima.clear();
        }
        let second = arrival_ns.div_euclid(NANOS_PER_SEC);
        match self.minima.back_mut() {
            Some(last) if last.0 == second => last.1 = last.1.min(offset),
            _ => {
                self.minima.push_back((second, offset));
                while self
                    .minima
                    .front()
                    .is_some_and(|f| f.0 < second - self.window_secs)
                {
                    self.minima.pop_front();
                }
            }
        }
        self.offset_ns = self.minima.iter().map(|m| m.1).min();
    }

    /// The device stamp on the host clock; None before the first observation.
    pub fn host_ns(&self, device_ns: i64) -> Option<u64> {
        Some(device_ns.saturating_add(self.offset_ns?).max(0) as u64)
    }

    pub fn offset_ns(&self) -> Option<i64> {
        self.offset_ns
    }
}

/// Observe and place in one: a stamped message's host time, never later than its arrival.
pub fn place(clock: &mut HostClock, device_ns: Option<i64>, arrival_ns: u64) -> u64 {
    let Some(device_ns) = device_ns.filter(|ns| *ns > 0) else {
        return arrival_ns;
    };
    clock.observe(device_ns, arrival_ns);
    clock
        .host_ns(device_ns)
        .map_or(arrival_ns, |at| at.min(arrival_ns))
}

/// Estimates when each camera frame was captured.
///
/// Example: frames 0, 1, 2, 3, 4 arrive at 0, 75, 130, 210, 280 ms. The camera shoots evenly (about every 70 ms);
/// the bumps are Wi-Fi delay, not when the picture was taken. So frame N gets `start + N x spacing`: a straight
/// line under the bumpy arrival times, resting on the fastest arrivals of the last `window`. That is the same
/// fastest-message rule HostClock uses for the pose stamps, so camera and odom stamps mean the same thing.
///
/// The Go2 sends no real capture time; its RTP timestamp goes up about 60 per frame, which is how we know N. The
/// line goes through frames whose N tops every earlier one (the very first frame is skipped, its timing is off).
/// The decoder can output frames out of order, so each decoded frame takes the smallest waiting N.
#[derive(Debug, Default)]
pub struct CaptureClock {
    max_waiting: usize,
    window_ns: u64,
    /// frames given to the decoder and not yet out of it: unwrapped ticks and their first packet's arrival
    waiting: BinaryHeap<Reverse<(i64, u64)>>,
    last_rtp: Option<u32>,
    unwrapped: i64,
    /// the frames the line goes through, newest last: (ticks, first arrival) within the window
    anchors: VecDeque<(i64, u64)>,
    first_seen: bool,
}

impl CaptureClock {
    pub fn new(max_waiting: usize, window_secs: f64) -> Self {
        Self {
            max_waiting,
            window_ns: (window_secs * NANOS_PER_SEC as f64) as u64,
            ..Self::default()
        }
    }

    fn unwrap(&mut self, rtp: u32) -> i64 {
        if let Some(last) = self.last_rtp {
            self.unwrapped += rtp.wrapping_sub(last) as i32 as i64;
        } else {
            self.unwrapped = rtp as i64;
        }
        self.last_rtp = Some(rtp);
        self.unwrapped
    }

    /// A frame went into the decoder: its RTP time and when its first packet arrived.
    pub fn submitted(&mut self, rtp: u32, first_arrival_ns: u64) {
        let ticks = self.unwrap(rtp);
        if !std::mem::replace(&mut self.first_seen, true) {
            // the very first frame goes out a reorder window early
        } else if self.anchors.back().is_none_or(|&(t, _)| ticks > t) {
            self.anchors.push_back((ticks, first_arrival_ns));
            while self
                .anchors
                .front()
                .is_some_and(|&(_, at)| at + self.window_ns < first_arrival_ns)
            {
                self.anchors.pop_front();
            }
        }
        self.waiting.push(Reverse((ticks, first_arrival_ns)));
        // a frame the decoder never hands out mustn't shift every later frame's time
        while self.waiting.len() > self.max_waiting {
            self.waiting.pop();
        }
    }

    /// The line's ns per tick and a point on it, once two anchors span some time.
    fn line(&self) -> Option<(f64, i64, i64)> {
        let (&(t_first, a_first), &(t_last, a_last)) =
            (self.anchors.front()?, self.anchors.back()?);
        if t_last <= t_first || a_last <= a_first {
            return None;
        }
        // a rough rate from the ends, then the fastest anchor in each half: the line rests on those two
        let rough = (a_last - a_first) as f64 / (t_last - t_first) as f64;
        let behind = |&(t, at): &(i64, u64)| at as f64 - (t - t_first) as f64 * rough;
        let half = self.anchors.len() / 2;
        let fastest = |range: std::collections::vec_deque::Iter<'_, (i64, u64)>| {
            range
                .min_by(|a, b| behind(a).total_cmp(&behind(b)))
                .copied()
        };
        let (t0, a0) = fastest(self.anchors.range(..half.max(1)))?;
        let (t1, a1) = fastest(self.anchors.range(half.max(1)..)).unwrap_or((t_last, a_last));
        let ns_per_tick = if t1 > t0 && a1 > a0 {
            (a1 - a0) as f64 / (t1 - t0) as f64
        } else {
            rough
        };
        let start = self
            .anchors
            .iter()
            .map(|&(t, at)| at as i64 - ((t - t0) as f64 * ns_per_tick) as i64)
            .min()?;
        Some((ns_per_tick, t0, start))
    }

    /// The capture time of the frame the decoder just handed out; its first packet's arrival until the line is known.
    pub fn decoded(&mut self) -> Option<u64> {
        let Reverse((ticks, first_arrival_ns)) = self.waiting.pop()?;
        let placed = match self.line() {
            Some((ns_per_tick, t0, start)) => {
                (start + ((ticks - t0) as f64 * ns_per_tick) as i64).max(0) as u64
            }
            None => first_arrival_ns,
        };
        Some(placed)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MS: i64 = 1_000_000;

    #[test]
    fn the_fastest_arrival_sets_the_offset() {
        let mut clock = HostClock::new(30.0, 1.0);
        assert_eq!(clock.host_ns(5), None);
        // robot 1000 s, host 1.7e9 s; the first arrives undelayed, later ones late
        let device = 1_000_000 * MS;
        let host = 1_700_000_000_000 * MS;
        clock.observe(device, host as u64);
        clock.observe(device + 5 * MS, (host + 9 * MS) as u64);
        clock.observe(device + 10 * MS, (host + 30 * MS) as u64);
        assert_eq!(clock.host_ns(device + 7), Some((host + 7) as u64));
    }

    #[test]
    fn a_robot_reboot_restarts_the_estimate() {
        let mut clock = HostClock::new(30.0, 1.0);
        let host = 1_700_000_000_000 * MS;
        clock.observe(1_000_000 * MS, host as u64);
        clock.observe(2_000 * MS, (host + 100_000 * MS) as u64);
        assert_eq!(
            clock.host_ns(2_000 * MS),
            Some((host + 100_000 * MS) as u64)
        );
    }

    #[test]
    fn place_falls_back_to_arrival_and_never_lands_after_it() {
        let mut clock = HostClock::new(30.0, 1.0);
        assert_eq!(place(&mut clock, None, 42), 42);
        assert_eq!(place(&mut clock, Some(0), 43), 43);
        assert_eq!(
            place(&mut clock, Some(1_000 * MS), (2_000 * MS) as u64),
            (2_000 * MS) as u64
        );
        // later and slower: placed by the earlier, faster one
        assert_eq!(
            place(&mut clock, Some(1_100 * MS), (2_150 * MS) as u64),
            (2_100 * MS) as u64
        );
    }

    #[test]
    fn frames_land_on_their_tick_spacing_at_the_measured_rate() {
        // 60 ticks a frame, a frame every 70 ms, arrivals jittered by 0..9 ms
        let mut clock = CaptureClock::new(16, 30.0);
        let base = 1_700_000_000_000 * MS as u64;
        let mut stamps = Vec::new();
        for i in 0..100u64 {
            let jitter = (i * 7 % 10) * MS as u64;
            clock.submitted(1_000 + 60 * i as u32, base + i * 70 * MS as u64 + jitter);
            stamps.push(clock.decoded().unwrap());
        }
        for (i, stamp) in stamps.iter().enumerate().skip(3) {
            let capture = base + i as u64 * 70 * MS as u64;
            let off = *stamp as i64 - capture as i64;
            // the first few rest on a handful of jittered arrivals; later the line is within a couple of ms
            let band = if i < 20 {
                -10 * MS..=10 * MS
            } else {
                -2 * MS..=2 * MS
            };
            assert!(band.contains(&off), "frame {i}: {off} ns");
        }
    }

    #[test]
    fn a_slow_anchor_frame_doesnt_shift_the_line() {
        // the frame the rate is anchored on arrives 40 ms late; the rest are on time
        let mut clock = CaptureClock::new(16, 30.0);
        let base = 1_700_000_000_000 * MS as u64;
        let mut last = 0;
        for i in 0..50u64 {
            let late = if i == 1 { 40 * MS as u64 } else { 0 };
            clock.submitted(60 * i as u32, base + i * 70 * MS as u64 + late);
            last = clock.decoded().unwrap();
        }
        let off = last as i64 - (base + 49 * 70 * MS as u64) as i64;
        assert!(off.abs() <= MS, "{off} ns");
    }

    #[test]
    fn a_reordered_frame_takes_the_earliest_ticks_waiting() {
        let mut clock = CaptureClock::new(16, 30.0);
        // decode order I P B: the decoder hands out I, then B (ticks 60) before P (ticks 120)
        clock.submitted(0, 1_000);
        assert_eq!(clock.decoded(), Some(1_000));
        clock.submitted(120, 2_000);
        clock.submitted(60, 2_100);
        assert_eq!(clock.decoded(), Some(2_100));
        assert_eq!(clock.decoded(), Some(2_000));
    }
}
