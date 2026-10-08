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
//! Moves a driver's capture timestamps onto the wall clock exactly.
//!
//! V4L2 stamps each frame on a clock the driver picks: CLOCK_MONOTONIC for uvcvideo, the ARM generic timer
//! (CNTVCT, ~14 s ahead) for the Jetson's GMSL capture, though its buffers claim CLOCK_MONOTONIC. The first
//! frame picks whichever clock reads within a second after its stamp; each stamp is then shifted by that clock's
//! offset to the wall clock, read between two reads of the clock and kept only when nothing ran in between.

use std::time::{SystemTime, UNIX_EPOCH};

/// Longest gap between the two clock reads around a wall read for that offset to be trusted.
const MAX_BRACKET_S: f64 = 1e-4;

pub type Clock = fn() -> f64;

pub fn monotonic_s() -> f64 {
    let mut now = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    // SAFETY: now is a valid timespec to write into.
    unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut now) };
    now.tv_sec as f64 + now.tv_nsec as f64 * 1e-9
}

/// Seconds on the ARM generic timer.
#[cfg(target_arch = "aarch64")]
pub fn soc_counter_s() -> f64 {
    let (count, hz): (u64, u64);
    // SAFETY: both registers are readable from user space on Linux; isb orders the counter read.
    unsafe {
        std::arch::asm!("isb", "mrs {}, cntvct_el0", out(reg) count);
        std::arch::asm!("mrs {}, cntfrq_el0", out(reg) hz);
    }
    count as f64 / hz as f64
}

pub fn wall_s() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs_f64())
        .unwrap_or(0.0)
}

/// Clocks a V4L2 driver may stamp frames on.
pub fn driver_clocks() -> Vec<Clock> {
    #[cfg(target_arch = "aarch64")]
    return vec![monotonic_s, soc_counter_s];
    #[cfg(not(target_arch = "aarch64"))]
    vec![monotonic_s]
}

pub struct CaptureClock {
    clocks: Vec<Clock>,
    wall: Clock,
    clock: Option<Clock>,
    wall_minus_clock_s: Option<f64>,
}

impl CaptureClock {
    pub fn new(clocks: Vec<Clock>, wall: Clock) -> Self {
        Self {
            clocks,
            wall,
            clock: None,
            wall_minus_clock_s: None,
        }
    }

    /// Wall-clock time (s) of a frame the driver stamped `capture_s`; None if no known clock matches.
    pub fn stamp(&mut self, capture_s: f64) -> Option<f64> {
        if self.clock.is_none() {
            self.clock = self
                .clocks
                .iter()
                .copied()
                .find(|c| (0.0..1.0).contains(&(c() - capture_s)));
        }
        let clock = self.clock?;
        let (before, wall, after) = (clock(), (self.wall)(), clock());
        if self.wall_minus_clock_s.is_none() || after - before < MAX_BRACKET_S {
            self.wall_minus_clock_s = Some(wall - (before + after) / 2.0);
        }
        Some(capture_s + self.wall_minus_clock_s?)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::Cell;

    thread_local! { static NOW: Cell<f64> = const { Cell::new(500.0) }; }
    fn monotonic() -> f64 {
        NOW.with(Cell::get)
    }
    fn soc_counter() -> f64 {
        NOW.with(Cell::get) + 14.0
    }
    fn wall() -> f64 {
        NOW.with(Cell::get) + 1000.0
    }

    #[test]
    fn stamps_move_by_the_driver_clock_offset_not_by_arrival() {
        let mut clock = CaptureClock::new(vec![monotonic, soc_counter], wall);
        let stamps: Vec<f64> = [
            (514.0, 0.030),
            (514.0 + 1.0 / 30.0, 0.009),
            (514.0 + 2.0 / 30.0, 0.050),
        ]
        .iter()
        .map(|&(capture, delay)| {
            NOW.with(|now| now.set(capture - 14.0 + delay));
            clock.stamp(capture).unwrap()
        })
        .collect();
        // Frame k started at 500 + k/30 on CLOCK_MONOTONIC, however late it was read.
        for (k, stamp) in stamps.iter().enumerate() {
            assert!(
                (stamp - (1500.0 + k as f64 / 30.0)).abs() < 1e-9,
                "frame {k}: {stamp}"
            );
        }
    }

    #[test]
    fn an_unknown_clock_gives_no_stamp() {
        NOW.with(|now| now.set(500.0));
        assert!(CaptureClock::new(vec![monotonic], wall)
            .stamp(9999.0)
            .is_none());
    }
}
