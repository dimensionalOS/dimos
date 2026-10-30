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

//! The robot's clock mapped onto ours.

use std::collections::VecDeque;

const NANOS_PER_SEC: i64 = 1_000_000_000;
/// How long the host-clock estimate remembers arrivals. Long enough to ride
/// out a burst of late deliveries, short enough to follow device drift.
const HOST_CLOCK_WINDOW_SECS: i64 = 30;
/// An offset this far from the estimate is a clock step on one side or the
/// other, not jitter, so the offset history no longer applies.
const HOST_CLOCK_RESET_NS: i64 = NANOS_PER_SEC;

/// Maps the device clock onto the host clock.
///
/// Delivery latency only ever makes a packet arrive later, so the smallest
/// `arrival - device` offset seen recently is the closest estimate of the true
/// one. One minimum per second of arrival time keeps the window cheap. A step
/// in either clock restarts the estimate, so stamps follow the host through a
/// time adjustment instead of lagging it for a window.
#[derive(Debug, Default)]
pub struct HostClock {
    minima: VecDeque<(i64, i64)>,
    offset_ns: Option<i64>,
}

impl HostClock {
    /// Fold in one packet's device stamp and host arrival time.
    pub fn observe(&mut self, device_ns: u64, arrival_secs: f64) {
        let arrival_ns = secs_to_ns(arrival_secs);
        let offset = arrival_ns - device_ns as i64;
        if self
            .offset_ns
            .is_some_and(|estimate| (offset - estimate).abs() >= HOST_CLOCK_RESET_NS)
        {
            tracing::warn!(
                estimate_ns = self.offset_ns,
                offset_ns = offset,
                "clock offset jumped, restarting host clock estimate"
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
                    .is_some_and(|f| f.0 < second - HOST_CLOCK_WINDOW_SECS)
                {
                    self.minima.pop_front();
                }
            }
        }
        self.offset_ns = self.minima.iter().map(|m| m.1).min();
    }

    /// The device stamp on the host clock. Unchanged until the first observation.
    pub fn host_ns(&self, device_ns: u64) -> u64 {
        match self.offset_ns {
            Some(offset) => (device_ns as i64).saturating_add(offset).max(0) as u64,
            None => device_ns,
        }
    }

    /// Whether any observation has set the offset yet.
    pub fn is_synced(&self) -> bool {
        self.offset_ns.is_some()
    }
}

/// Whole and fractional seconds converted separately, so an epoch-sized value
/// keeps sub-microsecond resolution.
fn secs_to_ns(secs: f64) -> i64 {
    secs.trunc() as i64 * NANOS_PER_SEC + (secs.fract() * NANOS_PER_SEC as f64).round() as i64
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn host_clock_keeps_the_smallest_offset_in_the_window() {
        let mut clock = HostClock::default();
        assert_eq!(clock.host_ns(5), 5);
        assert!(!clock.is_synced());
        // Device at 1000 s, host at 1.7e9 s: the true offset is the first,
        // undelayed arrival. Later packets arrive late and must not move it.
        let device = 1_000_000_000_000u64;
        clock.observe(device, 1_700_000_000.0);
        clock.observe(device + 5_000_000, 1_700_000_000.009);
        clock.observe(device + 10_000_000, 1_700_000_000.030);
        assert_eq!(clock.host_ns(device), 1_700_000_000_000_000_000);
        assert_eq!(clock.host_ns(device + 7), 1_700_000_000_000_000_007);
    }

    fn assert_near(actual: u64, expected: i64) {
        assert!(
            (actual as i64 - expected).abs() < 1_000,
            "{actual} vs {expected}"
        );
    }

    #[test]
    fn host_clock_forgets_offsets_outside_the_window() {
        let mut clock = HostClock::default();
        let device = 1_000_000_000_000u64;
        clock.observe(device, 1_700_000_000.0);
        // The device drifts 2 ms behind, so every later arrival carries a
        // larger offset. Inside the window the old minimum still wins.
        let mid_device = device + 10_000_000_000;
        clock.observe(mid_device, 1_700_000_010.002);
        assert_near(clock.host_ns(mid_device), 1_700_000_010_000_000_000);
        // Past the window the old minimum is gone and the estimate follows.
        let late_device = device + 60_000_000_000;
        clock.observe(late_device, 1_700_000_060.002);
        assert_near(clock.host_ns(late_device), 1_700_000_060_002_000_000);
    }

    #[test]
    fn host_clock_ignores_small_backwards_steps() {
        let mut clock = HostClock::default();
        let device = 1_000_000_000_000u64;
        clock.observe(device, 1_700_000_000.0);
        // Half a second back is reordering, not a reset, so the minimum holds.
        clock.observe(device - 500_000_000, 1_700_000_000.01);
        assert_eq!(clock.host_ns(device), 1_700_000_000_000_000_000);
    }

    #[test]
    fn host_clock_restarts_after_a_device_reset() {
        let mut clock = HostClock::default();
        clock.observe(1_000_000_000_000, 1_700_000_000.0);
        // The sensor reboots: device time drops to 2 s while the host moves on.
        clock.observe(2_000_000_000, 1_700_000_100.0);
        assert_eq!(clock.host_ns(2_000_000_000), 1_700_000_100_000_000_000);
    }

    #[test]
    fn host_clock_follows_a_host_clock_step() {
        // Either direction: the stamps step with the host at the next packet
        // instead of holding the stale minimum for the rest of the window.
        for step in [5.0, -2.0] {
            let mut clock = HostClock::default();
            let device = 1_000_000_000_000u64;
            for i in 0..10u64 {
                clock.observe(device + i * 5_000_000, 1_700_000_000.0 + i as f64 * 0.005);
            }
            let stepped_device = device + 50_000_000;
            let stepped_arrival = 1_700_000_000.05 + step;
            clock.observe(stepped_device, stepped_arrival);
            assert_near(clock.host_ns(stepped_device), secs_to_ns(stepped_arrival));
        }
    }
}
