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

//! A loop closure's warp run on a copy of the map while scans land in the live one and a cache, then swapped in with the cached scans replayed, one warp at a time so no scan is warped twice.

use std::collections::VecDeque;

use crate::mapper::{Mapper, Point, Pose};
use crate::reform::Warp;

/// A scan as the map took it, kept to replay onto a warped copy.
struct CachedFrame {
    /// The newest warp's number when the scan came in.
    epoch: u64,
    stamp: f64,
    points: Vec<Point>,
    pose: Pose,
}

/// A warp to run off the scan path: its number, the copy of the map to warp, and the warp.
pub struct WarpJob {
    pub epoch: u64,
    pub map: Mapper,
    pub warp: Warp,
}

#[derive(Default)]
pub struct BackgroundWarp {
    /// Warps received so far, which numbers the frame new scans are placed in.
    received: u64,
    /// The warp running in the background, if any.
    running: Option<u64>,
    waiting: VecDeque<(u64, Warp)>,
    cache: Vec<CachedFrame>,
}

impl BackgroundWarp {
    /// A closure's warp: the job to start now, or None when it queues behind the running one.
    pub fn on_warp(&mut self, warp: Warp, live: &Mapper) -> Option<WarpJob> {
        self.received += 1;
        if self.running.is_some() {
            self.waiting.push_back((self.received, warp));
            return None;
        }
        self.running = Some(self.received);
        Some(WarpJob {
            epoch: self.received,
            map: live.clone(),
            warp,
        })
    }

    /// A scan about to land in the live map, cached while a warp runs.
    pub fn on_frame(&mut self, stamp: f64, points: &[Point], pose: Pose) {
        if self.running.is_some() {
            self.cache.push(CachedFrame {
                epoch: self.received,
                stamp,
                points: points.to_vec(),
                pose,
            });
        }
    }

    /// The running warp finished on `warped`: make it live, replay the scans placed in its frame, and start the next waiting warp on a copy taken before later frames' scans replay.
    pub fn on_done(&mut self, epoch: u64, warped: Mapper, live: &mut Mapper) -> Option<WarpJob> {
        debug_assert_eq!(self.running, Some(epoch));
        *live = warped;
        let cache = std::mem::take(&mut self.cache);
        let (this_frame, later): (Vec<_>, Vec<_>) =
            cache.into_iter().partition(|frame| frame.epoch <= epoch);
        replay(live, &this_frame);
        let next = self.waiting.pop_front().map(|(next_epoch, warp)| WarpJob {
            epoch: next_epoch,
            map: live.clone(),
            warp,
        });
        replay(live, &later);
        self.running = next.as_ref().map(|job| job.epoch);
        if self.running.is_some() {
            self.cache = later;
        }
        next
    }
}

fn replay(live: &mut Mapper, frames: &[CachedFrame]) {
    for frame in frames {
        live.set_clock(frame.stamp);
        live.add_frame(frame.points.clone(), frame.pose);
    }
}

#[cfg(test)]
mod tests;
