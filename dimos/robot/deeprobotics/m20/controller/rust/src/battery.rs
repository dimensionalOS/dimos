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

//! rl_deploy's battery levels (SafeController::IsBatteryNormal 0x174e30; [battery] in m20_controller.toml).

use serde::Serialize;

use crate::config::BatteryConfig;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum BatteryLevel {
    #[default]
    Normal,
    Warning,
    Error,
}

/// One pack's reading: its voltage (V) and charge level (%).
#[derive(Debug, Clone, Copy)]
pub struct PackReading {
    pub voltage_v: f64,
    pub level_percent: u8,
}

/// The battery level the motor bus's battery frames give (a backend's part: it fills RobotState::battery).
#[derive(Debug, Clone)]
pub struct BatteryFeed {
    cfg: BatteryConfig,
    watch: BatteryWatch,
    pub level: Option<BatteryLevel>,
}

impl BatteryFeed {
    pub fn new(cfg: BatteryConfig) -> Self {
        BatteryFeed {
            cfg,
            watch: BatteryWatch::default(),
            level: None,
        }
    }

    /// A frame from the board: a battery report updates the level.
    #[cfg(feature = "ecan")]
    pub fn on_frame(&mut self, frame: &[u8]) {
        if let Ok(report) = m20_battery::BatteryReport::from_ethernet(frame) {
            let packs = report.packs().map(|p| PackReading {
                voltage_v: p.voltage_v as f64,
                level_percent: p.level_percent,
            });
            self.level = Some(self.watch.see(&self.cfg, &packs));
        }
    }
}

/// IsBatteryNormal's latched per-pack voltage errors.
#[derive(Debug, Clone, Default)]
pub struct BatteryWatch {
    voltage_error: u32,
}

impl BatteryWatch {
    /// One battery report's packs; the level they put the robot at.
    pub fn see(&mut self, cfg: &BatteryConfig, packs: &[PackReading]) -> BatteryLevel {
        let all = (1u32 << packs.len()) - 1;
        let (mut warn, mut error) = (0u32, 0u32);
        for (k, p) in packs.iter().enumerate() {
            if p.voltage_v <= cfg.voltage_error_v {
                self.voltage_error |= 1 << k;
            }
            if p.voltage_v >= cfg.voltage_clear_v {
                self.voltage_error &= !(1 << k);
            }
            if p.level_percent < cfg.warning_percent {
                warn |= 1 << k;
                if p.level_percent < cfg.error_percent {
                    error |= 1 << k;
                }
            }
        }
        if packs.is_empty() {
            BatteryLevel::Normal
        } else if self.voltage_error & all == all || error == all {
            BatteryLevel::Error
        } else if warn == all {
            BatteryLevel::Warning
        } else {
            BatteryLevel::Normal
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cfg() -> BatteryConfig {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("m20_controller.toml");
        crate::config::Config::load(&path).unwrap().battery
    }

    fn packs(v: f64, pct: u8) -> [PackReading; 2] {
        [PackReading {
            voltage_v: v,
            level_percent: pct,
        }; 2]
    }

    #[test]
    fn levels_are_is_battery_normal_s() {
        let c = cfg();
        let mut w = BatteryWatch::default();
        assert_eq!(w.see(&c, &packs(83.0, 90)), BatteryLevel::Normal);
        assert_eq!(w.see(&c, &packs(83.0, 20)), BatteryLevel::Warning);
        assert_eq!(w.see(&c, &packs(83.0, 5)), BatteryLevel::Error);
        // one pack low is not the robot low
        let mut mixed = packs(83.0, 5);
        mixed[1].level_percent = 90;
        assert_eq!(w.see(&c, &mixed), BatteryLevel::Normal);
        // 54 V latches the error until 58 V
        assert_eq!(w.see(&c, &packs(54.0, 90)), BatteryLevel::Error);
        assert_eq!(w.see(&c, &packs(57.0, 90)), BatteryLevel::Error);
        assert_eq!(w.see(&c, &packs(58.0, 90)), BatteryLevel::Normal);
    }
}
