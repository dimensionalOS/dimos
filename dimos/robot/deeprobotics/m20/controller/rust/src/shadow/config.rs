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

//! shadow.toml: where the inputs come from and where the log goes.

use std::path::Path;

use anyhow::{Context, Result};
use serde::Deserialize;

#[derive(Debug, Clone, Deserialize)]
pub struct CommandWriter {
    /// RTPS writer entity id, 8 hex digits
    pub entity: String,
    /// f32 index (after the 16-byte stamp) of x, y, yaw and, if present, z, roll, pitch
    pub x: usize,
    pub y: usize,
    pub yaw: usize,
    #[serde(default)]
    pub z: Option<usize>,
    #[serde(default)]
    pub roll: Option<usize>,
    #[serde(default)]
    pub pitch: Option<usize>,
    /// true: m/s and rad/s (divided by the runner's real_steer_divisor); false: normalized axes
    pub metric: bool,
}

/// A stand gait's pose input fed from a normalized steer axis: param = scale x axis.
#[derive(Debug, Clone, Deserialize)]
pub struct PoseInput {
    pub gait: u32,
    pub param: String,
    pub axis: String,
    pub scale: f64,
}

#[derive(Debug, Clone, Deserialize)]
pub struct ShadowConfig {
    /// the shim's live stream (it streams while a socket exists here)
    pub tap_socket: String,
    pub log_dir: String,
    /// start a new log file past this size, and delete the oldest past the total
    pub file_mb: u64,
    pub max_total_mb: u64,
    /// cycles are processed this long after they arrive, so the journal's lines for them are in
    pub lookahead_s: f64,
    /// the policy reads the joint state this many cycles before the one its targets first go out on
    pub state_lag: usize,
    /// interfaces the DDS sniffer listens on (never the motor bus)
    pub sniff_interfaces: Vec<String>,
    pub imu_port: u16,
    /// the kernel filter passes only UDP to this port range (RTPS)
    pub dds_port_range: [u16; 2],
    /// cycles held for processing at most; past it the oldest are dropped (counted) so memory stays bounded
    pub max_pending_cycles: usize,
    pub imu_sample_bytes: usize,
    pub command_writers: Vec<CommandWriter>,
    pub command_timeout_s: f64,
    pub pose_inputs: Vec<PoseInput>,
    /// journalctl -u this
    pub journal_unit: String,
    /// rl_deploy's own log (its state transitions survive a journal reset); {date} = YYYY_MMDD, local time
    pub rl_log: String,
    /// the AOS clock's offset from UTC (its log stamps are local time)
    pub utc_offset_s: i64,
    /// rl_deploy's enable frames' control word, and its "stand-in" gait
    pub enable_word: u16,
    pub stand_in_gait: u32,
    pub finetune_height_gait: u32,
    /// CPUs and nice level for every thread
    pub cpus: Vec<usize>,
    pub nice: i32,
    /// print a status line to stderr every this many cycles
    pub status_every: u64,
    /// IMU / steer history kept for lookups (s); journal events keep the newest one before this window too
    #[serde(default = "default_keep_s")]
    pub keep_s: f64,
    /// a segment's state is the journal's last transition up to this long after its first enable frame (s)
    #[serde(default = "default_state_lookup_slack_s")]
    pub state_lookup_slack_s: f64,
    /// control ticks run for cycles the shim missed (a gap in the stream), at most this many
    #[serde(default = "default_max_catchup_ticks")]
    pub max_catchup_ticks: u64,
    /// longest wait on the shim's stream while it is silent: bounds how late a stop or the last cycles are handled
    #[serde(default = "default_tap_idle_timeout_ms")]
    pub tap_idle_timeout_ms: u64,
    /// journal lines read back at start (the current state and gait), and lines of rl_deploy's own log
    #[serde(default = "default_journal_history_lines")]
    pub journal_history_lines: usize,
    #[serde(default = "default_rl_log_history_lines")]
    pub rl_log_history_lines: usize,
}

fn default_keep_s() -> f64 {
    30.0
}
fn default_state_lookup_slack_s() -> f64 {
    0.01
}
fn default_max_catchup_ticks() -> u64 {
    50
}
fn default_tap_idle_timeout_ms() -> u64 {
    200
}
fn default_journal_history_lines() -> usize {
    5000
}
fn default_rl_log_history_lines() -> usize {
    20000
}

impl ShadowConfig {
    pub fn load(path: &Path) -> Result<ShadowConfig> {
        let text =
            std::fs::read_to_string(path).with_context(|| format!("reading {}", path.display()))?;
        toml::from_str(&text).with_context(|| format!("parsing {}", path.display()))
    }
}
