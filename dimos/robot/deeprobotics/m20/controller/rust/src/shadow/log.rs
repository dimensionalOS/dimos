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

//! The shadow log: "M20SHDW1", a u32 header length, a JSON header, then one fixed 716-byte little-endian record per
//! control cycle (read by dimos.simulation.m20_policy_sim.shadow_summary). Files rotate at file_mb; the oldest go
//! once the directory passes max_total_mb.
//!
//! record: u64 t_ns | u32 flags (1 ours valid, 2 ours enable, 4 rl enable, 8 policy step, 16 imu valid) |
//! u16 rl control word | u8 state | u8 runner | u32 gait | u32 reserved | f32 quat 4, gyro 3, accel 3, q 16, dq 16 |
//! f32 axes 3 | f32 ours kp 16, q 16, kd 16, dq 16 | f32 rl kp 16, q 16, kd 16, dq 16

use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};

use anyhow::{Context, Result};

use crate::config::JOINTS;
use crate::types::{JointCommand, RobotState};

pub const MAGIC: &[u8; 8] = b"M20SHDW1";
pub const RECORD_SIZE: usize = 24 + 4 * (42 + 3 + 128);
pub const FLAG_OURS: u32 = 1;
pub const FLAG_OURS_ENABLE: u32 = 2;
pub const FLAG_RL_ENABLE: u32 = 4;
pub const FLAG_POLICY_STEP: u32 = 8;
pub const FLAG_IMU: u32 = 16;

pub struct Record<'a> {
    pub t_ns: u64,
    pub flags: u32,
    pub rl_word: u16,
    pub state: u8,
    pub runner: u8,
    pub gait: u32,
    pub input: &'a RobotState,
    pub axes: [f64; 3],
    pub ours: Option<&'a JointCommand>,
    pub rl: &'a JointCommand,
}

pub fn encode(r: &Record, out: &mut Vec<u8>) {
    out.extend_from_slice(&r.t_ns.to_le_bytes());
    out.extend_from_slice(&r.flags.to_le_bytes());
    out.extend_from_slice(&r.rl_word.to_le_bytes());
    out.push(r.state);
    out.push(r.runner);
    out.extend_from_slice(&r.gait.to_le_bytes());
    out.extend_from_slice(&0u32.to_le_bytes());
    let mut f = |v: f64| out.extend_from_slice(&(v as f32).to_le_bytes());
    let s = r.input;
    s.quat
        .iter()
        .chain(&s.gyro)
        .chain(&s.accel)
        .chain(&s.q)
        .chain(&s.dq)
        .for_each(|&v| f(v));
    r.axes.iter().for_each(|&v| f(v));
    let zero = JointCommand::default();
    for c in [r.ours.unwrap_or(&zero), r.rl] {
        for arr in [&c.kp, &c.q, &c.kd, &c.dq] {
            arr.iter().for_each(|&v| f(v));
        }
    }
    debug_assert_eq!(JOINTS, 16);
}

pub struct Logger {
    dir: PathBuf,
    header: Vec<u8>,
    file: Option<BufWriter<File>>,
    written: u64,
    file_bytes: u64,
    max_total: u64,
    buf: Vec<u8>,
}

impl Logger {
    pub fn new(
        dir: &Path,
        header_json: &serde_json::Value,
        file_mb: u64,
        max_total_mb: u64,
    ) -> Result<Logger> {
        std::fs::create_dir_all(dir).with_context(|| format!("creating {}", dir.display()))?;
        let json = serde_json::to_vec(header_json)?;
        let mut header = MAGIC.to_vec();
        header.extend_from_slice(&(json.len() as u32).to_le_bytes());
        header.extend_from_slice(&json);
        Ok(Logger {
            dir: dir.to_path_buf(),
            header,
            file: None,
            written: 0,
            file_bytes: file_mb.saturating_mul(1 << 20),
            max_total: max_total_mb.saturating_mul(1 << 20),
            buf: Vec::with_capacity(RECORD_SIZE),
        })
    }

    fn rotate(&mut self, t_ns: u64) -> Result<()> {
        if let Some(mut f) = self.file.take() {
            f.flush()?;
        }
        // never truncate an earlier file: a second rotation in the same ns (or a clock step back) gets a suffix
        let mut path = self.dir.join(format!("{t_ns}.m20shadow"));
        let mut n = 0;
        while path.exists() {
            n += 1;
            path = self.dir.join(format!("{t_ns}_{n}.m20shadow"));
        }
        let mut f = BufWriter::with_capacity(1 << 20, File::create(&path)?);
        f.write_all(&self.header)?;
        self.file = Some(f);
        self.written = 0;
        self.prune(&path)
    }

    /// Oldest first by name (the start ns), never the file being written (a clock step back names it lower).
    fn prune(&self, current: &Path) -> Result<()> {
        let mut files: Vec<(PathBuf, u64)> = std::fs::read_dir(&self.dir)?
            .filter_map(|e| e.ok())
            .filter(|e| e.path().extension().is_some_and(|x| x == "m20shadow"))
            .filter_map(|e| Some((e.path(), e.metadata().ok()?.len())))
            .collect();
        files.sort();
        let mut total: u64 = files.iter().map(|f| f.1).sum();
        for (path, len) in &files {
            if total <= self.max_total {
                break;
            }
            if path == current {
                continue;
            }
            let _ = std::fs::remove_file(path);
            total -= len;
        }
        Ok(())
    }

    pub fn write(&mut self, r: &Record) -> Result<()> {
        if self.file.is_none() || self.written >= self.file_bytes {
            self.rotate(r.t_ns)?;
        }
        self.buf.clear();
        encode(r, &mut self.buf);
        let file = self.file.as_mut().context("no log file open")?;
        file.write_all(&self.buf)?;
        self.written += self.buf.len() as u64;
        Ok(())
    }

    pub fn flush(&mut self) -> Result<()> {
        if let Some(f) = self.file.as_mut() {
            f.flush()?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn record_size() {
        let s = RobotState::default();
        let c = JointCommand::default();
        let r = Record {
            t_ns: 1,
            flags: 0,
            rl_word: 4,
            state: 0,
            runner: 255,
            gait: 0,
            input: &s,
            axes: [0.0; 3],
            ours: None,
            rl: &c,
        };
        let mut out = Vec::new();
        encode(&r, &mut out);
        assert_eq!(out.len(), RECORD_SIZE);
        assert_eq!(RECORD_SIZE, 716);
    }

    #[test]
    fn rotation_in_the_same_ns_keeps_the_earlier_file() {
        let dir = std::env::temp_dir().join(format!("m20shadow_log_test_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let mut log = Logger::new(&dir, &serde_json::json!({}), 0, 1 << 20).unwrap();
        let s = RobotState::default();
        let c = JointCommand::default();
        for _ in 0..3 {
            let r = Record {
                t_ns: 7,
                flags: 0,
                rl_word: 4,
                state: 0,
                runner: 255,
                gait: 0,
                input: &s,
                axes: [0.0; 3],
                ours: None,
                rl: &c,
            };
            log.write(&r).unwrap();
        }
        log.flush().unwrap();
        let files = std::fs::read_dir(&dir).unwrap().count();
        let _ = std::fs::remove_dir_all(&dir);
        assert_eq!(files, 3);
    }
}
