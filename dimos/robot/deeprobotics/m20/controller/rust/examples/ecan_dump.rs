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

//! An ECAN capture (m20-ecan tap) as flat records, exact as the frames carry them, for tools/policy_steps.py:
//! cargo run --release --example ecan_dump -- CAPTURE OUT.bin
//!
//! per frame: u64 t_ns, u8 kind (0 state, 1 command, 2 other-to-board), then
//!   state: 16 x (q, dq, tau) f32 (rl_deploy's joint space)
//!   command: u16 word, 16 x (motor q, dq, tau) f32 raw + kp, kd i16 (joint order)
use m20_controller::config::Config;
use m20_controller::frames::{decode_state, joint_index, Mapping};
use m20_ecan::{tap, Direction};
use std::io::Write;
fn main() {
    let a: Vec<String> = std::env::args().collect();
    let cfg = Config::load(
        &std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("m20_controller.toml"),
    )
    .unwrap();
    let map = Mapping::new(&cfg.ecan).unwrap();
    let bytes = std::fs::read(&a[1]).unwrap();
    let mut out = std::io::BufWriter::new(std::fs::File::create(&a[2]).unwrap());
    let (mut ns, mut nc) = (0, 0);
    for (t, d, f) in tap::read_capture(&bytes).unwrap() {
        match d {
            Direction::FromBoard => {
                if let Ok(Some(s)) = decode_state(f, cfg.ecan.ethertype, &map) {
                    out.write_all(&t.to_le_bytes()).unwrap();
                    out.write_all(&[0]).unwrap();
                    for i in 0..16 {
                        for v in [s.q[i], s.dq[i], s.tau[i]] {
                            out.write_all(&(v as f32).to_le_bytes()).unwrap();
                        }
                    }
                    ns += 1;
                }
            }
            Direction::ToBoard => {
                if f.len() >= 24 + 16 * 24 && f[14 + 4] == 0x12 && f[14 + 5] == 0x89 {
                    let p = &f[24..];
                    let mut rows = [[0u8; 16]; 16];
                    let mut word = 0u16;
                    for k in 0..16 {
                        let u = &p[k * 24..][..24];
                        let Some(i) = joint_index(u[1], u16::from_le_bytes([u[2], u[3]])) else {
                            continue;
                        };
                        if k == 0 {
                            word = u16::from_le_bytes([u[20], u[21]]);
                        }
                        rows[i][..12].copy_from_slice(&u[4..16]);
                        rows[i][12..16].copy_from_slice(&u[16..20]);
                    }
                    out.write_all(&t.to_le_bytes()).unwrap();
                    out.write_all(&[1]).unwrap();
                    out.write_all(&word.to_le_bytes()).unwrap();
                    for r in &rows {
                        out.write_all(r).unwrap();
                    }
                    nc += 1;
                } else {
                    out.write_all(&t.to_le_bytes()).unwrap();
                    out.write_all(&[2]).unwrap();
                }
            }
        }
    }
    eprintln!("{ns} states, {nc} commands");
}
