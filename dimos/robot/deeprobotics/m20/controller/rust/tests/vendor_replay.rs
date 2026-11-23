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

//! The docking core against charge_manager's own config and logs (NOS, copied to tests/fixtures): the real
//! charge_manager.toml loads to the numbers charge_manager derives, and the recorded vendor dock and undock (2026-11-21)
//! and a dock with two retries and a give-up (2026-11-06) replay pose for pose: every steer equal to the logged one to
//! the log's 6 decimals, every event at the same pose.

mod common;

use std::path::PathBuf;

use m20_controller::dock::vendor::{ChargeCore, ChargeManagerToml, ChargeParams};
use m20_controller::dock::vendor_log::{replay, Report};

fn fixture(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures")
        .join(name)
}

/// A fixture's text, gunzipped when it ends in .gz.
fn read(name: &str) -> String {
    use std::io::Read;
    let bytes = std::fs::read(fixture(name)).unwrap();
    if !name.ends_with(".gz") {
        return String::from_utf8_lossy(&bytes).into_owned();
    }
    let mut text = String::new();
    flate2::read::GzDecoder::new(&bytes[..])
        .read_to_string(&mut text)
        .unwrap();
    text
}

fn vendor() -> ChargeManagerToml {
    ChargeManagerToml::load(fixture("charge_manager.toml").to_str().unwrap())
        .unwrap()
        .0
}

/// The obstacle check logs a reading only when it finds one, ~every 100 ms (mOD_hz 10) or 200 ms
const OBSTACLE_HOLD_MS: f64 = 210.0;

fn run(log: &str) -> Report {
    let cfg = common::config();
    let params = ChargeParams::from_toml(&vendor());
    let text = read(log);
    let report = replay(
        &text,
        || ChargeCore::new(params.clone(), &cfg.dock.sequence),
        OBSTACLE_HOLD_MS,
    );
    println!("{log}: {report:?}");
    report
}

#[test]
fn the_real_charge_manager_toml_loads_as_charge_manager_reads_it() {
    let p = ChargeParams::from_toml(&vendor());
    assert_eq!(p.retries, 2);
    assert_eq!(p.charge_confirm_ms, 5000);
    assert_eq!(p.find_target_ms, 2000);
    assert_eq!(p.restart_after_waits, 400);
    assert_eq!(p.delta, [0.006, 0.003, 0.03]);
    assert_eq!(p.ext_delta, [0.09, 0.09, 0.7]);
    assert_eq!(p.min_speed, [0.18, 0.25, 0.1]);
    assert_eq!(p.ext_min_speed, [0.2, 0.25, 0.14]);
    assert_eq!(
        (p.pre_point_x, p.pre_exit_point_x, p.exit_pile_distance_x),
        (1.0, 0.6, 1.3)
    );
    assert!(!p.back_fixed && p.no_charge_go_back && !p.retry_in_place && p.all_battery_charge);
    assert_eq!(p.compensation_x, 0.0);
    assert!(p.obstacle_avoid);
    assert_eq!(p.obstacle_avoid_dis, 0.3);
    let t = vendor();
    assert_eq!(
        (t.grid_size, t.grid_occu_num, t.charge_pile_h),
        (0.3, 20, 0.25)
    );
    assert_eq!(
        (
            t.lidar_x_min,
            t.lidar_y_min,
            t.lidar_x_max,
            t.lidar_y_max,
            t.lidar_z_max
        ),
        (0.45, 0.25, 0.9, 0.3, 0.3)
    );
}

#[test]
fn the_recorded_vendor_dock_and_undock_replay_exactly() {
    let r = run("charge_manager_20261121_dock.log.gz");
    assert_eq!(r.mismatch_count, 0, "{:#?}", r.mismatches);
    assert_eq!((r.docks, r.undocks), (1, 1));
    assert!(r.sends > 100, "{r:?}");
}

#[test]
fn retries_and_the_give_up_replay_exactly() {
    let r = run("charge_manager_20261106_retries.log.gz");
    assert_eq!(r.mismatch_count, 0, "{:#?}", r.mismatches);
    assert!(r.docks >= 1 && r.undocks >= 1, "{r:?}");
}

/// reflective_column_node's output stage against its own log (2026-11-21, reduced to the lines it needs): every start
/// fix it published after a restart and every position it dropped as a jump, sweep for sweep.
#[test]
fn the_reflector_filter_publishes_and_drops_as_reflective_column_node_logged() {
    use m20_controller::dock::vendor::ReflectorFilter;
    let cfg = common::config();
    let text = read("reflective_column_node_20261121.txt.gz");
    let mut f = ReflectorFilter::new(&cfg.dock.filter);
    let (mut fixes, mut jumps, mut sweeps) = (0, 0, 0);
    let mut expect_fix = false;
    let mut dropped_jump = false;
    let mut last_out: Option<[f64; 3]> = None;
    for line in text.lines().filter(|l| !l.starts_with('#')) {
        let v: Vec<&str> = line.split(' ').collect();
        match v[0] {
            "R" => f.restart(),
            "P" => {
                assert!(
                    !dropped_jump,
                    "ours dropped a jump reflective_column_node did not log"
                );
                let n = |k: usize| v[k].parse::<f64>().unwrap();
                let (x, y, yaw) = (n(1), n(2), n(3));
                let was_started = f.started();
                let out = f.sweep(x, y, yaw);
                sweeps += 1;
                let in_range = (cfg.dock.filter.x_range[0]..=cfg.dock.filter.x_range[1])
                    .contains(&x)
                    && y.abs() <= cfg.dock.filter.max_y;
                expect_fix = !was_started && f.started();
                dropped_jump = was_started && in_range && out.is_none();
                last_out = out;
            }
            "F" => {
                assert!(expect_fix, "a start fix where ours published none");
                let out = last_out.expect("the start fix was published");
                let (x, y) = (v[1].parse::<f64>().unwrap(), v[2].parse::<f64>().unwrap());
                assert!(
                    (out[0] - x).abs() < 1e-6 && (out[1] - y).abs() < 1e-6,
                    "{out:?} vs {x} {y}"
                );
                fixes += 1;
            }
            "J" => {
                assert!(dropped_jump, "a logged jump ours published");
                dropped_jump = false;
                jumps += 1;
            }
            _ => {}
        }
    }
    println!("{sweeps} sweeps, {fixes} start fixes, {jumps} jumps");
    assert!(fixes > 10 && jumps >= 3);
}
