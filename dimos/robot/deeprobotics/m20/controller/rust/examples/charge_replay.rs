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

//! The docking core against charge_manager logs (NOS /var/opt/robot/log/*/charge_manager.*.log), one report per log:
//!
//!     cargo run --release --example charge_replay -- charge_manager.toml LOG...

use m20_controller::dock::vendor::{ChargeCore, ChargeManagerToml, ChargeParams};
use m20_controller::dock::vendor_log::replay;

fn main() -> anyhow::Result<()> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let Some((toml, logs)) = args.split_first() else {
        anyhow::bail!("usage: charge_replay charge_manager.toml LOG...");
    };
    let cfg = m20_controller::module::load_controller_config("")?;
    let params = ChargeParams::from_toml(&ChargeManagerToml::load(toml)?.0);
    let (mut poses, mut sends, mut bad, mut worst) = (0, 0, 0, 0.0f32);
    for log in logs {
        let text = String::from_utf8_lossy(&std::fs::read(log)?).into_owned();
        let r = replay(
            &text,
            || ChargeCore::new(params.clone(), &cfg.dock.sequence),
            210.0,
        );
        println!(
            "{log}: {} docks, {} undocks, {} poses, {} steers (max error {:.1e}), {} events, {} mismatches, {} at +-1 ms, {} \
             cut off",
            r.docks, r.undocks, r.poses, r.sends, r.max_steer_error, r.events, r.mismatch_count, r.clock_phase, r.cut
        );
        for (t, what) in &r.mismatches {
            let s = *t / 1000.0;
            println!(
                "  {:02}:{:02}:{:06.3} {what}",
                (s / 3600.0) as u32,
                (s / 60.0) as u32 % 60,
                s % 60.0
            );
        }
        poses += r.poses;
        sends += r.sends;
        bad += r.mismatch_count;
        worst = worst.max(r.max_steer_error);
    }
    println!("all: {poses} poses, {sends} steers (max error {worst:.1e}), {bad} mismatches");
    Ok(())
}
