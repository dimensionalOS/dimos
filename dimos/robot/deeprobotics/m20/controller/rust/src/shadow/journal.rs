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

//! rl_deploy's own log through journalctl: state transitions, the gait it actually switches to, height inputs.

use std::io::{BufRead, BufReader};
use std::process::{Child, Command, Stdio};
use std::sync::mpsc::Sender;

use anyhow::{Context, Result};

#[derive(Debug, Clone, PartialEq)]
pub enum Event {
    /// rl_deploy entered this state (the controller's state name)
    State(String),
    /// "target_gait --- X, last_target_gait --- Y": the gait rl_control now runs
    Gait(u32),
    /// "target_gait: X": the gait the operator asked for (a stand-in hands over to it)
    Target(u32),
    Height(f64),
}

/// (seconds since the epoch, event) from one journalctl short-iso-precise line, if it carries one.
pub fn parse_line(line: &str) -> Option<(f64, Event)> {
    let (stamp, rest) = line.split_once(' ')?;
    let t = iso_seconds(stamp)?;
    let msg = rest.split_once(": ").map(|(_, m)| m)?;
    if let Some((_, to)) = msg.split_once(" ------------> ") {
        return Some((t, Event::State(state_name(to))));
    }
    if let Some(r) = msg.strip_prefix("target_gait --- ") {
        let code = r.split(',').next()?.trim();
        return u32::from_str_radix(code, 16)
            .ok()
            .map(|g| (t, Event::Gait(g)));
    }
    if let Some(r) = msg.strip_prefix("target_gait: ") {
        return u32::from_str_radix(r.trim(), 16)
            .ok()
            .map(|g| (t, Event::Target(g)));
    }
    if let Some(v) = msg.strip_prefix("target_height_scale: ") {
        return v.trim().parse().ok().map(|h| (t, Event::Height(h)));
    }
    None
}

fn state_name(to: &str) -> String {
    match to.trim() {
        "idle_state" => "passive",
        "standup_state" => "stand_up",
        "liedown_state" => "lie_down",
        "rl_control" => "rl_control",
        "joint_damping" => "joint_damping",
        other => other,
    }
    .to_string()
}

/// A state transition from rl_deploy's own log file ("[2026-11-20 18:17:06.048.410] [INFO  ] [Trans]  ------------>
/// start_damping"); its stamps are local time, `utc_offset_s` east of UTC. The file keeps what the journal may drop.
pub fn parse_log_line(line: &str, utc_offset_s: i64) -> Option<(f64, Event)> {
    let rest = line.strip_prefix('[')?;
    let (stamp, rest) = rest.split_once(']')?;
    let (_, to) = rest.split_once("------------> ")?;
    let (date, clock) = stamp.split_once(' ')?;
    // "18:17:06.048.410" -> "18:17:06.048410"
    let mut parts = clock.splitn(2, '.');
    let (hms, frac) = (parts.next()?, parts.next().unwrap_or("0").replace('.', ""));
    let sign = if utc_offset_s < 0 { '-' } else { '+' };
    let off = utc_offset_s.abs();
    let iso = format!(
        "{date}T{hms}.{frac}{sign}{:02}{:02}",
        off / 3600,
        off % 3600 / 60
    );
    Some((iso_seconds(&iso)?, Event::State(state_name(to))))
}

/// Follow rl_deploy's log file (today's path; tail -F waits for it), state transitions to `tx`.
pub fn follow_log(path: &str, utc_offset_s: i64, tx: Sender<(f64, Event)>) -> Result<Child> {
    let mut child = Command::new("tail")
        .args(["-F", "-n", "20000", path])
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        .spawn()
        .context("starting tail")?;
    let out = child.stdout.take().context("tail stdout")?;
    std::thread::Builder::new()
        .name("shadow-rl-log".into())
        .spawn(move || {
            for line in BufReader::new(out).lines().map_while(Result::ok) {
                if let Some(e) = parse_log_line(&line, utc_offset_s) {
                    if tx.send(e).is_err() {
                        return;
                    }
                }
            }
        })?;
    Ok(child)
}

/// "2026-11-20T14:16:31.638834+0800" -> seconds since the epoch.
pub fn iso_seconds(s: &str) -> Option<f64> {
    let (date, rest) = s.split_once('T')?;
    let mut d = date.split('-').map(|x| x.parse::<i64>());
    let (y, mo, da) = (d.next()?.ok()?, d.next()?.ok()?, d.next()?.ok()?);
    let tz_at = rest.rfind(['+', '-'])?;
    let (clock, tz) = rest.split_at(tz_at);
    let mut c = clock.split(':');
    let (h, mi) = (
        c.next()?.parse::<i64>().ok()?,
        c.next()?.parse::<i64>().ok()?,
    );
    let sec: f64 = c.next()?.parse().ok()?;
    let sign = if tz.starts_with('-') { -1 } else { 1 };
    let tz = tz[1..].replace(':', "");
    let off = sign * (tz[..2].parse::<i64>().ok()? * 3600 + tz[2..4].parse::<i64>().ok()? * 60);
    // days from the civil date (Howard Hinnant's algorithm)
    let yy = if mo <= 2 { y - 1 } else { y };
    let era = yy.div_euclid(400);
    let yoe = yy - era * 400;
    let doy = (153 * (if mo > 2 { mo - 3 } else { mo + 9 }) + 2) / 5 + da - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    let days = era * 146097 + doe - 719468;
    Some((days * 86400 + h * 3600 + mi * 60 - off) as f64 + sec)
}

/// Follow the unit's journal from now on; events go to `tx`. The child is returned so it can be killed.
/// The last `history` lines come first, so the current state and gait are known at start.
pub fn follow(unit: &str, history: usize, tx: Sender<(f64, Event)>) -> Result<Child> {
    let mut child = Command::new("journalctl")
        .args([
            "-f",
            "-n",
            &history.to_string(),
            "-o",
            "short-iso-precise",
            "-u",
            unit,
        ])
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        .spawn()
        .context("starting journalctl")?;
    let out = child.stdout.take().context("journalctl stdout")?;
    std::thread::Builder::new()
        .name("shadow-journal".into())
        .spawn(move || {
            for line in BufReader::new(out).lines().map_while(Result::ok) {
                if let Some(e) = parse_line(&line) {
                    if tx.send(e).is_err() {
                        return;
                    }
                }
            }
        })?;
    Ok(child)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lines() {
        let (t, e) = parse_line("2026-11-20T14:16:31.643578+0800 host run.sh[407864]: idle_state ------------> standup_state").unwrap();
        assert_eq!(e, Event::State("stand_up".into()));
        assert!((t - 1795155391.643578).abs() < 1e-5, "{t}");
        let (_, e) = parse_line("2026-11-20T14:16:41.635308+0800 host run.sh[1]: target_gait --- 3003, last_target_gait --- 3001").unwrap();
        assert_eq!(e, Event::Gait(0x3003));
        let (_, e) =
            parse_line("2026-11-20T14:18:05.233305+0800 host run.sh[1]: target_height_scale: 0.1")
                .unwrap();
        assert_eq!(e, Event::Height(0.1));
        let (_, e) =
            parse_line("2026-11-20T14:16:41.635308+0800 host run.sh[1]: target_gait: 3003")
                .unwrap();
        assert_eq!(e, Event::Target(0x3003));
        let (t, e) = parse_log_line(
            "[2026-11-20 18:17:06.048.410] [INFO  ] [Trans]  ------------> start_damping ",
            28800,
        )
        .unwrap();
        assert_eq!(e, Event::State("start_damping".into()));
        assert!((t - 1795169826.048410).abs() < 1e-5, "{t}");
    }
}
