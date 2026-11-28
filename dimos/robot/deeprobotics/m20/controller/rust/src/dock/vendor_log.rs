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

//! [`super::vendor::ChargeCore`] against charge_manager's own log (NOS /var/opt/robot/log/*/charge_manager.*.log):
//! every pose it logged ("mInitYaw" docking, "get Target position" undocking) goes into the core at the logged time,
//! with the obstacle check's newest logged reading, its missed waits, its FinetuneStand start and its no-charge
//! retries; the core's steer must equal the logged sendX / sendY / sendYaw / sendZ (to the log's 6 decimals) and its
//! events the logged ones, pose for pose.

use super::vendor::{ChargeCore, Event, Failure, NoCharge, Outcome, TargetPose};

/// One line of charge_manager's log that the replay reads.
#[derive(Debug, Clone, PartialEq)]
enum Line {
    /// charge_manager (re)started: nothing carries over
    Started,
    StartMove,
    OverCharge,
    Stood,
    Gait(u32),
    /// a dock (true) or undock pose: charge_manager's (x, y, yaw) as logged
    Pose(bool, [f32; 3]),
    Send([f32; 4], bool),
    /// a send line cut off (charge_manager killed mid-write)
    Cut,
    StartPoint([f32; 3]),
    Staging,
    Fine,
    FinetuneDone,
    ObstacleStop,
    ExitStaging,
    ExitReached,
    Failed(Failure),
    Timeout,
    Restarted,
    Retry,
    Exhausted,
    ObstacleReading(f32),
}

/// (ms of the day's clock, line)
fn parse(text: &str) -> Vec<(f64, Line)> {
    let mut out = Vec::new();
    let mut t = 0.0;
    let num = |s: &str, key: &str| -> Option<f32> {
        let i = s.find(key)? + key.len();
        let rest = &s[i..];
        let end = rest
            .find(|c: char| !(c.is_ascii_digit() || c == '.' || c == '-'))
            .unwrap_or(rest.len());
        rest[..end].parse().ok()
    };
    for l in text.lines() {
        if l.starts_with('[') && l.len() > 30 {
            // [2026-11-21 13:27:49.431.614]
            let h = |a: usize, b: usize| l.get(a..b).and_then(|s| s.parse::<f64>().ok());
            if let (Some(hh), Some(mm), Some(ss), Some(ms), Some(us)) =
                (h(12, 14), h(15, 17), h(18, 20), h(21, 24), h(25, 28))
            {
                t = ((hh * 60.0 + mm) * 60.0 + ss) * 1000.0 + ms + us / 1000.0;
            }
        }
        let line = if l.contains("Start charge manager...") {
            Line::Started
        } else if l.contains("strat to move") {
            Line::StartMove
        } else if l.contains("Begin to over charge process") {
            Line::OverCharge
        } else if l.contains("Enable stand up command success")
            || l.contains("I have already stood up")
        {
            Line::Stood
        } else if let Some(i) = l.find("gait state change! new:0x") {
            match u32::from_str_radix(l[i + 25..].trim(), 16) {
                Ok(g) => Line::Gait(g),
                Err(_) => continue,
            }
        } else if l.contains("mInitYaw x:") || l.contains("get Target position: x:") {
            match (num(l, "x:"), num(l, "y:"), num(l, "yaw:")) {
                (Some(x), Some(y), Some(yaw)) => Line::Pose(l.contains("mInitYaw"), [x, y, yaw]),
                _ => continue,
            }
        } else if l.contains(" sendX:") {
            match (num(l, "sendX:"), num(l, "sendY:"), num(l, "sendYaw:")) {
                (Some(x), Some(y), Some(yaw)) => Line::Send(
                    [x, y, yaw, num(l, "sendZ:").unwrap_or(0.0)],
                    l.contains("sendZ"),
                ),
                _ => Line::Cut,
            }
        } else if l.contains("mInitX:") {
            match (num(l, "mInitX:"), num(l, "mInitY:"), num(l, "mInitYaw:")) {
                (Some(x), Some(y), Some(yaw)) => Line::StartPoint([x, y, yaw]),
                _ => continue,
            }
        } else if l.contains("已到达进桩预备点") {
            Line::Staging
        } else if l.contains("已到达精调点") {
            Line::Fine
        } else if l.contains("已到达桩上") {
            Line::FinetuneDone
        } else if l.contains("检测到障碍物，距离") {
            Line::ObstacleStop
        } else if l.contains("已到达退桩预备点") {
            Line::ExitStaging
        } else if l.contains("已到达指定位置") {
            Line::ExitReached
        } else if l.contains("more than 1 min, so return GOTOSTACKFAILED") {
            Line::Failed(Failure::ApproachTimeout)
        } else if l.contains("停障超时") {
            Line::Failed(Failure::ObstacleTimeout)
        } else if l.contains("退桩超时 超过1min") {
            Line::Failed(Failure::ExitTimeout)
        } else if l.contains("当前不在导航步态") {
            Line::Failed(Failure::NotNavigating)
        } else if l.contains("超时，停止运动") {
            Line::Timeout
        } else if l.contains("Try restart location success") {
            Line::Restarted
        } else if l.contains("检测到无充电电流，重试剩余次数") {
            Line::Retry
        } else if l.contains("检测到无充电电流，重试次数已用完") {
            Line::Exhausted
        } else if let Some(d) = num(l, "最近障碍物距离为: ") {
            Line::ObstacleReading(d)
        } else {
            continue;
        };
        out.push((t, line));
    }
    out
}

/// The replay's tally.
#[derive(Debug, Default, Clone)]
pub struct Report {
    pub docks: usize,
    pub undocks: usize,
    pub poses: usize,
    pub sends: usize,
    pub events: usize,
    /// the largest |ours - logged| over every steer component
    pub max_steer_error: f32,
    /// the first mismatches (time ms, what)
    pub mismatches: Vec<(f64, String)>,
    pub mismatch_count: usize,
    /// poses that match only at the logged time +-1 ms: a time limit within a millisecond, which the log's wall clock
    /// cannot place against charge_manager's steady clock
    pub clock_phase: usize,
    /// poses whose logged send was cut off
    pub cut: usize,
}

impl Report {
    fn bad(&mut self, t: f64, what: String) {
        self.mismatch_count += 1;
        if self.mismatches.len() < 20 {
            self.mismatches.push((t, what));
        }
    }
}

/// What a pose's logged lines say it did.
#[derive(Debug, Default)]
struct Logged {
    send: Option<[f32; 4]>,
    cut: bool,
    events: Vec<Line>,
}

/// Why `out` differs from what was logged (None: it does not).
fn compare(out: &Outcome, logged: &Logged) -> Option<String> {
    let ours: Vec<Line> = out
        .start_point
        .map(Line::StartPoint)
        .into_iter()
        .chain(out.event.iter().filter_map(event_line))
        .collect();
    if !(ours.len() == logged.events.len()
        && ours.iter().zip(&logged.events).all(|(a, b)| same(a, b)))
    {
        return Some(format!("events: ours {ours:?}, logged {:?}", logged.events));
    }
    if logged.cut {
        return None;
    }
    match (out.steer, logged.send) {
        (Some(s), Some(l)) => {
            let s = [s.x, s.y, s.yaw, s.z];
            let e = s
                .iter()
                .zip(l)
                .map(|(a, b)| (a - b).abs())
                .fold(0.0f32, f32::max);
            (e > PRINTED).then(|| format!("steer: ours {s:?}, logged {l:?}"))
        }
        (None, None) => None,
        (s, l) => Some(format!("steer: ours {s:?}, logged {l:?}")),
    }
}

fn event_line(e: &Event) -> Option<Line> {
    Some(match e {
        Event::StagingReached => Line::Staging,
        Event::FineReached(_) => Line::Fine,
        Event::FinetuneDone(_) => Line::FinetuneDone,
        Event::Obstacle(_) => Line::ObstacleStop,
        Event::ExitStagingReached => Line::ExitStaging,
        Event::ExitReached => Line::ExitReached,
        Event::Failed(f) => Line::Failed(*f),
    })
}

/// Logged numbers carry 6 decimals: a steer recomputed from logged (rounded) poses agrees to this (P + D of the
/// rounding, the logged steer's own rounding, f32 accumulation).
const PRINTED: f32 = 3e-6;

fn same(a: &Line, b: &Line) -> bool {
    match (a, b) {
        (Line::StartPoint(p), Line::StartPoint(q)) => {
            p.iter().zip(q).all(|(x, y)| (x - y).abs() <= PRINTED)
        }
        _ => a == b,
    }
}

/// Replay one log's text through `new()`'s core (a fresh one at every charge_manager start). The obstacle check's
/// logged reading counts for `obstacle_hold_ms` (it logs only what it finds).
pub fn replay(text: &str, new: impl Fn() -> ChargeCore, obstacle_hold_ms: f64) -> Report {
    let mut core = new();
    let lines = parse(text);
    let mut r = Report::default();
    let mut reading: Option<(f64, f32)> = None;
    let mut exit_pending = false;
    let mut finetune_pending = false;
    let mut mode_dock = None::<bool>;
    // a dock since charge_manager started set the start point (else the log began after the dock it comes from)
    let mut start_known = false;
    let mut i = 0;
    while i < lines.len() {
        let (t, line) = &lines[i];
        let now = t.floor() as u64;
        i += 1;
        match line {
            Line::ObstacleReading(d) => reading = Some((*t, *d)),
            Line::Started => {
                core = new();
                start_known = true;
                mode_dock = None;
                exit_pending = false;
                finetune_pending = false;
            }
            Line::StartMove => {
                start_known = true;
                core.begin_dock(now);
                mode_dock = Some(true);
                r.docks += 1;
            }
            Line::OverCharge => exit_pending = true,
            Line::Stood if exit_pending => {
                exit_pending = false;
                if !start_known {
                    // the undock's goal as it logs it, from before the log began
                    let goal = lines[i..]
                        .iter()
                        .take_while(|(_, l)| {
                            !matches!(l, Line::Started | Line::StartMove | Line::OverCharge)
                        })
                        .find_map(|(_, l)| match l {
                            Line::StartPoint(p) => Some(*p),
                            _ => None,
                        });
                    if let Some(p) = goal {
                        core.set_start_point(p);
                    }
                }
                core.begin_exit(now);
                mode_dock = Some(false);
                r.undocks += 1;
            }
            Line::Gait(0x3004) if finetune_pending => {
                finetune_pending = false;
                core.finetune_started(now);
            }
            Line::Timeout if mode_dock.is_some() => {
                core.wait_timed_out();
            }
            Line::Restarted => core.restarted(),
            Line::Retry => {
                if core.no_charge() != NoCharge::Retry {
                    r.bad(*t, "logged a retry, the core has none left".into());
                }
            }
            Line::Exhausted => {
                if core.no_charge() != NoCharge::Exhausted {
                    r.bad(*t, "logged no tries left, the core has some".into());
                }
                mode_dock = None;
            }
            // an undock pose stopped by an obstacle, its timeout or its gait check is not logged: its line is the pose
            Line::Pose(..) | Line::ObstacleStop | Line::Failed(_) if mode_dock.is_some() => {
                let (pose, dock) = match line {
                    Line::Pose(dock, [x, y, yaw]) => (Some([*x, *y, *yaw]), *dock),
                    _ => (None, mode_dock == Some(true)),
                };
                if pose.is_none() && dock && !matches!(line, Line::Failed(Failure::NotNavigating)) {
                    // docking logs the pose first; an event alone belongs to the pose before
                    continue;
                }
                // replay-only input: taken from the logged failure, so the live navigating check is not tested here
                let navigating = !matches!(line, Line::Failed(Failure::NotNavigating));
                let [x, y, yaw] = pose.unwrap_or([0.0; 3]);
                let comp = core.params.compensation_x;
                let p = TargetPose {
                    x: comp - x,
                    y: -y,
                    yaw,
                };
                let obstacle = reading
                    .filter(|(at, _)| t - at <= obstacle_hold_ms)
                    .map(|o| o.1);
                let before = core.clone();
                let mut out: Outcome = core.on_pose(now, p, navigating, obstacle);
                r.poses += 1;
                // this pose's logged lines: up to the next pose / timeout / stand-up
                let mut logged = Logged::default();
                if pose.is_none() {
                    logged.events.push(line.clone());
                }
                while let Some((_, l)) = lines.get(i) {
                    match l {
                        Line::Pose(..)
                        | Line::Timeout
                        | Line::Stood
                        | Line::StartMove
                        | Line::OverCharge
                        | Line::Started
                        | Line::Retry
                        | Line::Exhausted
                        | Line::Gait(_) => break,
                        // undocking, these are poses of their own
                        Line::Failed(_) | Line::ObstacleStop if !dock => break,
                        // undocking logs its goal with every pose
                        Line::StartPoint(_) if !dock => {}
                        Line::Send(s, _) => {
                            logged.send = Some(*s);
                            i += 1;
                            break;
                        }
                        Line::Cut => {
                            logged.cut = true;
                            i += 1;
                            break;
                        }
                        Line::ObstacleReading(d) => reading = Some((lines[i].0, *d)),
                        Line::Restarted => {}
                        other => logged.events.push(other.clone()),
                    }
                    i += 1;
                }
                if logged.cut {
                    r.cut += 1;
                }
                if let Some(why) = compare(&out, &logged) {
                    let shifted = [now + 1, now.saturating_sub(1)].into_iter().find_map(|t| {
                        let mut c = before.clone();
                        let o = c.on_pose(t, p, navigating, obstacle);
                        compare(&o, &logged).is_none().then_some((c, o))
                    });
                    match shifted {
                        Some((c, o)) => {
                            core = c;
                            out = o;
                            r.clock_phase += 1;
                        }
                        None => r.bad(*t, why),
                    }
                }
                r.events += logged.events.len();
                if let (Some(s), Some(l)) = (out.steer, logged.send) {
                    r.sends += 1;
                    let e = [s.x - l[0], s.y - l[1], s.yaw - l[2], s.z - l[3]]
                        .iter()
                        .fold(0.0f32, |m, d| m.max(d.abs()));
                    r.max_steer_error = r.max_steer_error.max(e);
                }
                if matches!(out.event, Some(Event::FineReached(_))) {
                    finetune_pending = true;
                }
                if matches!(out.event, Some(Event::Failed(_)) | Some(Event::ExitReached)) {
                    mode_dock = None;
                }
            }
            _ => {}
        }
    }
    r
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_the_vendors_lines() {
        let text = "[2026-11-21 13:27:51.043.568] [INFO  ] [ChargeManager] strat to move \n\
            [2026-11-21 13:27:58.527.151] [INFO  ] [ChargeManager] mInitYaw x:0.570704, y:0.102017, yaw:0.034962 \n\
            [2026-11-21 13:27:58.527.160] [INFO  ] [ChargeManager] ------------------------\n \
            sendX:0.679179,sendY:0.250000,sendYaw:-0.100000 sendZ:0.000000\n";
        let l = parse(text);
        assert_eq!(l[0].1, Line::StartMove);
        assert_eq!(l[1].1, Line::Pose(true, [0.570704, 0.102017, 0.034962]));
        assert_eq!(l[2].1, Line::Send([0.679179, 0.25, -0.1, 0.0], true));
        assert!((l[1].0 - ((13.0 * 60.0 + 27.0) * 60.0 + 58.0) * 1000.0 - 527.151).abs() < 1e-6);
    }
}
