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

// Golden vs candidate, in data-flow order: the first FAIL line is the first divergence.

use std::io::{self, Write};
use std::path::Path;

use crate::golden::{self, Frame, Pose, STATE_BLOCKS};

pub struct Tol {
    /// Max |Δ| on state and P entries.
    pub state: f64,
    /// Max ULP distance on plane coefficients (both sides selected).
    pub plane_ulp: i64,
}

const CHECKS: [&str; 8] = [
    "n_down",
    "feats_down_body",
    "selected",
    "neighbours",
    "plane",
    "n_eff",
    "state",
    "P",
];

/// Prints the report; true when every frame check is within tolerance.
pub fn compare(
    golden_dir: &Path,
    cand_dir: &Path,
    tol: &Tol,
    out: &mut impl Write,
) -> io::Result<bool> {
    let g = golden::read_frames(golden_dir.join("frames.bin"))?;
    let c = golden::read_frames(cand_dir.join("frames.bin"))?;
    writeln!(out, "frames: golden {} candidate {}", g.len(), c.len())?;
    let mut first: [Option<u32>; 8] = [None; 8];
    for (gf, cf) in g.iter().zip(&c) {
        for (check, detail) in diff_frame(gf, cf, tol, out)? {
            let slot = CHECKS.iter().position(|c| *c == check).unwrap();
            if first[slot].is_none() {
                first[slot] = Some(gf.idx);
                writeln!(out, "  first {check} break: {detail}")?;
            }
        }
    }
    let broken: Vec<String> = CHECKS
        .iter()
        .zip(first)
        .filter_map(|(c, f)| f.map(|f| format!("{c} (frame {f})")))
        .collect();
    let frames_ok = g.len() == c.len() && broken.is_empty();
    if broken.is_empty() {
        writeln!(out, "frames: PASS")?;
    } else {
        writeln!(out, "frames: FAIL {}", broken.join(", "))?;
    }
    let traj_ok = trajectory(
        &golden::read_tum(golden_dir.join("trajectory.tum"))?,
        &golden::read_tum(cand_dir.join("trajectory.tum"))?,
        out,
    )?;
    Ok(frames_ok && traj_ok)
}

/// One summary line; returns (check, detail) for every check this frame breaks.
fn diff_frame(
    g: &Frame,
    c: &Frame,
    tol: &Tol,
    out: &mut impl Write,
) -> io::Result<Vec<(&'static str, String)>> {
    let mut fails = Vec::new();
    let n = g.feats_down_body.len().min(c.feats_down_body.len());
    if g.feats_down_body.len() != c.feats_down_body.len() {
        fails.push((
            "n_down",
            format!(
                "golden {} candidate {}",
                g.feats_down_body.len(),
                c.feats_down_body.len()
            ),
        ));
    }

    let (mut feats_first, mut feats_max) = (None, 0f64);
    for i in 0..n {
        let (a, b) = (g.feats_down_body[i], c.feats_down_body[i]);
        if a != b {
            feats_first.get_or_insert(i);
            feats_max = feats_max.max(max_abs3(a, b));
        }
    }
    if let Some(i) = feats_first {
        fails.push((
            "feats_down_body",
            format!(
                "point {i}: golden {:?} candidate {:?}",
                g.feats_down_body[i], c.feats_down_body[i]
            ),
        ));
    }

    let (mut sel, mut sel_first) = (0usize, None);
    let (mut nbr, mut nbr_first) = (0usize, None);
    let (mut ulp, mut ulp_at) = (0i64, (0usize, 0usize));
    let (mut plane_over, mut plane_first) = (0usize, None);
    for i in 0..n {
        let (a, b) = (&g.points[i], &c.points[i]);
        if a.selected != b.selected {
            sel += 1;
            sel_first.get_or_insert(i);
        }
        if a.neighbours != b.neighbours {
            nbr += 1;
            nbr_first.get_or_insert(i);
        }
        if a.selected && b.selected {
            let mut over = false;
            for k in 0..4 {
                let d = ulp_diff(a.plane[k], b.plane[k]);
                over |= d > tol.plane_ulp;
                if d > ulp {
                    (ulp, ulp_at) = (d, (i, k));
                }
            }
            if over {
                plane_over += 1;
                plane_first.get_or_insert(i);
            }
        }
    }
    if let Some(i) = sel_first {
        fails.push((
            "selected",
            format!(
                "{sel} mismatches, first at point {i}: golden {} candidate {}",
                g.points[i].selected, c.points[i].selected
            ),
        ));
    }
    if let Some(i) = nbr_first {
        let (a, b) = (&g.points[i].neighbours, &c.points[i].neighbours);
        let j = a.iter().zip(b).position(|(x, y)| x != y);
        fails.push((
            "neighbours",
            format!(
                "{nbr} mismatches, first at point {i}: n_nbr {}/{}, first differing nbr {:?}: golden {:?} candidate {:?}",
                a.len(),
                b.len(),
                j,
                j.map(|j| a[j]),
                j.map(|j| b[j]),
            ),
        ));
    }
    if let Some(first) = plane_first {
        let (i, k) = ulp_at;
        let (x, y) = (g.points[i].plane[k], c.points[i].plane[k]);
        fails.push((
            "plane",
            format!(
                "{plane_over} points over tol, first at point {first}: golden {:?} candidate {:?}; worst point {i} coef {k}: golden {x:e} candidate {y:e} ({ulp} ulp, rel {:.1e})",
                g.points[first].plane,
                c.points[first].plane,
                ((x - y) / x).abs()
            ),
        ));
    }

    if g.n_eff != c.n_eff {
        fails.push(("n_eff", format!("golden {} candidate {}", g.n_eff, c.n_eff)));
    }

    let mut blocks = [0f64; 10];
    let (mut worst, mut worst_i) = (0f64, 0usize);
    for (b, (_, lo, hi)) in blocks.iter_mut().zip(STATE_BLOCKS) {
        for i in lo..hi {
            let d = (g.state[i] - c.state[i]).abs();
            *b = b.max(d);
            if d > worst {
                (worst, worst_i) = (d, i);
            }
        }
    }
    if worst > tol.state {
        let (name, lo, _) = STATE_BLOCKS
            .iter()
            .find(|(_, lo, hi)| (*lo..*hi).contains(&worst_i))
            .unwrap();
        fails.push((
            "state",
            format!(
                "{name}[{}]: golden {:e} candidate {:e} (Δ {worst:e})",
                worst_i - lo,
                g.state[worst_i],
                c.state[worst_i]
            ),
        ));
    }

    let (mut p_max, mut p_i) = (0f64, 0usize);
    for (i, (a, b)) in g.p.iter().zip(&c.p).enumerate() {
        let d = (a - b).abs();
        if d > p_max {
            (p_max, p_i) = (d, i);
        }
    }
    if p_max > tol.state {
        fails.push((
            "P",
            format!(
                "P[{},{}]: golden {:e} candidate {:e} (Δ {p_max:e})",
                p_i / 30,
                p_i % 30,
                g.p[p_i],
                c.p[p_i]
            ),
        ));
    }

    let ne = |a: u32, b: u32| {
        if a == b {
            a.to_string()
        } else {
            format!("{a}≠{b}")
        }
    };
    let names: Vec<String> = STATE_BLOCKS
        .iter()
        .zip(blocks)
        .map(|((n, _, _), d)| format!("{n} {d:.1e}"))
        .collect();
    writeln!(
        out,
        "f{:>3} ts {:.6} | n_down {} feats {:.1e}{} | sel≠ {sel} nbr≠ {nbr} plane {ulp} ulp | n_eff {} | {} | P {p_max:.1e} | {}",
        g.idx,
        g.lidar_ts,
        ne(g.feats_down_body.len() as u32, c.feats_down_body.len() as u32),
        feats_max,
        feats_first.map_or(String::new(), |i| format!("@{i}")),
        ne(g.n_eff, c.n_eff),
        names.join(" "),
        if fails.is_empty() { "OK" } else { "FAIL" },
    )?;
    Ok(fails)
}

/// Per-frame error, APE RMSE/max, final-pose error; no alignment (same start).
fn trajectory(g: &[Pose], c: &[Pose], out: &mut impl Write) -> io::Result<bool> {
    writeln!(
        out,
        "trajectory: golden {} candidate {} poses",
        g.len(),
        c.len()
    )?;
    let n = g.len().min(c.len());
    let (mut sum_t, mut sum_r) = (0f64, 0f64);
    let (mut max_t, mut max_r) = ((0f64, 0usize), (0f64, 0usize));
    let mut ts_mismatch = 0usize;
    let (mut dt, mut dr) = (0f64, 0f64);
    let mut first_div = None;
    let mut path = 0f64;
    for i in 0..n {
        let (a, b) = (&g[i], &c[i]);
        dt = norm3(sub3(a.pos, b.pos));
        dr = quat_angle(a.quat, b.quat);
        if dt > 1e-6 && first_div.is_none() {
            first_div = Some(i);
        }
        if i > 0 {
            path += norm3(sub3(a.pos, g[i - 1].pos));
        }
        ts_mismatch += usize::from(a.ts != b.ts);
        sum_t += dt * dt;
        sum_r += dr * dr;
        if dt > max_t.0 {
            max_t = (dt, i);
        }
        if dr > max_r.0 {
            max_r = (dr, i);
        }
        writeln!(
            out,
            "t{i:>4} ts {:.6} Δts {:.1e} trans {dt:.3e} m rot {dr:.3e} rad",
            a.ts,
            (a.ts - b.ts).abs()
        )?;
    }
    if n > 0 {
        writeln!(
            out,
            "APE trans RMSE {:.3e} m max {:.3e} @{}  rot RMSE {:.3e} rad max {:.3e} @{}  final trans {dt:.3e} m rot {dr:.3e} rad  first trans>1e-6 @{}  path {path:.3} m  ts mismatches {ts_mismatch}",
            (sum_t / n as f64).sqrt(),
            max_t.0,
            max_t.1,
            (sum_r / n as f64).sqrt(),
            max_r.0,
            max_r.1,
            first_div.map_or("none".into(), |i| i.to_string()),
        )?;
    }
    Ok(g.len() == c.len())
}

/// Rotation between unit quaternions: ‖a−b‖/‖a+b‖ = tan(θ/4), stable near 0 (acos is not).
fn quat_angle(a: [f64; 4], b: [f64; 4]) -> f64 {
    let sign = if (0..4).map(|k| a[k] * b[k]).sum::<f64>() < 0.0 {
        -1.0
    } else {
        1.0
    };
    let (mut d, mut s) = (0f64, 0f64);
    for k in 0..4 {
        d += (a[k] - sign * b[k]).powi(2);
        s += (a[k] + sign * b[k]).powi(2);
    }
    4.0 * d.sqrt().atan2(s.sqrt())
}

/// Distance in representable floats, sign-aware (-0.0 and 0.0 are 0 apart).
fn ulp_diff(a: f32, b: f32) -> i64 {
    let ord = |f: f32| {
        let i = f.to_bits() as i32;
        i64::from(if i < 0 { i32::MIN.wrapping_sub(i) } else { i })
    };
    (ord(a) - ord(b)).abs()
}

fn max_abs3<T: Into<f64> + Copy>(a: [T; 3], b: [T; 3]) -> f64 {
    (0..3)
        .map(|k| (a[k].into() - b[k].into()).abs())
        .fold(0.0, f64::max)
}

fn sub3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn norm3(v: [f64; 3]) -> f64 {
    v.iter().map(|x| x * x).sum::<f64>().sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn quat_angle_stable() {
        let q = [-0.392622901, -0.020033283, -0.001188510, 0.919480567];
        assert_eq!(quat_angle(q, q), 0.0);
        assert_eq!(quat_angle(q, q.map(|x| -x)), 0.0);
        let a = [0.0, 0.0, 0.0, 1.0];
        let b = [0.0, 0.0, (0.1f64 / 2.0).sin(), (0.1f64 / 2.0).cos()];
        assert!((quat_angle(a, b) - 0.1).abs() < 1e-12);
    }

    #[test]
    fn ulp() {
        assert_eq!(ulp_diff(1.0, 1.0), 0);
        assert_eq!(ulp_diff(0.0, -0.0), 0);
        assert_eq!(ulp_diff(1.0, f32::from_bits(1.0f32.to_bits() + 3)), 3);
        assert_eq!(ulp_diff(-1.0, f32::from_bits((-1.0f32).to_bits() + 2)), 2);
        assert_eq!(
            ulp_diff(-f32::MIN_POSITIVE, f32::MIN_POSITIVE),
            2 * (1 << 23)
        );
    }
}
