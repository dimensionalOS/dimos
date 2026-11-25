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

//! charge_manager's obstacle check (ObstacleDetection<PointXYZ>, obstacle_count_method = "grid"), on base_link points:
//! LidarCloudSegmentation keeps z in [charge_pile_h - body height, lidar_z_max], x in [0, lidar_x_max] looking
//! forward or [-lidar_x_max, 0] looking back, |y| <= lidar_y_max, outside the body box (|x| < lidar_x_min and |y| <
//! lidar_y_min); ObstacleDistanceGrid counts them in grid_size cells (indices truncated toward zero) and gives the
//! distance from (+-lidar_x_min, 0) to the centre of the nearest cell holding grid_occu_num or more.

use std::collections::BTreeMap;

use super::vendor::{ChargeManagerToml, Direction};
use crate::config::Robot;

/// The obstacle check's numbers (charge_manager.toml's mOD block).
#[derive(Debug, Clone, PartialEq)]
pub struct GridParams {
    pub z_max: f64,
    pub x_min: f64,
    pub y_min: f64,
    pub x_max: f64,
    pub y_max: f64,
    pub pile_h: f64,
    pub grid_size: f64,
    pub occupied: i32,
}

impl GridParams {
    pub fn from_toml(t: &ChargeManagerToml) -> GridParams {
        GridParams {
            z_max: t.lidar_z_max,
            x_min: t.lidar_x_min,
            y_min: t.lidar_y_min,
            x_max: t.lidar_x_max,
            y_max: t.lidar_y_max,
            pile_h: t.charge_pile_h,
            grid_size: t.grid_size,
            occupied: t.grid_occu_num,
        }
    }
}

/// The nearest occupied cell's distance (m), or None when no cell is occupied.
pub fn nearest_obstacle<I>(
    points: I,
    direction: Direction,
    body_height: f64,
    g: &GridParams,
) -> Option<f32>
where
    I: IntoIterator<Item = [f32; 3]>,
{
    // PassThrough limits are floats: z [-(height - pile_h), z_max], x by direction, y [-y_max, y_max]
    let z = [(-(body_height - g.pile_h)) as f32, g.z_max as f32];
    let x = match direction {
        Direction::Front => [0.0, g.x_max as f32],
        Direction::Back => [-(g.x_max as f32), 0.0],
    };
    let y = [-(g.y_max as f32), g.y_max as f32];
    let mut cells: BTreeMap<(i32, i32), i32> = BTreeMap::new();
    for p in points {
        let inside = |v: f32, r: [f32; 2]| r[0] <= v && v <= r[1];
        if !(inside(p[2], z) && inside(p[0], x) && inside(p[1], y)) {
            continue;
        }
        if !(g.x_min <= p[0].abs() as f64 || g.y_min <= p[1].abs() as f64) {
            continue;
        }
        let key = (
            (p[0] as f64 / g.grid_size) as i32,
            (p[1] as f64 / g.grid_size) as i32,
        );
        *cells.entry(key).or_insert(0) += 1;
    }
    let from = match direction {
        Direction::Front => g.x_min,
        Direction::Back => -g.x_min,
    } as f32;
    let mut nearest: Option<f32> = None;
    for (&(ix, iy), &n) in &cells {
        if n < g.occupied {
            continue;
        }
        let cy = ((iy as f64 + 0.5) * g.grid_size) as f32;
        let cx = ((ix as f64 + 0.5) * g.grid_size) as f32 - from;
        let d = cy.mul_add(cy, cx * cx).sqrt();
        if nearest.is_none_or(|m| d < m) {
            nearest = Some(d);
        }
    }
    nearest
}

/// The body height for the obstacle check (MotionInfo.height): base_link above the wheel axles, averaged over the legs
/// (each leg's axle depth below its hip through hipy and knee, tilted out by hipx), plus the wheel radius.
pub fn body_height(robot: &Robot, q: &[f64], wheel_radius: f64) -> f64 {
    let (l1, l2) = (robot.thigh_length, robot.shank_length);
    let depth: f64 = (0..4)
        .map(|leg| {
            let (hipx, hipy, knee) = (q[4 * leg], q[4 * leg + 1], q[4 * leg + 2]);
            (l1 * hipy.cos() + l2 * (hipy + knee).cos()) * hipx.cos()
        })
        .sum::<f64>()
        / 4.0;
    depth + wheel_radius
}

#[cfg(test)]
mod tests {
    use super::*;

    fn grid() -> GridParams {
        // NOS charge_manager.toml
        GridParams {
            z_max: 0.3,
            x_min: 0.45,
            y_min: 0.25,
            x_max: 0.9,
            y_max: 0.3,
            pile_h: 0.25,
            grid_size: 0.3,
            occupied: 20,
        }
    }

    fn block(x: f32, y: f32, z: f32, n: usize) -> Vec<[f32; 3]> {
        (0..n).map(|k| [x + k as f32 * 1e-4, y, z]).collect()
    }

    #[test]
    fn distances_are_the_vendors_cell_centres() {
        let g = grid();
        // a box 0.5 m ahead: cell (1, 0), centre 0.45 -> 0.15 (the vendor's logged 0.150000)
        assert_eq!(
            nearest_obstacle(block(0.5, 0.0, 0.0, 20), Direction::Front, 0.47, &g),
            Some(0.15)
        );
        // the dock's posts 0.735 m ahead: cell (2, 0) -> hypot(0.15, 0.3) (logged 0.335410 through every vendor dock)
        let d = nearest_obstacle(block(0.735, 0.187, 0.0, 30), Direction::Front, 0.47, &g).unwrap();
        assert!((d - 0.335_410_2).abs() < 1e-6, "{d}");
        // 19 points do not occupy a cell; behind is not ahead
        assert_eq!(
            nearest_obstacle(block(0.5, 0.0, 0.0, 19), Direction::Front, 0.47, &g),
            None
        );
        assert_eq!(
            nearest_obstacle(block(0.5, 0.0, 0.0, 30), Direction::Back, 0.47, &g),
            None
        );
        // behind: x -0.5 truncates to cell -1 (centre -0.15) -> 0.3 from -0.45, hypot 0.3354
        let d = nearest_obstacle(block(-0.5, 0.0, 0.0, 30), Direction::Back, 0.47, &g).unwrap();
        assert!((d - 0.335_410_2).abs() < 1e-6, "{d}");
    }

    #[test]
    fn the_floor_the_body_and_the_sides_are_cut() {
        let g = grid();
        // the floor 0.47 m below base_link and anything up to the pile's height above it
        assert_eq!(
            nearest_obstacle(block(0.5, 0.0, -0.47, 50), Direction::Front, 0.47, &g),
            None
        );
        assert_eq!(
            nearest_obstacle(block(0.5, 0.0, -0.23, 50), Direction::Front, 0.47, &g),
            None
        );
        assert!(nearest_obstacle(block(0.5, 0.0, -0.21, 50), Direction::Front, 0.47, &g).is_some());
        // inside the body box, above lidar_z_max, wider than lidar_y_max
        assert_eq!(
            nearest_obstacle(block(0.4, 0.1, 0.0, 50), Direction::Front, 0.47, &g),
            None
        );
        assert_eq!(
            nearest_obstacle(block(0.5, 0.0, 0.31, 50), Direction::Front, 0.47, &g),
            None
        );
        assert_eq!(
            nearest_obstacle(block(0.5, 0.31, 0.0, 50), Direction::Front, 0.47, &g),
            None
        );
        // beside the body (|y| >= lidar_y_min) counts even with |x| < lidar_x_min
        assert!(nearest_obstacle(block(0.1, 0.27, 0.0, 50), Direction::Front, 0.47, &g).is_some());
    }
}
