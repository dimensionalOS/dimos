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

//! Surface heights from the ray tracer's per-voxel planes; a voxel without one stands at its top face.

use ahash::AHashMap;

use crate::columns::ColumnIz;
use crate::voxel::VoxelKey;

/// A voxel's surface as a height field about its center, in meters: z = z0 + gx * dx + gy * dy.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SurfacePlane {
    z0: f32,
    gx: f32,
    gy: f32,
}

impl SurfacePlane {
    /// The tracer's fit `normal · (p - center) = offset`, None when NaN or tilted past `min_normal_z`.
    pub fn from_fit(fit @ [nx, ny, nz, offset]: [f32; 4], min_normal_z: f32) -> Option<Self> {
        (nz >= min_normal_z && fit.iter().all(|c| c.is_finite())).then(|| Self {
            z0: offset / nz,
            gx: -nx / nz,
            gy: -ny / nz,
        })
    }

    /// The level plane through the voxel center offset by `z0`, for tests.
    #[cfg(test)]
    pub fn level(z0: f32) -> Self {
        Self {
            z0,
            gx: 0.0,
            gy: 0.0,
        }
    }

    /// The plane through the voxel center offset by `z0` with slopes `gx` and `gy`, for tests.
    #[cfg(test)]
    pub fn sloped(z0: f32, gx: f32, gy: f32) -> Self {
        Self { z0, gx, gy }
    }

    /// Center height above its voxel's bottom face, held within the voxel pair the tracer fit.
    fn center_z(&self, voxel_size: f32) -> f32 {
        (0.5 * voxel_size + self.z0).clamp(-voxel_size, voxel_size)
    }

    /// Whether the two surfaces differ anywhere in a voxel by more than `tol` meters.
    pub fn moved(a: Option<Self>, b: Option<Self>, voxel_size: f32, tol: f32) -> bool {
        match (a, b) {
            (Some(a), Some(b)) => {
                let corner = 0.5 * voxel_size * ((a.gx - b.gx).abs() + (a.gy - b.gy).abs());
                (a.z0 - b.z0).abs() + corner > tol
            }
            (None, None) => false,
            _ => true,
        }
    }
}

/// A surface's slope, dz/dx then dz/dy.
pub type Slope = [f32; 2];

/// A surface's height at its column center, and its slope when a plane gave it.
pub type Surface = (f32, Option<Slope>);

/// The usable plane of each mapped voxel that has one.
pub type PlaneMap = AHashMap<VoxelKey, SurfacePlane>;

/// The occupied columns and their planes, with the robot's headroom and step.
pub struct Terrain<'a> {
    pub by_col: &'a ColumnIz,
    pub planes: &'a PlaneMap,
    pub voxel_size: f32,
    pub robot_height: f32,
    pub step_m: f32,
    /// How far fit noise alone moves a plane's height.
    pub noise_m: f32,
    /// The steepest slope a plane may have, dz per meter.
    pub max_slope: f32,
}

impl Terrain<'_> {
    fn plane(&self, key: VoxelKey) -> Option<SurfacePlane> {
        self.planes.get(&key).copied()
    }

    /// The voxel's surface.
    pub fn surface(&self, key: VoxelKey) -> Surface {
        let plane = self.plane(key);
        let slope = plane.map(|p| [p.gx, p.gy]);
        (key.2 as f32 * self.voxel_size + self.local_z(plane), slope)
    }

    /// Surface height above the voxel's bottom face at the column center, its top face without a plane.
    fn local_z(&self, plane: Option<SurfacePlane>) -> f32 {
        plane.map_or(self.voxel_size, |p| p.center_z(self.voxel_size))
    }

    /// Surface z at the column center.
    #[cfg(test)]
    pub fn z(&self, key: VoxelKey) -> f32 {
        self.surface(key).0
    }

    /// The cell has the robot's height clear between its surface and the next voxel up its column.
    pub fn standable(&self, key @ (ix, iy, iz): VoxelKey) -> bool {
        self.by_col
            .get((ix, iy))
            .is_none_or(|zs| self.clear_under(key, zs.get(zs.partition_point(|&z| z <= iz))))
    }

    /// `standable` for a cell whose next voxel up, if any, is at `next_iz`.
    pub fn clear_under(&self, key: VoxelKey, next_iz: Option<&i32>) -> bool {
        next_iz.is_none_or(|&next| self.clear_below(key, next))
    }

    /// `standable` for a cell whose next voxel up is at `next_iz`.
    pub fn clear_below(&self, key: VoxelKey, next_iz: i32) -> bool {
        let (cells, v) = (next_iz - key.2, self.voxel_size);
        let gap = cells as f32 * v;
        // The surface is within a voxel of the bottom face, so only gaps near the robot's height need the plane.
        if gap - v > self.robot_height {
            return true;
        }
        if gap + v < self.robot_height {
            return false;
        }
        match self.plane(key) {
            Some(plane) => gap - plane.center_z(v) >= self.robot_height,
            None => cells > (self.robot_height / v).ceil() as i32,
        }
    }

    /// The robot steps from `a` to `b`, any whole columns apart.
    #[cfg(test)]
    pub fn steppable(&self, a: VoxelKey, b: VoxelKey) -> bool {
        self.steps_onto(a, self.surface(a), b)
    }

    /// `steppable` from `a` with its surface already in hand.
    pub fn steps_onto(&self, a: VoxelKey, a_surface: Surface, b: VoxelKey) -> bool {
        let gap = surface_gap(a_surface, self.surface(b), (a, b), self.voxel_size);
        within_step(gap, self.voxel_size, self.step_m, self.noise_m)
    }

    /// The most z cells a step between adjacent columns can span: plane center within a voxel, edge half a slope further.
    pub fn step_reach_cells(&self) -> i32 {
        ((self.step_m + self.noise_m) / self.voxel_size + 2.0 + self.max_slope).floor() as i32
    }
}

/// The height gap the robot steps from surface `a` to surface `b`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Gap {
    /// Where the two planes meet halfway between the columns, in meters.
    Planes(f32),
    /// Between whole-voxel levels, in voxels.
    Voxels(i32),
}

/// See `Gap`.
pub fn surface_gap(
    (za, slope_a): Surface,
    (zb, slope_b): Surface,
    (a, b): (VoxelKey, VoxelKey),
    voxel_size: f32,
) -> Gap {
    let (Some([ax, ay]), Some([bx, by])) = (slope_a, slope_b) else {
        return Gap::Voxels((b.2 - a.2).abs());
    };
    let (hx, hy) = (
        0.5 * voxel_size * (b.0 - a.0) as f32,
        0.5 * voxel_size * (b.1 - a.1) as f32,
    );
    Gap::Planes(((zb - bx * hx - by * hy) - (za + ax * hx + ay * hy)).abs())
}

/// Whether a gap is within a step: past fit noise between planes, in whole voxels otherwise.
pub fn within_step(gap: Gap, voxel_size: f32, step_m: f32, noise_m: f32) -> bool {
    match gap {
        Gap::Planes(m) => climb(m, noise_m) <= step_m,
        Gap::Voxels(n) => n <= (step_m / voxel_size).floor() as i32,
    }
}

/// The climb in a rise between two surfaces: none within plane fit noise, so
/// a noisy flat floor costs nothing to cross.
pub fn climb(rise: f32, noise_m: f32) -> f32 {
    (rise - noise_m).max(0.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    const VOXEL: f32 = 0.1;

    fn terrain<'a>(by_col: &'a ColumnIz, planes: &'a PlaneMap) -> Terrain<'a> {
        Terrain {
            by_col,
            planes,
            voxel_size: VOXEL,
            robot_height: 0.3,
            step_m: 0.05,
            noise_m: 0.0,
            max_slope: 1.0,
        }
    }

    #[test]
    fn a_voxel_without_a_plane_stands_at_its_top_face() {
        let (by_col, planes) = (ColumnIz::default(), PlaneMap::new());
        assert!((terrain(&by_col, &planes).z((0, 0, 2)) - 0.3).abs() < 1e-6);
    }

    #[test]
    fn a_plane_center_is_held_within_its_voxel_pair() {
        let by_col = ColumnIz::default();
        let planes: PlaneMap = [
            ((0, 0, 0), SurfacePlane::level(0.2)),
            ((1, 0, 0), SurfacePlane::level(-0.2)),
        ]
        .into_iter()
        .collect();
        let t = terrain(&by_col, &planes);
        assert!((t.z((0, 0, 0)) - VOXEL).abs() < 1e-6);
        assert!((t.z((1, 0, 0)) + VOXEL).abs() < 1e-6);
    }

    #[test]
    fn a_fit_tilted_past_the_limit_is_dropped() {
        let steep = [0.8_f32, 0.0, 0.6, 0.0];
        assert!(SurfacePlane::from_fit(steep, 0.7).is_none());
        assert!(SurfacePlane::from_fit(steep, 0.5).is_some());
        assert!(SurfacePlane::from_fit([f32::NAN; 4], 0.0).is_none());
        assert!(SurfacePlane::from_fit([f32::NAN, 0.0, 1.0, 0.0], 0.0).is_none());
    }

    #[test]
    fn a_ramp_meets_itself_at_every_shared_face() {
        // 30 degrees: each column rises 5.8 cm, past the 5 cm step.
        let slope = 30.0_f32.to_radians().tan();
        let by_col = ColumnIz::default();
        let mut planes = PlaneMap::new();
        let mut keys = Vec::new();
        for ix in 0..10 {
            let z = ix as f32 * VOXEL * slope;
            let iz = (z / VOXEL).floor() as i32;
            let z0 = z - (iz as f32 + 0.5) * VOXEL;
            planes.insert((ix, 0, iz), SurfacePlane::sloped(z0, slope, 0.0));
            keys.push((ix, 0, iz));
        }
        let t = terrain(&by_col, &planes);
        for w in keys.windows(2) {
            assert!(t.steppable(w[0], w[1]), "{:?} -> {:?}", w[0], w[1]);
        }
    }

    #[test]
    fn plane_moves_compare_the_whole_voxel() {
        let flat = SurfacePlane::level(0.0);
        let tipped = SurfacePlane::sloped(0.0, 0.2, 0.0);
        assert!(!SurfacePlane::moved(Some(flat), Some(flat), VOXEL, 0.005));
        assert!(SurfacePlane::moved(Some(flat), Some(tipped), VOXEL, 0.005));
        assert!(SurfacePlane::moved(Some(flat), None, VOXEL, 0.005));
    }
}
