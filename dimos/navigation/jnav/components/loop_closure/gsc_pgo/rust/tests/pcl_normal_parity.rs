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

//! Bit-exactness goldens for the PCL NormalEstimation / cloud_degeneracy
//! port, against nixpkgs pcl-1.15.1 + Eigen 3.4.1 (the exact libraries the
//! C++ gsc_pgo binary links).
//!
//! Reference values come from a C++ harness (/tmp/pcl_normal_check/ref.cpp
//! in the porting session; nix-shell g++ 15.2 -O2, x86-64 baseline, no
//! -march, linked against nix glibc 2.42 — the same glibc/libm the deployed
//! pgo binary resolves atan2f/sinf/cosf from) that runs
//! pcl::NormalEstimation<PointXYZI, Normal> with setKSearch(10) + the
//! normal-scatter degeneracy pipeline from simple_pgo.cpp on two seeded
//! synthetic clouds mimicking the real failure regime: points 30-80 m from
//! the origin with ~0.1 m local structure, where PCL's single-pass f32
//! covariance suffers severe cancellation. The per-point normals must match
//! bit for bit (asserted via an FNV-1a hash over all 720 f32 bit patterns
//! per cloud, plus spot checks); e_min/e_mid go through an f64 eigensolver
//! on each side (Eigen SelfAdjointEigenSolver vs jacobi_eigen) and must
//! agree after the final f32 cast.

use dimos_gsc_pgo::pointcloud::{cloud_degeneracy, estimate_normals};

/// LCG identical to the C++ harness.
struct Lcg(u64);

impl Lcg {
    fn next_f32(&mut self) -> f32 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((self.0 >> 40) as f32) / (1u32 << 24) as f32
    }
}

/// Cloud A: rough tilted plane ~50-60 m from the origin with ~0.06 m noise
/// (the grass-field regime). Cloud B: ground plane plus two thin walls (a
/// well-constrained corner). Both draw 3 LCG values per point from a shared
/// stream, in the C++ harness's order.
fn make_clouds() -> (Vec<[f32; 3]>, Vec<[f32; 3]>) {
    let mut rng = Lcg(12345);
    let mut a = Vec::with_capacity(240);
    for _ in 0..240 {
        let (u, v, w) = (rng.next_f32(), rng.next_f32(), rng.next_f32());
        let x = 43.0f32 + u * 7.0f32;
        let y = -58.0f32 + v * 7.0f32;
        let z = 5.0f32 + 0.05f32 * (x - 46.0f32) + 0.03f32 * (y + 55.0f32) + (w - 0.5f32) * 0.06f32;
        a.push([x, y, z]);
    }
    let mut b = Vec::with_capacity(240);
    for i in 0..240 {
        let (u, v, w) = (rng.next_f32(), rng.next_f32(), rng.next_f32());
        if i % 3 == 0 {
            b.push([
                61.0f32 + u * 4.0f32,
                34.0f32 + v * 4.0f32,
                2.0f32 + (w - 0.5f32) * 0.04f32,
            ]);
        } else if i % 3 == 1 {
            b.push([
                61.0f32 + (w - 0.5f32) * 0.04f32,
                34.0f32 + u * 4.0f32,
                2.0f32 + v * 2.0f32,
            ]);
        } else {
            b.push([
                61.0f32 + u * 4.0f32,
                34.0f32 + (w - 0.5f32) * 0.04f32,
                2.0f32 + v * 2.0f32,
            ]);
        }
    }
    (a, b)
}

/// FNV-1a 64 over the little-endian bytes of each normal component, in
/// point order — the same digest the C++ harness prints.
fn normals_fnv(normals: &[[f32; 3]]) -> u64 {
    let mut hash = 1469598103934665603u64;
    for n in normals {
        for c in n {
            for byte in c.to_bits().to_le_bytes() {
                hash ^= byte as u64;
                hash = hash.wrapping_mul(1099511628211);
            }
        }
    }
    hash
}

fn check_cloud(
    name: &str,
    cloud: &[[f32; 3]],
    spot: &[(usize, [u32; 3])],
    fnv: u64,
    e_bits: (u32, u32),
) {
    let normals = estimate_normals(cloud, 10);
    assert_eq!(normals.len(), cloud.len());
    for &(i, expected) in spot {
        for c in 0..3 {
            assert_eq!(
                normals[i][c].to_bits(),
                expected[c],
                "{name} normal[{i}][{c}]: got {:e} (0x{:08x}), PCL has 0x{:08x}",
                normals[i][c],
                normals[i][c].to_bits(),
                expected[c]
            );
        }
    }
    assert_eq!(
        normals_fnv(&normals),
        fnv,
        "{name}: per-point normal bit stream diverges from PCL (hash mismatch)"
    );
    let (e_min, e_mid) = cloud_degeneracy(cloud);
    assert_eq!(
        e_min.to_bits(),
        e_bits.0,
        "{name} e_min: got {:e} (0x{:08x}), PCL has 0x{:08x}",
        e_min,
        e_min.to_bits(),
        e_bits.0
    );
    assert_eq!(
        e_mid.to_bits(),
        e_bits.1,
        "{name} e_mid: got {:e} (0x{:08x}), PCL has 0x{:08x}",
        e_mid,
        e_mid.to_bits(),
        e_bits.1
    );
}

// The goldens are bit-exact against x86-64 Linux glibc PCL. The normal-scatter
// eigensolver (pcl_compute_roots_f32) calls libm atan2f/cosf/sinf, whose results
// are arch/libm-specific (Apple libm, and even arm64 glibc, differ from x86-64
// glibc on cancellation-sensitive cubic roots), so a few normals diverge off the
// reference platform. IEEE-only paths (e.g. the ICP parity test) match
// everywhere; run this one only on x86-64 Linux.
#[test]
#[cfg_attr(
    not(all(target_arch = "x86_64", target_os = "linux")),
    ignore = "bit-exact vs x86-64 Linux glibc PCL; libm transcendentals differ off that platform"
)]
fn normals_and_degeneracy_match_pcl_bit_for_bit() {
    let (a, b) = make_clouds();
    check_cloud(
        "A",
        &a,
        &[
            (0, [0x3d1ddc19, 0x3d1dc96a, 0xbf7f9ea2]),
            (1, [0x3d644514, 0x3d0707a9, 0xbf7f7679]),
            (2, [0x3d8685ab, 0x3d064ea1, 0xbf7f4f27]),
            (3, [0x3d47b232, 0x3cef25bb, 0xbf7f961b]),
            (4, [0x3d3d3fe7, 0x3d3c2625, 0xbf7f74c1]),
            (235, [0x3d70792b, 0x3d07fff8, 0xbf7f6ac3]),
            (236, [0x3d031556, 0x3c767f6f, 0xbf7fd703]),
            (237, [0x3d8f8d29, 0x3d0a06ee, 0xbf7f397f]),
            (238, [0x3d10b7df, 0xbc730595, 0xbf7fcfde]),
            (239, [0x3d089372, 0x3c592c05, 0xbf7fd5cc]),
        ],
        0x1af0bddbf2429b67,
        (0x39af2ebf, 0x39dc236a),
    );
    check_cloud(
        "B",
        &b,
        &[
            (0, [0x3b7fb9f2, 0xbbd5188f, 0xbf7ffe1e]),
            (1, [0xbf5565c7, 0xbe85c9ef, 0xbef92df6]),
            (2, [0xbc59fcfb, 0xbf7ff7f8, 0x3c07170f]),
            (3, [0x3c5541b7, 0x3b2cb6e3, 0xbf7ffa38]),
            (4, [0xbf7f5708, 0xbd8fedf9, 0xbc6df1e3]),
            (235, [0xbf560359, 0xbea636db, 0xbee2849f]),
            (236, [0xbd7da705, 0xbf7f814a, 0x3bade47f]),
            (237, [0xbb4bd35a, 0x3b339866, 0xbf7fff70]),
            (238, [0xbf7ffc22, 0xbc29279f, 0x3b5df915]),
            (239, [0xbe84a8c7, 0xbf76e21b, 0x3d59a855]),
        ],
        0x1d69bd639e760738,
        (0x3e955f23, 0x3ea1d9ea),
    );
}
