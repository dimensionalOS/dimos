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

//! dock::libm::sincosf against glibc 2.31's (aarch64, the boards' Ubuntu 20.04: rl_deploy's feet rotation calls it):
//! tests/fixtures/sincosf_glibc231.bin.gz, 4000 f32 (x, sin, cos) with |x| < 120 (40k random draws in [-0.5, 0.5],
//! [-4, 4] and [-150, 150] went through glibc 2.31 in an arm64 Ubuntu 20.04 container; every one below 120 matched).

use std::io::Read;

#[test]
fn sincosf_matches_glibc_bit_for_bit() {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures/sincosf_glibc231.bin.gz");
    let mut bytes = Vec::new();
    flate2::read::GzDecoder::new(std::fs::File::open(path).unwrap())
        .read_to_end(&mut bytes)
        .unwrap();
    let v: Vec<f32> = bytes
        .chunks(4)
        .map(|c| f32::from_le_bytes(c.try_into().unwrap()))
        .collect();
    let mut misses = Vec::new();
    for r in v.chunks(3) {
        let (s, c) = m20_controller::dock::libm::sincosf(r[0]);
        if s.to_bits() != r[1].to_bits() || c.to_bits() != r[2].to_bits() {
            misses.push(format!(
                "x {}: ours ({s}, {c}) glibc ({}, {})",
                r[0], r[1], r[2]
            ));
        }
    }
    assert!(
        misses.is_empty(),
        "{} of {} differ:\n{}",
        misses.len(),
        v.len() / 3,
        misses.join("\n")
    );
}
