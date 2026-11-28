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

//! The glibc 2.31 float functions reflective_column_node calls (Ubuntu 20.04 arm64 libm.so.6), as that build computes
//! them: sysdeps/ieee754/flt-32 e_atan2f.c / s_atanf.c (fdlibm) and s_sinf.c, with the fused multiply-adds GCC put in
//! (read from its disassembly). The platform's own atan2f / sinf differ by an ulp now and then, which moves a post's
//! centre by ~3 nm and so flips a printed sixth decimal.

#[allow(clippy::excessive_precision)] // fdlibm's table as glibc writes it; each literal rounds to the same f32
const ATAN_HI: [f32; 4] = [
    4.6364760399e-01,
    7.8539812565e-01,
    9.8279368877e-01,
    1.5707962513e+00,
];
#[allow(clippy::excessive_precision)] // fdlibm's table as glibc writes it; each literal rounds to the same f32
const ATAN_LO: [f32; 4] = [
    5.0121582440e-09,
    3.7748947079e-08,
    3.4473217170e-08,
    7.5497894159e-08,
];
#[allow(clippy::excessive_precision)] // fdlibm's table as glibc writes it; each literal rounds to the same f32
const AT: [f32; 11] = [
    3.3333334327e-01,
    -2.0000000298e-01,
    1.4285714924e-01,
    -1.1111110449e-01,
    9.0908870101e-02,
    -7.6918758452e-02,
    6.6610731184e-02,
    -5.8335702866e-02,
    4.9768779427e-02,
    -3.6531571299e-02,
    1.6285819933e-02,
];

/// s_atanf.c's __atanf.
pub fn atanf(x: f32) -> f32 {
    let hx = x.to_bits() as i32;
    let ix = hx & 0x7fffffff;
    if ix >= 0x4c000000 {
        if ix > 0x7f800000 {
            return x + x;
        }
        return if hx > 0 {
            ATAN_HI[3] + ATAN_LO[3]
        } else {
            -ATAN_HI[3] - ATAN_LO[3]
        };
    }
    let (id, x) = if ix < 0x3ee00000 {
        if ix < 0x31000000 && 1.0e30f32 + x > 1.0 {
            return x;
        }
        (-1, x)
    } else {
        let a = x.abs();
        if ix < 0x3f980000 {
            if ix < 0x3f300000 {
                (0, a.mul_add(2.0, -1.0) / (a + 2.0))
            } else {
                (1, (a - 1.0) / (a + 1.0))
            }
        } else if ix < 0x401c0000 {
            (2, (a - 1.5) / a.mul_add(1.5, 1.0))
        } else {
            (3, -1.0 / a)
        }
    };
    let z = x * x;
    let w = z * z;
    let odd = w
        .mul_add(AT[10], AT[8])
        .mul_add(w, AT[6])
        .mul_add(w, AT[4])
        .mul_add(w, AT[2])
        .mul_add(w, AT[0]);
    let even = w
        .mul_add(AT[9], AT[7])
        .mul_add(w, AT[5])
        .mul_add(w, AT[3])
        .mul_add(w, AT[1])
        * w;
    // z * odd + even: s1 + s2 fused
    let s = x * odd.mul_add(z, even);
    if id < 0 {
        return x - s;
    }
    let id = id as usize;
    let z = ATAN_HI[id] - ((s - ATAN_LO[id]) - x);
    if hx < 0 {
        -z
    } else {
        z
    }
}

/// e_atan2f.c's __ieee754_atan2f.
#[allow(clippy::excessive_precision)] // e_atan2f.c's constants as written; each literal rounds to the same f32
pub fn atan2f(y: f32, x: f32) -> f32 {
    const PI_O_4: f32 = 7.8539818525e-01;
    const PI_O_2: f32 = 1.5707963705e+00;
    const PI: f32 = 3.1415927410e+00;
    const PI_LO: f32 = -8.7422776573e-08;
    const TINY: f32 = 1.0e-30;
    let (hx, hy) = (x.to_bits() as i32, y.to_bits() as i32);
    let (ix, iy) = (hx & 0x7fffffff, hy & 0x7fffffff);
    if ix > 0x7f800000 || iy > 0x7f800000 {
        return x + y;
    }
    if hx == 0x3f800000 {
        return atanf(y);
    }
    let m = ((hy >> 31) & 1) | ((hx >> 30) & 2);
    if iy == 0 {
        return match m {
            0 | 1 => y,
            2 => PI + TINY,
            _ => -PI - TINY,
        };
    }
    if ix == 0 {
        return if hy < 0 {
            -PI_O_2 - TINY
        } else {
            PI_O_2 + TINY
        };
    }
    if ix == 0x7f800000 {
        return match (iy == 0x7f800000, m) {
            (true, 0) => PI_O_4 + TINY,
            (true, 1) => -PI_O_4 - TINY,
            (true, 2) => 3.0f32.mul_add(PI_O_4, TINY),
            (true, _) => (-3.0f32).mul_add(PI_O_4, -TINY),
            (false, 0) => 0.0,
            (false, 1) => -0.0,
            (false, 2) => PI + TINY,
            (false, _) => -PI - TINY,
        };
    }
    if iy == 0x7f800000 {
        return if hy < 0 {
            -PI_O_2 - TINY
        } else {
            PI_O_2 + TINY
        };
    }
    let k = (iy - ix) >> 23;
    let z = if k > 60 {
        0.5f32.mul_add(PI_LO, PI_O_2)
    } else if hx < 0 && k < -60 {
        0.0
    } else {
        atanf((y / x).abs())
    };
    match m {
        0 => z,
        1 => -z,
        2 => PI - (z - PI_LO),
        _ => (z - PI_LO) - PI,
    }
}

/// s_sincosf_data.c's __sincosf_table: sign[4], hpi_inv, hpi, c0..c4, s1..s3 (the second table negates c0..c4).
struct SinCos {
    sign: [f64; 4],
    hpi_inv: f64,
    hpi: f64,
    c: [f64; 5],
    s: [f64; 3],
}

const fn table(cs: f64) -> SinCos {
    let c = [
        f64::from_bits(0x3FF0000000000000),
        f64::from_bits(0xBFDFFFFFFD0C621C),
        f64::from_bits(0x3FA55553E1068F19),
        f64::from_bits(0xBF56C087E89A359D),
        f64::from_bits(0x3EF99343027BF8C3),
    ];
    SinCos {
        sign: [1.0, -1.0, -1.0, 1.0],
        // 0x1.45F306DC9C883p-1, 0x1.921FB54442D18p0
        hpi_inv: f64::from_bits(0x3FE45F306DC9C883),
        hpi: f64::from_bits(0x3FF921FB54442D18),
        c: [c[0] * cs, c[1] * cs, c[2] * cs, c[3] * cs, c[4] * cs],
        s: [
            f64::from_bits(0xBFC555545995A603),
            f64::from_bits(0x3F81107605230BC4),
            f64::from_bits(0xBF2994EB3774CF24),
        ],
    }
}

const SINCOS: [SinCos; 2] = [table(1.0), table(-1.0)];

/// sincosf_poly.h's sinf_poly as compiled (fmadd where the disassembly has them).
fn sinf_poly(x: f64, x2: f64, p: &SinCos, n: i64) -> f32 {
    if n & 1 == 0 {
        let x3 = x * x2;
        let s1 = x2.mul_add(p.s[2], p.s[1]);
        let x7 = x3 * x2;
        let s = x3.mul_add(p.s[0], x);
        s1.mul_add(x7, s) as f32
    } else {
        let x4 = x2 * x2;
        let c2 = x2.mul_add(p.c[4], p.c[3]);
        let c1 = x2.mul_add(p.c[1], p.c[0]);
        let x6 = x2 * x4;
        let c = x4.mul_add(p.c[2], c1);
        c2.mul_add(x6, c) as f32
    }
}

fn abstop12(x: f32) -> u32 {
    (x.to_bits() >> 20) & 0x7ff
}

/// s_sinf.c's sinf for |y| < 120 (reduce_fast with aarch64's round / lround); beyond, the platform's.
pub fn sinf(y: f32) -> f32 {
    let x = y as f64;
    if abstop12(y) < abstop12(std::f32::consts::FRAC_PI_4) {
        if abstop12(y) < abstop12(f32::from_bits(0x39800000)) {
            return y;
        }
        return sinf_poly(x, x * x, &SINCOS[0], 0);
    }
    if abstop12(y) >= abstop12(120.0) {
        return y.sin();
    }
    let r = x * SINCOS[0].hpi_inv;
    let n = r.round() as i64;
    let x = (-SINCOS[0].hpi).mul_add(r.round(), x);
    let s = SINCOS[0].sign[(n & 3) as usize];
    let p = &SINCOS[if n & 2 != 0 { 1 } else { 0 }];
    sinf_poly(x * s, x * x, p, n)
}

/// s_sincosf.c's sincosf for |y| < 120 (as sinf: reduce_fast, the cos polynomial on n ^ 1); beyond, the platform's.
pub fn sincosf(y: f32) -> (f32, f32) {
    let x = y as f64;
    if abstop12(y) < abstop12(std::f32::consts::FRAC_PI_4) {
        if abstop12(y) < abstop12(f32::from_bits(0x39800000)) {
            return (y, 1.0);
        }
        let x2 = x * x;
        return (
            sinf_poly(x, x2, &SINCOS[0], 0),
            sinf_poly(x, x2, &SINCOS[0], 1),
        );
    }
    if abstop12(y) >= abstop12(120.0) {
        return (y.sin(), y.cos());
    }
    let r = x * SINCOS[0].hpi_inv;
    let n = r.round() as i64;
    let x = (-SINCOS[0].hpi).mul_add(r.round(), x);
    let s = SINCOS[0].sign[(n & 3) as usize];
    let p = &SINCOS[if n & 2 != 0 { 1 } else { 0 }];
    (
        sinf_poly(x * s, x * x, p, n),
        sinf_poly(x * s, x * x, p, n ^ 1),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Read;

    /// u32 words of a gzipped fixture: glibc 2.31's own results (tools/glibc_libm_fixtures.c, Ubuntu 20.04 arm64).
    fn words(name: &str) -> Vec<u32> {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("tests/fixtures")
            .join(name);
        let mut bytes = Vec::new();
        flate2::read::GzDecoder::new(std::fs::File::open(path).unwrap())
            .read_to_end(&mut bytes)
            .unwrap();
        bytes
            .as_chunks::<4>()
            .0
            .iter()
            .map(|b| u32::from_le_bytes(*b))
            .collect()
    }

    #[test]
    fn atan2f_is_glibcs_to_the_bit() {
        let w = words("glibc231_atan2f.bin.gz");
        for &[y, x, want] in w.as_chunks::<3>().0 {
            let (y, x) = (f32::from_bits(y), f32::from_bits(x));
            assert_eq!(atan2f(y, x).to_bits(), want, "atan2f({y:e}, {x:e})");
        }
    }

    #[test]
    fn sinf_is_glibcs_to_the_bit() {
        let w = words("glibc231_sinf.bin.gz");
        for &[v, want] in w.as_chunks::<2>().0 {
            let v = f32::from_bits(v);
            assert_eq!(sinf(v).to_bits(), want, "sinf({v:e})");
        }
    }
}
