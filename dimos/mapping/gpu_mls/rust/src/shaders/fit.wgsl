// Pooled plane fit of each voxel in the fit list, the CPU pooled_normal.
// Appended to map_common.wgsl.

const NEIGHBORHOOD: u32 = 27u;
const REWEIGHT_ITERS: u32 = 3u;
const PLANE_SIGMA_FRAC: f32 = 0.5;
const MIN_SUPPORT: f32 = 0.5;
const EIGEN_DEGENERATE: f32 = 1e-12;
const JACOBI_SWEEPS: u32 = 8u;

struct Eigen {
    values: vec3<f32>,
    smallest: vec3<f32>,
}

// Cyclic Jacobi on a symmetric 3x3, eigenvalues ascending. f32 has no closed
// form accurate enough for thin planes, unlike the CPU's f64 one.
fn sym3_eigen(m: mat3x3<f32>) -> Eigen {
    var a = m;
    var vecs = mat3x3<f32>(vec3<f32>(1.0, 0.0, 0.0), vec3<f32>(0.0, 1.0, 0.0), vec3<f32>(0.0, 0.0, 1.0));
    for (var sweep = 0u; sweep < JACOBI_SWEEPS; sweep++) {
        let off = a[0][1] * a[0][1] + a[0][2] * a[0][2] + a[1][2] * a[1][2];
        let diag = a[0][0] * a[0][0] + a[1][1] * a[1][1] + a[2][2] * a[2][2];
        if (off <= 1e-14 * diag) {
            break;
        }
        for (var pair = 0; pair < 3; pair++) {
            let p = select(select(1, 0, pair == 1), 0, pair == 0);
            let q = select(select(2, 2, pair == 1), 1, pair == 0);
            let apq = a[p][q];
            if (abs(apq) < 1e-30) {
                continue;
            }
            let theta = (a[q][q] - a[p][p]) / (2.0 * apq);
            let t = sign(theta) / (abs(theta) + sqrt(theta * theta + 1.0));
            let tt = select(t, 1.0, theta == 0.0);
            let c = 1.0 / sqrt(tt * tt + 1.0);
            let s = tt * c;
            var rot = mat3x3<f32>(vec3<f32>(1.0, 0.0, 0.0), vec3<f32>(0.0, 1.0, 0.0), vec3<f32>(0.0, 0.0, 1.0));
            rot[p][p] = c;
            rot[q][q] = c;
            rot[q][p] = s;
            rot[p][q] = -s;
            a = transpose(rot) * a * rot;
            vecs = vecs * rot;
        }
    }
    var values = vec3<f32>(a[0][0], a[1][1], a[2][2]);
    var order = vec3<i32>(0, 1, 2);
    for (var i = 0; i < 2; i++) {
        for (var j = 0; j < 2 - i; j++) {
            if (values[j] > values[j + 1]) {
                let v = values[j];
                values[j] = values[j + 1];
                values[j + 1] = v;
                let o = order[j];
                order[j] = order[j + 1];
                order[j + 1] = o;
            }
        }
    }
    return Eigen(values, normalize(vecs[order[0]]));
}

// The normal when the eigenvalues are clearly planar, w = 1; w = 0 otherwise.
fn classify(eig: Eigen) -> vec4<f32> {
    let e2 = max(eig.values.z, 0.0);
    if (e2 < EIGEN_DEGENERATE) {
        return vec4<f32>(0.0);
    }
    let l0 = sqrt(max(eig.values.x, 0.0));
    let l1 = sqrt(max(eig.values.y, 0.0));
    let l2 = sqrt(e2);
    let linearity = (l2 - l1) / l2;
    let planarity = (l1 - l0) / l2;
    let scattering = l0 / l2;
    if (planarity < linearity || planarity < scattering) {
        return vec4<f32>(0.0);
    }
    return vec4<f32>(eig.smallest, 1.0);
}

fn outer(a: vec3<f32>, b: vec3<f32>) -> mat3x3<f32> {
    return mat3x3<f32>(a * b.x, a * b.y, a * b.z);
}

// The pooled plane: its normal (w = 1 when planar), and the mean of the
// final reweighted neighborhood about the voxel center, a point on it.
struct PooledFit {
    normal: vec4<f32>,
    mean: vec3<f32>,
}

// Neighbor k of 27 is offset (k / 9 - 1, k / 3 % 3 - 1, k % 3 - 1).
fn neighbor_offset(k: u32) -> vec3<i32> {
    return vec3<i32>(i32(k / 9u) - 1, i32(k / 3u % 3u) - 1, i32(k % 3u) - 1);
}

fn pooled_normal(key: vec3<i32>) -> vec4<f32> {
    var counts: array<f32, 27>;
    var sums: array<vec3<f32>, 27>;
    var seconds: array<mat3x3<f32>, 27>;
    var used = 0u;
    var n_raw = 0u;
    let vs = params.voxel_size;
    for (var dx = -1; dx <= 1; dx++) {
        for (var dy = -1; dy <= 1; dy++) {
            for (var dz = -1; dz <= 1; dz++) {
                let v = find_voxel(key + vec3<i32>(dx, dy, dz));
                if (v == NO_SLOT) {
                    continue;
                }
                let count = moment_count(v);
                if (count == 0u) {
                    continue;
                }
                let ni = f32(count);
                let sum = vec3<f32>(moment_value(v, 0u), moment_value(v, 1u), moment_value(v, 2u)) * vs;
                let xx = moment_value(v, 3u);
                let xy = moment_value(v, 4u);
                let xz = moment_value(v, 5u);
                let yy = moment_value(v, 6u);
                let yz = moment_value(v, 7u);
                let zz = moment_value(v, 8u);
                let m2 = mat3x3<f32>(vec3<f32>(xx, xy, xz), vec3<f32>(xy, yy, yz), vec3<f32>(xz, yz, zz)) * (vs * vs);
                // Shift this voxel's center-relative moments to the target center.
                let d = vec3<f32>(f32(dx), f32(dy), f32(dz)) * vs;
                counts[used] = ni;
                sums[used] = sum + d * ni;
                seconds[used] = m2 + outer(sum, d) + outer(d, sum) + outer(d, d) * ni;
                used += 1u;
                n_raw += count;
            }
        }
    }
    if (n_raw < NORMAL_MIN_POINTS) {
        return vec4<f32>(0.0);
    }

    let sigma = PLANE_SIGMA_FRAC * vs;
    let two_sig2 = 2.0 * sigma * sigma;
    var weights: array<f32, 27>;
    for (var k = 0u; k < used; k++) {
        weights[k] = 1.0;
    }
    var fitted = false;
    var last: Eigen;
    for (var iter = 0u; iter < REWEIGHT_ITERS; iter++) {
        var wn = 0.0;
        var s = vec3<f32>(0.0);
        var t = mat3x3<f32>(vec3<f32>(0.0), vec3<f32>(0.0), vec3<f32>(0.0));
        for (var k = 0u; k < used; k++) {
            wn += weights[k] * counts[k];
            s += sums[k] * weights[k];
            t += seconds[k] * weights[k];
        }
        if (wn < 1e-6) {
            break;
        }
        let mean = s / wn;
        let cov = t * (1.0 / wn) - outer(mean, mean);
        let eig = sym3_eigen(cov);
        for (var k = 0u; k < used; k++) {
            let dist = abs(dot(eig.smallest, sums[k] / counts[k] - mean));
            weights[k] = exp(-(dist * dist) / two_sig2);
        }
        last = eig;
        fitted = true;
    }
    var kept = 0.0;
    for (var k = 0u; k < used; k++) {
        kept += weights[k] * counts[k];
    }
    if (!fitted || kept < MIN_SUPPORT * f32(n_raw)) {
        return vec4<f32>(0.0);
    }
    return classify(last);
}

@compute @workgroup_size(64)
fn fit_normals(
    @builtin(workgroup_id) wg: vec3<u32>,
    @builtin(num_workgroups) nwg: vec3<u32>,
    @builtin(local_invocation_index) li: u32,
) {
    let i = thread_index(wg, nwg, li);
    if (i >= list_len(C_FITS)) {
        return;
    }
    let v = lists[L_FITS * params.list_capacity + i];
    let fit = pooled_normal(key_of_voxel(v));
    set_normal(v, pack_normal(select(vec3<f32>(0.0, 0.0, 1.0), fit.xyz, fit.w > 0.0)));
    if (fit.w > 0.0) {
        atomicOr(&voxel_state[v], S_PLANAR);
    } else {
        atomicAnd(&voxel_state[v], ~S_PLANAR);
    }
    atomicAnd(&voxel_state[v], ~S_STALE);
}
