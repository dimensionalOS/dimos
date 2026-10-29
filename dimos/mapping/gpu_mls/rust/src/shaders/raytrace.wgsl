// One thread per ray: the CPU find_misses_along_ray walk. Pass 0 lists the
// stale voxels a ray needs fitted, pass 1 lists the misses. Appended to
// map_common.wgsl.

// Stands in for the CPU walk's infinities, which fast math does not keep.
const FAR: f32 = 3.0e38;
const MAX_WALK_STEPS: u32 = 1048576u;

// Rust's signum: +0 steps forward, -0 back.
fn walk_step(d: f32) -> i32 {
    return select(1, -1, (bitcast<u32>(d) >> 31u) != 0u);
}

fn first_crossing(p: f32, d: f32, vox: i32, step: i32) -> f32 {
    if (d == 0.0) {
        return FAR;
    }
    let boundary = select(f32(vox), f32(vox + 1), step > 0) * params.voxel_size;
    return (boundary - p) / d;
}

fn crossing_interval(d: f32) -> f32 {
    if (d == 0.0) {
        return FAR;
    }
    return params.voxel_size / abs(d);
}

// Fine cells of `key` the segment between t0 and t1 crosses, as a 64-bit mask.
fn crossed_fine_cells(origin: vec3<f32>, delta: vec3<f32>, t0: f32, t1: f32, key: vec3<i32>) -> vec2<u32> {
    let divisor = i32(params.fine_divisor);
    let fine_size = params.voxel_size / f32(divisor);
    var l = clamp(
        vec3<i32>(floor((origin + t0 * delta) / fine_size)) - key * divisor,
        vec3<i32>(0),
        vec3<i32>(divisor - 1),
    );
    var step = vec3<i32>(0);
    var t = vec3<f32>(FAR);
    var dt = vec3<f32>(FAR);
    for (var a = 0; a < 3; a++) {
        if (delta[a] != 0.0) {
            step[a] = select(-1, 1, delta[a] > 0.0);
            let boundary = f32(key[a] * divisor + l[a] + select(0, 1, step[a] > 0)) * fine_size;
            t[a] = (boundary - origin[a]) / delta[a];
            dt[a] = fine_size / abs(delta[a]);
        }
    }
    var mask = vec2<u32>(0u);
    for (var guard = 0; guard < 64; guard++) {
        let index = u32((l.x * divisor + l.y) * divisor + l.z);
        mask[index >> 5u] |= 1u << (index & 31u);
        if (min(t.x, min(t.y, t.z)) >= t1) {
            break;
        }
        var axis = 2;
        if (t.x <= t.y && t.x <= t.z) {
            axis = 0;
        } else if (t.y <= t.z) {
            axis = 1;
        }
        l[axis] += step[axis];
        t[axis] += dt[axis];
        if (l[axis] < 0 || l[axis] >= divisor) {
            break;
        }
    }
    return mask;
}

@compute @workgroup_size(64)
fn trace_rays(
    @builtin(workgroup_id) wg: vec3<u32>,
    @builtin(num_workgroups) nwg: vec3<u32>,
    @builtin(local_invocation_index) li: u32,
) {
    let i = thread_index(wg, nwg, li) * params.ray_step;
    if (i >= params.point_count) {
        return;
    }
    let origin = params.origin.xyz;
    let end = points[i].xyz;
    let endpoint = voxel_of(end);
    let origin_voxel = voxel_of(origin);
    if (all(origin_voxel == endpoint)) {
        return;
    }
    let d = end - origin;
    var cell = origin_voxel;
    let step = vec3<i32>(walk_step(d.x), walk_step(d.y), walk_step(d.z));
    var tx = first_crossing(origin.x, d.x, cell.x, step.x);
    var ty = first_crossing(origin.y, d.y, cell.y, step.y);
    var tz = first_crossing(origin.z, d.z, cell.z, step.z);
    let dt = vec3<f32>(crossing_interval(d.x), crossing_interval(d.y), crossing_interval(d.z));

    let endpoint_center = voxel_center(endpoint);
    let shadow_sq = params.shadow_depth * params.shadow_depth;
    let grace_sq = params.grace_depth * params.grace_depth;
    let ray_len = sqrt(dot(d, d));
    let t_max = 1.0 + params.shadow_depth / max(ray_len, 1.1920929e-7);
    let ray_unit = d / max(ray_len, 1.1920929e-7);

    var cached_brick = vec3<i32>(0x7fffffff);
    var cached_slot = NO_SLOT;
    var past_endpoint = false;
    for (var guard = 0u; guard < MAX_WALK_STEPS; guard++) {
        let t_enter = min(tx, min(ty, tz));
        if (t_enter > t_max) {
            return;
        }
        if (t_enter >= 1.0) {
            past_endpoint = true;
        }
        if (tx < ty) {
            if (tx < tz) {
                cell.x += step.x;
                tx += dt.x;
            } else {
                cell.z += step.z;
                tz += dt.z;
            }
        } else if (ty < tz) {
            cell.y += step.y;
            ty += dt.y;
        } else {
            cell.z += step.z;
            tz += dt.z;
        }

        if (all(cell == endpoint)) {
            past_endpoint = true;
            continue;
        }
        let offset = voxel_center(cell) - endpoint_center;
        let dist_sq = dot(offset, offset);
        if (past_endpoint) {
            if (dist_sq > shadow_sq) {
                return;
            }
        } else if (dist_sq < grace_sq) {
            continue;
        }

        let brick = brick_of(cell);
        if (any(brick != cached_brick)) {
            cached_brick = brick;
            cached_slot = find_brick(brick);
        }
        if (cached_slot == NO_SLOT) {
            continue;
        }
        let v = voxel_at(cached_slot * BRICK_VOXELS + local_index(cell));
        if (v == NO_SLOT) {
            continue;
        }
        let state = atomicLoad(&voxel_state[v]);
        if ((state & S_EXISTS) == 0u) {
            continue;
        }
        if (atomicLoad(&stamps[v * 3u + STAMP_HIT]) == params.frame) {
            continue;
        }
        if (params.fine_divisor >= 2u) {
            let observed = vec2<u32>(atomicLoad(&fine[v * 2u]), atomicLoad(&fine[v * 2u + 1u]));
            if (any(observed != vec2<u32>(0u))) {
                let crossed = crossed_fine_cells(origin, d, t_enter, min(tx, min(ty, tz)), cell);
                if (all((crossed & observed) == vec2<u32>(0u))) {
                    continue;
                }
            }
        }
        if (params.trace_pass == 0u) {
            if ((state & S_STALE) != 0u && atomicExchange(&stamps[v * 3u + STAMP_FIT], params.frame) != params.frame) {
                push(L_FITS, C_FITS, v);
            }
            continue;
        }
        let spare = (state & S_PLANAR) != 0u && abs(dot(ray_unit, unpack_normal(normal_of(v)))) < params.graze_cos;
        if (!spare && atomicExchange(&stamps[v * 3u + STAMP_MISS], params.frame) != params.frame) {
            push(L_MISSES, C_MISSES, v);
        }
    }
}
