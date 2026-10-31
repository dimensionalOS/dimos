// Point ingest: brick allocation, hit marking, health and moment updates,
// misses, and normal staleness. Appended to map_common.wgsl.

const SEED_HEALTH: i32 = 1;
const OVERFLOW_TABLE: u32 = 256u;
const OVERFLOW_BRICKS: u32 = 512u;
const OVERFLOW_VOXELS: u32 = 1024u;

@group(1) @binding(0) var<storage, read_write> dispatch_args: array<u32>;

fn write_args(slot: u32, threads: u32) {
    let groups = (threads + 63u) / 64u;
    dispatch_args[slot * 3u] = min(groups, 65535u);
    dispatch_args[slot * 3u + 1u] = (groups + 65534u) / 65535u;
    dispatch_args[slot * 3u + 2u] = 1u;
}

// Indirect sizes for every list, the brick pool at one thread per site, then
// one thread per voxel.
@compute @workgroup_size(1)
fn prepare_dispatch() {
    for (var list = 0u; list < LISTS; list++) {
        write_args(list, list_len(list + 1u));
    }
    let bricks = min(atomicLoad(&counters[C_BRICKS]), params.brick_capacity);
    write_args(LISTS, bricks * BRICK_VOXELS);
    write_args(LISTS + 1u, voxel_count());
}

@compute @workgroup_size(64)
fn insert_bricks(
    @builtin(workgroup_id) wg: vec3<u32>,
    @builtin(num_workgroups) nwg: vec3<u32>,
    @builtin(local_invocation_index) li: u32,
) {
    let i = thread_index(wg, nwg, li);
    if (i >= params.point_count) {
        return;
    }
    let b = brick_of(voxel_of(points[i].xyz));
    if (!brick_in_range(b)) {
        return;
    }
    let key = pack_brick(b);
    // A full pool takes no new keys: slotless keys would fill the table and
    // make every lookup probe all of it. Only the frame that fills the pool
    // adds any, at most one per point, which the table's headroom covers.
    let full = atomicLoad(&counters[C_BRICKS]) >= params.brick_capacity;
    var entry = hash_key(key) & params.table_mask;
    for (var probe = 0u; probe <= params.table_mask; probe++) {
        let k = atomicLoad(&table_keys[entry]);
        if (k == key) {
            return;
        }
        if (k == EMPTY_KEY) {
            if (full) {
                atomicOr(&counters[C_OVERFLOW], OVERFLOW_BRICKS);
                return;
            }
            let r = atomicCompareExchangeWeak(&table_keys[entry], EMPTY_KEY, key);
            if (r.exchanged) {
                push(L_NEW, C_NEW, entry);
                return;
            }
            if (r.old_value == key) {
                return;
            }
            // A weak exchange may fail spuriously on an empty entry; retry it.
            if (r.old_value == EMPTY_KEY) {
                continue;
            }
        }
        entry = (entry + 1u) & params.table_mask;
    }
    atomicOr(&counters[C_OVERFLOW], OVERFLOW_TABLE);
}

@compute @workgroup_size(64)
fn assign_bricks(
    @builtin(workgroup_id) wg: vec3<u32>,
    @builtin(num_workgroups) nwg: vec3<u32>,
    @builtin(local_invocation_index) li: u32,
) {
    let i = thread_index(wg, nwg, li);
    if (i >= list_len(C_NEW)) {
        return;
    }
    let entry = lists[L_NEW * params.list_capacity + i];
    let slot = atomicAdd(&counters[C_BRICKS], 1u);
    if (slot >= params.brick_capacity) {
        table_slots[entry] = NO_SLOT;
        atomicOr(&counters[C_OVERFLOW], OVERFLOW_BRICKS);
        return;
    }
    table_slots[entry] = slot;
    let key = atomicLoad(&table_keys[entry]);
    let b = vec3<i32>(
        i32(key & 2047u) - 1024,
        i32((key >> 11u) & 2047u) - 1024,
        i32(key >> 22u) - 512,
    );
    brick_coords[slot] = vec4<i32>(b, 0);
}

// The first point into a site without a voxel reserves it for assign_voxels.
@compute @workgroup_size(64)
fn reserve_voxels(
    @builtin(workgroup_id) wg: vec3<u32>,
    @builtin(num_workgroups) nwg: vec3<u32>,
    @builtin(local_invocation_index) li: u32,
) {
    let i = thread_index(wg, nwg, li);
    if (i >= params.point_count) {
        return;
    }
    let key = voxel_of(points[i].xyz);
    let slot = find_brick(brick_of(key));
    if (slot == NO_SLOT) {
        return;
    }
    let site = slot * BRICK_VOXELS + local_index(key);
    if (atomicLoad(&brick_voxels[site]) != 0u) {
        return;
    }
    if (atomicCompareExchangeWeak(&brick_voxels[site], 0u, RESERVED_VOXEL).exchanged) {
        push(L_NEW_VOXELS, C_NEW_VOXELS, site);
    }
}

// Ids are never reused, so a new id's per-voxel words are still zero.
@compute @workgroup_size(64)
fn assign_voxels(
    @builtin(workgroup_id) wg: vec3<u32>,
    @builtin(num_workgroups) nwg: vec3<u32>,
    @builtin(local_invocation_index) li: u32,
) {
    let i = thread_index(wg, nwg, li);
    if (i >= list_len(C_NEW_VOXELS)) {
        return;
    }
    let site = lists[L_NEW_VOXELS * params.list_capacity + i];
    let v = atomicAdd(&counters[C_VOXELS], 1u);
    if (v >= params.voxel_capacity) {
        atomicStore(&brick_voxels[site], 0u);
        atomicOr(&counters[C_OVERFLOW], OVERFLOW_VOXELS);
        return;
    }
    voxel_data[v * VOXEL_DATA_WORDS + 3u] = site;
    atomicStore(&brick_voxels[site], v + 1u);
}

// Every pool brick back into an empty, larger table.
@compute @workgroup_size(64)
fn rehash(
    @builtin(workgroup_id) wg: vec3<u32>,
    @builtin(num_workgroups) nwg: vec3<u32>,
    @builtin(local_invocation_index) li: u32,
) {
    let slot = thread_index(wg, nwg, li);
    if (slot >= min(atomicLoad(&counters[C_BRICKS]), params.brick_capacity)) {
        return;
    }
    let key = pack_brick(brick_coords[slot].xyz);
    var entry = hash_key(key) & params.table_mask;
    for (var probe = 0u; probe <= params.table_mask; probe++) {
        let r = atomicCompareExchangeWeak(&table_keys[entry], EMPTY_KEY, key);
        if (r.exchanged) {
            table_slots[entry] = slot;
            return;
        }
        if (r.old_value != EMPTY_KEY) {
            entry = (entry + 1u) & params.table_mask;
        }
    }
    atomicOr(&counters[C_OVERFLOW], OVERFLOW_TABLE);
}

// First point into a voxel this frame lists it as hit, creating it when absent.
@compute @workgroup_size(64)
fn mark_hits(
    @builtin(workgroup_id) wg: vec3<u32>,
    @builtin(num_workgroups) nwg: vec3<u32>,
    @builtin(local_invocation_index) li: u32,
) {
    let i = thread_index(wg, nwg, li);
    if (i >= params.point_count) {
        return;
    }
    let v = find_voxel(voxel_of(points[i].xyz));
    if (v == NO_SLOT) {
        return;
    }
    if (atomicExchange(&stamps[v * 3u + STAMP_HIT], params.frame) == params.frame) {
        return;
    }
    push(L_HITS, C_HITS, v);
    if (!voxel_exists(v)) {
        atomicStore(&voxel_state[v], S_EXISTS | S_STALE | S_SURFACE_STALE | S_CREATED | S_REPORT);
        set_health(v, select(params.min_health, SEED_HEALTH, params.mode == MODE_SEED));
        set_next_fit(v, NORMAL_MIN_POINTS);
        set_normal(v, 0u);
    }
}

@compute @workgroup_size(64)
fn apply_hits(
    @builtin(workgroup_id) wg: vec3<u32>,
    @builtin(num_workgroups) nwg: vec3<u32>,
    @builtin(local_invocation_index) li: u32,
) {
    let i = thread_index(wg, nwg, li);
    if (i >= list_len(C_HITS) || params.mode == MODE_SEED) {
        return;
    }
    let v = lists[L_HITS * params.list_capacity + i];
    let health = health_of(v);
    if (health < params.max_health) {
        set_health(v, health + 1);
        atomicOr(&voxel_state[v], S_REPORT);
    }
}

fn add_fixed(word: u32, value: i32) {
    atomicAdd(&moments[word], bitcast<u32>(value));
}

fn to_fixed(x: f32) -> i32 {
    return i32(round(x * FIXED_SCALE));
}

@compute @workgroup_size(64)
fn accumulate(
    @builtin(workgroup_id) wg: vec3<u32>,
    @builtin(num_workgroups) nwg: vec3<u32>,
    @builtin(local_invocation_index) li: u32,
) {
    let i = thread_index(wg, nwg, li);
    if (i >= params.point_count) {
        return;
    }
    let p = points[i].xyz;
    let key = voxel_of(p);
    let v = find_voxel(key);
    if (v == NO_SLOT) {
        return;
    }
    if (params.mode == MODE_SEED && (atomicLoad(&voxel_state[v]) & S_CREATED) == 0u) {
        return;
    }
    let u = (p - voxel_center(key)) * params.inv_voxel;
    let base = v * MOMENT_WORDS;
    atomicAdd(&moments[base], 1u);
    add_fixed(base + 1u, to_fixed(u.x));
    add_fixed(base + 2u, to_fixed(u.y));
    add_fixed(base + 3u, to_fixed(u.z));
    add_fixed(base + 4u, to_fixed(u.x * u.x));
    add_fixed(base + 5u, to_fixed(u.x * u.y));
    add_fixed(base + 6u, to_fixed(u.x * u.z));
    add_fixed(base + 7u, to_fixed(u.y * u.y));
    add_fixed(base + 8u, to_fixed(u.y * u.z));
    add_fixed(base + 9u, to_fixed(u.z * u.z));

    let divisor = i32(params.fine_divisor);
    if (divisor >= 2) {
        let inv_fine = f32(divisor) / params.voxel_size;
        let corner = vec3<f32>(key) * params.voxel_size;
        let l = clamp(vec3<i32>(floor((p - corner) * inv_fine)), vec3<i32>(0), vec3<i32>(divisor - 1));
        let index = u32((l.x * divisor + l.y) * divisor + l.z);
        let bit = 1u << (index & 31u);
        if ((atomicOr(&fine[v * 2u + (index >> 5u)], bit) & bit) == 0u) {
            atomicOr(&voxel_state[v], S_REPORT);
        }
    }
}

// Advance each hit voxel's refit milestone and list the voxels whose
// neighborhoods changed materially.
@compute @workgroup_size(64)
fn settle_hits(
    @builtin(workgroup_id) wg: vec3<u32>,
    @builtin(num_workgroups) nwg: vec3<u32>,
    @builtin(local_invocation_index) li: u32,
) {
    let i = thread_index(wg, nwg, li);
    if (i >= list_len(C_HITS)) {
        return;
    }
    let v = lists[L_HITS * params.list_capacity + i];
    let count = moment_count(v);
    var milestone = next_fit_of(v);
    var crossed = false;
    while (milestone <= count) {
        crossed = true;
        milestone += max(milestone / 2u, 1u);
    }
    // Halving keeps the mean and covariance, so the fit is unchanged.
    if (count >= MOMENT_HALVING_COUNT) {
        let base = v * MOMENT_WORDS;
        atomicStore(&moments[base], count >> 1u);
        for (var w = 1u; w < MOMENT_WORDS; w++) {
            atomicStore(&moments[base + w], bitcast<u32>(bitcast<i32>(atomicLoad(&moments[base + w])) >> 1u));
        }
        milestone = max(milestone >> 1u, (count >> 1u) + 1u);
    }
    set_next_fit(v, milestone);
    let created = (atomicAnd(&voxel_state[v], ~S_CREATED) & S_CREATED) != 0u;
    if (created || (crossed && params.mode != MODE_SEED)) {
        push(L_CHANGED, C_CHANGED, v);
    }
}

@compute @workgroup_size(64)
fn apply_misses(
    @builtin(workgroup_id) wg: vec3<u32>,
    @builtin(num_workgroups) nwg: vec3<u32>,
    @builtin(local_invocation_index) li: u32,
) {
    let i = thread_index(wg, nwg, li);
    if (i >= list_len(C_MISSES)) {
        return;
    }
    let v = lists[L_MISSES * params.list_capacity + i];
    let h = health_of(v) - 1;
    if (h > params.min_health) {
        set_health(v, h);
        return;
    }
    atomicStore(&voxel_state[v], 0u);
    set_health(v, 0);
    for (var w = 0u; w < MOMENT_WORDS; w++) {
        atomicStore(&moments[v * MOMENT_WORDS + w], 0u);
    }
    atomicStore(&fine[v * 2u], 0u);
    atomicStore(&fine[v * 2u + 1u], 0u);
    set_next_fit(v, NORMAL_MIN_POINTS);
    set_normal(v, 0u);
    push(L_CHANGED, C_CHANGED, v);
}

@compute @workgroup_size(64)
fn mark_stale(
    @builtin(workgroup_id) wg: vec3<u32>,
    @builtin(num_workgroups) nwg: vec3<u32>,
    @builtin(local_invocation_index) li: u32,
) {
    let i = thread_index(wg, nwg, li);
    if (i >= list_len(C_CHANGED)) {
        return;
    }
    let key = key_of_voxel(lists[L_CHANGED * params.list_capacity + i]);
    for (var dx = -1; dx <= 1; dx++) {
        for (var dy = -1; dy <= 1; dy++) {
            for (var dz = -1; dz <= 1; dz++) {
                let n = find_voxel(key + vec3<i32>(dx, dy, dz));
                if (n != NO_SLOT && voxel_exists(n)) {
                    atomicOr(&voxel_state[n], S_STALE | S_SURFACE_STALE);
                }
            }
        }
    }
}
