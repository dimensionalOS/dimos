// Each voxel a frame hit or missed, with its key, health and fine mask after
// the frame, so the host can mirror the map without reading it all back,
// then the keys of the voxels it changed materially (created, removed, or
// past a refit milestone), which is what moves a surface.

@group(1) @binding(0) var<storage, read_write> report: array<u32>;

// Key x, y, z, health, fine mask low, high.
const REPORT_WORDS: u32 = 6u;
// The health word of a voxel the frame removed.
const REPORT_REMOVED: u32 = 0x80000000u;

fn report_voxel(slot: u32, v: u32) {
    let key = key_of_voxel(v);
    let base = slot * REPORT_WORDS;
    report[base] = bitcast<u32>(key.x);
    report[base + 1u] = bitcast<u32>(key.y);
    report[base + 2u] = bitcast<u32>(key.z);
    report[base + 3u] = select(REPORT_REMOVED, bitcast<u32>(health_of(v)), voxel_exists(v));
    report[base + 4u] = atomicLoad(&fine[v * 2u]);
    report[base + 5u] = atomicLoad(&fine[v * 2u + 1u]);
}

@compute @workgroup_size(64)
fn report_hits(
    @builtin(workgroup_id) wg: vec3<u32>,
    @builtin(num_workgroups) nwg: vec3<u32>,
    @builtin(local_invocation_index) li: u32,
) {
    let i = thread_index(wg, nwg, li);
    if (i >= list_len(C_HITS)) {
        return;
    }
    report_voxel(i, lists[L_HITS * params.list_capacity + i]);
}

// After the hits, so the report is hits then misses.
@compute @workgroup_size(64)
fn report_misses(
    @builtin(workgroup_id) wg: vec3<u32>,
    @builtin(num_workgroups) nwg: vec3<u32>,
    @builtin(local_invocation_index) li: u32,
) {
    let i = thread_index(wg, nwg, li);
    if (i >= list_len(C_MISSES)) {
        return;
    }
    report_voxel(list_len(C_HITS) + i, lists[L_MISSES * params.list_capacity + i]);
}

// After the misses: the changed list, key only.
@compute @workgroup_size(64)
fn report_changed(
    @builtin(workgroup_id) wg: vec3<u32>,
    @builtin(num_workgroups) nwg: vec3<u32>,
    @builtin(local_invocation_index) li: u32,
) {
    let i = thread_index(wg, nwg, li);
    if (i >= list_len(C_CHANGED)) {
        return;
    }
    let key = key_of_voxel(lists[L_CHANGED * params.list_capacity + i]);
    let base = (list_len(C_HITS) + list_len(C_MISSES) + i) * REPORT_WORDS;
    report[base] = bitcast<u32>(key.x);
    report[base + 1u] = bitcast<u32>(key.y);
    report[base + 2u] = bitcast<u32>(key.z);
}
