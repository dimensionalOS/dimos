// Brick map bindings and helpers shared by every map kernel.

struct MapParams {
    origin: vec4<f32>,
    voxel_size: f32,
    inv_voxel: f32,
    shadow_depth: f32,
    grace_depth: f32,
    graze_cos: f32,
    min_health: i32,
    max_health: i32,
    fine_divisor: u32,
    point_count: u32,
    ray_step: u32,
    frame: u32,
    mode: u32,
    table_mask: u32,
    brick_capacity: u32,
    list_capacity: u32,
    trace_pass: u32,
    voxel_capacity: u32,
    // The grace and shadow depths squared, in cells.
    grace_cells_sq: f32,
    shadow_cells_sq: f32,
    pad: u32,
}

const MODE_TRACE: u32 = 0u;
const MODE_SEED: u32 = 1u;
const MODE_INGEST: u32 = 2u;

const EMPTY_KEY: u32 = 0xffffffffu;
const NO_SLOT: u32 = 0xffffffffu;
// A brick_voxels entry: 0 for no voxel, else its voxel id plus one, or this
// while the voxel's id is being assigned.
const RESERVED_VOXEL: u32 = 0xffffffffu;
const BRICK_BITS: i32 = 3;
const BRICK_VOXELS: u32 = 512u;

const C_BRICKS: u32 = 0u;
const C_NEW: u32 = 1u;
const C_HITS: u32 = 2u;
const C_FITS: u32 = 3u;
const C_MISSES: u32 = 4u;
const C_CHANGED: u32 = 5u;
const C_NEW_VOXELS: u32 = 6u;
const C_OVERFLOW: u32 = 7u;
const C_VOXELS: u32 = 8u;
// Hits the frame's report kept: those it changed.
const C_REPORT_HITS: u32 = 9u;

const L_NEW: u32 = 0u;
const L_HITS: u32 = 1u;
const L_FITS: u32 = 2u;
const L_MISSES: u32 = 3u;
const L_CHANGED: u32 = 4u;
const L_NEW_VOXELS: u32 = 5u;
const LISTS: u32 = 6u;

const S_EXISTS: u32 = 1u;
const S_STALE: u32 = 2u;
const S_CREATED: u32 = 4u;
// The cached normal is a plane; clear when the fit found none.
const S_PLANAR: u32 = 8u;
// The surface fit cached in voxel_data is out of date: the voxel or one of
// its 26 neighbors changed materially since it was made (or it never was).
const S_SURFACE_STALE: u32 = 16u;
// The cached surface fit found a plane.
const S_SURFACE_PLANAR: u32 = 32u;
// This frame changed the voxel's health or fine cells, so the report lists it.
const S_REPORT: u32 = 64u;

const STAMP_HIT: u32 = 0u;
const STAMP_FIT: u32 = 1u;
const STAMP_MISS: u32 = 2u;

// Count, then sum (3) and second moment (xx, xy, xz, yy, yz, zz) about the
// voxel center in voxel units, as 32-bit fixed point. A point adds at most
// FIXED_SCALE / 2 to any word, and settle_hits halves a voxel's moments once
// its count reaches MOMENT_HALVING_COUNT, so no word overflows.
const MOMENT_WORDS: u32 = 10u;
const FIXED_SCALE: f32 = 65536.0;
const MOMENT_HALVING_COUNT: u32 = 16384u;

const NORMAL_MIN_POINTS: u32 = 3u;

// voxel_data words: health, next refit count, packed normal, brick site, then
// the cached surface fit: its height above the voxel's bottom face
// (NO_FIT_HEIGHT when the plane gives none) and its packed upward normal.
const VOXEL_DATA_WORDS: u32 = 6u;

@group(0) @binding(0) var<uniform> params: MapParams;
@group(0) @binding(1) var<storage, read_write> table_keys: array<atomic<u32>>;
@group(0) @binding(2) var<storage, read_write> table_slots: array<u32>;
@group(0) @binding(3) var<storage, read_write> brick_coords: array<vec4<i32>>;
// Per brick site (slot * BRICK_VOXELS + local index), its voxel's id.
@group(0) @binding(4) var<storage, read_write> brick_voxels: array<atomic<u32>>;
@group(0) @binding(5) var<storage, read_write> counters: array<atomic<u32>>;
@group(0) @binding(6) var<storage, read_write> lists: array<u32>;
@group(0) @binding(7) var<storage, read_write> points: array<vec4<f32>>;
// Per voxel id from here on.
@group(0) @binding(8) var<storage, read_write> voxel_state: array<atomic<u32>>;
@group(0) @binding(9) var<storage, read_write> stamps: array<atomic<u32>>;
@group(0) @binding(10) var<storage, read_write> fine: array<atomic<u32>>;
@group(0) @binding(11) var<storage, read_write> moments: array<atomic<u32>>;
@group(0) @binding(12) var<storage, read_write> voxel_data: array<u32>;

fn thread_index(wg: vec3<u32>, nwg: vec3<u32>, local: u32) -> u32 {
    return (wg.y * nwg.x + wg.x) * 64u + local;
}

fn voxel_of(p: vec3<f32>) -> vec3<i32> {
    return vec3<i32>(floor(p * params.inv_voxel));
}

fn brick_of(key: vec3<i32>) -> vec3<i32> {
    return key >> vec3<u32>(3u);
}

fn local_index(key: vec3<i32>) -> u32 {
    let l = vec3<u32>(key & vec3<i32>(7));
    return (l.x << 6u) | (l.y << 3u) | l.z;
}

// 11 bits x, 11 bits y, 10 bits z, biased; the all-ones key is reserved as empty.
fn pack_brick(b: vec3<i32>) -> u32 {
    let u = vec3<u32>(b + vec3<i32>(1024, 1024, 512));
    return (u.x & 2047u) | ((u.y & 2047u) << 11u) | ((u.z & 1023u) << 22u);
}

fn brick_in_range(b: vec3<i32>) -> bool {
    let inside = all(b >= vec3<i32>(-1024, -1024, -512)) && all(b < vec3<i32>(1023, 1023, 511));
    return inside;
}

fn hash_key(key: u32) -> u32 {
    var h = key;
    h ^= h >> 16u;
    h *= 0x85ebca6bu;
    h ^= h >> 13u;
    h *= 0xc2b2ae35u;
    h ^= h >> 16u;
    return h;
}

// The brick's pool slot, NO_SLOT when absent.
fn find_brick(b: vec3<i32>) -> u32 {
    if (!brick_in_range(b)) {
        return NO_SLOT;
    }
    let key = pack_brick(b);
    var i = hash_key(key) & params.table_mask;
    for (var probe = 0u; probe <= params.table_mask; probe++) {
        let k = atomicLoad(&table_keys[i]);
        if (k == key) {
            return table_slots[i];
        }
        if (k == EMPTY_KEY) {
            return NO_SLOT;
        }
        i = (i + 1u) & params.table_mask;
    }
    return NO_SLOT;
}

// The voxel id at a brick site, NO_SLOT when it has none.
fn voxel_at(site: u32) -> u32 {
    let entry = atomicLoad(&brick_voxels[site]);
    if (entry == 0u || entry == RESERVED_VOXEL) {
        return NO_SLOT;
    }
    return entry - 1u;
}

// The voxel id of a key, NO_SLOT when it has none.
fn find_voxel(key: vec3<i32>) -> u32 {
    let slot = find_brick(brick_of(key));
    if (slot == NO_SLOT) {
        return NO_SLOT;
    }
    return voxel_at(slot * BRICK_VOXELS + local_index(key));
}

fn health_of(v: u32) -> i32 {
    return bitcast<i32>(voxel_data[v * VOXEL_DATA_WORDS]);
}

fn set_health(v: u32, h: i32) {
    voxel_data[v * VOXEL_DATA_WORDS] = bitcast<u32>(h);
}

fn next_fit_of(v: u32) -> u32 {
    return voxel_data[v * VOXEL_DATA_WORDS + 1u];
}

fn set_next_fit(v: u32, count: u32) {
    voxel_data[v * VOXEL_DATA_WORDS + 1u] = count;
}

// Octahedral-packed unit normal, meaningful where S_PLANAR is set.
fn normal_of(v: u32) -> u32 {
    return voxel_data[v * VOXEL_DATA_WORDS + 2u];
}

fn set_normal(v: u32, packed: u32) {
    voxel_data[v * VOXEL_DATA_WORDS + 2u] = packed;
}

fn surface_fit_height(v: u32) -> f32 {
    return bitcast<f32>(voxel_data[v * VOXEL_DATA_WORDS + 4u]);
}

fn surface_fit_normal(v: u32) -> u32 {
    return voxel_data[v * VOXEL_DATA_WORDS + 5u];
}

fn set_surface_fit(v: u32, height: f32, normal: u32) {
    voxel_data[v * VOXEL_DATA_WORDS + 4u] = bitcast<u32>(height);
    voxel_data[v * VOXEL_DATA_WORDS + 5u] = normal;
}

fn site_of(v: u32) -> u32 {
    return voxel_data[v * VOXEL_DATA_WORDS + 3u];
}

fn voxel_exists(v: u32) -> bool {
    return (atomicLoad(&voxel_state[v]) & S_EXISTS) != 0u;
}

fn push(list: u32, counter: u32, value: u32) {
    let i = atomicAdd(&counters[counter], 1u);
    if (i < params.list_capacity) {
        lists[list * params.list_capacity + i] = value;
    } else {
        atomicOr(&counters[C_OVERFLOW], 1u << list);
    }
}

fn list_len(counter: u32) -> u32 {
    return min(atomicLoad(&counters[counter]), params.list_capacity);
}

fn voxel_center(key: vec3<i32>) -> vec3<f32> {
    return (vec3<f32>(key) + vec3<f32>(0.5)) * params.voxel_size;
}

// The key of a brick site from its brick's recorded coordinate.
fn key_of_site(site: u32) -> vec3<i32> {
    let b = brick_coords[site / BRICK_VOXELS].xyz;
    let l = site % BRICK_VOXELS;
    return (b << vec3<u32>(3u)) | vec3<i32>(i32(l >> 6u), i32((l >> 3u) & 7u), i32(l & 7u));
}

fn key_of_voxel(v: u32) -> vec3<i32> {
    return key_of_site(site_of(v));
}

fn voxel_count() -> u32 {
    return min(atomicLoad(&counters[C_VOXELS]), params.voxel_capacity);
}

// Read a fixed-point moment word as f32 in voxel units.
fn moment_value(v: u32, word: u32) -> f32 {
    return f32(bitcast<i32>(atomicLoad(&moments[v * MOMENT_WORDS + 1u + word]))) / FIXED_SCALE;
}

fn sign_not_zero(p: vec2<f32>) -> vec2<f32> {
    return select(vec2<f32>(-1.0), vec2<f32>(1.0), p >= vec2<f32>(0.0));
}

fn pack_normal(n: vec3<f32>) -> u32 {
    let p = n.xy / (abs(n.x) + abs(n.y) + abs(n.z));
    let q = select(p, (vec2<f32>(1.0) - abs(p.yx)) * sign_not_zero(p), n.z < 0.0);
    return pack2x16snorm(q);
}

fn unpack_normal(packed: u32) -> vec3<f32> {
    let q = unpack2x16snorm(packed);
    var n = vec3<f32>(q, 1.0 - abs(q.x) - abs(q.y));
    let t = max(-n.z, 0.0);
    n.x += select(t, -t, n.x >= 0.0);
    n.y += select(t, -t, n.y >= 0.0);
    return normalize(n);
}

fn moment_count(v: u32) -> u32 {
    return atomicLoad(&moments[v * MOMENT_WORDS]);
}
