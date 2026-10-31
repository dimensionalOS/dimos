// Surface extraction over a dense window of columns: occupancy from the
// brick map, per-cell plane heights, the standable test, bit-parallel round
// hole closing per z slice (32 slices per word op), and jump-flood boundary
// distance. Appended to map_common.wgsl and fit.wgsl.
//
// Heights are sub-voxel: each occupied cell carries the height of its voxel's
// pooled plane at the column center (its top face when it has no plane), and
// headroom, slope and step links are judged in meters from those planes.
//
// Column bit layout: word (y * W + x) * ZW + z / 32, bit z % 32.
// Cell layout: (z * H + y) * W + x.
//
// Boundary distance runs per plane: a connected surface (4-neighbor links with a
// step of at most step_m between the planes) projected from above.
// Where a connected surface overlaps itself in x,y (a ramp under a balcony),
// it splits into sheets: within a column, a cell more than a step above the
// surface's cell below it starts the next sheet. Each (surface, sheet) piece
// connected under equal sheets is a plane; cells of one plane sharing a
// column (a hole filled at two adjacent z) share that column's distance.

struct SurfaceParams {
    window_min: vec4<i32>,
    // W, H, D, ZW
    size: vec4<u32>,
    closing_cells: i32,
    // floor(step_m / voxel_size); links are looked for one cell further.
    step_cells: i32,
    hole_span_cells: i32,
    support_radius: i32,
    support_z_tol: i32,
    voxel_size: f32,
    robot_height_m: f32,
    step_m: f32,
    // A plane whose normal has |z| below this is too steep to stand on; -1 disables the gate.
    min_up: f32,
    merge_m: f32,
    merge_cos: f32,
    // 1 when `columns` limits the passes to its read columns and the export
    // to its write columns; 0 runs and writes the whole window.
    masked: u32,
    // Window-local columns export_cells writes out: x0, y0, x1, y1 inclusive.
    write_box: vec4<i32>,
    // Window-local z range export_cells writes out: lo, hi inclusive.
    write_z: vec4<i32>,
}

struct FloodStep {
    step: i32,
    // 1 reads seeds_a and writes seeds_b, 0 the other way.
    a_to_b: u32,
}

const NO_SEED: u32 = 0xffffffffu;

@group(1) @binding(0) var<uniform> sp: SurfaceParams;
@group(1) @binding(1) var<storage, read_write> occupied: array<atomic<u32>>;
@group(1) @binding(2) var<storage, read_write> standable: array<u32>;
@group(1) @binding(3) var<storage, read_write> grown: array<u32>;
@group(1) @binding(4) var<storage, read_write> surface: array<u32>;
@group(1) @binding(5) var<storage, read_write> boundary: array<u32>;
@group(1) @binding(6) var<storage, read_write> seeds_a: array<u32>;
@group(1) @binding(7) var<storage, read_write> seeds_b: array<u32>;
@group(1) @binding(8) var<storage, read_write> boundary_distance: array<f32>;
// Two union-find parent arrays over cells: components, then planes.
@group(1) @binding(9) var<storage, read_write> labels: array<atomic<u32>>;
@group(1) @binding(10) var<storage, read_write> heights: array<f32>;
// Octahedral-packed upward normals, meaningful where the planar bit is set.
@group(1) @binding(11) var<storage, read_write> cell_normals: array<u32>;
@group(1) @binding(12) var<storage, read_write> planar: array<atomic<u32>>;
// Bit per window column (y * W + x): the columns the passes read, then the
// columns the export writes, then a bit per window brick column holding any
// read column.
@group(1) @binding(13) var<storage, read_write> columns: array<u32>;
@group(2) @binding(0) var<uniform> flood: FloodStep;

fn mask_words() -> u32 {
    return (sp.size.x * sp.size.y + 31u) / 32u;
}

fn mask_bit(section: u32, i: u32) -> bool {
    return ((columns[section * mask_words() + i / 32u] >> (i & 31u)) & 1u) != 0u;
}

// A column the passes compute; the others read as empty.
fn column_active(x: i32, y: i32) -> bool {
    return sp.masked == 0u || mask_bit(0u, u32(y) * sp.size.x + u32(x));
}

fn column_writes(x: i32, y: i32) -> bool {
    return sp.masked == 0u || mask_bit(1u, u32(y) * sp.size.x + u32(x));
}

// Whether window brick column (bx, by) holds any active column.
fn brick_column_active(bx: u32, by: u32) -> bool {
    let lo = sp.window_min.xy >> vec2<u32>(3u);
    let hi = (sp.window_min.xy + vec2<i32>(sp.size.xy) - 1) >> vec2<u32>(3u);
    let across = u32(hi.x - lo.x + 1);
    return sp.masked == 0u || mask_bit(2u, by * across + bx);
}

fn column_word(x: i32, y: i32, z: i32) -> u32 {
    return (u32(y) * sp.size.x + u32(x)) * sp.size.w + u32(z) / 32u;
}

fn in_window_xy(x: i32, y: i32) -> bool {
    return x >= 0 && y >= 0 && x < i32(sp.size.x) && y < i32(sp.size.y);
}

fn occupied_at(x: i32, y: i32, z: i32) -> bool {
    if (!in_window_xy(x, y) || z < 0 || z >= i32(sp.size.z)) {
        return false;
    }
    return ((atomicLoad(&occupied[column_word(x, y, z)]) >> (u32(z) & 31u)) & 1u) != 0u;
}

fn bit_at(x: i32, y: i32, z: i32, which: u32) -> bool {
    if (!in_window_xy(x, y) || z < 0 || z >= i32(sp.size.z)) {
        return false;
    }
    let w = column_word(x, y, z);
    var word = 0u;
    switch which {
        case 0u: { word = standable[w]; }
        case 1u: { word = surface[w]; }
        case 2u: { word = grown[w]; }
        case 4u: { word = atomicLoad(&planar[w]); }
        default: { word = boundary[w]; }
    }
    return ((word >> (u32(z) & 31u)) & 1u) != 0u;
}

fn column_has_occupied(x: i32, y: i32) -> bool {
    if (!in_window_xy(x, y)) {
        return false;
    }
    let base = column_word(x, y, 0);
    for (var w = 0u; w < sp.size.w; w++) {
        if (atomicLoad(&occupied[base + w]) != 0u) {
            return true;
        }
    }
    return false;
}

fn cell_index(x: i32, y: i32, z: i32) -> u32 {
    return (u32(z) * sp.size.y + u32(y)) * sp.size.x + u32(x);
}

// A boundary seed is its cell index, so it carries the boundary's z.
fn seed_dist_sq(x: i32, y: i32, seed: u32) -> i32 {
    if (seed == NO_SEED) {
        return 0x7fffffff;
    }
    let dx = i32(seed % sp.size.x) - x;
    let dy = i32((seed / sp.size.x) % sp.size.y) - y;
    return dx * dx + dy * dy;
}

const NO_DISTANCE: f32 = 3.0e38;

// Squared meters to a seed counting the surface's rise, as the CPU's walked edges do.
fn seed_dist_sq_3d(x: i32, y: i32, z: i32, seed: u32) -> f32 {
    if (seed == NO_SEED) {
        return NO_DISTANCE;
    }
    let flat = f32(seed_dist_sq(x, y, seed)) * sp.voxel_size * sp.voxel_size;
    let rise = heights[seed] - heights[cell_index(x, y, z)];
    return flat + rise * rise;
}

// The brick coordinate of the window's low corner; dispatches over the
// window's bricks count from it.
fn window_brick_lo() -> vec3<i32> {
    return sp.window_min.xyz >> vec3<u32>(3u);
}

// Healthy voxels inside the window set their column bit. One group per
// window brick, which leaves at once when the map has no such brick.
@compute @workgroup_size(64)
fn build_occupancy(
    @builtin(workgroup_id) wg: vec3<u32>,
    @builtin(local_invocation_index) li: u32,
) {
    if (!brick_column_active(wg.x, wg.y)) {
        return;
    }
    let slot = find_brick(window_brick_lo() + vec3<i32>(wg));
    if (slot == NO_SLOT) {
        return;
    }
    for (var r = 0u; r < BRICK_VOXELS / 64u; r++) {
        let site = slot * BRICK_VOXELS + r * 64u + li;
        let v = voxel_at(site);
        if (v == NO_SLOT || !voxel_exists(v) || health_of(v) <= 0) {
            continue;
        }
        let local = key_of_site(site) - sp.window_min.xyz;
        if (any(local < vec3<i32>(0)) || any(vec3<u32>(local) >= sp.size.xyz) || !column_active(local.x, local.y)) {
            continue;
        }
        atomicOr(&occupied[column_word(local.x, local.y, local.z)], 1u << (u32(local.z) & 31u));
    }
}

fn bottom_face(z: i32) -> f32 {
    return f32(sp.window_min.z + z) * sp.voxel_size;
}

fn is_planar(x: i32, y: i32, z: i32) -> bool {
    return bit_at(x, y, z, 4u);
}

fn cell_normal(x: i32, y: i32, z: i32) -> vec3<f32> {
    return unpack_normal(cell_normals[cell_index(x, y, z)]);
}

fn too_steep(x: i32, y: i32, z: i32) -> bool {
    return is_planar(x, y, z) && cell_normal(x, y, z).z < sp.min_up;
}

// The cell's surface height at a horizontal offset (m) from its column
// center: its plane where it has one, else its stored height. The rise is
// bounded to a 63 degree slope so a near-vertical plane cannot run away.
fn height_at(x: i32, y: i32, z: i32, offset: vec2<f32>) -> f32 {
    let h = heights[cell_index(x, y, z)];
    if (!is_planar(x, y, z)) {
        return h;
    }
    let n = cell_normal(x, y, z);
    let reach = 2.0 * max(abs(offset.x), abs(offset.y));
    return h + clamp(-dot(n.xy, offset) / max(n.z, 1e-3), -reach, reach);
}

// The lowest point of the cell's surface over its column square, the
// underside a robot below would meet; its bottom face without a plane.
fn underside(x: i32, y: i32, z: i32) -> f32 {
    let bottom = bottom_face(z);
    if (!is_planar(x, y, z)) {
        return bottom;
    }
    let n = cell_normal(x, y, z);
    let drop = (abs(n.x) + abs(n.y)) * 0.5 * sp.voxel_size / max(n.z, 1e-3);
    return clamp(heights[cell_index(x, y, z)] - drop, bottom, bottom + sp.voxel_size);
}

// The underside of the first occupied cell above z, or far away.
fn ceiling_above(x: i32, y: i32, z: i32) -> f32 {
    for (var above = z + 1; above < i32(sp.size.z); above++) {
        if (occupied_at(x, y, above)) {
            return underside(x, y, above);
        }
    }
    return 3.0e38;
}

// Two surface cells, a hop of whole columns apart, meet within a step: their
// planes evaluated halfway between their column centers differ by at most step_m.
fn within_step(x: i32, y: i32, z: i32, nx: i32, ny: i32, nz: i32) -> bool {
    let half = vec2<f32>(f32(nx - x), f32(ny - y)) * (0.5 * sp.voxel_size);
    return abs(height_at(x, y, z, half) - height_at(nx, ny, nz, -half)) <= sp.step_m;
}

// A workgroup of 64 threads takes half a brick: 4 x-slabs, 4 x 8 x 8
// voxels. Their neighborhoods all fall in that block grown by one voxel
// (6 x 10 x 10), whose moments are loaded once into workgroup memory so the
// fit's repeated passes over 27 neighbors read there. The voxels that need a
// fit (a few dozen in a typical brick) are packed onto the first lanes.
const TILE_SLABS: u32 = 4u;
const TILE_CELLS: u32 = 600u;
var<workgroup> tile_counts: array<u32, 600>;
var<workgroup> tile_moments: array<f32, 5400>;
var<workgroup> tile_bricks: array<u32, 27>;
var<workgroup> tile_list: array<u32, 256>;
var<workgroup> tile_len: atomic<u32>;
var<workgroup> tile_any: u32;

// Tile index of block-relative (x in -1..4, y and z in -1..8).
fn tile_index(p: vec3<i32>) -> u32 {
    return u32((p.x + 1) * 100 + (p.y + 1) * 10 + (p.z + 1));
}

fn tile_count(i: u32) -> f32 {
    return f32(tile_counts[i]);
}

// The tile frame: meters from the block's center, shared by every cell so
// the fit's passes sum moments without shifting them. Within 5.5 voxels of
// it, f32 keeps a thin plane's covariance to about 1e-7 of its spread.
fn tile_center(p: vec3<i32>) -> vec3<f32> {
    return (vec3<f32>(p) - vec3<f32>(1.5, 3.5, 3.5)) * sp.voxel_size;
}

fn tile_mean(i: u32) -> vec3<f32> {
    return vec3<f32>(tile_moments[i * 9u], tile_moments[i * 9u + 1u], tile_moments[i * 9u + 2u]);
}

// The cell's second moment about the tile frame's origin, xx xy xz yy yz zz.
fn tile_second(i: u32) -> mat3x3<f32> {
    let m = i * 9u + 3u;
    let xx = tile_moments[m];
    let xy = tile_moments[m + 1u];
    let xz = tile_moments[m + 2u];
    let yy = tile_moments[m + 3u];
    let yz = tile_moments[m + 4u];
    let zz = tile_moments[m + 5u];
    return mat3x3<f32>(vec3<f32>(xx, xy, xz), vec3<f32>(xy, yy, yz), vec3<f32>(xz, yz, zz));
}

// Load voxel `v`'s moments (about its center, block-relative `p`) into tile
// cell `c`, moved to the tile frame.
fn tile_store(c: u32, v: u32, p: vec3<i32>) {
    let vs = sp.voxel_size;
    let n = f32(moment_count(v));
    let sum = vec3<f32>(moment_value(v, 0u), moment_value(v, 1u), moment_value(v, 2u)) * vs;
    let m2 = mat3x3<f32>(
        vec3<f32>(moment_value(v, 3u), moment_value(v, 4u), moment_value(v, 5u)),
        vec3<f32>(moment_value(v, 4u), moment_value(v, 6u), moment_value(v, 7u)),
        vec3<f32>(moment_value(v, 5u), moment_value(v, 7u), moment_value(v, 8u)),
    ) * (vs * vs);
    let d = tile_center(p);
    let moved = m2 + outer(sum, d) + outer(d, sum) + outer(d, d) * n;
    let mean = sum / n + d;
    let base = c * 9u;
    tile_moments[base] = mean.x;
    tile_moments[base + 1u] = mean.y;
    tile_moments[base + 2u] = mean.z;
    tile_moments[base + 3u] = moved[0][0];
    tile_moments[base + 4u] = moved[0][1];
    tile_moments[base + 5u] = moved[0][2];
    tile_moments[base + 6u] = moved[1][1];
    tile_moments[base + 7u] = moved[1][2];
    tile_moments[base + 8u] = moved[2][2];
}

// The pooled plane of the voxel at block-relative `at`, from the tile: the
// CPU pooled_normal, except the reweighting starts from each neighbor's height
// off this voxel's own points instead of uniform, so at a step edge the fit
// settles on the voxel's own level rather than a tilted plane through both.
fn tile_fit(at: vec3<i32>) -> PooledFit {
    var n_raw = 0u;
    for (var k = 0u; k < NEIGHBORHOOD; k++) {
        n_raw += tile_counts[tile_index(at + neighbor_offset(k))];
    }
    if (n_raw < NORMAL_MIN_POINTS) {
        return PooledFit(vec4<f32>(0.0), vec3<f32>(0.0));
    }
    let sigma = PLANE_SIGMA_FRAC * sp.voxel_size;
    let two_sig2 = 2.0 * sigma * sigma;
    let own = tile_index(at);
    let level = tile_counts[own] > 0u;
    let own_z = tile_mean(own).z;
    // The previous pass's plane; none yet, so weights start from the level.
    var plane_n = vec3<f32>(0.0);
    var plane_p = vec3<f32>(0.0);
    var fitted = false;
    var last: Eigen;
    for (var iter = 0u; iter <= REWEIGHT_ITERS; iter++) {
        var wn = 0.0;
        var s = vec3<f32>(0.0);
        var t = mat3x3<f32>(vec3<f32>(0.0), vec3<f32>(0.0), vec3<f32>(0.0));
        for (var k = 0u; k < NEIGHBORHOOD; k++) {
            let i = tile_index(at + neighbor_offset(k));
            if (tile_counts[i] == 0u) {
                continue;
            }
            let mean = tile_mean(i);
            var w = 1.0;
            if (fitted) {
                let dist = dot(plane_n, mean - plane_p);
                w = exp(-(dist * dist) / two_sig2);
            } else if (level) {
                let rise = mean.z - own_z;
                w = exp(-(rise * rise) / two_sig2);
            }
            let wi = w * tile_count(i);
            wn += wi;
            s += mean * wi;
            if (iter < REWEIGHT_ITERS) {
                t += tile_second(i) * w;
            }
        }
        // The pass after the last fit only tallies the support kept.
        if (iter == REWEIGHT_ITERS) {
            if (!fitted || wn < MIN_SUPPORT * f32(n_raw)) {
                return PooledFit(vec4<f32>(0.0), vec3<f32>(0.0));
            }
            return PooledFit(classify(last), s / wn - tile_center(at));
        }
        if (wn < 1e-6) {
            return PooledFit(vec4<f32>(0.0), vec3<f32>(0.0));
        }
        let mean = s / wn;
        let cov = t * (1.0 / wn) - outer(mean, mean);
        last = sym3_eigen(cov);
        plane_n = last.smallest;
        plane_p = mean;
        fitted = true;
    }
    return PooledFit(vec4<f32>(0.0), vec3<f32>(0.0));
}

// Each healthy voxel's surface height and normal from its pooled plane. A
// voxel inside a solid stack is never a surface nor an underside, so it
// keeps its top face and skips the fit.
@compute @workgroup_size(64)
fn fit_cells(
    @builtin(workgroup_id) wg: vec3<u32>,
    @builtin(local_invocation_index) li: u32,
) {
    // Dispatched two groups per window brick along x, each taking a half
    // brick; a brick the map lacks leaves at once.
    let half = wg.x % 2u;
    let slot = find_brick(window_brick_lo() + vec3<i32>(i32(wg.x / 2u), i32(wg.y), i32(wg.z)));
    let first = slot * BRICK_VOXELS + half * TILE_SLABS * 64u;
    // Workgroup memory starts uninitialized (see Gpu::pipeline).
    if (li == 0u) {
        atomicStore(&tile_len, 0u);
        tile_any = select(0u, 1u, slot != NO_SLOT && brick_column_active(wg.x / 2u, wg.y));
    }
    if (workgroupUniformLoad(&tile_any) == 0u) {
        return;
    }
    for (var r = 0u; r < TILE_SLABS; r++) {
        let site = first + r * 64u + li;
        let v = voxel_at(site);
        if (v == NO_SLOT || !voxel_exists(v) || health_of(v) <= 0) {
            continue;
        }
        let local = key_of_site(site) - sp.window_min.xyz;
        if (any(local < vec3<i32>(0)) || any(vec3<u32>(local) >= sp.size.xyz)) {
            continue;
        }
        let i = cell_index(local.x, local.y, local.z);
        heights[i] = bottom_face(local.z) + sp.voxel_size;
        cell_normals[i] = 0u;
        if (occupied_at(local.x, local.y, local.z - 1) && occupied_at(local.x, local.y, local.z + 1)) {
            continue;
        }
        // Refit only where the moments moved since the cached fit was made.
        let state = atomicLoad(&voxel_state[v]);
        if ((state & S_SURFACE_STALE) != 0u) {
            tile_list[atomicAdd(&tile_len, 1u)] = r * 64u + li;
        } else if ((state & S_SURFACE_PLANAR) != 0u) {
            apply_surface_fit(v, local, i);
        }
    }
    workgroupBarrier();
    if (li == 0u) {
        tile_any = atomicLoad(&tile_len);
    }
    let len = workgroupUniformLoad(&tile_any);
    if (len == 0u) {
        return;
    }

    let home = brick_coords[slot].xyz;
    let block_x = i32(half * TILE_SLABS);
    if (li < 27u) {
        let offset = neighbor_offset(li);
        tile_bricks[li] = select(find_brick(home + offset), slot, all(offset == vec3<i32>(0)));
    }
    workgroupBarrier();
    for (var c = li; c < TILE_CELLS; c += 64u) {
        let p = vec3<i32>(i32(c / 100u) - 1, i32(c / 10u % 10u) - 1, i32(c % 10u) - 1);
        let brick_local = vec3<i32>(block_x + p.x, p.y, p.z);
        let step = brick_local >> vec3<u32>(3u);
        let b = tile_bricks[u32((step.x + 1) * 9 + (step.y + 1) * 3 + (step.z + 1))];
        var count = 0u;
        if (b != NO_SLOT) {
            let nv = voxel_at(b * BRICK_VOXELS + local_index(brick_local));
            if (nv != NO_SLOT) {
                count = moment_count(nv);
            }
            if (count > 0u) {
                tile_store(c, nv, p);
            }
        }
        tile_counts[c] = count;
    }
    workgroupBarrier();

    for (var j = li; j < len; j += 64u) {
        let offset = tile_list[j];
        let key = key_of_site(first + offset);
        let cell = key - sp.window_min.xyz;
        let ci = cell_index(cell.x, cell.y, cell.z);
        let cell_bottom = bottom_face(cell.z);
        let l = offset % 64u;
        let fit = tile_fit(vec3<i32>(i32(offset / 64u), i32((l >> 3u) & 7u), i32(l & 7u)));
        let v = voxel_at(first + offset);
        if (fit.normal.w <= 0.0) {
            atomicAnd(&voxel_state[v], ~(S_SURFACE_STALE | S_SURFACE_PLANAR));
            continue;
        }
        let n = select(fit.normal.xyz, -fit.normal.xyz, fit.normal.z < 0.0);
        var above = NO_FIT_HEIGHT;
        if (n.z > 1e-3) {
            // The plane through the pooled mean, at the column center; kept within
            // half a voxel of this one so a sparse voxel cannot drift off.
            let h = voxel_center(key).z + fit.mean.z + dot(n.xy, fit.mean.xy) / n.z;
            above = clamp(h - cell_bottom, -0.5 * sp.voxel_size, 1.5 * sp.voxel_size);
        }
        set_surface_fit(v, above, pack_normal(n));
        atomicAnd(&voxel_state[v], ~S_SURFACE_STALE);
        atomicOr(&voxel_state[v], S_SURFACE_PLANAR);
        apply_surface_fit(v, cell, ci);
    }
}

// A cached fit's height when its plane is too steep to give one.
const NO_FIT_HEIGHT: f32 = 3.0e38;

// Write a voxel's cached surface fit into its window cell.
fn apply_surface_fit(v: u32, cell: vec3<i32>, ci: u32) {
    let above = surface_fit_height(v);
    if (above != NO_FIT_HEIGHT) {
        heights[ci] = bottom_face(cell.z) + above;
    }
    cell_normals[ci] = surface_fit_normal(v);
    atomicOr(&planar[column_word(cell.x, cell.y, cell.z)], 1u << (u32(cell.z) & 31u));
}

// A voxel and the one above it holding the same plane (a floor on a grid
// line) are one surface: the upper takes their mean height and the lower
// stops being an obstacle under it.
@compute @workgroup_size(8, 8, 1)
fn merge_columns(@builtin(global_invocation_id) gid: vec3<u32>) {
    if (gid.x >= sp.size.x || gid.y >= sp.size.y) {
        return;
    }
    let x = i32(gid.x);
    let y = i32(gid.y);
    if (!column_active(x, y)) {
        return;
    }
    for (var z = 0; z + 1 < i32(sp.size.z); z++) {
        if (!occupied_at(x, y, z) || !occupied_at(x, y, z + 1) || !is_planar(x, y, z) || !is_planar(x, y, z + 1)) {
            continue;
        }
        let lower = cell_index(x, y, z);
        let upper = cell_index(x, y, z + 1);
        let agree = dot(cell_normal(x, y, z), cell_normal(x, y, z + 1)) >= sp.merge_cos;
        if (agree && abs(heights[upper] - heights[lower]) <= sp.merge_m) {
            heights[upper] = 0.5 * (heights[upper] + heights[lower]);
            atomicAnd(&occupied[column_word(x, y, z)], ~(1u << (u32(z) & 31u)));
        }
    }
}

// Per column: occupied cells with robot_height of clear space between their
// plane and the underside of the next occupied cell above, unless too steep.
@compute @workgroup_size(8, 8, 1)
fn standable_columns(@builtin(global_invocation_id) gid: vec3<u32>) {
    if (gid.x >= sp.size.x || gid.y >= sp.size.y) {
        return;
    }
    let x = i32(gid.x);
    let y = i32(gid.y);
    let base = column_word(x, y, 0);
    for (var w = 0u; w < sp.size.w; w++) {
        standable[base + w] = 0u;
    }
    if (!column_active(x, y)) {
        return;
    }
    var ceiling = 3.0e38;
    for (var z = i32(sp.size.z) - 1; z >= 0; z--) {
        if (!occupied_at(x, y, z)) {
            continue;
        }
        if (!too_steep(x, y, z) && ceiling - heights[cell_index(x, y, z)] >= sp.robot_height_m) {
            standable[base + u32(z) / 32u] |= 1u << (u32(z) & 31u);
        }
        ceiling = underside(x, y, z);
    }
}

// A filled cell's height: the mean of the standable heights in its slice
// within the closing radius, one of which closing guarantees.
fn filled_height(x: i32, y: i32, z: i32) -> f32 {
    let n = sp.closing_cells;
    var sum = 0.0;
    var count = 0.0;
    for (var dy = -n; dy <= n; dy++) {
        for (var dx = -n; dx <= n; dx++) {
            if (dx * dx + dy * dy > n * n || !bit_at(x + dx, y + dy, z, 0u)) {
                continue;
            }
            sum += heights[cell_index(x + dx, y + dy, z)];
            count += 1.0;
        }
    }
    return select(bottom_face(z) + sp.voxel_size, sum / count, count > 0.0);
}

// The bits of column (x, y) at z + shift, aligned to word `word`'s z range:
// bit b holds z = 32 * word + b + shift. Outside the window reads empty.
fn shifted_word(which: u32, x: i32, y: i32, word: i32, shift: i32) -> u32 {
    if (!in_window_xy(x, y)) {
        return 0u;
    }
    let base = column_word(x, y, 0);
    let words = i32(sp.size.w);
    let start = word * 32 + shift;
    let first = start >> 5u;
    let offset = u32(start & 31);
    var out = 0u;
    if (first >= 0 && first < words) {
        out = column_bits(which, base + u32(first)) >> offset;
    }
    if (offset != 0u && first + 1 >= 0 && first + 1 < words) {
        out |= column_bits(which, base + u32(first + 1)) << (32u - offset);
    }
    return out;
}

fn column_bits(which: u32, w: u32) -> u32 {
    switch which {
        case 0u: { return standable[w]; }
        case 2u: { return grown[w]; }
        default: { return atomicLoad(&occupied[w]); }
    }
}

const BITS_OCCUPIED: u32 = 5u;

// Grow: a cell is grown when a standable cell of its slice lies within the
// closing radius (Euclidean, in cells). 32 slices per word op.
@compute @workgroup_size(8, 8, 1)
fn grow_columns(@builtin(global_invocation_id) gid: vec3<u32>) {
    if (gid.x >= sp.size.x || gid.y >= sp.size.y) {
        return;
    }
    let x = i32(gid.x);
    let y = i32(gid.y);
    let n = sp.closing_cells;
    let base = column_word(x, y, 0);
    if (!column_active(x, y)) {
        for (var w = 0u; w < sp.size.w; w++) {
            grown[base + w] = 0u;
        }
        return;
    }
    for (var w = 0u; w < sp.size.w; w++) {
        var bits = 0u;
        for (var dy = -n; dy <= n; dy++) {
            for (var dx = -n; dx <= n; dx++) {
                if (dx * dx + dy * dy > n * n || !in_window_xy(x + dx, y + dy)) {
                    continue;
                }
                bits |= standable[column_word(x + dx, y + dy, 0) + w];
            }
        }
        grown[base + w] = bits;
    }
}

// Shrink the grown set by the same disk, outside the window unfilled, then
// keep the filled cells with headroom and nearby support, as close_at_z.
@compute @workgroup_size(8, 8, 1)
fn compose_surface(@builtin(global_invocation_id) gid: vec3<u32>) {
    if (gid.x >= sp.size.x || gid.y >= sp.size.y) {
        return;
    }
    let x = i32(gid.x);
    let y = i32(gid.y);
    let n = sp.closing_cells;
    let base = column_word(x, y, 0);
    if (!column_active(x, y)) {
        for (var w = 0u; w < sp.size.w; w++) {
            surface[base + w] = 0u;
        }
        return;
    }
    for (var w = 0u; w < sp.size.w; w++) {
        let original = standable[base + w];
        var closed = 0u;
        if (n > 0) {
            closed = grown[base + w];
            for (var dy = -n; dy <= n && closed != 0u; dy++) {
                for (var dx = -n; dx <= n; dx++) {
                    if (dx * dx + dy * dy > n * n) {
                        continue;
                    }
                    if (!in_window_xy(x + dx, y + dy)) {
                        closed = 0u;
                        break;
                    }
                    closed &= grown[column_word(x + dx, y + dy, 0) + w];
                }
            }
        }
        var added = closed & ~original;
        if (added != 0u) {
            var support = 0u;
            let r = sp.support_radius;
            for (var dy = -r; dy <= r; dy++) {
                for (var dx = -r; dx <= r; dx++) {
                    for (var dz = -sp.support_z_tol; dz <= sp.support_z_tol; dz++) {
                        support |= shifted_word(BITS_OCCUPIED, x + dx, y + dy, i32(w), dz);
                    }
                }
            }
            added &= support;
        }
        // Headroom in meters, from a height borrowed off the standable cells around.
        var candidates = added;
        while (candidates != 0u) {
            let bit = firstTrailingBit(candidates);
            candidates &= candidates - 1u;
            let z = i32(w * 32u + bit);
            let h = filled_height(x, y, z);
            if (ceiling_above(x, y, z) - h >= sp.robot_height_m) {
                heights[cell_index(x, y, z)] = h;
            } else {
                added &= ~(1u << bit);
            }
        }
        surface[base + w] = original | added;
    }
}

// The neighbor column holds a surface cell within a step of this one.
fn surface_linked(x: i32, y: i32, z: i32, nx: i32, ny: i32) -> bool {
    let reach = sp.step_cells + 1;
    for (var dz = -reach; dz <= reach; dz++) {
        if (bit_at(nx, ny, z + dz, 1u) && within_step(x, y, z, nx, ny, z + dz)) {
            return true;
        }
    }
    return false;
}

// The CPU edge_in_direction: the first non-empty column within the hole span
// decides, an all-empty span is an edge.
fn edge_in_direction(x: i32, y: i32, z: i32, dx: i32, dy: i32) -> bool {
    for (var k = 1; k <= sp.hole_span_cells; k++) {
        let nx = x + dx * k;
        let ny = y + dy * k;
        if (!column_has_occupied(nx, ny)) {
            continue;
        }
        let reach = sp.step_cells + 1;
        for (var dz = -reach; dz <= reach; dz++) {
            if (bit_at(nx, ny, z + dz, 0u) && within_step(x, y, z, nx, ny, z + dz)) {
                return false;
            }
        }
        return true;
    }
    return true;
}

@compute @workgroup_size(8, 8, 1)
fn find_boundaries(@builtin(global_invocation_id) gid: vec3<u32>) {
    if (gid.x >= sp.size.x || gid.y >= sp.size.y) {
        return;
    }
    let x = i32(gid.x);
    let y = i32(gid.y);
    let base = column_word(x, y, 0);
    for (var w = 0u; w < sp.size.w; w++) {
        boundary[base + w] = 0u;
    }
    if (!column_active(x, y)) {
        return;
    }
    let dirs = array<vec2<i32>, 4>(vec2<i32>(-1, 0), vec2<i32>(1, 0), vec2<i32>(0, -1), vec2<i32>(0, 1));
    for (var z = 0; z < i32(sp.size.z); z++) {
        if (!bit_at(x, y, z, 1u)) {
            continue;
        }
        var is_boundary = false;
        for (var k = 0; k < 4; k++) {
            let d = dirs[k];
            if (!surface_linked(x, y, z, x + d.x, y + d.y) && edge_in_direction(x, y, z, d.x, d.y)) {
                is_boundary = true;
                break;
            }
        }
        if (is_boundary) {
            boundary[base + u32(z) / 32u] |= 1u << (u32(z) & 31u);
        }
    }
}


const COMPONENTS: u32 = 0u;

fn planes_base() -> u32 {
    return sp.size.x * sp.size.y * sp.size.z;
}

fn is_surface(x: i32, y: i32, z: i32) -> bool {
    return bit_at(x, y, z, 1u);
}

fn uf_find(base: u32, c: u32) -> u32 {
    var x = c;
    for (var guard = 0u; guard < 0xffffffu; guard++) {
        let p = atomicLoad(&labels[base + x]);
        if (p == x) {
            break;
        }
        x = p;
    }
    return x;
}

// Hook the larger root under the smaller with atomicMin, retrying when a
// concurrent hook got there first (Playne and Hawick).
fn uf_union(base: u32, a: u32, b: u32) {
    var x = uf_find(base, a);
    var y = uf_find(base, b);
    for (var guard = 0u; guard < 0xffffffu; guard++) {
        if (x == y) {
            return;
        }
        if (x < y) {
            let t = x;
            x = y;
            y = t;
        }
        let old = atomicMin(&labels[base + x], y);
        if (old == x) {
            return;
        }
        x = uf_find(base, old);
        y = uf_find(base, y);
    }
}

// The next surface cell up a column, -1 past the top. The planes passes run
// one thread per column over its surface cells; only those carry labels.
fn next_surface(x: i32, y: i32, word: ptr<function, u32>, bits: ptr<function, u32>) -> i32 {
    while (*bits == 0u) {
        if (*word >= sp.size.w) {
            return -1;
        }
        *bits = surface[column_word(x, y, 0) + *word];
        *word += 1u;
    }
    let z = (*word - 1u) * 32u + firstTrailingBit(*bits);
    *bits &= *bits - 1u;
    return i32(z);
}

@compute @workgroup_size(8, 8, 1)
fn uf_init(@builtin(global_invocation_id) gid: vec3<u32>) {
    if (gid.x >= sp.size.x || gid.y >= sp.size.y) {
        return;
    }
    let x = i32(gid.x);
    let y = i32(gid.y);
    var word = 0u;
    var bits = 0u;
    for (var z = next_surface(x, y, &word, &bits); z >= 0; z = next_surface(x, y, &word, &bits)) {
        let c = cell_index(x, y, z);
        atomicStore(&labels[c], c);
        atomicStore(&labels[planes_base() + c], c);
    }
}

// Which sheet of its component a cell is on, counted from below: cells of
// the component in this column within a step of the one beneath share a sheet.
fn component_rank(x: i32, y: i32, z: i32, component: u32) -> u32 {
    var rank = 0u;
    var previous = -1.0e38;
    for (var below = 0; below <= z; below++) {
        if (!is_surface(x, y, below) || atomicLoad(&labels[cell_index(x, y, below)]) != component) {
            continue;
        }
        let h = heights[cell_index(x, y, below)];
        if (previous > -1.0e37 && h - previous > sp.step_m) {
            rank += 1u;
        }
        previous = h;
    }
    return rank;
}

// Union each surface cell with its linked +x and +y neighbors; the plane pass
// also requires equal rank.
fn link_neighbors(x: i32, y: i32, z: i32, planes: bool) {
    let c = cell_index(x, y, z);
    var rank = 0u;
    var component = 0u;
    if (planes) {
        component = atomicLoad(&labels[c]);
        rank = component_rank(x, y, z, component);
    }
    let base = select(COMPONENTS, planes_base(), planes);
    let dirs = array<vec2<i32>, 2>(vec2<i32>(1, 0), vec2<i32>(0, 1));
    for (var k = 0; k < 2; k++) {
        let nx = x + dirs[k].x;
        let ny = y + dirs[k].y;
        let reach = sp.step_cells + 1;
        for (var dz = -reach; dz <= reach; dz++) {
            let nz = z + dz;
            if (!is_surface(nx, ny, nz) || !within_step(x, y, z, nx, ny, nz)) {
                continue;
            }
            if (planes && component_rank(nx, ny, nz, component) != rank) {
                continue;
            }
            uf_union(base, c, cell_index(nx, ny, nz));
        }
    }
}

@compute @workgroup_size(8, 8, 1)
fn link_components(@builtin(global_invocation_id) gid: vec3<u32>) {
    if (gid.x >= sp.size.x || gid.y >= sp.size.y) {
        return;
    }
    let x = i32(gid.x);
    let y = i32(gid.y);
    var word = 0u;
    var bits = 0u;
    for (var z = next_surface(x, y, &word, &bits); z >= 0; z = next_surface(x, y, &word, &bits)) {
        link_neighbors(x, y, z, false);
    }
}

@compute @workgroup_size(8, 8, 1)
fn link_planes(@builtin(global_invocation_id) gid: vec3<u32>) {
    if (gid.x >= sp.size.x || gid.y >= sp.size.y) {
        return;
    }
    let x = i32(gid.x);
    let y = i32(gid.y);
    var word = 0u;
    var bits = 0u;
    for (var z = next_surface(x, y, &word, &bits); z >= 0; z = next_surface(x, y, &word, &bits)) {
        link_neighbors(x, y, z, true);
    }
}

fn flatten(gid: vec3<u32>, base: u32) {
    if (gid.x >= sp.size.x || gid.y >= sp.size.y) {
        return;
    }
    let x = i32(gid.x);
    let y = i32(gid.y);
    var word = 0u;
    var bits = 0u;
    for (var z = next_surface(x, y, &word, &bits); z >= 0; z = next_surface(x, y, &word, &bits)) {
        let c = cell_index(x, y, z);
        atomicStore(&labels[base + c], uf_find(base, c));
    }
}

@compute @workgroup_size(8, 8, 1)
fn flatten_components(@builtin(global_invocation_id) gid: vec3<u32>) {
    flatten(gid, COMPONENTS);
}

@compute @workgroup_size(8, 8, 1)
fn flatten_planes(@builtin(global_invocation_id) gid: vec3<u32>) {
    flatten(gid, planes_base());
}

struct PlaneInfo {
    planes: u32,
    cells: u32,
    capacity: u32,
    _pad: u32,
}

@group(3) @binding(0) var<uniform> plane_info: PlaneInfo;
@group(3) @binding(1) var<storage, read_write> plane_count: array<atomic<u32>>;
// Per plane: min x, min y, max x, max y, in window columns.
@group(3) @binding(2) var<storage, read_write> plane_bounds: array<atomic<i32>>;
// Per plane: first plane cell, min x, min y, width.
@group(3) @binding(3) var<storage, read_write> plane_table: array<vec4<u32>>;
@group(3) @binding(4) var<storage, read_write> plane_cells_a: array<u32>;
@group(3) @binding(5) var<storage, read_write> plane_cells_b: array<u32>;
@group(3) @binding(6) var<storage, read_write> plane_owner: array<u32>;
// Per plane cell: the z of the plane's cell in that column, -1 where it has none.
@group(3) @binding(7) var<storage, read_write> plane_z: array<i32>;
// export_cells rows of EXPORT_WORDS: x, y, z, height bits, boundary distance bits.
@group(3) @binding(8) var<storage, read_write> export_rows: array<u32>;
@group(3) @binding(9) var<storage, read_write> export_count: array<atomic<u32>>;

fn plane_of_cell(c: u32) -> u32 {
    return seeds_a[atomicLoad(&labels[planes_base() + c])];
}

// Number each plane root; the ids index plane_bounds and plane_table.
@compute @workgroup_size(8, 8, 1)
fn number_planes(@builtin(global_invocation_id) gid: vec3<u32>) {
    if (gid.x >= sp.size.x || gid.y >= sp.size.y) {
        return;
    }
    let x = i32(gid.x);
    let y = i32(gid.y);
    var word = 0u;
    var bits = 0u;
    for (var z = next_surface(x, y, &word, &bits); z >= 0; z = next_surface(x, y, &word, &bits)) {
        let c = cell_index(x, y, z);
        if (atomicLoad(&labels[planes_base() + c]) != c) {
            continue;
        }
        let id = atomicAdd(&plane_count[0], 1u);
        if (id >= plane_info.capacity) {
            seeds_a[c] = NO_SEED;
            continue;
        }
        seeds_a[c] = id;
        atomicStore(&plane_bounds[id * 4u], x);
        atomicStore(&plane_bounds[id * 4u + 1u], y);
        atomicStore(&plane_bounds[id * 4u + 2u], x);
        atomicStore(&plane_bounds[id * 4u + 3u], y);
    }
}

@compute @workgroup_size(8, 8, 1)
fn bound_planes(@builtin(global_invocation_id) gid: vec3<u32>) {
    if (gid.x >= sp.size.x || gid.y >= sp.size.y) {
        return;
    }
    let x = i32(gid.x);
    let y = i32(gid.y);
    var word = 0u;
    var bits = 0u;
    for (var z = next_surface(x, y, &word, &bits); z >= 0; z = next_surface(x, y, &word, &bits)) {
        let id = plane_of_cell(cell_index(x, y, z));
        if (id == NO_SEED) {
            continue;
        }
        atomicMin(&plane_bounds[id * 4u], x);
        atomicMin(&plane_bounds[id * 4u + 1u], y);
        atomicMax(&plane_bounds[id * 4u + 2u], x);
        atomicMax(&plane_bounds[id * 4u + 3u], y);
    }
}

// The plane whose cell range holds plane cell t; the table is sorted by first cell.
fn owner_of(t: u32) -> u32 {
    var lo = 0u;
    var hi = plane_info.planes;
    while (hi - lo > 1u) {
        let mid = (lo + hi) / 2u;
        if (plane_table[mid].x <= t) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    return lo;
}

fn plane_height(id: u32) -> u32 {
    let next = select(plane_info.cells, plane_table[id + 1u].x, id + 1u < plane_info.planes);
    return (next - plane_table[id].x) / plane_table[id].w;
}

@compute @workgroup_size(64)
fn init_plane_cells(
    @builtin(workgroup_id) wg: vec3<u32>,
    @builtin(num_workgroups) nwg: vec3<u32>,
    @builtin(local_invocation_index) li: u32,
) {
    let t = thread_index(wg, nwg, li);
    if (t >= plane_info.cells) {
        return;
    }
    plane_owner[t] = owner_of(t);
    plane_cells_a[t] = NO_SEED;
    plane_z[t] = -1;
}

fn plane_cell(id: u32, x: i32, y: i32) -> u32 {
    let e = plane_table[id];
    return e.x + (u32(y) - e.z) * e.w + (u32(x) - e.y);
}

@compute @workgroup_size(8, 8, 1)
fn scatter_boundaries(@builtin(global_invocation_id) gid: vec3<u32>) {
    if (gid.x >= sp.size.x || gid.y >= sp.size.y) {
        return;
    }
    let x = i32(gid.x);
    let y = i32(gid.y);
    var word = 0u;
    var bits = 0u;
    for (var z = next_surface(x, y, &word, &bits); z >= 0; z = next_surface(x, y, &word, &bits)) {
        let id = plane_of_cell(cell_index(x, y, z));
        if (id != NO_SEED && plane_z[plane_cell(id, x, y)] < 0) {
            plane_z[plane_cell(id, x, y)] = z;
        }
        if (!bit_at(x, y, z, 3u)) {
            continue;
        }
        if (id != NO_SEED && !plane_open_in_column(x, y, id)) {
            plane_cells_a[plane_cell(id, x, y)] = cell_index(x, y, z);
        }
    }
}

// The plane has a non-boundary cell in this column: on a smeared floor its sheet
// stacks, and a boundary on one layer must not seed the layer walked above it.
fn plane_open_in_column(x: i32, y: i32, id: u32) -> bool {
    var word = 0u;
    var bits = 0u;
    for (var z = next_surface(x, y, &word, &bits); z >= 0; z = next_surface(x, y, &word, &bits)) {
        if (!bit_at(x, y, z, 3u) && plane_of_cell(cell_index(x, y, z)) == id) {
            return true;
        }
    }
    return false;
}

// The flood weighs the rise where the plane has a cell to rise from.
fn flood_dist_sq(x: i32, y: i32, z: i32, seed: u32) -> f32 {
    if (seed == NO_SEED) {
        return NO_DISTANCE;
    }
    if (z < 0) {
        return f32(seed_dist_sq(x, y, seed)) * sp.voxel_size * sp.voxel_size;
    }
    return seed_dist_sq_3d(x, y, z, seed);
}

fn plane_read(t: u32) -> u32 {
    if (flood.a_to_b != 0u) {
        return plane_cells_a[t];
    }
    return plane_cells_b[t];
}

@compute @workgroup_size(64)
fn plane_flood_pass(
    @builtin(workgroup_id) wg: vec3<u32>,
    @builtin(num_workgroups) nwg: vec3<u32>,
    @builtin(local_invocation_index) li: u32,
) {
    let t = thread_index(wg, nwg, li);
    if (t >= plane_info.cells) {
        return;
    }
    let id = plane_owner[t];
    let e = plane_table[id];
    let local = t - e.x;
    let lx = i32(local % e.w);
    let ly = i32(local / e.w);
    let width = i32(e.w);
    let height = i32(plane_height(id));
    let x = lx + i32(e.y);
    let y = ly + i32(e.z);
    var best = plane_read(t);
    let z = plane_z[t];
    var best_d = flood_dist_sq(x, y, z, best);
    for (var oy = -1; oy <= 1; oy++) {
        for (var ox = -1; ox <= 1; ox++) {
            let qx = lx + ox * flood.step;
            let qy = ly + oy * flood.step;
            if ((ox == 0 && oy == 0) || qx < 0 || qy < 0 || qx >= width || qy >= height) {
                continue;
            }
            let seed = plane_read(e.x + u32(qy * width + qx));
            let d = flood_dist_sq(x, y, z, seed);
            if (d < best_d) {
                best = seed;
                best_d = d;
            }
        }
    }
    if (flood.a_to_b != 0u) {
        plane_cells_b[t] = best;
    } else {
        plane_cells_a[t] = best;
    }
}

// The flood weighs the rise from one z per column; the seeds around the cell
// are weighed again from the cell's own z.
fn nearest_seed_3d(id: u32, x: i32, y: i32, z: i32) -> f32 {
    let e = plane_table[id];
    let x0 = i32(e.y);
    let y0 = i32(e.z);
    let x1 = x0 + i32(e.w);
    let y1 = y0 + i32(plane_height(id));
    var best = NO_DISTANCE;
    for (var oy = -1; oy <= 1; oy++) {
        for (var ox = -1; ox <= 1; ox++) {
            let qx = x + ox;
            let qy = y + oy;
            if (qx < x0 || qy < y0 || qx >= x1 || qy >= y1) {
                continue;
            }
            best = min(best, seed_dist_sq_3d(x, y, z, plane_read(plane_cell(id, qx, qy))));
        }
    }
    // A boundary of the plane one link away, which an open column keeps out of the flood.
    let dirs = array<vec2<i32>, 4>(vec2<i32>(-1, 0), vec2<i32>(1, 0), vec2<i32>(0, -1), vec2<i32>(0, 1));
    for (var k = 0; k < 4; k++) {
        let nx = x + dirs[k].x;
        let ny = y + dirs[k].y;
        if (!in_window_xy(nx, ny)) {
            continue;
        }
        var word = 0u;
        var bits = 0u;
        for (var nz = next_surface(nx, ny, &word, &bits); nz >= 0; nz = next_surface(nx, ny, &word, &bits)) {
            if (bit_at(nx, ny, nz, 3u) && plane_of_cell(cell_index(nx, ny, nz)) == id) {
                best = min(best, seed_dist_sq_3d(x, y, z, cell_index(nx, ny, nz)));
            }
        }
    }
    return best;
}

// Distance in meters from each surface cell to the nearest boundary cell of its
// plane, rise included, 0 on boundary cells, -1 on a plane with no boundary. Cells
// off the surface are left as they were.
@compute @workgroup_size(8, 8, 1)
fn measure_boundaries(@builtin(global_invocation_id) gid: vec3<u32>) {
    if (gid.x >= sp.size.x || gid.y >= sp.size.y) {
        return;
    }
    let x = i32(gid.x);
    let y = i32(gid.y);
    var word = 0u;
    var bits = 0u;
    for (var z = next_surface(x, y, &word, &bits); z >= 0; z = next_surface(x, y, &word, &bits)) {
        let c = cell_index(x, y, z);
        var d = -1.0;
        if (bit_at(x, y, z, 3u)) {
            d = 0.0;
        } else {
            let id = plane_of_cell(c);
            if (id != NO_SEED) {
                let best = nearest_seed_3d(id, x, y, z);
                // Any non-boundary cell is at least one cell from a boundary.
                if (best < NO_DISTANCE) {
                    d = max(sqrt(best), sp.voxel_size);
                }
            }
        }
        boundary_distance[c] = d;
    }
}

// The plane id of every surface cell, for inspecting the split.
@compute @workgroup_size(8, 8, 1)
fn export_planes(@builtin(global_invocation_id) gid: vec3<u32>) {
    if (gid.x >= sp.size.x || gid.y >= sp.size.y || gid.z >= sp.size.z) {
        return;
    }
    let x = i32(gid.x);
    let y = i32(gid.y);
    let z = i32(gid.z);
    let c = cell_index(x, y, z);
    var id = NO_SEED;
    if (is_surface(x, y, z)) {
        id = plane_of_cell(c);
    }
    seeds_b[c] = id;
}

const EXPORT_WORDS: u32 = 5u;

fn export_row(x: i32, y: i32, z: u32, height: f32, boundary_dist: f32) {
    let i = atomicAdd(&export_count[0], 1u);
    if (i >= arrayLength(&export_rows) / EXPORT_WORDS) {
        return;
    }
    let base = i * EXPORT_WORDS;
    export_rows[base] = bitcast<u32>(x + sp.window_min.x);
    export_rows[base + 1u] = bitcast<u32>(y + sp.window_min.y);
    export_rows[base + 2u] = z;
    export_rows[base + 3u] = bitcast<u32>(height);
    export_rows[base + 4u] = bitcast<u32>(boundary_dist);
}

// Every surface cell of the write box and z range with its height and boundary
// distance, packed for readback.
@compute @workgroup_size(8, 8, 1)
fn export_cells(@builtin(global_invocation_id) gid: vec3<u32>) {
    let x = sp.write_box.x + i32(gid.x);
    let y = sp.write_box.y + i32(gid.y);
    if (x > sp.write_box.z || y > sp.write_box.w || !column_writes(x, y)) {
        return;
    }
    let base = column_word(x, y, 0);
    for (var w = 0u; w < sp.size.w; w++) {
        var bits = surface[base + w];
        while (bits != 0u) {
            let b = countTrailingZeros(bits);
            bits &= bits - 1u;
            let z = w * 32u + b;
            if (z >= sp.size.z || i32(z) > sp.write_z.y) {
                break;
            }
            if (i32(z) < sp.write_z.x) {
                continue;
            }
            let c = cell_index(x, y, i32(z));
            export_row(x, y, bitcast<u32>(i32(z) + sp.window_min.z), heights[c], boundary_distance[c]);
        }
    }
}
