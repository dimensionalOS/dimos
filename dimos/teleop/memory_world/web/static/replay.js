// Timeline replay: the voxel map as it was at any moment of the recording,
// scrubbed entirely on the client.
//
// The server hands out segments (see replay.py): a table of every voxel a
// keyframe's stretch of the recording can show, plus each scan's diff as
// (slot, op) pairs. The state of the map at scan s is a visibility byte per
// slot: the keyframe's voxels start visible, then the diffs of the scans
// after the keyframe are applied forward — or un-applied backward, because a
// diff with add and remove swapped is its own inverse. Seeking is therefore
// byte flips, never a hash lookup, and the render buffer is refilled with the
// visible slots in one linear pass.

import * as THREE from 'https://esm.sh/three@0.160.0';
import { SPRITE_FRAGMENT_SHADER, SPRITE_VERTEX_GLSL, spriteUniforms } from '/static_mw/voxel_sprites.js';

const OP_ADD = 1;
const OP_REMOVE = 2;
const SEGMENT_CACHE = 6;            // parsed segments kept, ~2 MB each
const FRAME_TOLERANCE_S = 0.25;     // no camera frame closer than this: show none
const FRESH_COLOR = [1.0, 0.55, 0.2];

export class ReplaySegment {
    /** Parse a /replay/segment/{n} response body. */
    constructor(buffer, index) {
        const view = new DataView(buffer);
        const headerLength = view.getUint32(0, true);
        this.header = JSON.parse(new TextDecoder().decode(new Uint8Array(buffer, 4, headerLength)));
        const base = 4 + headerLength;
        this.number = this.header.segment;
        this.keyframeScan = this.header.keyframe.scan;
        this.keyframeCount = this.header.keyframe.n;
        this.slotCount = this.header.slots;
        this.scans = this.header.scans;            // [{index, ts, n}], first one is the keyframe's own
        this.firstScan = this.scans.length ? this.scans[0].index : this.keyframeScan;
        this.lastScan = this.scans.length ? this.scans[this.scans.length - 1].index : this.keyframeScan;

        const table = new Int16Array(buffer, base, this.slotCount * 3);
        const entries = this.scans.reduce((sum, scan) => sum + scan.n, 0);
        const slotsOffset = base + this.header.slots_offset;
        this.slots = new Uint32Array(buffer, slotsOffset, entries);
        this.ops = new Uint8Array(buffer, slotsOffset + entries * 4, entries);
        // visibility an entry sets when applied forward / un-applied backward
        this.forward = new Uint8Array(entries);
        this.backward = new Uint8Array(entries);
        for (let e = 0; e < entries; e++) {
            this.forward[e] = this.ops[e] === OP_ADD ? 1 : 0;
            this.backward[e] = this.ops[e] === OP_REMOVE ? 1 : 0;
        }
        this.scanStart = new Uint32Array(this.scans.length + 1);
        this.scans.forEach((scan, i) => { this.scanStart[i + 1] = this.scanStart[i] + scan.n; });

        // Voxel centres in the map frame, once per segment.
        const [ox, oy, oz] = index.origin;
        const size = index.voxel_size;
        this.offsets = new Float32Array(this.slotCount * 3);
        for (let i = 0; i < this.slotCount; i++) {
            this.offsets[i * 3] = (table[i * 3] + ox + 0.5) * size;
            this.offsets[i * 3 + 1] = (table[i * 3 + 1] + oy + 0.5) * size;
            this.offsets[i * 3 + 2] = (table[i * 3 + 2] + oz + 0.5) * size;
        }
        this.bytes = buffer.byteLength;
    }

    /** Entry range [start, end) of the diff for scan *index*. */
    entryRange(index) {
        const i = index - this.firstScan;
        return [this.scanStart[i], this.scanStart[i + 1]];
    }
}

/** One draw of every visible voxel as sphere sprites, coloured by height in the shader. */
export class ReplayLayer extends THREE.Points {
    constructor(voxelSize, height, colors) {
        const geometry = new THREE.BufferGeometry();
        geometry.setDrawRange(0, 0);
        const material = new THREE.ShaderMaterial({
            uniforms: {
                ...spriteUniforms(voxelSize),
                floorZ: { value: height.floor },
                spanZ: { value: height.span },
                color0: { value: new THREE.Vector3(...colors[0]) },
                color1: { value: new THREE.Vector3(...colors[1]) },
                color2: { value: new THREE.Vector3(...colors[2]) },
                freshColor: { value: new THREE.Vector3(...FRESH_COLOR) },
            },
            vertexShader: `${SPRITE_VERTEX_GLSL}
                attribute float fresh;
                uniform float floorZ, spanZ;
                uniform vec3 color0, color1, color2, freshColor;
                varying vec3 vColor;
                void main() {
                    float t = clamp((position.z - floorZ) / spanZ, 0.0, 1.0);
                    vec3 ramp = t < 0.5 ? mix(color0, color1, t * 2.0) : mix(color1, color2, t * 2.0 - 1.0);
                    vColor = mix(ramp, freshColor, fresh);
                    vec4 mvPosition = modelViewMatrix * vec4(position, 1.0);
                    gl_PointSize = spritePointSize(mvPosition);
                    gl_Position = projectionMatrix * mvPosition;
                }`,
            fragmentShader: SPRITE_FRAGMENT_SHADER,
        });
        super(geometry, material);
        this.frustumCulled = false;
        this._capacity = 0;
        this.fraction = 1.0;               // quality governor: draw this share of the visible voxels
        this.drawn = 0;
        this._ensureCapacity(65536);
    }

    _ensureCapacity(count) {
        if (count <= this._capacity) return;
        this._capacity = Math.max(count, Math.ceil(this._capacity * 1.5));
        const positions = new THREE.BufferAttribute(new Float32Array(this._capacity * 3), 3);
        const fresh = new THREE.BufferAttribute(new Float32Array(this._capacity), 1);
        positions.setUsage(THREE.DynamicDrawUsage);
        fresh.setUsage(THREE.DynamicDrawUsage);
        this.geometry.setAttribute('position', positions);
        this.geometry.setAttribute('fresh', fresh);
    }

    /** Refill the sprite buffers from a segment's table and a visibility byte per slot. */
    fill(segment, visible, freshSlots) {
        this._ensureCapacity(segment.slotCount);
        const out = this.geometry.getAttribute('position').array;
        const fresh = this.geometry.getAttribute('fresh').array;
        const src = segment.offsets;
        const fraction = this.fraction;
        let n = 0;
        for (let slot = 0; slot < segment.slotCount; slot++) {
            if (!visible[slot]) continue;
            // a fixed pseudo-random share, so thinning is even and stable between seeks
            if (fraction < 1 && (Math.imul(slot, 2654435761) >>> 0) / 4294967296 > fraction) continue;
            out[n * 3] = src[slot * 3]; out[n * 3 + 1] = src[slot * 3 + 1]; out[n * 3 + 2] = src[slot * 3 + 2];
            fresh[n] = freshSlots[slot];
            n++;
        }
        this.geometry.getAttribute('position').needsUpdate = true;
        this.geometry.getAttribute('fresh').needsUpdate = true;
        this.geometry.setDrawRange(0, n);
        this.drawn = n;
    }

    clear() {
        this.geometry.setDrawRange(0, 0);
        this.drawn = 0;
    }
}

/** Fetches index, segments and camera frames; owns the seek state and the timeline UI. */
export class ReplayController {
    constructor({ scene, baseUrl, diag, ui }) {
        this.scene = scene;
        this.baseUrl = baseUrl;
        this.diag = diag || (() => {});
        this.ui = ui;                        // {bar, scrub, playBtn, timeLabel, exitBtn}
        this.index = null;
        this.layer = null;
        this.segments = new Map();           // number -> ReplaySegment
        this.pending = new Map();            // number -> Promise
        this.segment = null;                 // the one the state below belongs to
        this.visible = null;                 // Uint8Array(slotCount)
        this.freshSlots = null;              // Uint8Array(slotCount), the last scan's additions
        this._freshList = [];
        this.scan = -1;                      // scan index currently shown
        this.targetScan = -1;
        this.active = false;
        this.playing = false;
        this.stats = { seeks: 0, lastSeekMs: 0, maxSeekMs: 0, fetches: 0, bytes: 0, frames: 0 };
        this._frameShown = null;             // ts of the camera frame on the HUD
        this._frameWanted = null;
        this._frameBusy = false;
        this._frameCache = new Map();        // ts -> {bitmap, meta}
    }

    async load() {
        const response = await fetch(`${this.baseUrl}/replay/index`);
        if (!response.ok) throw new Error(`replay index: ${response.status}`);
        this.index = await response.json();
        this.index.keyframeScans = this.index.keyframes.map((k) => k.scan);
        this.t0 = this.index.scans[0];
        this.t1 = this.index.scans[this.index.scans.length - 1];
        this.layer = new ReplayLayer(this.index.voxel_size, this.index.height, this.index.colors);
        this.scene.attachReplay(this.layer, this.index.hfov_deg);
        this._bindUi();
        this.diag('replay_index_loaded', {
            scans: this.index.scans.length, keyframes: this.index.keyframes.length,
            frames: this.index.frames.length, seconds: Number((this.t1 - this.t0).toFixed(1)),
        });
        return this.index;
    }

    // ---- seeking -------------------------------------------------------------

    scanAt(ts) {
        const scans = this.index.scans;
        let lo = 0, hi = scans.length;
        while (lo < hi) { const mid = (lo + hi) >> 1; if (scans[mid] <= ts) lo = mid + 1; else hi = mid; }
        return Math.max(0, lo - 1);
    }

    segmentOf(scan) {
        const keys = this.index.keyframeScans;
        let lo = 0, hi = keys.length;
        while (lo < hi) { const mid = (lo + hi) >> 1; if (keys[mid] <= scan) lo = mid + 1; else hi = mid; }
        return Math.max(0, lo - 1);
    }

    /** Show the map as of time *ts* (seconds, absolute). Returns true when applied now. */
    seek(ts) {
        return this.seekScan(this.scanAt(ts));
    }

    seekScan(scan) {
        if (!this.index) return false;
        scan = Math.max(0, Math.min(this.index.scans.length - 1, scan | 0));
        this.targetScan = scan;
        this.setActive(true);
        this._updateTimeline();
        this._wantFrame(this.index.scans[scan]);
        const number = this.segmentOf(scan);
        const segment = this.segments.get(number);
        if (!segment) {
            this._fetchSegment(number).then(() => {
                // only the latest target matters once the data is here
                if (this.targetScan === scan || this.segmentOf(this.targetScan) === number) this.seekScan(this.targetScan);
            }).catch((e) => this.diag('replay_segment_failed', { number, error: String(e.message || e) }));
            return false;
        }
        const started = performance.now();
        this._applyScan(segment, scan);
        const ms = performance.now() - started;
        this.stats.seeks++;
        this.stats.lastSeekMs = ms;
        this.stats.maxSeekMs = Math.max(this.stats.maxSeekMs, ms);
        this._prefetchAround(number);
        return true;
    }

    _applyScan(segment, scan) {
        if (this.segment !== segment) {
            this.segment = segment;
            this.visible = new Uint8Array(segment.slotCount);
            this.visible.fill(1, 0, segment.keyframeCount);
            this.freshSlots = new Uint8Array(segment.slotCount);
            this._freshList = [];
            this.scan = segment.keyframeScan;
        }
        const visible = this.visible;
        const { slots, ops, forward, backward } = segment;
        // forward: apply diffs (scan, target]; backward: un-apply (target, scan]
        while (this.scan < scan) {
            this.scan++;
            const [start, end] = segment.entryRange(this.scan);
            for (let e = start; e < end; e++) visible[slots[e]] = forward[e];
        }
        while (this.scan > scan) {
            const [start, end] = segment.entryRange(this.scan);
            for (let e = start; e < end; e++) visible[slots[e]] = backward[e];
            this.scan--;
        }
        // what this scan's lidar added glows, so the sweep can be watched
        for (const slot of this._freshList) this.freshSlots[slot] = 0;
        this._freshList = [];
        if (scan > segment.keyframeScan) {
            const [start, end] = segment.entryRange(scan);
            for (let e = start; e < end; e++) {
                if (ops[e] === OP_ADD) { this.freshSlots[slots[e]] = 1; this._freshList.push(slots[e]); }
            }
        }
        this.layer.fill(segment, visible, this.freshSlots);
    }

    /** Re-run the fill after a quality change without moving in time. */
    refill() {
        if (this.segment && this.scan >= 0) this.layer.fill(this.segment, this.visible, this.freshSlots);
    }

    // ---- segments --------------------------------------------------------------

    _fetchSegment(number) {
        let promise = this.pending.get(number);
        if (promise) return promise;
        promise = fetch(`${this.baseUrl}/replay/segment/${number}`)
            .then((r) => { if (!r.ok) throw new Error(`segment ${number}: ${r.status}`); return r.arrayBuffer(); })
            .then((buffer) => {
                const segment = new ReplaySegment(buffer, this.index);
                this.segments.set(number, segment);
                this.stats.fetches++;
                this.stats.bytes += buffer.byteLength;
                this._evictSegments();
                this.diag('replay_segment', { number, slots: segment.slotCount, scans: segment.scans.length, bytes: buffer.byteLength });
                return segment;
            })
            .finally(() => this.pending.delete(number));
        this.pending.set(number, promise);
        return promise;
    }

    _prefetchAround(number) {
        for (const n of [number + 1, number - 1]) {
            if (n >= 0 && n < this.index.keyframes.length && !this.segments.has(n)) this._fetchSegment(n).catch(() => {});
        }
    }

    _evictSegments() {
        while (this.segments.size > SEGMENT_CACHE) {
            let farthest = null, distance = -1;
            for (const n of this.segments.keys()) {
                const d = Math.abs(n - this.segmentOf(this.targetScan));
                if (d > distance) { distance = d; farthest = n; }
            }
            if (farthest === null || this.segments.get(farthest) === this.segment) break;
            this.segments.delete(farthest);
        }
    }

    // ---- camera frame ----------------------------------------------------------

    _wantFrame(ts) {
        const frames = this.index.frames;
        let lo = 0, hi = frames.length;
        while (lo < hi) { const mid = (lo + hi) >> 1; if (frames[mid] < ts) lo = mid + 1; else hi = mid; }
        const candidates = [frames[lo - 1], frames[lo]].filter((t) => t !== undefined);
        if (!candidates.length) return;
        const nearest = candidates.reduce((a, b) => (Math.abs(b - ts) < Math.abs(a - ts) ? b : a));
        if (Math.abs(nearest - ts) > FRAME_TOLERANCE_S) return;
        if (nearest === this._frameShown) return;
        this._frameWanted = nearest;
        this._pumpFrame();
    }

    async _pumpFrame() {
        if (this._frameBusy) return;
        while (this._frameWanted !== null && this._frameWanted !== this._frameShown) {
            const ts = this._frameWanted;
            this._frameBusy = true;
            try {
                let frame = this._frameCache.get(ts);
                if (!frame) {
                    const response = await fetch(`${this.baseUrl}/replay/frame?t=${ts}`);
                    if (!response.ok) throw new Error(`frame: ${response.status}`);
                    const meta = JSON.parse(response.headers.get('X-Camera-Pose') || '{}');
                    const bitmap = await createImageBitmap(await response.blob(), { imageOrientation: 'flipY' });
                    frame = { bitmap, meta };
                    this._frameCache.set(ts, frame);
                    if (this._frameCache.size > 60) {
                        const oldest = this._frameCache.keys().next().value;
                        this._frameCache.get(oldest).bitmap.close();
                        this._frameCache.delete(oldest);
                    }
                    this.stats.frames++;
                }
                this.scene.setCameraFrame(frame.bitmap, frame.meta);
                this._frameShown = ts;
            } catch (e) {
                this.diag('replay_frame_failed', { error: String(e.message || e) });
                this._frameShown = ts; // don't spin on a missing frame
            } finally {
                this._frameBusy = false;
            }
        }
    }

    // ---- playback and UI ---------------------------------------------------------

    setActive(active) {
        if (this.active === active) return;
        this.active = active;
        this.scene.setReplayActive(active);
        if (this.ui) this.ui.bar.classList.toggle('replaying', active);
        if (!active) { this.playing = false; this._updateTimeline(); }
        this.diag('replay_active', { active });
    }

    /** Leave replay: the full map comes back. */
    exit() {
        this.setActive(false);
    }

    play(on = !this.playing) {
        this.playing = on && Boolean(this.index);
        if (this.playing) {
            if (this.targetScan >= this.index.scans.length - 1) this.seekScan(0);
            this._playTime = this.index.scans[Math.max(0, this.targetScan)];
            this._playClock = performance.now();
        }
        this._updateTimeline();
    }

    /** Called every render frame; advances playback in real time. */
    tick() {
        if (!this.playing) return;
        const now = performance.now();
        this._playTime += (now - this._playClock) / 1000;
        this._playClock = now;
        if (this._playTime >= this.t1) { this._playTime = this.t1; this.playing = false; }
        const scan = this.scanAt(this._playTime);
        if (scan !== this.targetScan) this.seekScan(scan);
        else this._updateTimeline();
    }

    currentTime() {
        return this.targetScan >= 0 ? this.index.scans[this.targetScan] : this.t0;
    }

    _bindUi() {
        const ui = this.ui;
        if (!ui) return;
        ui.bar.hidden = false;
        ui.scrub.min = 0;
        ui.scrub.max = this.index.scans.length - 1;
        ui.scrub.step = 1;
        ui.scrub.value = 0;
        ui.scrub.addEventListener('input', () => { this.playing = false; this.seekScan(Number(ui.scrub.value)); });
        ui.playBtn.addEventListener('click', () => this.play());
        ui.exitBtn.addEventListener('click', () => this.exit());
        this._updateTimeline();
    }

    _updateTimeline() {
        const ui = this.ui;
        if (!ui || !this.index) return;
        const scan = Math.max(0, this.targetScan);
        if (Number(ui.scrub.value) !== scan) ui.scrub.value = scan;
        const clock = (t) => {
            const s = Math.max(0, Math.round(t - this.t0));
            return `${Math.floor(s / 60)}:${String(s % 60).padStart(2, '0')}`;
        };
        ui.timeLabel.textContent = `${clock(this.index.scans[scan])} / ${clock(this.t1)}`;
        ui.playBtn.textContent = this.playing ? '❚❚' : '▶';
    }

    /** For automated checks. */
    state() {
        return {
            active: this.active,
            scan: this.scan,
            target: this.targetScan,
            segment: this.segment ? this.segment.number : null,
            drawn: this.layer ? this.layer.drawn : 0,
            cached: Array.from(this.segments.keys()),
            frame: this._frameShown,
            ...this.stats,
        };
    }
}
