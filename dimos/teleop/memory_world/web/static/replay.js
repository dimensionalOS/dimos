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
// Parsed segments kept in memory, as a byte budget: a segment is the whole map at
// its keyframe plus five seconds of diffs, so on a big map a late one is tens of
// megabytes. Phones get the small budget.
const CACHE_BUDGET_DESKTOP = 400e6;
const CACHE_BUDGET_PHONE = 80e6;
const PRELOAD_IDLE_MS = 1200;       // no seek for this long -> keep loading segments outward
const LOADING_LABEL_AFTER_MS = 150; // a fetch shorter than this never shows "loading"
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
                color3: { value: new THREE.Vector3(...colors[3]) },
                freshColor: { value: new THREE.Vector3(...FRESH_COLOR) },
            },
            vertexShader: `${SPRITE_VERTEX_GLSL}
                attribute float fresh;
                uniform float floorZ, spanZ;
                uniform vec3 color0, color1, color2, color3, freshColor;
                varying vec3 vColor;
                void main() {
                    float t = clamp((position.z - floorZ) / spanZ, 0.0, 1.0) * 3.0;   // four stops, equally spaced like the server's
                    vec3 ramp = t < 1.0 ? mix(color0, color1, t) : t < 2.0 ? mix(color1, color2, t - 1.0) : mix(color2, color3, t - 2.0);
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
        this.stats = { seeks: 0, lastSeekMs: 0, maxSeekMs: 0, fetches: 0, bytes: 0, frames: 0, aborted: 0, preloaded: 0 };
        this.cacheBudget = (navigator.maxTouchPoints > 0 && (navigator.deviceMemory || 4) <= 4)
            ? CACHE_BUDGET_PHONE : CACHE_BUDGET_DESKTOP;
        this.cacheBytes = 0;                 // raw bytes of the parsed segments held
        this._preloadTimer = null;
        this._loadingSince = null;
        this._frameShown = null;             // ts of the camera frame on the HUD
        this._frameWanted = null;
        this._frameBusy = false;
        this._frameCache = new Map();        // ts -> {bitmap, meta}
    }

    async load() {
        const response = await fetch(`${this.baseUrl}/replay/index`);
        if (!response.ok) {
            // The server says why: "replay building" is worth waiting for, a failed build is not.
            const detail = await response.json().then((body) => body.detail).catch(() => null);
            throw new Error(detail || `replay index: ${response.status}`);
        }
        const index = await response.json();
        // A rebuild replaces the diff stream, so an index fetched mid-build can
        // arrive with a scan or two in it. That renders a dead 0:00 / 0:00
        // timeline, which reads as broken; wait for the real thing instead.
        if (!index.scans || index.scans.length < 2) {
            throw new Error(`replay index still building (${(index.scans || []).length} scans)`);
        }
        this.index = index;
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
        this._schedulePreload();
        if (!segment) {
            // A drag across the bar asks for many segments in a row; only the one under
            // the thumb matters, so the others' downloads are dropped.
            this._abortPendingExcept(number);
            this._setLoading(true, number);
            this._fetchSegment(number).then(() => {
                // only the latest target matters once the data is here
                if (this.targetScan === scan || this.segmentOf(this.targetScan) === number) this.seekScan(this.targetScan);
            }).catch((e) => {
                if (e && e.name === 'AbortError') return;
                this.diag('replay_segment_failed', { number, error: String(e.message || e) });
                this.playing = false;  // autoplay would retry this fetch ten times a second
                this._setLoading(false);
                if (this.ui) this.ui.timeLabel.textContent = 'segment failed, scrub again';
            });
            return false;
        }
        this._setLoading(false);
        const started = performance.now();
        this._applyScan(segment, scan);
        const ms = performance.now() - started;
        this.stats.seeks++;
        this.stats.lastSeekMs = ms;
        this.stats.maxSeekMs = Math.max(this.stats.maxSeekMs, ms);
        this._evictSegments();   // the one just left is no longer protected
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
        if (this.onScan) this.onScan(scan);
    }

    /** Re-run the fill after a quality change without moving in time. */
    refill() {
        if (this.segment && this.scan >= 0) this.layer.fill(this.segment, this.visible, this.freshSlots);
    }

    // ---- segments --------------------------------------------------------------

    _fetchSegment(number, { preload = false } = {}) {
        const inFlight = this.pending.get(number);
        if (inFlight) return inFlight.promise;
        const controller = new AbortController();
        const promise = fetch(`${this.baseUrl}/replay/segment/${number}`, { signal: controller.signal })
            .then(async (r) => {
                if (!r.ok) {
                    const detail = await r.json().then((body) => body.detail).catch(() => null);
                    throw new Error(detail || `segment ${number}: ${r.status}`);
                }
                return r.arrayBuffer();
            })
            .then((buffer) => {
                const segment = new ReplaySegment(buffer, this.index);
                segment.bytes = buffer.byteLength;
                this.segments.set(number, segment);
                this.cacheBytes += buffer.byteLength;
                this.stats.fetches++;
                this.stats.bytes += buffer.byteLength;
                if (preload) this.stats.preloaded++;
                this._evictSegments();
                this.diag('replay_segment', { number, slots: segment.slotCount, scans: segment.scans.length, bytes: buffer.byteLength, preload });
                return segment;
            })
            .finally(() => this.pending.delete(number));
        this.pending.set(number, { promise, controller });
        return promise;
    }

    /** Drop every download except the segment the viewer is waiting for. */
    _abortPendingExcept(number) {
        for (const [n, entry] of this.pending) {
            if (n !== number) { entry.controller.abort(); this.stats.aborted++; }
        }
    }

    _prefetchAround(number) {
        for (const n of [number + 1, number - 1]) {
            if (n >= 0 && n < this.index.keyframes.length && !this.segments.has(n)) this._fetchSegment(n).catch(() => {});
        }
    }

    /** Segments farthest from the target go first, until the cache fits its byte budget. */
    _evictSegments() {
        const target = this.segmentOf(this.targetScan);
        while (this.cacheBytes > this.cacheBudget && this.segments.size > 1) {
            let farthest = null, distance = -1;
            for (const [n, segment] of this.segments) {
                if (segment === this.segment || n === target) continue;   // on screen, or about to be
                const d = Math.abs(n - this.segmentOf(this.targetScan));
                if (d > distance) { distance = d; farthest = n; }
            }
            if (farthest === null) break;
            this.cacheBytes -= this.segments.get(farthest).bytes || 0;
            this.segments.delete(farthest);
        }
    }

    /** Stop every download and timer and free the frames; the controller is done. */
    dispose() {
        this._loadingSince = null;  // a pending loading-label timer then does nothing
        this._disposed = true;
        if (this._preloadTimer) clearTimeout(this._preloadTimer);
        this._preloadTimer = null;
        this._abortPendingExcept(-1);
        for (const frame of this._frameCache.values()) frame.bitmap.close();
        this._frameCache.clear();
        this.segments.clear();
        this.cacheBytes = 0;
        this.index = null;   // seek/tick/preload all return without an index
    }

    // ---- gradual preloading ----------------------------------------------------

    /** After a quiet moment, keep loading segments outward from the current one,
     *  one at a time, until the byte budget is spent; a scrub to a loaded segment
     *  is then instant. Every seek resets the quiet timer. */
    _schedulePreload() {
        if (this._preloadTimer) clearTimeout(this._preloadTimer);
        this._preloadTimer = setTimeout(() => this._preloadNext(), PRELOAD_IDLE_MS);
    }

    _nextToPreload() {
        if (!this.index) return null;
        const total = this.index.keyframes.length;
        const centre = this.segmentOf(Math.max(0, this.targetScan));
        for (let d = 1; d < total; d++) {
            for (const n of [centre + d, centre - d]) {
                if (n >= 0 && n < total && !this.segments.has(n) && !this.pending.has(n)) return n;
            }
        }
        return null;
    }

    _preloadNext() {
        this._preloadTimer = null;
        if (!this.index || this.cacheBytes >= this.cacheBudget * 0.9) return;
        if (this.pending.size) { this._schedulePreload(); return; }   // a seek's own download first
        const n = this._nextToPreload();
        if (n === null) return;
        // The budget check uses the largest segment seen so far as the estimate of the next.
        let largest = 0;
        for (const seg of this.segments.values()) largest = Math.max(largest, seg.bytes || 0);
        if (this.cacheBytes + largest > this.cacheBudget) return;
        this._fetchSegment(n, { preload: true })
            .then(() => this._schedulePreload())
            .catch(() => {});
    }

    // ---- loading state -----------------------------------------------------------

    /** The scrubber shows "loading" while the segment under the thumb downloads. */
    _setLoading(on, number = null) {
        if (!this.ui) return;
        if (on) {
            if (this._loadingSince === null) this._loadingSince = performance.now();
            if (performance.now() - this._loadingSince < LOADING_LABEL_AFTER_MS) {
                // Re-read the target when the timer fires: a drag may have moved on.
                setTimeout(() => { if (this._loadingSince !== null) this._setLoading(true, this.segmentOf(this.targetScan)); }, LOADING_LABEL_AFTER_MS);
                return;
            }
            this.ui.bar.classList.add('loading');
            this.ui.timeLabel.textContent = `loading ${number !== null ? `${number + 1}/${this.index.keyframes.length}` : ''}…`;
        } else {
            this._loadingSince = null;
            this.ui.bar.classList.remove('loading');
            this._updateTimeline();
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
        this._frameWanted = nearest;   // also when it is already shown: a download in flight must not replace it
        if (nearest !== this._frameShown) this._pumpFrame();
    }

    async _pumpFrame() {
        if (this._frameBusy) return;
        while (this._frameWanted !== null && this._frameWanted !== this._frameShown) {
            const ts = this._frameWanted;
            this._frameBusy = true;
            try {
                let frame = this._frameCache.get(ts);
                if (frame) { this._frameCache.delete(ts); this._frameCache.set(ts, frame); }   // LRU: bump on a hit
                else {
                    const response = await fetch(`${this.baseUrl}/replay/frame?t=${ts}`);
                    if (!response.ok) throw new Error(`frame: ${response.status}`);
                    const meta = JSON.parse(response.headers.get('X-Camera-Pose') || '{}');
                    const bitmap = await createImageBitmap(await response.blob(), { imageOrientation: 'flipY' });
                    if (this._disposed) { bitmap.close(); return; }   // disconnected mid-download
                    frame = { bitmap, meta };
                    this._frameCache.set(ts, frame);
                    for (const [old, cached] of this._frameCache) {   // oldest first; never the one on screen
                        if (this._frameCache.size <= 60) break;
                        if (old === this._frameShown) continue;
                        cached.bitmap.close();
                        this._frameCache.delete(old);
                    }
                    this.stats.frames++;
                }
                if (ts !== this._frameWanted) continue;   // the scrubber moved on while this one loaded
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
        this._playTime += ((now - this._playClock) / 1000) * (this.speed || 1);  // speed: the tour plays faster
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
        // handler properties, not addEventListener: each reconnect makes a new controller
        ui.scrub.oninput = () => { this.playing = false; this.seekScan(Number(ui.scrub.value)); };
        ui.playBtn.onclick = () => this.play();
        ui.exitBtn.onclick = () => this.exit();
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
        if (!ui.bar.classList.contains('loading')) ui.timeLabel.textContent = `${clock(this.index.scans[scan])} / ${clock(this.t1)}`;
        ui.playBtn.textContent = this.playing ? '❚❚' : '▶';
    }

    /** For automated checks. */
    state() {
        return {
            cacheBytes: this.cacheBytes, cacheBudget: this.cacheBudget, cachedSegments: this.segments.size,
            loading: !!(this.ui && this.ui.bar.classList.contains('loading')),
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
