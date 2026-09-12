// "Explain Hyperspace": a museum tour through the world itself. Each station
// is a card of text plus a live scene: the camera flies to the exhibit, the
// layers that matter are switched on, and where a step needs data (a query,
// a route) the tour asks the server for it, so what is shown is real.

import * as THREE from 'https://esm.sh/three@0.160.0';

const DEFAULT_QUESTION = 'a chair';
const REPLAY_SPEED = 6;
const ROOF_HEIGHT_M = 2.3;   // above the floor, the overview's roof cut
const OUTDOOR_HEIGHT_SPAN_M = 12;  // more height than this = no ceiling to cut
const OVERVIEW_MAX_M = 140;  // a city-scale map is shown from this high at most, around the path's middle

export class Tour {
    constructor({ scene, heatmap, pyramids, flight, results, baseUrl, diag, replay, ask, ui }) {
        this.scene = scene;
        this.heatmap = heatmap;
        this.pyramids = pyramids;
        this.flight = flight;
        this.results = results;
        this.baseUrl = baseUrl;
        this.diag = diag || (() => {});
        this.replayOf = replay || (() => null);
        this.ask = ask || (async () => null);
        this.ui = ui;
        this.active = false;
        this.index = -1;
        this.question = DEFAULT_QUESTION;
        this._placards = new THREE.Group();
        this._placards.name = 'placards';
        this._placards.visible = false;
        scene._frameRotate.add(this._placards);
        this._saved = null;
        this._replayWatch = null;
        this.stations = this._stations();
        this._bindUi();
    }

    // ---- the exhibits --------------------------------------------------------

    _stations() {
        return [
            {
                title: 'A recording you can walk through',
                body: () => `A robot drove through this building once and recorded what it saw:
                    lidar scans, camera frames and the <b>tf</b> tree that says where every sensor was.
                    Everything in this world was rebuilt from that recording — ${this._voxelCount()} voxels,
                    ${this._frameCount()} camera frames.
                    <ul><li><b>Hyperspace</b> is the part that lets you ask it questions in plain words.</li>
                    <li>Drag to look, W A S D to walk, the wheel to scale; this tour flies you between exhibits.</li></ul>`,
                enter: () => {
                    this._layers({ voxels: true, heat: false, pyramids: false, photos: false });
                    this._overview(0.62);
                },
            },
            {
                title: 'The map: ray tracing',
                body: () => `Every lidar scan is placed by tf and traced from the sensor outward.
                    Voxels a ray passes through are <b>carved free</b>; voxels a ray ends in gain <b>support</b>.
                    A voxel stays only when several scans agree on it, which is what turns lidar fuzz into walls and floors.
                    <ul><li>8 cm voxels, coloured by height.</li>
                    <li>The scrubber replays the map growing; orbit mode circles the robot as it goes.</li></ul>`,
                enter: () => {
                    this._layers({ voxels: true, heat: false, pyramids: false, photos: false });
                    const replay = this.replayOf();
                    this._roof(false);
                    if (replay && replay.index) {
                        replay.seekScan(0);
                        replay.setActive(true);
                        replay.play(true);
                        replay.speed = REPLAY_SPEED;
                        // Circle the robot from just above it, inside the rooms rather than in the ceiling.
                        this.scene._desktopPitch = -0.2;
                        this.scene.camera.rotation.set(-0.2, this.scene._desktopYaw, 0);
                        this.scene.setOrbit(true);
                        this.scene._orbit.distance = 5;
                        this._replayWatch = replay;
                    } else {
                        this._overview(0.8);
                    }
                },
                leave: () => {
                    const replay = this._replayWatch;
                    this._replayWatch = null;
                    if (replay) {
                        replay.play(false);
                        replay.speed = 1;
                        replay.exit();
                    }
                    this.scene.setOrbit(false);
                },
            },
            {
                title: 'Seeing: keyframes and patches',
                body: () => `Colour frames are embedded at 5 Hz with <b>SigLIP 2</b>, a vision-language model,
                    as a <b>24 × 24 grid of patch vectors</b> that live in the same space as text.
                    A rolling buffer of 11 frames keeps only the sharp, novel ones as <b>keyframes</b>.
                    <ul><li>Each patch stores one vector and one fused depth (RealSense, refined depth2depth-style).</li>
                    <li><b>No pose is stored</b>: a keyframe is just a camera frame and a timestamp. Where it was is looked up in tf later.</li></ul>`,
                enter: () => {
                    this._layers({ voxels: true, heat: false, pyramids: false, photos: true });
                    const at = this._trailPoint(0.35);
                    const ahead = this._trailPoint(0.4);
                    this._roof(false);
                    if (at && ahead) {
                        const yaw = Math.atan2(-(ahead[0] - at[0]), ahead[1] - at[1]);
                        this.flight.lookAt([at[0], at[1], at[2] + 0.6], { distance: 3.5, yaw, pitch: -0.15 });
                    } else {
                        this._overview(0.7);
                    }
                },
            },
            {
                title: 'Asking',
                body: () => `"${this.question}" is embedded by the same model's text tower. Every stored patch is scored by
                    <b>contrast</b>: its similarity to the question minus its similarity to background prompts
                    such as "a wall", "a floor", "an office". Patches above a threshold are <b>hot</b>.
                    <ul><li>${this._answerLine()}</li>
                    <li>The search is a single matrix product over every patch of every keyframe — a few milliseconds.</li></ul>`,
                enter: async () => {
                    const mine = this.index;
                    this._layers({ voxels: true, heat: false, pyramids: true, photos: false });
                    if (!this.results || !this.results.count || this.results.queryText !== this.question) {
                        await this.ask(this.question);
                    }
                    if (!this._onStation(mine)) return;   // stepped away while the server answered
                    this._overview(0.75);
                },
            },
            {
                title: 'Hot pyramids',
                body: () => `A hot patch is placed through tf <b>at query time</b> — so a loop closure moves old answers for free —
                    and becomes its own little <b>view frustum</b>, cut to a thin slice around its measured depth
                    (0.99 to 1.01 of it). No point cloud is needed to say where the patch was looking.
                    <ul><li>Each line pyramid here is one hot patch from one keyframe.</li>
                    <li>Yellow: pyramids behind the current place; blue: the others.</li></ul>`,
                enter: () => {
                    this._layers({ voxels: true, heat: false, pyramids: true, photos: false });
                    this._atCluster(0, 5.5);
                },
            },
            {
                title: 'Voxel counting',
                body: () => `Each pyramid is rasterized into a sparse voxel grid. Per voxel: the <b>max</b> over a frame's patches,
                    then a <b>log-sum-exp</b> across frames, times the square root of the number of <b>distinct viewing
                    directions</b>. Seen from many places: confident. Seen once: weak.
                    <ul><li>Scores are normalized so the 99.9th-percentile voxel is 1.0; the map shows everything above 0.3.</li>
                    <li>Dark red is faint, yellow-white is certain.</li></ul>`,
                enter: () => {
                    this._layers({ voxels: true, heat: true, pyramids: true, photos: false });
                    this._atCluster(0, 5.0);
                },
            },
            {
                title: 'Places',
                body: () => `Hot voxels that touch form <b>clusters</b>, ranked by <b>how many
                    viewpoints saw them</b>, then by score.
                    <b>← →</b> step from place to place; the camera flies to each and only its evidence stays lit.
                    <ul><li>${this._clusterLine()}</li>
                    <li>The pictures hang where the camera stood when it took them, so you can check the answer yourself.</li></ul>`,
                enter: () => {
                    this._layers({ voxels: true, heat: true, pyramids: false, photos: false });
                    if (this.results && this.results.count) this.results.go(0);
                    else this._overview(0.75);
                },
            },
            {
                title: 'Navigation',
                body: () => `DimOS's <b>MLS planner</b> reads the same voxel map as a <b>multi-level surface</b>: every voxel with
                    head-room above it is a standable cell, cells become a graph with terrain costs (steps, wall clearance),
                    and the route is planned in <b>3D</b> on that graph, so stairs, ramps and a mezzanine all count.
                    <ul><li>${this._routeLine()}</li><li>The green tube is what the navigation stack would drive.</li></ul>`,
                enter: async () => {
                    const mine = this.index;
                    this._layers({ voxels: true, heat: true, pyramids: false, photos: false });
                    if (this.results && this.results.count) {
                        const route = this.results.route || await this.results.navigate();
                        // The first navigate builds the MLS graph over the whole map and takes
                        // seconds. Stepping on with the arrow keys during it is ordinary use,
                        // and without this the resolved body cut the roof and flew the camera
                        // on whatever station the user had reached by then.
                        if (!this._onStation(mine)) return;
                        this._refresh();
                        if (route && route.points && route.points.length) {
                            const mid = route.points[Math.floor(route.points.length / 2)];
                            const span = Math.max(route.length_m || 8, 6);
                            this._roof(true);
                            this.flight.lookAt(mid, { distance: span * 0.9, pitch: -0.75 });
                            return;
                        }
                    }
                    this._overview(0.75);
                },
            },
            {
                hands_on: true,  // the controls come back: this station asks the user to use them
                title: 'Your turn',
                body: () => `Type a question in the bar at the top, or hold the microphone button and ask.
                    <ul><li><b>← →</b> places · <b>N</b> route · <b>O</b> orbit the robot · <b>P</b> see through the camera</li>
                    <li>The scrubber replays the recording; the menu (☰) picks any tf frame to orbit and toggles the layers.</li>
                    <li>Exit the tour to roam freely.</li></ul>`,
                enter: () => {
                    this._layers({ voxels: true, heat: true, pyramids: false, photos: false });
                    if (this.results && this.results.count) this._atCluster(this.results.current < 0 ? 0 : this.results.current, 4.0);
                    else this._overview(0.8);
                },
            },
        ];
    }

    // ---- helpers the exhibits use ------------------------------------------------

    /** A point `fraction` of the way along the robot's path, robot frame, or null. */
    _trailPoint(fraction) {
        const trail = this.scene._trailPositions;
        if (!trail || trail.length < 6) return null;
        const n = trail.length / 3;
        const i = Math.min(n - 1, Math.max(0, Math.floor(n * fraction)));
        return [trail[i * 3], trail[i * 3 + 1], trail[i * 3 + 2]];
    }

    _voxelCount() {
        const n = this.scene._cloudData ? this.scene._cloudData.n : 0;
        return n ? n.toLocaleString() : 'the';
    }

    _frameCount() {
        const replay = this.replayOf();
        const frames = replay && replay.index && replay.index.frames ? replay.index.frames.length : 0;
        return frames ? frames.toLocaleString() : 'its';
    }

    _answerLine() {
        const r = this.results;
        if (!r || !r.count) return 'Asking now…';
        const stats = this.heatmap && this.heatmap.header && this.heatmap.header.stats;
        const hot = stats && stats.hot_patches ? `${stats.hot_patches.toLocaleString()} hot patches` : 'hot patches found';
        const secs = this.heatmap && this.heatmap.header ? ` in ${this.heatmap.header.seconds.toFixed(2)} s` : '';
        return `Just now: ${hot}${secs}.`;
    }

    _clusterLine() {
        const r = this.results;
        if (!r || !r.count) return 'No answer yet.';
        const best = r.clusters[0];
        return `${r.count} place${r.count === 1 ? '' : 's'} for "${r.queryText}"; the best scores ${best.peak.toFixed(2)} with ${best.n_views ?? best.n_evidence} views.`;
    }

    _routeLine() {
        const r = this.results;
        if (r && r.route) {
            const via = r.route.planner === 'mls' ? 'MLS 3D planner' : '2D costmap fallback';
            return `Route to place #${(r.route.cluster ?? 0) + 1}: ${r.route.length_m} m, ${r.route.cells} waypoints (${via}).`;
        }
        return 'Planning a route to the best place…';
    }

    _layers({ voxels, heat, pyramids, photos }) {
        const scene = this.scene;
        if (scene._cloudWanted !== voxels) scene.toggleCloud();
        if (this.heatmap) this.heatmap.setVisible(heat);
        if (this.pyramids) this.pyramids.setVisible(pyramids);
        if (scene._imageQuadGroup.visible !== photos) scene.toggleImages();
        for (const [id, on] of [['layerVoxels', voxels], ['layerHeat', heat], ['layerPyramids', pyramids], ['layerPhotos', photos]]) {
            const box = document.getElementById(id);
            if (box) box.checked = on;
        }
    }

    /** Fly high above the map's centre, looking down into it with the roof cut away. */
    _overview(fraction) {
        const bounds = this._bounds();
        if (!bounds) return;
        let [centre, extent] = bounds;
        this._roof(true);
        let distance = Math.max(8, extent * fraction);
        if (distance > OVERVIEW_MAX_M) {
            // Kilometres of streets do not fit one view: hover over the middle of the ride instead.
            const mid = this._trailPoint(0.5);
            if (mid) centre = mid;
            distance = OVERVIEW_MAX_M;
        }
        this.flight.lookAt(centre, { distance, pitch: -1.05 });
    }

    /** Cut the voxels above head height (on/off), so an overview shows rooms, not ceilings.
     *  Outdoors (or on a hilly ride) there is no ceiling and no single floor, so nothing is cut. */
    /** Still on the station that started this, and still touring?
     *
     *  `!this.active` alone is not enough: it catches Exit but not the forward arrow, and
     *  a station body resuming after its own `await` is exactly as able to cut the ceiling
     *  and fly the camera on somebody else's station as it is after the tour closes.
     */
    _onStation(index) {
        return this.active && !this._disposed && this.index === index;
    }

    _roof(cut) {
        // Only while the tour is on. A station resuming after Exit used to cut the ceiling
        // and leave it cut: setRoofCut has three callers and all of them are in here, so
        // with the tour closed nothing could put it back for the rest of the session.
        if (!this.active) return;
        if (!cut || this._outdoors()) { this.scene.setRoofCut(null); return; }
        const replay = this.replayOf();
        const floor = replay && replay.index && replay.index.height ? replay.index.height.floor : null;
        const trail = this._trailPoint(0.5);
        const base = floor !== null ? floor : (trail ? trail[2] - 0.5 : 0);
        this.scene.setRoofCut(base + ROOF_HEIGHT_M);
    }

    /** A map whose heights span more than a building's is outdoors or multi-level. */
    _outdoors() {
        const replay = this.replayOf();
        const span = replay && replay.index && replay.index.height ? replay.index.height.span : 0;
        return span > OUTDOOR_HEIGHT_SPAN_M;
    }

    _bounds() {
        const d = this.scene._cloudData;
        if (!d || !d.n) return null;
        let minX = Infinity, minY = Infinity, minZ = Infinity, maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;
        const step = Math.max(1, Math.floor(d.n / 20000));
        for (let i = 0; i < d.n; i += step) {
            const x = d.positions[i * 3], y = d.positions[i * 3 + 1], z = d.positions[i * 3 + 2];
            if (x < minX) minX = x; if (x > maxX) maxX = x;
            if (y < minY) minY = y; if (y > maxY) maxY = y;
            if (z < minZ) minZ = z; if (z > maxZ) maxZ = z;
        }
        const centre = [(minX + maxX) / 2, (minY + maxY) / 2, minZ + (maxZ - minZ) * 0.2];
        return [centre, Math.hypot(maxX - minX, maxY - minY)];
    }

    _atCluster(index, distance) {
        this._roof(true);   // look into the room from above the walls, not through the ceiling
        const r = this.results;
        if (r && r.count) {
            const cluster = r.clusters[Math.min(index, r.count - 1)];
            r.go(cluster.index, { fly: false });
            this.flight.lookAt(cluster.centre, { distance, pitch: -0.5, yaw: r._yawFromEvidence(cluster.index) });
        } else {
            this._overview(0.75);
        }
    }

    // ---- placards: numbered signs in the world at the exhibits -------------------

    dispose() { this._disposed = true; }  // a station still entering then draws nothing

    _buildPlacards() {
        if (!this.active) return;   // a resumed station must not put the signs back
        this._clearPlacards();
        const bounds = this._bounds();
        const anchors = [];
        if (bounds) anchors.push([1, bounds[0]]);
        const seen = this._trailPoint(0.35);
        if (seen) anchors.push([3, seen]);
        const r = this.results;
        if (r && r.count) anchors.push([7, r.clusters[0].centre]);
        if (r && r.route && r.route.points.length) anchors.push([8, r.route.points[r.route.points.length - 1]]);
        for (const [number, at] of anchors) {
            const sprite = this._placard(`${number}`, this.stations[number - 1].title);
            sprite.position.set(at[0], at[1], at[2] + 1.6);
            this._placards.add(sprite);
        }
        this._placards.visible = true;
    }

    _placard(number, title) {
        const canvas = document.createElement('canvas');
        canvas.width = 512; canvas.height = 128;
        const ctx = canvas.getContext('2d');
        ctx.fillStyle = 'rgba(6, 9, 15, 0.85)';
        ctx.fillRect(0, 0, 512, 128);
        ctx.fillStyle = '#7af0a8';
        ctx.fillRect(0, 0, 10, 128);
        ctx.font = 'bold 64px -apple-system, system-ui, sans-serif';
        ctx.fillStyle = '#7af0a8';
        ctx.fillText(number, 28, 88);
        ctx.font = '30px -apple-system, system-ui, sans-serif';
        ctx.fillStyle = '#d8e6f4';
        ctx.fillText(title, 100, 78);
        const texture = new THREE.CanvasTexture(canvas);
        texture.colorSpace = THREE.SRGBColorSpace;
        const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: texture, depthTest: false, transparent: true }));
        sprite.scale.set(2.4, 0.6, 1);
        sprite.renderOrder = 10;
        return sprite;
    }

    _clearPlacards() {
        while (this._placards.children.length) {
            const child = this._placards.children.pop();
            child.material.map.dispose();
            child.material.dispose();
        }
    }

    // ---- flow --------------------------------------------------------------------

    _bindUi() {
        const { prevBtn, nextBtn, exitBtn } = this.ui || {};
        // handler properties, not addEventListener: each reconnect makes a new tour
        if (prevBtn) prevBtn.onclick = () => this.prev();
        if (nextBtn) nextBtn.onclick = () => this.next();
        if (exitBtn) exitBtn.onclick = () => this.exit();
    }

    start(station = 0) {
        if (!this.active) {
            this.active = true;
            const r = this.results;
            if (r && r.count && r.queryText) this.question = r.queryText;
            this._saved = {
                heat: this.heatmap ? this.heatmap.visible : true,
                pyramids: this.pyramids ? this.pyramids.visible : false,
                photos: this.scene._imageQuadGroup.visible,
                voxels: this.scene._cloudWanted,
            };
            document.body.classList.add('touring');
            if (this.ui && this.ui.panel) this.ui.panel.classList.add('open');
            // A presentation shows the whole map from above: pin full quality (the
            // governor's lower levels cull far voxels) and restore automatic on exit.
            this._saved.quality = this.scene._qualityAuto ? null : this.scene._quality;
            this.scene._hudGroup.visible = false;   // minimap + answer panel: the card carries the words
            this.scene._hudGroupPinnedOff = true;   // an answer must not bring it back mid-tour
            this.scene.setQuality(0);
            this._buildPlacards();
            this.diag('tour_start', { station });
        }
        this.goTo(station);
    }

    exit() {
        if (!this.active) return;
        const current = this.stations[this.index];
        if (current && current.leave) current.leave();
        this.active = false;
        this.index = -1;
        document.body.classList.remove('touring', 'touring-hands-on');
        this.scene._hudGroupPinnedOff = false;
        if (this.ui && this.ui.panel) this.ui.panel.classList.remove('open');
        this._placards.visible = false;
        this._clearPlacards();
        this.scene.setRoofCut(null);
        if (this._saved) {
            if (this.heatmap) this.heatmap.setVisible(this._saved.heat);
            if (this.pyramids) this.pyramids.setVisible(this._saved.pyramids);
            // The photos too, through the toggle so the layer box follows.
            if (this.scene._imageQuadGroup.visible !== this._saved.photos) this.scene.toggleImages();
            if (this.scene._cloudWanted !== this._saved.voxels) this.scene.toggleCloud();
            for (const [id, on] of [['layerHeat', this._saved.heat], ['layerPyramids', this._saved.pyramids]]) {
                const box = document.getElementById(id);
                if (box) box.checked = on;
            }
            this.scene.setQuality(this._saved.quality ?? null);
            // The box is the truth: it can be unchecked through the menu mid-tour.
            this.scene._hudGroup.visible = this.scene._hudPanel.visible;
        }
        this.diag('tour_exit');
    }

    next() {
        if (this.index + 1 >= this.stations.length) { this.exit(); return; }
        this.goTo(this.index + 1);
    }

    prev() {
        this.goTo(Math.max(0, this.index - 1));
    }

    goTo(index) {
        index = Math.max(0, Math.min(this.stations.length - 1, index));
        const previous = this.stations[this.index];
        if (previous && previous.leave && this.index !== index) previous.leave();
        this.index = index;
        const station = this.stations[index];
        document.body.classList.toggle('touring-hands-on', Boolean(station.hands_on));
        this._refresh();
        // A station that awaits (4 asks the server, 8 plans a route, and the first plan
        // builds the MLS graph over the whole map) resumes AFTER an Exit that happened
        // while it waited. `_disposed` is not that check -- only a websocket drop sets it
        // -- so the tour has to ask whether it is still on this station.
        const result = station.enter();
        if (result && typeof result.then === 'function') {
            result.then(() => {
                if (!this._onStation(index)) return;
                this._refresh();
                this._buildPlacards();
            }).catch((e) => this.diag('tour_station_failed', { index, error: String(e.message || e) }));
        }
        this.diag('tour_station', { index, title: station.title });
    }

    _refresh() {
        const ui = this.ui;
        if (!ui || this.index < 0) return;
        const station = this.stations[this.index];
        ui.kicker.textContent = `How it works · ${this.index + 1} of ${this.stations.length}`;
        ui.title.textContent = station.title;
        ui.body.innerHTML = station.body();
        ui.dots.innerHTML = this.stations.map((_, i) => `<i class="${i === this.index ? 'on' : ''}"></i>`).join('');
        ui.prevBtn.disabled = this.index === 0;
        ui.nextBtn.textContent = this.index === this.stations.length - 1 ? 'Finish' : 'Next ▶';
    }

    tick() {
        const replay = this._replayWatch;
        if (replay && replay.index && replay.targetScan >= replay.index.scans.length - 1) {
            replay.play(false);
        }
    }

    state() {
        return {
            active: this.active,
            index: this.index,
            title: this.index >= 0 ? this.stations[this.index].title : null,
            stations: this.stations.length,
            placards: this._placards.children.length,
        };
    }
}
