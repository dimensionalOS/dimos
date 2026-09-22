// "Dimos Spatial Reasoning": a museum tour through the world itself, explaining how
// CLIP/SigLIP embeddings turn a sentence into places on a map. Each station is a card
// of text plus a live scene: the camera flies to the exhibit, the layers that matter
// are switched on, and where a step needs data (a query, a route) the tour asks the
// server for it, so what is shown is real.

import * as THREE from 'https://esm.sh/three@0.160.0';
import { desktopLookAngles } from '/static_mw/world_frame.js';

const DEFAULT_QUESTION = 'a chair';
// The windowed question the "Asking about part of it" station runs, and the slice it
// runs over: the first half of the recording, as fractions of its length.
const WINDOW_QUESTION = 'a person';
const WINDOW_SPAN = [0.0, 0.5];
const REPLAY_SPEED = 6;
const ROOF_HEIGHT_M = 2.3;   // above the floor, the overview's roof cut
const OUTDOOR_HEIGHT_SPAN_M = 12;  // more height than this = no ceiling to cut
const OVERVIEW_MAX_M = 140;  // a city-scale map is shown from this high at most, around the path's middle

export class Tour {
    constructor({ scene, flight, results, baseUrl, diag, replay, ask, ui }) {
        this.scene = scene;
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
        this._windowAsked = false;
        this._windowAnswer = null;
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
                    Everything in this world was rebuilt from that recording &mdash; ${this._voxelCount()} voxels,
                    ${this._frameCount()} camera frames.
                    <ul><li>This tour is about the part that lets you ask it questions in plain words:
                    <b>CLIP embeddings</b> recorded alongside the frames, and a <b>vector-database
                    lookup</b> over them.</li>
                    <li>Drag to look, W A S D to walk, the wheel to scale; the tour flies you between exhibits.</li></ul>`,
                enter: () => {
                    this._layers({ voxels: true, photos: false });
                    this._overview(0.62);
                },
            },
            {
                title: 'The map: ray tracing',
                body: () => `Every lidar scan is placed by tf and traced from the sensor outward.
                    Voxels a ray passes through are <b>carved free</b>; voxels a ray ends in gain <b>support</b>.
                    A voxel stays only when several scans agree on it, which is what turns lidar fuzz into walls and floors.
                    <ul><li>8 cm voxels, coloured by height.</li>
                    <li>The scrubber replays the map growing; orbit mode circles the robot as it goes.</li></ul>
                    <p>None of this knows what anything <i>is</i>. That is the next four stations.</p>`,
                enter: () => {
                    this._layers({ voxels: true, photos: false });
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
                title: 'Embedding what it saw',
                body: () => `Every few colour frames are run through <b>SigLIP 2</b>, a CLIP-style vision-language
                    model, by the same <b>DimOS</b> call anything else in the platform uses for a picture &mdash;
                    <code>model.embed(image)</code>, the one <code>EmbedImages</code> runs over a recording.
                    It turns a picture into a <b>vector</b> &mdash; a list of about a thousand numbers &mdash; and the
                    trick is that it puts <i>pictures and sentences in the same space</i>. A photo of a chair and the
                    words "a chair" land near each other; a photo of a fire door lands somewhere else.
                    <ul><li><b>One vector per image</b>, and nothing finer. The picture is the unit.</li>
                    <li>Each vector is recorded into the memory store alongside the frame, so this is done once
                    and the question is cheap.</li>
                    <li><b>No pose is stored with them</b>: a frame is a picture and a timestamp. Where the camera
                    was is looked up in tf afterwards, so a better tf moves every old answer for free.</li></ul>`,
                enter: () => {
                    this._layers({ voxels: true, photos: true });
                    const at = this._trailPoint(0.35);
                    const ahead = this._trailPoint(0.4);
                    this._roof(false);
                    if (at && ahead) {
                        // Through the shared helper, not `atan2` on the raw robot-frame
                        // delta: `flight.lookAt` reads `yaw` in WORLD space, so an
                        // unspun one is out by exactly the world's turn angle.
                        const { yaw } = desktopLookAngles(this.scene._worldGroup, [
                            ahead[0] - at[0], ahead[1] - at[1], 0,
                        ]);
                        this.flight.lookAt([at[0], at[1], at[2] + 0.6], { distance: 3.5, yaw, pitch: -0.15 });
                    } else {
                        this._overview(0.7);
                    }
                },
            },
            {
                title: 'Asking in words: a vector-database lookup',
                body: () => `"${this.question}" goes through the <b>same model's text tower</b> and comes out as a
                    vector in that same space. Answering is then a <b>vector-database lookup</b>: DimOS's own
                    <code>Stream.search(vector, k)</code> hands the query to the store's vector index, which ranks
                    every recorded frame by <b>cosine similarity</b> and returns the closest.
                    <ul><li>${this._answerLine()}</li>
                    <li>The same call any DimOS memory uses. Nothing here scores vectors by hand, and no copy of
                    the index is held in memory &mdash; the database does the search.</li>
                    <li>Nothing was labelled and no detector was trained. The model already knew what a chair looks
                    like, so the recording did not have to be annotated to be searchable.</li></ul>`,
                enter: async () => {
                    const mine = this.index;
                    const run = this._run;
                    this._layers({ voxels: true, photos: false });
                    if (!this.results || !this.results.count || this.results.queryText !== this.question) {
                        const answer = await this.ask(this.question);
                        // WHY it came back empty matters: a refusal (no index yet, an
                        // empty question) is not "nothing matched", and saying so of a
                        // search that never ran is the one thing these cards must not do.
                        // The server's own sentence is the honest one, so it is kept.
                        //
                        // Worked out LOCALLY and only then assigned. Writing it first and
                        // clearing it if the run had moved on let a stale answer wipe the
                        // refusal the CURRENT run had already put on screen.
                        const refusal = answer
                            ? (answer.success === false ? answer.answer : null)
                            : 'The question could not be asked.';
                        // `_asked` says the question came back, whatever it came back as.
                        // It belongs to THIS run: an answer arriving after an exit and a
                        // restart is not an answer to the question the new run asked.
                        if (this._onRun(run)) {
                            this._askRefused = refusal;
                            this._asked = true;
                        }
                    }
                    if (!this._onStation(mine, run)) return;   // stepped away while the server answered
                    this._overview(0.75);
                },
            },
            {
                title: 'Where it was seen from',
                body: () => `A matching frame says <i>when</i> the thing was seen. <b>tf</b> turns that into
                    <i>where the camera stood</i> &mdash; and that is deliberately all this claims. One vector per
                    image knows the picture matched, not which pixel did, so nothing is projected into the map to
                    guess the object's own coordinates.
                    <ul><li>${this._clusterLine()}</li>
                    <li>The robot lingers, so dozens of frames catch the same thing: matches closer together than a
                    couple of metres are collapsed into one <b>place</b>, keeping the best-scoring frame.</li>
                    <li><b>&larr; &rarr;</b> steps from place to place; the camera flies to each and only its
                    evidence stays lit. The pictures hang where the camera stood when it took them, so you can
                    check the answer yourself rather than taking the score on trust.</li></ul>`,
                enter: () => {
                    this._layers({ voxels: true, photos: true });
                    if (this.results && this.results.count) this.results.go(0);
                    else this._overview(0.75);
                },
            },
            {
                title: 'Walking there',
                body: () => `The answer is a place in the same map the robot drives on, so it can be <b>navigated to</b>.
                    DimOS's <b>MLS planner</b> reads the voxel map as a <b>multi-level surface</b>: every voxel with
                    head-room above it is a standable cell, cells become a graph with terrain costs (steps, wall
                    clearance), and the route is planned in <b>3D</b> on that graph, so stairs, ramps and a mezzanine
                    all count.
                    <ul><li>${this._routeLine()}</li><li>The green tube is what the navigation stack would drive.</li></ul>`,
                enter: async () => {
                    const mine = this.index;
                    const run = this._run;
                    this._layers({ voxels: true, photos: false });
                    if (this.results && this.results.count) {
                        const route = this.results.route || await this.results.navigate();
                        // `navigate()` resolves to null when the planner finds nothing,
                        // and leaves `route` null with it: without this the sentence said
                        // "planning" for ever after the attempt had already failed. As
                        // with the question, it belongs to the run that asked for it.
                        if (this._onRun(run)) this._routed = true;
                        // The first navigate builds the MLS graph over the whole map and takes
                        // seconds. Stepping on with the arrow keys during it is ordinary use,
                        // and without this the resolved body cut the roof and flew the camera
                        // on whatever station the user had reached by then.
                        if (!this._onStation(mine, run)) return;
                        this._refresh();
                        if (route && route.points && route.points.length) {
                            const mid = route.points[Math.floor(route.points.length / 2)];
                            const span = Math.max(route.length_m || 8, 6);
                            this._roof(true);
                            this.flight.lookAt(mid, { distance: span * 0.9, pitch: -0.75 });
                            return;
                        }
                    } else if (this._onRun(run)) {
                        // Nothing was asked of the planner because there was nowhere to
                        // send it, and that is an ANSWER: `_routed` stayed false here and
                        // the card said "Planning a route..." for ever on a question that
                        // matched nothing -- the case the two sentences above were fixed
                        // for, in the one place that was missed.
                        this._routed = true;
                    }
                    this._overview(0.75);
                },
            },
            {
                title: 'Asking about part of it',
                body: () => `The lookup takes a <b>slice of the recording</b> as well as a sentence, so a question
                    can be about <i>when</i> as much as <i>what</i>: "in the first half, were there any people".
                    The frames outside the slice are simply not candidates.
                    <ul><li>${this._windowLine()}</li>
                    <li>This is the same tool an <b>agent</b> drives. The LLM turns "the first half" into the
                    fractions <code>0.0</code> and <code>0.5</code>, calls <code>find_in_memory</code>, and answers
                    from what comes back &mdash; the lookup you just watched, not a second one that resembles it.</li>
                    <li>A ranking always returns its top row, so a best match below the floor is reported as
                    <b>nothing found</b>. That is what lets the answer be "no".</li></ul>`,
                enter: async () => {
                    const mine = this.index;
                    const run = this._run;
                    this._layers({ voxels: true, photos: true });
                    if (!this._windowAsked) {
                        const answer = await this.ask(WINDOW_QUESTION, WINDOW_SPAN);
                        if (this._onRun(run)) {
                            this._windowAnswer = answer;
                            this._windowAsked = true;
                        }
                    }
                    if (!this._onStation(mine, run)) return;   // stepped away while the server answered
                    this._refresh();
                    this._overview(0.75);
                },
            },
            {
                hands_on: true,  // the controls come back: this station asks the user to use them
                title: 'Your turn',
                body: () => `Type a question in the bar at the top, or hold the microphone button and ask.
                    <ul><li><b>&larr; &rarr;</b> places &middot; <b>N</b> route &middot; <b>O</b> orbit the robot &middot; <b>P</b> see through the camera</li>
                    <li>The scrubber replays the recording; the menu (&#9776;) picks any tf frame to orbit and toggles the layers.</li>
                    <li>Exit the tour to roam freely.</li></ul>`,
                enter: () => {
                    this._layers({ voxels: true, photos: false });
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

    _ourAnswer() {
        // `results` is ONE object, shared with the rest of the page:
        // every `query_result` broadcast overwrites them, including another viewer's
        // question and the operator's own search-bar or voice query racing the tour's.
        // Only the answer to the question THIS station asked says anything about this
        // station -- measured, a tour whose own question came back refused reported the
        // previous answer's "hot patches found" as its own result and swallowed the
        // refusal entirely. The station body already compares `queryText` before asking;
        // the lines that report the answer did not.
        const answer = this.results;
        return answer && answer.queryText === this.question ? answer : null;
    }

    _answerLine() {
        const r = this._ourAnswer();
        if (!r || !r.count) {
            if (this._askRefused) return this._askRefused;
            return this._asked ? `Nothing in the recording matched "${this.question}".` : 'Asking now…';
        }
        const best = r.clusters[0];
        return `Just now: ${r.count} place${r.count === 1 ? '' : 's'} for "${r.queryText}",`
            + ` the closest at cosine ${best.peak.toFixed(3)}.`;
    }

    _clusterLine() {
        const r = this._ourAnswer();
        if (!r || !r.count) {
            if (this._askRefused) return this._askRefused;
            return this._asked ? 'That question matched no place in this recording.' : 'No answer yet.';
        }
        const best = r.clusters[0];
        return `${r.count} place${r.count === 1 ? '' : 's'} for "${r.queryText}"; the best scores ${best.peak.toFixed(2)}.`;
    }

    /** What the windowed question came back with, in the station's own words. */
    _windowLine() {
        if (!this._windowAsked) return `Asking "${WINDOW_QUESTION}" over the first half…`;
        const answer = this._windowAnswer;
        if (!answer) return 'The question could not be asked.';
        if (answer.success === false) {
            // The server's own sentence. A refusal names WHY -- below the floor, no index
            // -- and rewriting it here as "nothing found" would report a search that never
            // ran as a search that found nothing.
            return answer.answer;
        }
        const places = (answer.metadata && answer.metadata.places) || [];
        const first = places.length ? places[0].seconds_into_recording : null;
        const when = first === null ? '' : `, the first ${first}s in`;
        return `In the first half: ${places.length} place${places.length === 1 ? '' : 's'}`
            + ` for "${WINDOW_QUESTION}"${when}.`;
    }

    _routeLine() {
        const r = this._ourAnswer();
        if (r && r.route) {
            const via = r.route.planner === 'mls' ? 'MLS 3D planner' : '2D costmap fallback';
            return `Route to place #${(r.route.cluster ?? 0) + 1}: ${r.route.length_m} m, ${r.route.cells} waypoints (${via}).`;
        }
        if (this._routed) {
            return r && r.count
                ? 'No route to the best place: nothing standable joins here to there.'
                : 'Nothing to route to: the question matched no place in this recording.';
        }
        return 'Planning a route to the best place…';
    }

    _layers({ voxels, photos }) {
        const scene = this.scene;
        if (scene._cloudWanted !== voxels) scene.toggleCloud();
        if (scene._imageQuadGroup.visible !== photos) scene.toggleImages();
        for (const [id, on] of [['layerVoxels', voxels], ['layerPhotos', photos]]) {
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
    _onRun(run) {
        // An answer belongs to the RUN that asked for it, not to the station the viewer
        // happens to be standing on when it arrives. Stepping on with the arrow keys
        // while the server thinks is ordinary use, and gating the outcome on the station
        // threw the answer away: the Places card then read "No answer yet." about a
        // question that had been answered.
        return this.active && !this._disposed && (run === undefined || run === this._run);
    }

    _onStation(index, run) {
        // The RUN matters as much as the station: exiting and starting again lands on the
        // same index, so a request still in flight from the previous run passed this test
        // and wrote its outcome over a pending one -- the restarted tour said "Nothing in
        // the recording matched" while its own question was still being asked.
        return (
            this.active
            && !this._disposed
            && this.index === index
            && (run === undefined || run === this._run)
        );
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
        if (r && r.count) anchors.push([6, r.clusters[0].centre]);
        if (r && r.route && r.route.points.length) anchors.push([7, r.route.points[r.route.points.length - 1]]);
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
        // A fresh run has asked nothing and routed nowhere yet, whatever the last one
        // ended up saying, and carries its own number so a request still in flight from
        // the last one cannot answer for it. OUTSIDE the `active` guard: `main.js` binds
        // T and the tour button to `start()` with no guard of their own, and the keydown
        // handler deliberately falls through to them on a hands-on station -- so pressing
        // T on the last station restarts a LIVE tour, and the flags stayed as the old run
        // had left them, asserting a negative answer to a question not yet asked.
        this._asked = false;
        this._routed = false;
        this._askRefused = null;
        // Same rule as the flags above: a restarted tour must re-ask its windowed
        // question rather than report the last run's answer as this run's.
        this._windowAsked = false;
        this._windowAnswer = null;
        this._run = (this._run || 0) + 1;
        // The question the tour explains is whatever is on screen NOW, restart or not:
        // asking something new and then pressing T left the cards explaining the old
        // question while the results bar answered the new one, and station 4 then re-asked
        // the old one and threw away the answer the user had just got.
        // Whatever question is on screen, ANSWERED OR NOT: a search that came back with
        // no places is still the question the viewer asked, and taking only the ones with
        // places left the tour explaining -- and re-asking -- the one before it.
        const showing = this.results;
        if (showing && showing.queryText) this.question = showing.queryText;
        if (!this.active) {
            this.active = true;
            this._saved = {
                photos: this.scene._imageQuadGroup.visible,
                voxels: this.scene._cloudWanted,
            };
            document.body.classList.add('touring');
            if (this.ui && this.ui.panel) this.ui.panel.classList.add('open');
            // A presentation shows the whole map from above: pin full quality (the
            // governor's lower levels cull far voxels) and restore automatic on exit.
            this._saved.quality = this.scene._qualityAuto ? null : this.scene._quality;
            this.scene._hudGroup.visible = false;   // the answer panel: the card carries the words
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
            // Through the toggles so the layer boxes follow.
            if (this.scene._imageQuadGroup.visible !== this._saved.photos) this.scene.toggleImages();
            if (this.scene._cloudWanted !== this._saved.voxels) this.scene.toggleCloud();
            this.scene.setQuality(this._saved.quality ?? null);
            // The box is the truth: it can be unchecked through the menu mid-tour.
            this.scene._hudGroup.visible = true;   // the answer panel lives here
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
        // A NUMBER, whatever the caller handed over. `window.app.tourStart` is the console
        // API and its argument is a station; `tourStart('a chair')` -- a question is the
        // obvious thing to pass it -- made `index` NaN, `stations[NaN]` undefined, and
        // threw out of `start()` half way: the tour left ACTIVE with no station, the body
        // class on, the HUD hidden, quality pinned, and `next()` throwing on NaN + 1.
        // Measured live: `{active: true, index: null}`, which no interaction can produce.
        index = Math.trunc(Number(index)) || 0;
        index = Math.max(0, Math.min(this.stations.length - 1, index));
        // The question the tour explains is whatever is on screen NOW, the same rule
        // `start()` follows and for the same reason. Stepping between stations is a
        // deliberate move by the viewer, and the last station is the one where they can
        // type: ask something there and press back, and the stations act on the new
        // answer -- flying to its places, drawing its route -- while the cards, which
        // ask whether the answer is THEIRS, denied an answer that was on screen. Not in
        // `_refresh()`, which also runs when an answer merely ARRIVES: adopting it there
        // is how a tour came to report someone else's answer as its own.
        const showing = this.results;
        if (showing && showing.queryText) this.question = showing.queryText;
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
        ui.kicker.textContent = `Dimos Spatial Reasoning · ${this.index + 1} of ${this.stations.length}`;
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
