// Stepping through an answer's clusters: next/prev flies the camera to each
// blob of the heat map, lights it up, shows only its evidence frames, and can
// ask the server for a route to it.

import * as THREE from 'https://esm.sh/three@0.160.0';
import { sightLineFor } from '/static_mw/evidence.js';

const ROUTE_COLOR = 0x64ff8f;
const ROUTE_RADIUS_M = 0.06;
const MIN_VIEW_DISTANCE_M = 2.5;
const MAX_VIEW_DISTANCE_M = 9.0;

export class ResultsNav {
    constructor({ scene, heatmap, pyramids, flight, baseUrl, diag, ui }) {
        this.scene = scene;
        this.heatmap = heatmap;
        this.pyramids = pyramids;
        this.flight = flight;
        this.baseUrl = baseUrl;
        this.diag = diag || (() => {});
        this.ui = ui;
        this.clusters = [];
        this.queryText = '';
        this.current = -1;
        this.route = null;
        this._routeGroup = new THREE.Group();
        this._routeGroup.name = 'route';
        scene._frameRotate.add(this._routeGroup);
        this.onChange = null;
        this._bindUi();
    }

    get count() {
        return this.clusters.length;
    }

    _bindUi() {
        const { prevBtn, nextBtn, orbitBtn, navigateBtn, closeBtn } = this.ui || {};
        // handler properties, not addEventListener: each reconnect makes a new nav
        if (prevBtn) prevBtn.onclick = () => this.prev();
        if (nextBtn) nextBtn.onclick = () => this.next();
        if (orbitBtn) orbitBtn.onclick = () => this.orbitCurrent();
        if (navigateBtn) navigateBtn.onclick = () => this.navigate();
        if (closeBtn) closeBtn.onclick = () => this.clear();
    }

    /** A new answer arrived (the `query_result` message). */
    setResult(msg) {
        // The last answer's corridor belongs to the last answer: a new question that
        // never reaches a picture would otherwise leave a hole in the map for good.
        if (this.scene.setSightLine) this.scene.setSightLine(null);
        this.clusters = msg.clusters || [];
        this.queryId = msg.query_id || null;
        this.queryText = msg.query_text || '';
        this.current = -1;
        this.clearRoute();
        this.scene.clusterFilter = -1;
        if (this.heatmap) this.heatmap.setCurrent(-1);
        if (this.pyramids) this.pyramids.setCurrent(-1);
        this._render();
        this.diag('results', { clusters: this.clusters.length, engine: msg.engine });
    }

    clear() {
        this.clusters = [];
        this.current = -1;
        this.queryId = null;   // there is no answer, so there is no answer to belong to
        if (this.scene.clearAnswer) this.scene.clearAnswer();
        this.clearRoute();
        this.scene.clusterFilter = -1;
        if (this.heatmap) this.heatmap.clear();
        if (this.pyramids) this.pyramids.clear();
        // The status line is the last thing that happened, and closing the answer IS the
        // last thing. Left alone it went on reading "Route to #1: 23.14 m" over an empty
        // world -- the route, the places and the pictures all gone.
        this._status('Answer closed');
        this._render();
    }

    /** Fly to cluster `index`, light it, and show only its pictures. */
    go(index, { fly = true } = {}) {
        if (!this.clusters.length) return false;
        index = ((index % this.clusters.length) + this.clusters.length) % this.clusters.length;
        const cluster = this.clusters[index];
        this.current = index;
        this.scene.clusterFilter = index;
        if (this.heatmap) this.heatmap.setCurrent(index);
        if (this.pyramids) this.pyramids.setCurrent(index);
        // A photo pinned with P belongs to the place it was pinned in: carried to the
        // next one it hides that place's pictures while we fly toward one of them.
        this.scene._queryImageCursor = -1;
        this._showEvidence(index);
        let viewpoint = null;
        // Cleared whether or not we fly: the tour steps through places with fly:false, and
        // a corridor left from the last one is a wedge missing for the rest of the tour.
        if (this.scene.setSightLine) this.scene.setSightLine(null);
        if (fly && this.flight) {
            // Stand where the first picture of this place was taken, looking the way it
            // looked. Flying to the voxels instead put the camera at an arbitrary
            // distance from a blob, facing whichever way, and you had to work out what
            // you were looking at; from the camera's own pose the answer is just there.
            viewpoint = this._firstEvidence(index);
            // A corridor is the way to SEE a photograph. With Photos off there is nothing
            // to see past, so cutting one takes geometry out of the map and shows nothing
            // in its place -- which is what the tour's Places station did.
            if (viewpoint && this.scene._photosPinnedOff) viewpoint = null;
            if (viewpoint) {
                this.flight.lookAt(viewpoint.position, {
                    distance: 0, yaw: viewpoint.yaw, pitch: viewpoint.pitch,
                });
                // And take the map out from between the viewer and that picture, or
                // standing where it was taken just puts you inside the wall it shows.
                if (this.scene.setSightLine) {
                    this.scene.setSightLine(viewpoint.position, viewpoint.at, viewpoint.radius);
                }
            } else {
                const distance = Math.max(MIN_VIEW_DISTANCE_M, Math.min(MAX_VIEW_DISTANCE_M, cluster.radius * 3 + 1.5));
                this.flight.lookAt(cluster.centre, { distance, yaw: this._yawFromEvidence(index) });
            }
        }
        this._render();
        // Which of the two it did: a place whose pictures have not arrived still falls
        // back to the voxels, and from the outside the difference is invisible.
        this.diag('results_go', {
            index,
            centre: cluster.centre,
            from: viewpoint ? viewpoint.position : null,
        });
        if (this.onChange) this.onChange(index, cluster);
        return true;
    }

    next() { return this.go(this.current + 1); }
    prev() { return this.go(this.current < 0 ? this.clusters.length - 1 : this.current - 1); }

    /** Where the first picture of the cluster was taken, and which way it faced.
     *  Null when the pictures have not arrived, or were never sent for this cluster:
     *  only the best few places get them, so the rest still fall back to the voxels. */
    _firstEvidence(index) {
        const header = (this.scene._queryImages || []).find((h) => h && h.cluster === index);
        if (!header || !header.position || !header.forward) return null;
        const f = header.forward;
        const [, at, halfWidth] = sightLineFor(header);
        return {
            position: header.position,
            at,
            radius: halfWidth,
            yaw: Math.atan2(-f[0], f[1]),               // robot -> three, as below
            pitch: Math.asin(Math.max(-1, Math.min(1, f[2]))),   // robot z is up
        };
    }

    /** Look from where the best picture of the cluster was taken (robot -> three yaw). */
    _yawFromEvidence(index) {
        const header = (this.scene._queryImages || []).find((h) => h && h.cluster === index);
        if (!header) return null;
        const f = header.forward;
        return Math.atan2(-f[0], f[1]);   // three: x = x, z = -y; yaw = atan2(-fx, -fz) = atan2(-fx, fy)
    }

    _showEvidence(index) {
        const scene = this.scene;
        // Cluster filter FIRST, then the visibility rule, which owns whether a photo and
        // its marks are on screen and knows the user turned Photos off. The other order
        // put the ring and the line back for the chosen cluster while its photo stayed
        // hidden -- a circle hanging in mid-air, which is the thing the rule exists to
        // prevent. The tour's Places station does exactly that: photos off, then go(0).
        for (const child of scene._highlightGroup.children) {
            if (child.userData && child.userData.cluster !== undefined) {
                child.visible = child.userData.cluster === index;
            }
        }
        if (scene._applyQueryImageVisibility) scene._applyQueryImageVisibility();
    }

    /** Orbit this place, or stop orbiting if this is the place already being orbited. */
    orbitCurrent() {
        if (this.scene.isOrbiting()) {
            this.scene.setOrbit(false);
            this._syncOrbitLabel();
            if (this.onOrbitChange) this.onOrbitChange();
            this.diag('results_orbit', { index: this.current, on: false });
            return;
        }
        if (this.current < 0 && !this.go(0, { fly: false })) return;
        const cluster = this.clusters[this.current];
        this.scene.setOrbitTarget(cluster.centre);
        this.scene._orbit.distance = Math.max(2.0, cluster.radius * 3 + 1.0);
        this.scene.setOrbit(true);
        this._syncOrbitLabel();
        if (this.onOrbitChange) this.onOrbitChange();  // the toolbar's own orbit button
        this.diag('results_orbit', { index: this.current, on: true });
    }

    /** The button says what pressing it will do, whoever last changed orbit. */
    _syncOrbitLabel() {
        const button = (this.ui || {}).orbitBtn;
        if (!button) return;
        button.textContent = this.scene.isOrbiting() ? 'Stop orbit' : 'Orbit';
    }

    dispose() { this._disposed = true; }  // an in-flight /navigate reply then does nothing

    async navigate() {
        if (this.current < 0 && !this.go(0, { fly: false })) return null;
        const cluster = this.current;
        try {
            const response = await fetch(`${this.baseUrl}/navigate`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ cluster, query_id: this.queryId }),
            });
            const body = await response.json();
            if (this._disposed) return null;
            if (!response.ok) {
                this._status(`No route: ${body.detail || response.status}`);
                return null;
            }
            // The server also broadcasts it; drawing here makes the click feel instant.
            this.setRoute(body);
            return body;
        } catch (e) {
            this._status(`Route failed: ${e.message || e}`);
            return null;
        }
    }

    /** Draw a route (the `route` message or the /navigate reply). */
    setRoute(msg) {
        // A route belongs to the answer it was planned for, and planning takes seconds.
        // Close the answer in those seconds and the tube arrives anyway, pointing into an
        // empty world at a place that is no longer lit. The server refuses to plan for a
        // REPLACED answer, but it cannot see a viewer close one, and its own check and its
        // broadcast are not one step -- so the answer on screen decides here.
        if (!this.clusters.length) return;
        if (msg && msg.query_id && msg.query_id !== this.queryId) return;
        this.clearRoute();
        const points = (msg && msg.points) || [];
        if (points.length < 2) return;
        this.route = msg;
        const curve = new THREE.CatmullRomCurve3(points.map((p) => new THREE.Vector3(p[0], p[1], p[2])), false, 'centripetal');
        const tube = new THREE.Mesh(
            new THREE.TubeGeometry(curve, Math.max(24, points.length * 2), ROUTE_RADIUS_M, 8, false),
            new THREE.MeshBasicMaterial({ color: ROUTE_COLOR }),
        );
        this._routeGroup.add(tube);
        const end = points[points.length - 1];
        const flag = new THREE.Mesh(
            new THREE.ConeGeometry(0.18, 0.45, 12),
            new THREE.MeshBasicMaterial({ color: ROUTE_COLOR }),
        );
        flag.position.set(end[0], end[1], end[2] + 0.3);
        flag.rotation.x = Math.PI / 2;
        this._routeGroup.add(flag);
        this._status(`Route to #${(msg.cluster ?? 0) + 1}: ${msg.length_m} m`);
        this._render();
        this.diag('route_drawn', { cluster: msg.cluster, length_m: msg.length_m, points: points.length });
    }

    clearRoute() {
        this.route = null;
        while (this._routeGroup.children.length) {
            const child = this._routeGroup.children.pop();
            child.geometry.dispose();
            child.material.dispose();
        }
    }

    _status(text) {
        if (this.ui && this.ui.status) this.ui.status.textContent = text;
    }

    _render() {
        const ui = this.ui;
        if (!ui || !ui.bar) return;
        const n = this.clusters.length;
        ui.bar.hidden = n === 0;
        if (n === 0) return;
        const k = this.current;
        ui.counter.textContent = k < 0 ? `${n} place${n === 1 ? '' : 's'}` : `${k + 1} / ${n}`;
        const cluster = k < 0 ? this.clusters[0] : this.clusters[k];
        // n_views, not n_evidence: the latter is how many pictures are shown, capped, so
        // it read 8 for every strong place while the answer above it said 66.
        const views = cluster.n_views ?? cluster.n_evidence;
        ui.label.textContent = k < 0
            ? `${this.queryText} — press ▶ to visit the best`
            : `${this.queryText} · ${cluster.peak.toFixed(2)} · ${views} view${views === 1 ? '' : 's'}`;
        if (ui.navigateBtn) ui.navigateBtn.textContent = this.route && this.route.cluster === k ? `Route ${this.route.length_m} m` : 'Navigate';
    }
}
