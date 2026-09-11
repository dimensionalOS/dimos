// Stepping through an answer's clusters: next/prev flies the camera to each
// blob of the heat map, lights it up, shows only its evidence frames, and can
// ask the server for a route to it.

import * as THREE from 'https://esm.sh/three@0.160.0';

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
        if (prevBtn) prevBtn.addEventListener('click', () => this.prev());
        if (nextBtn) nextBtn.addEventListener('click', () => this.next());
        if (orbitBtn) orbitBtn.addEventListener('click', () => this.orbitCurrent());
        if (navigateBtn) navigateBtn.addEventListener('click', () => this.navigate());
        if (closeBtn) closeBtn.addEventListener('click', () => this.clear());
    }

    /** A new answer arrived (the `query_result` message). */
    setResult(msg) {
        this.clusters = msg.clusters || [];
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
        this.clearRoute();
        this.scene.clusterFilter = -1;
        if (this.heatmap) this.heatmap.clear();
        if (this.pyramids) this.pyramids.clear();
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
        this._showEvidence(index);
        if (fly && this.flight) {
            const distance = Math.max(MIN_VIEW_DISTANCE_M, Math.min(MAX_VIEW_DISTANCE_M, cluster.radius * 3 + 1.5));
            this.flight.lookAt(cluster.centre, { distance, yaw: this._yawFromEvidence(index) });
        }
        this._render();
        this.diag('results_go', { index, centre: cluster.centre });
        if (this.onChange) this.onChange(index, cluster);
        return true;
    }

    next() { return this.go(this.current + 1); }
    prev() { return this.go(this.current < 0 ? this.clusters.length - 1 : this.current - 1); }

    /** Look from where the best picture of the cluster was taken (robot -> three yaw). */
    _yawFromEvidence(index) {
        const header = (this.scene._queryImages || []).find((h) => h && h.cluster === index);
        if (!header) return null;
        const f = header.forward;
        return Math.atan2(-f[0], f[1]);   // three: x = x, z = -y; yaw = atan2(-fx, -fz) = atan2(-fx, fy)
    }

    _showEvidence(index) {
        const scene = this.scene;
        (scene._queryImageMeshes || []).forEach((mesh, i) => {
            if (!mesh) return;
            const header = scene._queryImages[i];
            mesh.visible = !header || header.cluster === undefined || header.cluster === index;
        });
        for (const child of scene._highlightGroup.children) {
            if (child.userData && child.userData.cluster !== undefined) {
                child.visible = child.userData.cluster === index;
            }
        }
    }

    orbitCurrent() {
        if (this.current < 0 && !this.go(0, { fly: false })) return;
        const cluster = this.clusters[this.current];
        this.scene.setOrbitTarget(cluster.centre);
        this.scene._orbit.distance = Math.max(2.0, cluster.radius * 3 + 1.0);
        this.scene.setOrbit(true);
        this.diag('results_orbit', { index: this.current });
    }

    async navigate() {
        if (this.current < 0 && !this.go(0, { fly: false })) return null;
        const cluster = this.current;
        try {
            const response = await fetch(`${this.baseUrl}/navigate`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ cluster }),
            });
            const body = await response.json();
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
        const views = cluster.n_evidence;
        ui.label.textContent = k < 0
            ? `${this.queryText} — press ▶ to visit the best`
            : `${this.queryText} · ${cluster.peak.toFixed(2)} · ${views} view${views === 1 ? '' : 's'}`;
        if (ui.navigateBtn) ui.navigateBtn.textContent = this.route && this.route.cluster === k ? `Route ${this.route.length_m} m` : 'Navigate';
    }
}
