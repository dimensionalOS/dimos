// Stepping through an answer's places: next and previous fly the camera to each
// point or box the answer put on the map, from the hung frame that shows it when
// one is near, else to a spot in front of it.

const SAME_PLACE_M = 0.5;          // a box this close to a point is the same place
const FRAME_REACH_M = 8.0;         // a hung frame this far from a place can show it
const FRAME_FACING_COS = 0.5;      // and must face it within about 60 degrees
const MIN_VIEW_DISTANCE_M = 2.5;
const MAX_VIEW_DISTANCE_M = 9.0;

export class ResultsNav {
    constructor({ scene, flight, diag, ui }) {
        this.scene = scene;
        this.flight = flight;
        this.diag = diag || (() => {});
        this.ui = ui;
        this.stops = [];
        this.queryId = null;
        this.current = -1;
        this._bindUi();
    }

    _bindUi() {
        const { prevBtn, nextBtn, closeBtn } = this.ui || {};
        if (prevBtn) prevBtn.onclick = () => this.prev();
        if (nextBtn) nextBtn.onclick = () => this.next();
        if (closeBtn) closeBtn.onclick = () => this.clear();
    }

    get count() {
        return this.stops.length;
    }

    /** A new or extended answer arrived (the `query_result` message). */
    setResult(msg) {
        const stops = [];
        for (const point of msg.points || []) {
            if (!point.position) continue;
            stops.push({ position: point.position, label: point.label || '', extent: point.extent || null });
        }
        for (const box of msg.boxes || []) {
            if (!box.center) continue;
            const near = stops.some((s) => Math.hypot(
                s.position[0] - box.center[0], s.position[1] - box.center[1], s.position[2] - box.center[2],
            ) < SAME_PLACE_M);
            if (!near) stops.push({ position: box.center, label: box.label || 'box', extent: box.extent || null });
        }
        const sameAnswer = msg.query_id && msg.query_id === this.queryId;
        this.stops = stops;
        this.queryId = msg.query_id || null;
        if (!sameAnswer || this.current >= stops.length) this.current = -1;
        this._render();
        this.diag('results', { stops: stops.length });
    }

    clear() {
        this.stops = [];
        this.current = -1;
        this.queryId = null;
        this._render();
    }

    next() { return this.go(this.current + 1); }
    prev() { return this.go(this.current < 0 ? this.stops.length - 1 : this.current - 1); }

    /** Fly to stop `index`, wrapping at either end. */
    go(index) {
        if (!this.stops.length || !this.flight) return false;
        index = ((index % this.stops.length) + this.stops.length) % this.stops.length;
        const stop = this.stops[index];
        this.current = index;
        const header = this._frameShowing(stop);
        if (header && this.flight.viewFrom(header)) {
            this.scene._queryImageCursor = header.index;
            this.scene._queryImageMeshes.forEach((mesh, i) => { if (mesh) mesh.visible = i === header.index; });
        } else {
            const size = stop.extent ? Math.max(...stop.extent) : 1.0;
            const distance = Math.max(MIN_VIEW_DISTANCE_M, Math.min(MAX_VIEW_DISTANCE_M, size * 1.5 + 1.5));
            this.scene._queryImageCursor = -1;
            this.scene._queryImageMeshes.forEach((mesh) => { if (mesh) mesh.visible = true; });
            this.flight.lookAt(stop.position, { distance });
        }
        this._render();
        this.diag('results_go', { index, position: stop.position, from_frame: Boolean(header) });
        return true;
    }

    /** The hung frame nearest a place that also faces it, or null. */
    _frameShowing(stop) {
        let best = null;
        let bestScore = Infinity;
        for (const header of this.scene._queryImages || []) {
            if (!header || !header.position || !header.forward) continue;
            const dx = stop.position[0] - header.position[0];
            const dy = stop.position[1] - header.position[1];
            const dz = stop.position[2] - header.position[2];
            const distance = Math.hypot(dx, dy, dz);
            if (distance > FRAME_REACH_M || distance < 1e-3) continue;
            const f = header.forward;
            const facing = (dx * f[0] + dy * f[1] + dz * f[2]) / (distance * Math.hypot(f[0], f[1], f[2]));
            if (facing < FRAME_FACING_COS) continue;
            const score = distance * (2 - facing);
            if (score < bestScore) { bestScore = score; best = header; }
        }
        return best;
    }

    _render() {
        const ui = this.ui;
        if (!ui || !ui.bar) return;
        const n = this.stops.length;
        ui.bar.hidden = n === 0;
        if (n === 0) return;
        const k = this.current;
        ui.counter.textContent = k < 0 ? `${n} place${n === 1 ? '' : 's'}` : `${k + 1} / ${n}`;
        ui.label.textContent = k < 0 ? 'press ▶ or → to visit them' : this.stops[k].label;
    }
}
