// Hot pyramids: the frusta of the camera patches that scored, drawn as lines
// from each camera through its patch's rectangle to the near and far caps.
// Off by default; the tour and the menu switch them on to show how the heat
// map was made.

import * as THREE from 'https://esm.sh/three@0.160.0';

const COLOR_CURRENT = new THREE.Color(0xffd166);
const COLOR_OTHER = new THREE.Color(0x4a6fa5);
const COLOR_LOOSE = new THREE.Color(0x3a4256);

export class PyramidLayer {
    /** @param {THREE.Object3D} parent the robot-frame group */
    constructor(parent) {
        this.group = new THREE.Group();
        this.group.name = 'pyramids';
        this.group.visible = false;
        parent.add(this.group);
        this.lines = null;
        this.pyramids = [];
        this.current = -1;
    }

    set(pyramids) {
        this.clear();
        this.pyramids = pyramids || [];
        if (!this.pyramids.length) return;
        // 4 apex->near edges, 4 near rect, 4 far rect, 4 near->far = 16 segments = 32 points each.
        const positions = new Float32Array(this.pyramids.length * 32 * 3);
        const colors = new Float32Array(this.pyramids.length * 32 * 3);
        let k = 0;
        const put = (p) => { positions[k++] = p[0]; positions[k++] = p[1]; positions[k++] = p[2]; };
        for (const py of this.pyramids) {
            const { apex, near, far } = py;
            for (let i = 0; i < 4; i++) { put(apex); put(near[i]); }
            for (let i = 0; i < 4; i++) { put(near[i]); put(near[(i + 1) % 4]); }
            for (let i = 0; i < 4; i++) { put(far[i]); put(far[(i + 1) % 4]); }
            for (let i = 0; i < 4; i++) { put(near[i]); put(far[i]); }
        }
        const geometry = new THREE.BufferGeometry();
        geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
        geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
        this.lines = new THREE.LineSegments(
            geometry,
            new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, opacity: 0.85, depthWrite: false }),
        );
        this.lines.frustumCulled = false;
        this.group.add(this.lines);
        this._paint();
    }

    setCurrent(cluster) {
        this.current = cluster;
        this._paint();
    }

    _paint() {
        if (!this.lines) return;
        const colors = this.lines.geometry.getAttribute('color');
        const arr = colors.array;
        this.pyramids.forEach((py, index) => {
            let c = COLOR_OTHER;
            if (py.cluster < 0) c = COLOR_LOOSE;
            else if (this.current < 0 || py.cluster === this.current) c = COLOR_CURRENT;
            const gain = 0.45 + 0.55 * Math.min(1, py.score / 0.3);
            for (let i = 0; i < 32; i++) {
                const o = (index * 32 + i) * 3;
                arr[o] = c.r * gain; arr[o + 1] = c.g * gain; arr[o + 2] = c.b * gain;
            }
        });
        colors.needsUpdate = true;
    }

    setVisible(on) {
        this.group.visible = !!on;
    }

    get visible() {
        return this.group.visible;
    }

    /** The apexes (camera positions) of the pyramids of one cluster, for the tour's camera. */
    apexes(cluster) {
        return this.pyramids.filter((p) => cluster < 0 || p.cluster === cluster).map((p) => p.apex);
    }

    clear() {
        if (this.lines) {
            this.group.remove(this.lines);
            this.lines.geometry.dispose();
            this.lines.material.dispose();
            this.lines = null;
        }
        this.pyramids = [];
    }
}
