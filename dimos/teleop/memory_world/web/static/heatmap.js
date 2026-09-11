// The heat map an answer paints over the world: one sphere sprite per hot
// voxel, coloured by score, with the cluster being looked at lit up and the
// rest dimmed. Payload layout is MSG_HEATMAP in messages.py.

import * as THREE from 'https://esm.sh/three@0.160.0';
import { SPRITE_FRAGMENT_SHADER, SPRITE_VERTEX_GLSL, spriteUniforms } from '/static_mw/voxel_sprites.js';

// Score 0.3 (the server's cutoff) .. 1.0 -> dark red .. yellow-white.
const HEAT_STOPS = [
    [0.30, [0.55, 0.05, 0.10]],
    [0.55, [0.95, 0.30, 0.05]],
    [0.80, [1.00, 0.70, 0.10]],
    [1.00, [1.00, 0.98, 0.75]],
];
const DIM_OTHER_CLUSTERS = 0.28;
const DIM_UNCLUSTERED = 0.18;
const SPRITE_SCALE = 1.35;   // drawn a little larger than the voxel so blobs read as solid

const VERTEX_SHADER = `
    ${SPRITE_VERTEX_GLSL}
    attribute vec3 color;
    varying vec3 vColor;
    void main() {
        vColor = color;
        vec4 mvPosition = modelViewMatrix * vec4(position, 1.0);
        gl_PointSize = spritePointSize(mvPosition) * ${SPRITE_SCALE.toFixed(2)};
        gl_Position = projectionMatrix * mvPosition;
    }
`;

export function heatColor(score) {
    const s = Math.max(HEAT_STOPS[0][0], Math.min(1, score));
    for (let i = 1; i < HEAT_STOPS.length; i++) {
        const [s1, c1] = HEAT_STOPS[i];
        if (s <= s1) {
            const [s0, c0] = HEAT_STOPS[i - 1];
            const t = (s - s0) / (s1 - s0);
            return [c0[0] + (c1[0] - c0[0]) * t, c0[1] + (c1[1] - c0[1]) * t, c0[2] + (c1[2] - c0[2]) * t];
        }
    }
    return HEAT_STOPS[HEAT_STOPS.length - 1][1];
}

export class HeatmapLayer {
    /** @param {THREE.Object3D} parent the robot-frame group (scene._frameRotate) */
    constructor(parent) {
        this.group = new THREE.Group();
        this.group.name = 'heatmap';
        parent.add(this.group);
        this.points = null;
        this.n = 0;
        this.positions = null;   // Float32Array n*3, robot frame
        this.scores = null;      // Float32Array n, 0..1
        this.clusterOf = null;   // Int16Array n, -1 = none
        this.clusters = 0;
        this.current = -1;
        this.header = null;
        this.visible = true;
    }

    /** Replace the map with the voxels of a MSG_HEATMAP frame. */
    set(header, payload) {
        this.clear();
        const n = header.n | 0;
        this.header = header;
        this.n = n;
        this.clusters = header.clusters | 0;
        if (n === 0) return;
        const bytes = new Uint8Array(payload);
        this.positions = new Float32Array(payload.slice(0, n * 12));
        const scoreBytes = bytes.subarray(n * 12, n * 13);
        this.scores = new Float32Array(n);
        for (let i = 0; i < n; i++) this.scores[i] = scoreBytes[i] / 255;
        // Copy: the int16 block starts at 13n, which is odd for odd n.
        this.clusterOf = new Int16Array(payload.slice(n * 13, n * 15));

        const geometry = new THREE.BufferGeometry();
        geometry.setAttribute('position', new THREE.BufferAttribute(this.positions, 3));
        geometry.setAttribute('color', new THREE.BufferAttribute(new Float32Array(n * 3), 3));
        const material = new THREE.ShaderMaterial({
            uniforms: spriteUniforms(header.voxel_size || 0.1),
            vertexShader: VERTEX_SHADER,
            fragmentShader: SPRITE_FRAGMENT_SHADER,
            transparent: false,
        });
        this.points = new THREE.Points(geometry, material);
        this.points.frustumCulled = false;
        this.points.visible = this.visible;
        this.group.add(this.points);
        this.current = -1;
        this._paint();
    }

    /** Light one cluster (or all, with -1). */
    setCurrent(index) {
        this.current = index;
        this._paint();
    }

    _paint() {
        if (!this.points) return;
        const colors = this.points.geometry.getAttribute('color');
        const arr = colors.array;
        for (let i = 0; i < this.n; i++) {
            const c = heatColor(this.scores[i]);
            const cluster = this.clusterOf[i];
            let gain = 1;
            if (cluster < 0) gain = DIM_UNCLUSTERED;
            else if (this.current >= 0 && cluster !== this.current) gain = DIM_OTHER_CLUSTERS;
            arr[i * 3] = c[0] * gain;
            arr[i * 3 + 1] = c[1] * gain;
            arr[i * 3 + 2] = c[2] * gain;
        }
        colors.needsUpdate = true;
    }

    setVisible(on) {
        this.visible = on;
        if (this.points) this.points.visible = on;
    }

    /** Voxels of one cluster, robot frame, for camera framing. */
    clusterPositions(index) {
        const out = [];
        if (!this.positions) return out;
        for (let i = 0; i < this.n; i++) {
            if (this.clusterOf[i] === index) out.push([this.positions[i * 3], this.positions[i * 3 + 1], this.positions[i * 3 + 2]]);
        }
        return out;
    }

    clear() {
        if (this.points) {
            this.group.remove(this.points);
            this.points.geometry.dispose();
            this.points.material.dispose();
            this.points = null;
        }
        this.n = 0;
        this.positions = this.scores = this.clusterOf = null;
        this.current = -1;
        this.header = null;
    }
}
