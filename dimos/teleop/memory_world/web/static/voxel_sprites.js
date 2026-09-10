// Voxels drawn as round point sprites shaded like spheres.
//
// A sphere's silhouette is a disc from anywhere, so one vertex per voxel with
// gl_PointSize set to the sphere's projected diameter, and a fragment shader
// that lights the disc as a ball, looks like a sphere mesh at these sizes for
// a fraction of the vertex work of an instanced box.

import * as THREE from 'https://esm.sh/three@0.160.0';

// Height of the framebuffer being rendered, in device pixels. Every sprite
// material shares this uniform and the scene writes it once per frame.
export const viewportHeight = { value: 1 };

// View-space light, the same one the cube map used.
const LIGHT_DIR = new THREE.Vector3(2, 4, 3).normalize();

const size = new THREE.Vector2();

// Uniforms plus the projected diameter of a sphere of voxelSize at a
// view-space position. The model matrix column length carries the world scale.
export const SPRITE_VERTEX_GLSL = `
    uniform float voxelSize;
    uniform float viewportHeight;
    float spritePointSize(vec4 mvPosition) {
        float scale = length(modelViewMatrix[0].xyz);
        return voxelSize * scale * projectionMatrix[1][1] * 0.5 * viewportHeight / max(-mvPosition.z, 1e-4);
    }
`;

// Discard outside the disc, then light the sphere normal the disc implies.
export const SPRITE_FRAGMENT_SHADER = `
    uniform vec3 lightDir;
    varying vec3 vColor;
    void main() {
        vec2 p = gl_PointCoord * 2.0 - 1.0;
        float r2 = dot(p, p);
        if (r2 > 1.0) discard;
        vec3 n = vec3(p.x, -p.y, sqrt(1.0 - r2));
        float light = 0.45 + 0.75 * max(dot(n, lightDir), 0.0);
        gl_FragColor = vec4(vColor * light, 1.0);
    }
`;

export function spriteUniforms(voxelSize) {
    return {
        voxelSize: { value: voxelSize },
        viewportHeight,
        lightDir: { value: LIGHT_DIR },
    };
}

// The height of whatever is being drawn to: the XR layer while presenting,
// else the canvas drawing buffer.
export function viewportHeightPx(renderer) {
    const session = renderer.xr.isPresenting ? renderer.xr.getSession() : null;
    if (session) {
        const state = session.renderState;
        if (state.baseLayer) return state.baseLayer.framebufferHeight;
        const layer = state.layers && state.layers[0];
        if (layer && layer.textureHeight) return layer.textureHeight;
    }
    return renderer.getDrawingBufferSize(size).y;
}
