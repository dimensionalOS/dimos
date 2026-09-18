// Voxels drawn as point sprites: shaded spheres, or real cubes.
//
// A sphere's silhouette is a disc from anywhere, so one vertex per voxel with
// gl_PointSize set to the sphere's projected diameter, and a fragment shader
// that lights the disc as a ball, looks like a sphere mesh at these sizes for
// a fraction of the vertex work of an instanced box.
//
// The cube style is a real cube, not a square standing in for one: the fragment
// shader intersects the view ray with the voxel's axis-aligned box and shades the
// face it hits. That gives the hexagonal silhouette a cube actually has from an
// angle, and three faces at three different brightnesses -- which is what reads as
// a solid block. It used to be a flat square with a darker rim, which from any
// angle but face-on looks like a tile, because that is what it was.

import * as THREE from 'https://esm.sh/three@0.160.0';

// Height of the framebuffer being rendered, in device pixels. Every sprite
// material shares this uniform and the scene writes it once per frame.
export const viewportHeight = { value: 1 };

// 0 = shaded spheres, 1 = flat cube faces. Shared by every voxel material; the
// menu flips it.
export const voxelStyle = { value: 0 };

// View-space light, the same one the cube map used.
const LIGHT_DIR = new THREE.Vector3(2, 4, 3).normalize();

const size = new THREE.Vector2();

// Uniforms plus the projected diameter of a sphere of voxelSize at a
// view-space position. The model matrix column length carries the world scale.
// CUBE_SPAN: a cube's longest projection is its space diagonal, so the sprite has to
// be sqrt(3) wider than the sphere's to hold the corners. Anything less clips them and
// the silhouette goes back to being a square.
const CUBE_SPAN = 1.7320508;

export const SPRITE_VERTEX_GLSL = `
    uniform float voxelSize;
    uniform float viewportHeight;
    uniform float voxelStyle;
    varying vec3 vVoxelView;     // the voxel centre, in view space
    varying float vSpriteRadius; // half the sprite's view-space extent at that depth
    varying mat3 vCubeAxes;      // the voxel's world axes, expressed in view space
    float spritePointSize(vec4 mvPosition) {
        float scale = length(modelViewMatrix[0].xyz);
        float span = voxelStyle > 0.5 ? ${CUBE_SPAN.toFixed(7)} : 1.0;
        vVoxelView = mvPosition.xyz;
        vSpriteRadius = voxelSize * scale * span * 0.5;
        vCubeAxes = mat3(
            normalize(modelViewMatrix[0].xyz),
            normalize(modelViewMatrix[1].xyz),
            normalize(modelViewMatrix[2].xyz)
        );
        return voxelSize * scale * span * projectionMatrix[1][1] * 0.5 * viewportHeight
            / max(-mvPosition.z, 1e-4);
    }
`;

// Discard outside the disc, then light the sphere normal the disc implies.
export const SPRITE_FRAGMENT_SHADER = `
    uniform vec3 lightDir;
    uniform float voxelStyle;
    varying vec3 vColor;
    varying vec3 vVoxelView;
    varying float vSpriteRadius;
    varying mat3 vCubeAxes;
    void main() {
        vec2 p = gl_PointCoord * 2.0 - 1.0;
        if (voxelStyle > 0.5) {
            // Where this fragment sits in view space, on the plane through the centre,
            // and the ray the eye casts through it (the eye is the origin in view space).
            vec3 onPlane = vVoxelView + vec3(p.x, -p.y, 0.0) * vSpriteRadius;
            vec3 dir = normalize(onPlane);
            // Into the cube's own frame, where it is an axis-aligned box of half-size h.
            // Transpose, by dotting against each axis: multiplying by vCubeAxes is the
            // forward rotation, not the inverse; the axes are orthonormal, so this is it.
            vec3 localEye = vec3(dot(-vVoxelView, vCubeAxes[0]), dot(-vVoxelView, vCubeAxes[1]),
                                 dot(-vVoxelView, vCubeAxes[2]));
            vec3 localDir = vec3(dot(dir, vCubeAxes[0]), dot(dir, vCubeAxes[1]),
                                 dot(dir, vCubeAxes[2]));
            float h = vSpriteRadius / ${CUBE_SPAN.toFixed(7)};
            // Sign-preserving reciprocal. GLSL has no vector less-than, and sign() returns
            // 0 for a ray exactly parallel to a slab; forcing +1 there makes t huge, which
            // is what "never crosses this pair of planes" should mean.
            vec3 way = vec3(localDir.x >= 0.0 ? 1.0 : -1.0,
                            localDir.y >= 0.0 ? 1.0 : -1.0,
                            localDir.z >= 0.0 ? 1.0 : -1.0);
            vec3 inv = way / max(abs(localDir), vec3(1e-6));
            vec3 t0 = (-vec3(h) - localEye) * inv;
            vec3 t1 = (vec3(h) - localEye) * inv;
            vec3 tNear = min(t0, t1);
            vec3 tFar = max(t0, t1);
            float tIn = max(max(tNear.x, tNear.y), tNear.z);
            float tOut = min(min(tFar.x, tFar.y), tFar.z);
            if (tIn > tOut || tOut < 0.0) discard;  // the ray misses: outside the silhouette
            // The slab that won the max is the face that was hit.
            vec3 face = step(vec3(tIn - 1e-5), tNear) * -sign(localDir);
            vec3 n = normalize(vCubeAxes * face);
            float light = 0.42 + 0.72 * max(dot(n, lightDir), 0.0);
            gl_FragColor = vec4(vColor * light, 1.0);
            return;
        }
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
        voxelStyle,
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
