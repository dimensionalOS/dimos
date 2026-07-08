// Single shared WebXR renderer for all VR cockpits (Go2 vr.js, arm vrarm.js).
//
// A canvas has exactly ONE WebGL context. If each cockpit constructed its own
// THREE.WebGLRenderer on #canvas, the second would fight the first for the
// context and renderer.xr.setSession() would build an XRWebGLBinding against a
// stale/invalid session ("parameter 1 is not of type 'XRSession'"). So both
// cockpits must share this one instance.

import * as THREE from 'three';

let renderer = null;

export function getVRRenderer() {
    if (!renderer) {
        const canvas = document.getElementById('canvas');
        // xrCompatible at context creation — on Quest, without it the GL context
        // is unusable by the immersive session and video panels come up black.
        renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true, xrCompatible: true });
        renderer.autoClear = true;
        renderer.xr.enabled = true;
        renderer.xr.setReferenceSpaceType('local-floor');
    }
    return renderer;
}
