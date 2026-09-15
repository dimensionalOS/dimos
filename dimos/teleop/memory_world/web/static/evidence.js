// The photographs behind an answer, hung where their cameras stood.
//
// Each one is a quad on its camera's image plane, `distance_m` in front of where
// that camera was and sized by its field of view, with thin lines back to the eye
// so the frustum reads. A ring marks the pixel the answer actually matched and a
// line runs from it to the voxel that pixel produced, so the photo is visibly the
// reason for the highlight rather than something that happens to be beside it.

import * as THREE from 'https://esm.sh/three@0.160.0';

/** Hang the frame behind an answer on its camera's image plane: a quad
 *  `distance_m` in front of where the camera stood, sized by its field of
 *  view, with thin lines back to the camera so the frustum reads. */
/** Where a photograph hangs and how wide it is there, as `[from, to, radius]` for
 *  `scene.setSightLine`. `[null, null, 0]` when the header cannot say, which turns the
 *  corridor off rather than guessing at one.
 *
 *  Here rather than in a caller because both the place stepper and the P key need it, and
 *  a second copy of this arithmetic is a second thing to keep in step with the header. */
export function sightLineFor(header) {
    if (!header || !header.position || !header.forward) return [null, null, 0];
    const f = header.forward;
    const at = header.distance_m
        ? [0, 1, 2].map((k) => header.position[k] + f[k] * header.distance_m)
        : header.point;
    const halfWidth = header.distance_m && header.hfov_deg
        ? header.distance_m * Math.tan((header.hfov_deg * Math.PI) / 360)
        : 0.8;
    if (!at) return [null, null, 0];
    return [header.position, at, halfWidth];
}

export function addQueryImage(scene, header, jpegArrayBuffer) {
    if (header.query_id !== scene._activeQueryId) return;
    const blob = new Blob([jpegArrayBuffer], { type: 'image/jpeg' });
    createImageBitmap(blob, { imageOrientation: 'flipY' }).then((bitmap) => {
        if (header.query_id !== scene._activeQueryId) { bitmap.close(); return; }
        const texture = new THREE.Texture(bitmap);
        texture.flipY = false;
        texture.colorSpace = THREE.SRGBColorSpace;
        texture.generateMipmaps = false;
        texture.minFilter = THREE.LinearFilter;
        texture.needsUpdate = true;

        const eye = new THREE.Vector3(...header.position);
        const forward = new THREE.Vector3(...header.forward).normalize();
        const up = new THREE.Vector3(...header.up).normalize();
        const distance = header.distance_m || 1.0;
        const width = 2 * distance * Math.tan(THREE.MathUtils.degToRad(header.hfov_deg || 70) / 2);
        const height = width / (header.aspect || 16 / 9);

        const quad = new THREE.Mesh(
            new THREE.PlaneGeometry(width, height),
            new THREE.MeshBasicMaterial({ map: texture, side: THREE.DoubleSide }),
        );
        quad.position.copy(eye).addScaledVector(forward, distance);
        // lookAt works in world space; the group is a child of the frame rotation.
        scene._frameRotate.updateWorldMatrix(true, false);
        quad.up.copy(up).transformDirection(scene._frameRotate.matrixWorld);
        scene._highlightGroup.add(quad);
        // lookAt aims the plane's front (+z, where the texture reads
        // correctly) at the eye. Turning it away showed the back face,
        // which is the picture mirrored.
        quad.lookAt(scene._frameRotate.localToWorld(eye.clone()));

        const right = new THREE.Vector3().crossVectors(forward, up).normalize();
        const centre = quad.position.clone();
        const corners = [[-1, -1], [1, -1], [1, 1], [-1, 1]].map(([sx, sy]) =>
            centre.clone().addScaledVector(right, sx * width / 2).addScaledVector(up, sy * height / 2));
        const segments = [];
        for (const corner of corners) segments.push(eye.clone(), corner);
        for (let i = 0; i < 4; i++) segments.push(corners[i], corners[(i + 1) % 4]);
        const frustum = new THREE.LineSegments(
            new THREE.BufferGeometry().setFromPoints(segments),
            new THREE.LineBasicMaterial({
                color: header.cluster === 0 || header.index === 0 ? 0xff5c3a : 0xffb347,
            }),
        );
        if (header.cluster !== undefined) frustum.userData.cluster = header.cluster;

        scene._highlightGroup.add(frustum);
        scene._queryImages[header.index] = header;
        scene._queryImageMeshes[header.index] = quad;
        // Frames from nearby poses overlap; while standing at one camera only its frame
        // shows, while a cluster is selected only that cluster's do, and with Photos off
        // none do. The frustum is a highlight, not a photo, so it follows the cluster only.
        const filter = scene.clusterFilter;
        if (filter >= 0 && header.cluster !== undefined && header.cluster !== filter) {
            frustum.visible = false;
        }
        scene._applyQueryImageVisibility();
        scene.diag('query_image_placed', { index: header.index, width: Number(width.toFixed(2)) });
    }).catch((e) => {
        scene.diag('query_image_failed', { index: header.index, error: String(e.message || e) });
    });
}
