// The capture-pose photographs: every picture in the recording, hung where it was taken.
//
// Lives beside scene.js rather than in it because that file is at the repository's 75 KB
// per-file limit, and this is the one self-contained group in it: the poses, the JPEG
// bytes as they arrive, and the level-of-detail rule that decides which of them are
// decoded right now. Mixed into `WorldScene.prototype`, so `this` is the scene.

import * as THREE from 'https://esm.sh/three@0.160.0';

// Thumbnails decode only near the viewer, and only this many at once.
export const IMAGE_RENDER_DISTANCE_M = 12.0;
export const IMAGE_QUAD_BUDGET = 24;
export const IMAGE_LOD_INTERVAL_S = 0.2;     // how often the visible set is recomputed

// A second tier, for the photo you have actually walked up to. The thumbnail is 192 px,
// which reads fine across the room and is visibly mush at arm's length, so a marker this
// close is re-fetched from the recording at `MARKER_SHARP_SIZE_PX` and swapped in place.
export const IMAGE_SHARP_DISTANCE_M = 7.0;
// Hysteresis: upgrade at 7 m, drop back at 10. Equal thresholds make a viewer standing on
// the boundary fetch and release the same photo every LOD tick.
export const IMAGE_SHARP_RELEASE_M = 10.0;
// What actually bounds GPU memory, since a sharp texture is ~45x the pixels of a
// thumbnail: 15 of them decoded at 1280x720 RGBA is ~55 MB, against ~1.2 MB for the same
// count of thumbnails. The quad budget does NOT bound it -- that counts quads, and a quad
// costs what its texture costs. `_updateSharpTier` also clamps this to the quality level's
// own `quad_budget`, so a headset dropping frames sheds sharp photos before anything else.
export const IMAGE_SHARP_BUDGET = 15;
export const MARKER_SHARP_SIZE_PX = 1280;    // the server clamps this to its own ceiling

export const photoMarkerMethods = {
    setImagePoses(header, payloadArrayBuffer) {
        this._releaseAllThumbnails();
        this._thumbnailBytes.clear();
        this._thumbnailUndecodable.clear();
        this._sharpUnavailable.clear();
        this._imagePoseMeta = [];

        const n = header.n | 0;
        if (n === 0) return;

        const positions = new Float32Array(payloadArrayBuffer, 0, n * 3);
        const quats = new Float32Array(payloadArrayBuffer, n * 12, n * 4);

        // Default PlaneGeometry normal is +Z. Rotate so the normal points along
        // robot -X (i.e. "behind" the capture direction), so the image is seen
        // face-on when the viewer stands in front of the pose.
        const faceBackward = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), -Math.PI / 2);
        const standUpright = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), -Math.PI / 2);

        for (let i = 0; i < n; i++) {
            const quat = new THREE.Quaternion(
                quats[i * 4 + 0], quats[i * 4 + 1], quats[i * 4 + 2], quats[i * 4 + 3],
            ).multiply(faceBackward).multiply(standUpright);
            this._imagePoseMeta.push({
                id: header.ids?.[i] ?? null,
                // The recording stamp, which is how the sharp re-fetch names this frame.
                // Null where the header did not carry one: that marker simply never
                // upgrades, rather than fetching `t=undefined` and 404ing every tick.
                ts: header.timestamps?.[i] ?? null,
                rx: positions[i * 3 + 0],
                ry: positions[i * 3 + 1],
                rz: positions[i * 3 + 2],
                quadQuat: quat,
            });
        }
        this.diag('image_poses_loaded', { n });
    },

    /** Thumbnails arrive once and are kept as JPEG bytes; decoding is deferred
     *  to `_updateImageLod` so only nearby poses ever cost a texture. */
    addImageThumbnail(index, jpegArrayBuffer) {
        if (!this._imagePoseMeta[index]) return;
        this._thumbnailBytes.set(index, jpegArrayBuffer);
        this._thumbnailUndecodable.delete(index);   // different bytes, a fresh chance
    },

    /** Keep decoded thumbnails to the nearest `IMAGE_QUAD_BUDGET` poses within
     *  `IMAGE_RENDER_DISTANCE_M`, measured in world metres so zooming out drops
     *  quads rather than piling up hundreds of textured, sorted transparents. */
    _updateImageLod(dt) {
        this._imageLodAccumS += dt;
        if (this._imageLodAccumS < IMAGE_LOD_INTERVAL_S) return;
        this._imageLodAccumS = 0;
        if (!this._imageQuadGroup.visible || this._imagePoseMeta.length === 0) return;
        // An answer that brought its own photographs has already said which pictures are
        // relevant; the capture-pose markers are every photo in the recording, and drawing
        // the nearest two dozen of those beside the answer's own is what made it impossible
        // to tell which pictures the answer was actually claiming.
        if (this._queryImageMeshes.some(Boolean)) {
            this._releaseAllThumbnails();
            return;
        }

        const eye = this._imageQuadGroup.worldToLocal(this.camera.getWorldPosition(new THREE.Vector3()));
        const scale = this._worldGroup.scale.x || 1;
        // A query answer is a handful of poses anywhere in the recording, so the
        // distance cutoff would hide the very thing the user asked to see. The
        // budget alone is enough to bound the cost there.
        const level = this._qualityLevel();
        const budget = Math.min(IMAGE_QUAD_BUDGET, level.quad_budget);
        // One id space on the wire: the server snaps every engine's answer to marker
        // ids before publishing, so an answer that selects nothing genuinely has nothing.
        const selected = this._selectedImageIds.size > 0 ? this._selectedImageIds : null;
        const maxDist = selected
            ? Infinity
            : Math.min(IMAGE_RENDER_DISTANCE_M, Number.isFinite(level.voxel_range_m) ? level.voxel_range_m : Infinity) / scale;

        const candidates = [];
        for (let i = 0; i < this._imagePoseMeta.length; i++) {
            const meta = this._imagePoseMeta[i];
            if (selected && !selected.has(meta.id)) continue;
            if (!this._thumbnailBytes.has(i)) continue;
            // One that has already failed to decode never will: the bytes do not change,
            // and the budget is small. Left in, the nearest corrupt thumbnail was decoded
            // again every 0.2 s and held one of four slots for good -- a frame further
            // away, whole and decodable, was never drawn at all.
            if (this._thumbnailUndecodable.has(i)) continue;
            const dx = meta.rx - eye.x;
            const dy = meta.ry - eye.y;
            const dz = meta.rz - eye.z;
            const dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
            if (dist > maxDist) continue;
            candidates.push([dist, i]);
        }
        candidates.sort((a, b) => a[0] - b[0]);
        const wanted = new Set(candidates.slice(0, budget).map(([, i]) => i));

        for (const index of Array.from(this._imageQuadsByIndex.keys())) {
            if (!wanted.has(index)) this._releaseThumbnail(index);
        }
        for (const index of wanted) {
            if (!this._imageQuadsByIndex.has(index)) this._decodeThumbnail(index);
        }

        this._updateSharpTier(candidates, budget, scale);
    },

    /** Swap the nearest few quads to the recording's own frame instead of the thumbnail.
     *
     *  Runs off the SAME distance list the thumbnail pass just built, so a marker can only
     *  become sharp if it is already drawn -- there is no second notion of "near", and no
     *  fetch for a photo that is not on screen.
     */
    _updateSharpTier(candidates, budget, scale) {
        const sharpBudget = Math.min(IMAGE_SHARP_BUDGET, budget);
        const upgradeAt = IMAGE_SHARP_DISTANCE_M / scale;
        const releaseAt = IMAGE_SHARP_RELEASE_M / scale;

        const wantSharp = new Set();
        if (sharpBudget > 0) {
            // `candidates` is already sorted nearest-first by the thumbnail pass.
            for (const [dist, index] of candidates) {
                if (wantSharp.size >= sharpBudget) break;
                if (dist > upgradeAt) break;
                if (this._sharpUnavailable.has(index)) continue;
                wantSharp.add(index);
            }
        }
        // Hysteresis, and only for quads that are still drawn: one that is already sharp
        // keeps its texture out to the release radius rather than flipping at the edge.
        const distanceOf = new Map(candidates.map(([dist, index]) => [index, dist]));
        for (const index of this._sharpByIndex) {
            if (!this._imageQuadsByIndex.has(index)) continue;
            const dist = distanceOf.get(index);
            if (dist !== undefined && dist <= releaseAt && wantSharp.size < sharpBudget) {
                wantSharp.add(index);
            }
        }

        for (const index of Array.from(this._sharpByIndex)) {
            if (!wantSharp.has(index)) this._downgradeToThumbnail(index);
        }
        for (const index of wantSharp) {
            if (!this._sharpByIndex.has(index)) this._upgradeToSharp(index);
        }
    },

    /** Fetch this marker's frame at full size and swap it onto the quad already drawn. */
    _upgradeToSharp(index) {
        if (this._sharpFetching.has(index)) return;
        const meta = this._imagePoseMeta[index];
        if (!meta || meta.ts === null || meta.ts === undefined) return;
        if (!this._imageQuadsByIndex.has(index)) return;   // nothing on screen to swap onto
        const base = this.baseUrl ?? '';
        this._sharpFetching.add(index);
        const url = `${base}/replay/frame?t=${meta.ts}&size=${MARKER_SHARP_SIZE_PX}`;
        fetch(url)
            .then((response) => {
                if (!response.ok) throw new Error(`HTTP ${response.status}`);
                return response.blob();
            })
            .then((blob) => createImageBitmap(blob, { imageOrientation: 'flipY' }))
            .then((bitmap) => {
                this._sharpFetching.delete(index);
                const quad = this._imageQuadsByIndex.get(index);
                // The viewer can walk away while the fetch is in flight; there is then no
                // quad to swap onto and the bitmap is dropped. A quad that was released
                // and rebuilt meanwhile is the SAME marker, so swapping onto it is right.
                if (!quad) { bitmap.close(); return; }
                const texture = new THREE.Texture(bitmap);
                texture.flipY = false;
                texture.colorSpace = THREE.SRGBColorSpace;
                texture.generateMipmaps = false;
                texture.minFilter = THREE.LinearFilter;
                texture.needsUpdate = true;
                this._swapQuadTexture(quad, texture);
                this._sharpByIndex.add(index);
            })
            .catch((e) => {
                this._sharpFetching.delete(index);
                // A recording can simply have no frame at this stamp (the route answers
                // 404), and retrying it every 0.2 s would hold a slot in the small sharp
                // budget for good -- the same failure mode `_thumbnailUndecodable` exists
                // to stop for decoding. One attempt per marker per recording.
                this._sharpUnavailable.add(index);
                this.diag('marker_sharp_failed', { index, error: String(e.message || e) });
            });
    },

    /** Put the thumbnail back, so the sharp textures stay bounded by their own budget. */
    _downgradeToThumbnail(index) {
        this._sharpByIndex.delete(index);
        const quad = this._imageQuadsByIndex.get(index);
        const bytes = this._thumbnailBytes.get(index);
        if (!quad || !bytes) return;
        const blob = new Blob([bytes], { type: 'image/jpeg' });
        createImageBitmap(blob, { imageOrientation: 'flipY' }).then((bitmap) => {
            // Re-approached while the decode was in flight: the sharp texture is the one
            // wanted now, so keep it rather than stepping back down onto the thumbnail.
            if (!this._imageQuadsByIndex.has(index) || this._sharpByIndex.has(index)) {
                bitmap.close();
                return;
            }
            const texture = new THREE.Texture(bitmap);
            texture.flipY = false;
            texture.colorSpace = THREE.SRGBColorSpace;
            texture.generateMipmaps = false;
            texture.minFilter = THREE.LinearFilter;
            texture.needsUpdate = true;
            this._swapQuadTexture(this._imageQuadsByIndex.get(index), texture);
        }).catch(() => {});   // the thumbnail already drew once; leaving the sharp one is fine
    },

    /** Replace a quad's texture, releasing the one it was showing.
     *
     *  `dispose()` alone drops only the GPU copy -- the ImageBitmap behind it stays in
     *  memory until it is closed, which at 1280x720 is ~3.5 MB a swap.
     */
    _swapQuadTexture(quad, texture) {
        const old = quad.material.map;
        quad.material.map = texture;
        quad.material.needsUpdate = true;
        if (old) {
            if (old.image && old.image.close) old.image.close();
            old.dispose();
        }
    },

    _decodeThumbnail(index) {
        if (this._thumbnailDecoding.has(index)) return;
        this._thumbnailDecoding.add(index);
        // Three.js doesn't reliably apply flipY to ImageBitmap textures, so we
        // flip at decode time and disable the texture's own flip — otherwise the
        // photos render upside down.
        // The BYTES this decode is for. A decode is in flight for a while, and
        // `addImageThumbnail` can replace the bytes under it in that window -- which is
        // how a resend of a thumbnail that arrived corrupt is meant to fix it. The
        // failure that comes back afterwards belongs to the bytes that are gone: marking
        // the INDEX undecodable on it condemned the good replacement unseen, since
        // `_updateImageLod` then skipped that index forever.
        const bytes = this._thumbnailBytes.get(index);
        const stale = () => this._thumbnailBytes.get(index) !== bytes;
        const blob = new Blob([bytes], { type: 'image/jpeg' });
        createImageBitmap(blob, { imageOrientation: 'flipY' }).then((bitmap) => {
            this._thumbnailDecoding.delete(index);
            const meta = this._imagePoseMeta[index];
            if (!meta || this._imageQuadsByIndex.has(index) || stale()) {
                bitmap.close();
                return;
            }
            const texture = new THREE.Texture(bitmap);
            texture.flipY = false;
            texture.colorSpace = THREE.SRGBColorSpace;
            texture.generateMipmaps = false;
            texture.minFilter = THREE.LinearFilter;
            texture.needsUpdate = true;
            const quad = new THREE.Mesh(
                this._imageQuadGeom,
                new THREE.MeshBasicMaterial({ map: texture, side: THREE.DoubleSide }),
            );
            quad.position.set(meta.rx, meta.ry, meta.rz);
            quad.quaternion.copy(meta.quadQuat);
            this._imageQuadGroup.add(quad);
            this._imageQuadsByIndex.set(index, quad);
        }).catch((e) => {
            this._thumbnailDecoding.delete(index);
            if (stale()) return;  // these bytes are not the ones on offer any more
            this._thumbnailUndecodable.add(index);
            this.diag('thumbnail_decode_failed', { index, error: String(e.message || e) });
        });
    },

    _releaseThumbnail(index) {
        this._sharpByIndex.delete(index);
        const quad = this._imageQuadsByIndex.get(index);
        if (!quad) return;
        this._imageQuadGroup.remove(quad);
        quad.material.map.image.close();   // the ImageBitmap; dispose() only drops the GPU copy
        quad.material.map.dispose();
        quad.material.dispose();
        this._imageQuadsByIndex.delete(index);
    },

    _releaseAllThumbnails() {
        for (const index of Array.from(this._imageQuadsByIndex.keys())) this._releaseThumbnail(index);
    },
};
