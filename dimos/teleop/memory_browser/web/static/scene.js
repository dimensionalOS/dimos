// Three.js scene — curved arc of thumbnails ("the ribbon") + active highlight.
//
// We keep all visual state (positions, base scales, materials) inside this
// module so the rest of the app only has to call:
//
//   const scene = new RibbonScene(gl);
//   scene.setExpectedSize(n);                    // from timeline_summary
//   scene.addThumbnail(index, jpegBytes, ts);    // from MSG_THUMBNAIL frames
//   scene.setActiveIndex(i);                     // from active_index updates
//   scene.clearFocus();                          // from clear_focus
//   scene.render(frame, xrRefSpace, layer);      // each XR frame

import * as THREE from 'https://esm.sh/three@0.160.0';

// ----------------------------------------------------------------------
// Vertical layout (metres above the floor).
//
//   2.45  ┌─ side pane / preview tops
//         │     SCREEN ROW       (centre Y = 2.05)
//   1.65  └─ side pane / preview bottoms
//   1.55     ← gap
//   1.39  ┌─ active ribbon panel top
//         │     RIBBON STRIP     (centre Y = 1.30)
//   1.21  └─ ribbon bottom
//   1.13       caption (centre)
//
//    .85       DISC CONSOLE      (front-right of user, tilted up)
// ----------------------------------------------------------------------

// Ribbon row.
const RIBBON_RADIUS = 1.6;
const RIBBON_HEIGHT = 1.30;
const RIBBON_ARC_DEG = 140;
const PANEL_W = 0.22;            // bigger cards
const PANEL_H = 0.125;           // 16:9 source aspect
const ACTIVE_SCALE = 1.7;
const ACTIVE_FORWARD = 0.14;
// Mild slant per non-active panel — angled around its own Y so adjacent cards
// look stacked instead of overlaid. Active panel lerps back to face-on (0).
const PANEL_SLANT = 0.52;        // rad (~30°), projected width = W * cos(slant)
const HOVER_LERP = 0.18;
const DISMISS_DURATION_MS = 280;
// Per-slot HSL hue spread so adjacent untextured panels are easy to tell apart.
const SLOT_HUE_STEP = 0.018;

// Caption directly below the highlighted ribbon panel — small enough to fit
// between the ribbon and the timeline tick row.
const CAPTION_W = 0.42;
const CAPTION_H = 0.07;
const CAPTION_DROP = 0.16;

// Screen row — preview centre-front with side panes at ±SIDE_ANGLE on the
// same 1.7 m radius arc so they're equidistant from the user.
const PREVIEW_W = 1.40;
const PREVIEW_H = 0.80;
const PREVIEW_HEIGHT = 2.05;     // raised so screen bottoms clear active ribbon top
const PREVIEW_DISTANCE = 1.7;
const PREVIEW_FADE_LERP = 0.20;
const SIDE_W = 0.65;
const SIDE_H = 0.65;
const DASH_W = 0.50;
const DASH_H = 0.80;
const SIDE_ANGLE = 0.85;         // rad (~48.7°) — clearance for the wider preview
// Minimap marker (the dot showing where the active frame was captured).
const MARKER_RADIUS = 0.018;
// Tech-frame styling: thin wireframe cubes outline each pane.
const FRAME_DEPTH = 0.005;       // thin shell — was 0.04 (looked too chunky)
const FRAME_COLOR = 0x4cd9ff;
const FRAME_OPACITY = 0.55;
// Timeline bar below the ribbon (and below the caption).
const TIMELINE_HEIGHT_OFFSET = -0.27;  // metres below ribbon centerline
const TIMELINE_TICK_W = 0.012;
const TIMELINE_TICK_H = 0.025;
const TIMELINE_TICK_COLOR = 0x4cd9ff;
const TIMELINE_ACTIVE_COLOR = 0xffe07a;
const TIMELINE_ACTIVE_W = 0.022;
const TIMELINE_ACTIVE_H = 0.06;

export class RibbonScene {
    constructor(diag) {
        this.diag = diag || (() => {});

        // Let Three.js own the canvas + context. With an externally-created
        // hidden canvas, the WebXR layer binding inside Three.js's WebXRManager
        // can fail silently — frames tick, but nothing reaches the headset's
        // compositor. Owning the canvas avoids that whole class of bug.
        this.three = new THREE.WebGLRenderer({
            alpha: true,
            antialias: false,
        });
        this.three.setSize(window.innerWidth || 800, window.innerHeight || 600);
        this.three.setPixelRatio(window.devicePixelRatio || 1);
        this.three.xr.enabled = true;
        this.three.setClearColor(0x000000, 0);

        const dom = this.three.domElement;
        dom.style.position = 'fixed';
        dom.style.top = '0';
        dom.style.left = '0';
        dom.style.width = '100vw';
        dom.style.height = '100vh';
        dom.style.zIndex = '50';
        dom.style.pointerEvents = 'none';
        document.body.appendChild(dom);

        this.scene = new THREE.Scene();
        this.scene.add(new THREE.AmbientLight(0xffffff, 1.0));

        this.camera = new THREE.PerspectiveCamera(60, 1, 0.05, 100);

        // Reusable square geometry centered at origin.
        this._panelGeom = new THREE.PlaneGeometry(PANEL_W, PANEL_H);

        // Body-locked group that holds the ribbon. Each frame we set its
        // position to the user's XZ and rotation to their yaw, so the ribbon
        // is always in front no matter where they turn or walk.
        this._ribbonGroup = new THREE.Group();
        this.scene.add(this._ribbonGroup);

        // Big preview pane above the ribbon — shows whichever frame is active.
        this._previewMat = new THREE.MeshBasicMaterial({
            color: 0xffffff,
            transparent: true,
            opacity: 0.0,
            side: THREE.DoubleSide,
        });
        this._previewMesh = new THREE.Mesh(
            new THREE.PlaneGeometry(PREVIEW_W, PREVIEW_H),
            this._previewMat,
        );
        this._previewMesh.position.set(0, PREVIEW_HEIGHT, -PREVIEW_DISTANCE);
        this._previewMesh.lookAt(0, PREVIEW_HEIGHT, 0);
        this._previewFrame = this._makeFrame(PREVIEW_W, PREVIEW_H);
        this._previewMesh.add(this._previewFrame);
        this._ribbonGroup.add(this._previewMesh);
        this._previewTargetOpacity = 0.0;

        // Minimap pane on the left. A solid dark-blue placeholder until the
        // global map JPEG arrives. A small marker mesh tracks the active pose.
        this._minimapMat = new THREE.MeshBasicMaterial({
            color: 0x18233a,
            transparent: true,
            opacity: 0.85,
            side: THREE.DoubleSide,
        });
        this._minimapMesh = new THREE.Mesh(
            new THREE.PlaneGeometry(SIDE_W, SIDE_H),
            this._minimapMat,
        );
        this._minimapMesh.position.set(
            -PREVIEW_DISTANCE * Math.sin(SIDE_ANGLE),
            PREVIEW_HEIGHT,
            -PREVIEW_DISTANCE * Math.cos(SIDE_ANGLE),
        );
        this._minimapMesh.lookAt(0, PREVIEW_HEIGHT, 0);
        this._minimapMesh.add(this._makeFrame(SIDE_W, SIDE_H));
        this._ribbonGroup.add(this._minimapMesh);

        // Marker is a child of the minimap mesh so its local x/y coords map
        // directly into the minimap's plane. Z is just in front to avoid
        // z-fighting with the map texture.
        this._markerMat = new THREE.MeshBasicMaterial({
            color: 0xff3344,
            transparent: true,
            opacity: 0.0,
        });
        this._markerMesh = new THREE.Mesh(
            new THREE.CircleGeometry(MARKER_RADIUS, 16),
            this._markerMat,
        );
        this._markerMesh.position.set(0, 0, 0.001);
        this._minimapMesh.add(this._markerMesh);
        this._markerTargetOpacity = 0.0;
        this._mapBounds = null;

        // Dashboard pane on the right — text rendered to a 2D canvas, used
        // as a CanvasTexture so it can be updated on each active_index change
        // without round-tripping through the GPU texture upload path.
        this._dashCanvas = document.createElement('canvas');
        this._dashCanvas.width = 512;
        this._dashCanvas.height = 720;
        this._dashCtx = this._dashCanvas.getContext('2d');
        this._dashTex = new THREE.CanvasTexture(this._dashCanvas);
        this._dashTex.colorSpace = THREE.SRGBColorSpace;
        this._dashMat = new THREE.MeshBasicMaterial({
            map: this._dashTex,
            transparent: true,
            opacity: 0.95,
            side: THREE.DoubleSide,
        });
        this._dashMesh = new THREE.Mesh(
            new THREE.PlaneGeometry(DASH_W, DASH_H),
            this._dashMat,
        );
        this._dashMesh.position.set(
            PREVIEW_DISTANCE * Math.sin(SIDE_ANGLE),
            PREVIEW_HEIGHT,
            -PREVIEW_DISTANCE * Math.cos(SIDE_ANGLE),
        );
        this._dashMesh.lookAt(0, PREVIEW_HEIGHT, 0);
        this._dashMesh.add(this._makeFrame(DASH_W, DASH_H));
        this._ribbonGroup.add(this._dashMesh);
        // Cached metadata array; populated by `setFrameMetaBatch`.
        this._frameMeta = [];
        this._renderDashboard(null);

        // Timeline bar — a row of small ticks below the ribbon arc, with one
        // brighter tick marking the current cursor position. Built when
        // `setExpectedSize` is called.
        this._timelineGroup = new THREE.Group();
        this._ribbonGroup.add(this._timelineGroup);
        this._timelineTickGeom = new THREE.PlaneGeometry(TIMELINE_TICK_W, TIMELINE_TICK_H);
        this._timelineTickMat = new THREE.MeshBasicMaterial({
            color: TIMELINE_TICK_COLOR,
            transparent: true,
            opacity: 0.6,
        });
        this._timelineCursor = null;

        // Caption strip that floats just below the highlighted ribbon panel,
        // showing the frame index + timestamp at a glance. Texture is a 2D
        // canvas redrawn on each setActiveIndex.
        this._captionCanvas = document.createElement('canvas');
        this._captionCanvas.width = 480;
        this._captionCanvas.height = 80;
        this._captionCtx = this._captionCanvas.getContext('2d');
        this._captionTex = new THREE.CanvasTexture(this._captionCanvas);
        this._captionTex.colorSpace = THREE.SRGBColorSpace;
        this._captionMat = new THREE.MeshBasicMaterial({
            map: this._captionTex,
            transparent: true,
            opacity: 0.0,
            side: THREE.DoubleSide,
        });
        this._captionMesh = new THREE.Mesh(
            new THREE.PlaneGeometry(CAPTION_W, CAPTION_H),
            this._captionMat,
        );
        // Position is set per active-index in `_updateCaption` so it tracks
        // whichever ribbon panel is highlighted.
        this._captionMesh.position.set(0, RIBBON_HEIGHT - CAPTION_DROP, -RIBBON_RADIUS + 0.02);
        this._ribbonGroup.add(this._captionMesh);
        this._captionTargetOpacity = 0.0;

        // index -> { mesh, currentScale, forwardOffset, dismissedAt, textured, texture }
        this._panels = new Map();
        this._n = 0;
        this._activeIndex = -1;
        this._dismissed = false;

        // Anchor flag: ribbon is positioned once (on first XR frame, when we
        // know where the user is looking) and then stays world-fixed so the
        // user can walk around it without it following.
        this._anchored = false;

        // Wrist-roll dial — sits on a "console" plane below the user, in
        // front of them. Tilted up so the user can glance down at it like a
        // real desk dashboard. Spins only while the right grip is engaged.
        this._handDiscMount = new THREE.Group();
        // Set off to the right and a bit further forward so it reads as a
        // separate console element rather than a screen control.
        this._handDiscMount.position.set(0.55, 0.85, -0.85);
        // Tilt the dial face up toward the user (~45°) so spokes are readable
        // without crouching.
        this._handDiscMount.rotation.x = -Math.PI / 4;
        this._handDiscMount.add(this._makeFrame(0.30, 0.30));
        this._ribbonGroup.add(this._handDiscMount);

        this._handDiscSpinner = new THREE.Group();
        this._handDiscMount.add(this._handDiscSpinner);
        this._handDiscSpinner.add(this._buildHandDiscVisual());
        this._wristRoll = 0.0;
        this._wristRollBase = null;  // captured at engage time
    }

    _buildHandDiscVisual() {
        const group = new THREE.Group();
        const accent = new THREE.Color(FRAME_COLOR);
        const accentMat = new THREE.LineBasicMaterial({
            color: accent,
            transparent: true,
            opacity: 0.85,
        });
        const dimMat = new THREE.LineBasicMaterial({
            color: accent,
            transparent: true,
            opacity: 0.45,
        });

        // Outer ring (TorusGeometry edges as a circle of segments).
        const ring = new THREE.Mesh(
            new THREE.TorusGeometry(0.06, 0.003, 6, 48),
            new THREE.MeshBasicMaterial({
                color: accent,
                transparent: true,
                opacity: 0.75,
            }),
        );
        group.add(ring);

        // 12 radial spokes drawn as a single LineSegments primitive.
        const spokeVerts = [];
        for (let i = 0; i < 12; i++) {
            const a = (i / 12) * Math.PI * 2;
            const cx = Math.cos(a), sy = Math.sin(a);
            spokeVerts.push(cx * 0.022, sy * 0.022, 0, cx * 0.058, sy * 0.058, 0);
        }
        const spokeGeom = new THREE.BufferGeometry();
        spokeGeom.setAttribute(
            'position',
            new THREE.Float32BufferAttribute(spokeVerts, 3),
        );
        group.add(new THREE.LineSegments(spokeGeom, dimMat));

        // North tick — a bright wedge so wrist roll is unambiguous.
        const tick = new THREE.Mesh(
            new THREE.PlaneGeometry(0.014, 0.024),
            new THREE.MeshBasicMaterial({
                color: 0xffe07a,
                transparent: true,
                opacity: 0.95,
                side: THREE.DoubleSide,
            }),
        );
        tick.position.set(0, 0.072, 0);
        group.add(tick);

        // Centre cross-hair so the disc reads as a dial even when textured
        // panels are bright behind it.
        const crossVerts = new Float32Array([
            -0.015, 0, 0, 0.015, 0, 0,
            0, -0.015, 0, 0, 0.015, 0,
        ]);
        const crossGeom = new THREE.BufferGeometry();
        crossGeom.setAttribute('position', new THREE.BufferAttribute(crossVerts, 3));
        group.add(new THREE.LineSegments(crossGeom, accentMat));

        return group;
    }

    _makeFrame(w, h) {
        // Wireframe outline cube wrapping a plane of size (w, h). The "Tron"
        // look the original placeholder cube had, applied as a tech border
        // around each pane. Thin LineSegments so it stays cheap.
        const geom = new THREE.BoxGeometry(w * 1.05, h * 1.05, FRAME_DEPTH);
        const edges = new THREE.EdgesGeometry(geom);
        const mat = new THREE.LineBasicMaterial({
            color: FRAME_COLOR,
            transparent: true,
            opacity: FRAME_OPACITY,
        });
        return new THREE.LineSegments(edges, mat);
    }

    _buildTimeline(n) {
        // Clear previous timeline ticks (e.g., on reconnect with a different N).
        while (this._timelineGroup.children.length) {
            const c = this._timelineGroup.children.pop();
            if (c.geometry && c !== this._timelineTickGeom) c.geometry.dispose?.();
        }
        if (n < 2) return;

        const arcRad = (RIBBON_ARC_DEG * Math.PI) / 180;
        const y = RIBBON_HEIGHT + TIMELINE_HEIGHT_OFFSET;
        for (let i = 0; i < n; i++) {
            const t = i / (n - 1);
            const theta = (t - 0.5) * arcRad;
            const x = RIBBON_RADIUS * Math.sin(theta);
            const z = -RIBBON_RADIUS * Math.cos(theta);
            const mesh = new THREE.Mesh(this._timelineTickGeom, this._timelineTickMat);
            mesh.position.set(x, y, z);
            mesh.lookAt(0, y, 0);
            this._timelineGroup.add(mesh);
        }

        // Cursor — bigger, brighter, on top of the tick row.
        if (this._timelineCursor) {
            this._timelineCursor.geometry?.dispose?.();
            this._timelineCursor.material?.dispose?.();
        }
        const cursorGeom = new THREE.PlaneGeometry(TIMELINE_ACTIVE_W, TIMELINE_ACTIVE_H);
        const cursorMat = new THREE.MeshBasicMaterial({
            color: TIMELINE_ACTIVE_COLOR,
            transparent: true,
            opacity: 0.0,
        });
        const cursor = new THREE.Mesh(cursorGeom, cursorMat);
        this._timelineGroup.add(cursor);
        this._timelineCursor = cursor;
        this._timelineCursorTargetOpacity = 0.0;
    }

    _updateTimelineCursor(index) {
        if (!this._timelineCursor || this._n < 2) return;
        const t = index / (this._n - 1);
        const arcRad = (RIBBON_ARC_DEG * Math.PI) / 180;
        const theta = (t - 0.5) * arcRad;
        const y = RIBBON_HEIGHT + TIMELINE_HEIGHT_OFFSET;
        const x = RIBBON_RADIUS * Math.sin(theta);
        const z = -RIBBON_RADIUS * Math.cos(theta);
        this._timelineCursor.position.set(x, y, z);
        this._timelineCursor.lookAt(0, y, 0);
        this._timelineCursorTargetOpacity = 1.0;
    }

    async setSession(session, perFrame) {
        // After the session is granted, re-affirm XR-compatibility on the
        // gl context. The `xrCompatible: true` flag at context creation is
        // best-effort — if no XR device existed yet, the flag silently fails.
        const gl = this.three.getContext();
        if (gl && gl.makeXRCompatible) {
            try {
                await gl.makeXRCompatible();
                this.diag('gl_xr_compatible');
            } catch (e) {
                this.diag('make_xr_compatible_failed', { error: String(e.message || e) });
            }
        }

        this.three.xr.setReferenceSpaceType('local-floor');
        await this.three.xr.setSession(session);
        this.diag('three_xr_session_set');

        this.three.setAnimationLoop((time, frame) => {
            if (perFrame) perFrame(frame);
            this._tick(frame);
            this.three.render(this.scene, this.camera);
        });
    }

    setExpectedSize(n) {
        this._n = n;
        this._buildTimeline(n);
    }

    addThumbnail(index, jpegBytes, _ts) {
        if (this._panels.has(index)) return;  // dedupe (resends are harmless)
        // Create the panel IMMEDIATELY with a per-slot solid colour. That way
        // it's visible in VR the moment it's added, even if the JPEG decode
        // takes time or fails outright. We swap in the texture when ready.
        const hue = (index * SLOT_HUE_STEP) % 1.0;
        const color = new THREE.Color().setHSL(hue, 0.65, 0.55);
        const mat = new THREE.MeshBasicMaterial({
            color,
            transparent: true,
            opacity: 1.0,
            side: THREE.DoubleSide,
        });
        const mesh = new THREE.Mesh(this._panelGeom, mat);
        this._positionAtIndex(mesh, index);
        // Tech-frame outline around each ribbon panel.
        mesh.add(this._makeFrame(PANEL_W, PANEL_H));
        this._ribbonGroup.add(mesh);
        this._panels.set(index, {
            mesh,
            currentScale: 1.0,
            forwardOffset: 0.0,
            currentSlant: PANEL_SLANT,
            dismissedAt: null,
            textured: false,
            texture: null,
        });
        if (index === 0) this.diag('first_panel_added', { panels: this._panels.size });

        // Decode JPEG via HTMLImageElement -> Texture. More reliable on Quest
        // than TextureLoader for blob URLs.
        const blob = new Blob([jpegBytes], { type: 'image/jpeg' });
        const url = URL.createObjectURL(blob);
        const img = new Image();
        img.onload = () => {
            const tex = new THREE.Texture(img);
            tex.colorSpace = THREE.SRGBColorSpace;
            tex.needsUpdate = true;
            mat.map = tex;
            mat.color.setRGB(1, 1, 1);  // unmodulated texture
            mat.needsUpdate = true;
            const p = this._panels.get(index);
            if (p) {
                p.textured = true;
                p.texture = tex;
            }
            // If this image happens to be the currently-active one, push it
            // into the preview pane right now (the user already pointed at it
            // before the JPEG decoded).
            if (index === this._activeIndex) {
                this._showPreview(tex);
            }
            URL.revokeObjectURL(url);
            if (index === 0) this.diag('first_texture_loaded');
        };
        img.onerror = (e) => {
            this.diag('texture_load_failed', { index, error: String(e && e.message || e) });
            URL.revokeObjectURL(url);
        };
        img.src = url;
    }

    _positionAtIndex(mesh, index) {
        const n = Math.max(this._n || 1, 1);
        const arcRad = (RIBBON_ARC_DEG * Math.PI) / 180;
        const t = n === 1 ? 0.5 : index / (n - 1);
        const theta = (t - 0.5) * arcRad;
        const x = RIBBON_RADIUS * Math.sin(theta);
        const z = -RIBBON_RADIUS * Math.cos(theta);
        mesh.position.set(x, RIBBON_HEIGHT, z);
        // Every panel faces the arc centre (where the user is standing) so
        // all images present their textured faces toward the viewer with the
        // same orientation, regardless of arc angle.
        mesh.lookAt(0, RIBBON_HEIGHT, 0);
    }

    setActiveIndex(i) {
        this._activeIndex = i;
        this._dismissed = false;
        const p = this._panels.get(i);
        if (p && p.texture) {
            this._showPreview(p.texture);
        } else {
            this._previewTargetOpacity = 0.0;
        }
        const meta = this._frameMeta[i] || null;
        this._renderDashboard(meta);
        this._updateMarker(meta);
        this._updateTimelineCursor(i);
        this._updateCaption(i, meta);
    }

    clearFocus() {
        const was = this._panels.get(this._activeIndex);
        if (was) was.dismissedAt = performance.now();
        this._activeIndex = -1;
        this._dismissed = true;
        this._hidePreview();
        this._renderDashboard(null);
        this._markerTargetOpacity = 0.0;
        this._timelineCursorTargetOpacity = 0.0;
        this._captionTargetOpacity = 0.0;
    }

    _updateCaption(index, meta) {
        if (!this._captionMesh || !this._captionCtx) return;

        // Position the caption directly below whichever ribbon panel is
        // highlighted. We pull it inward by ACTIVE_FORWARD (plus a small
        // extra) so the caption is *in front of* the popped-out panel rather
        // than coplanar with the un-popped ribbon — otherwise the active
        // panel occludes the caption sitting behind it.
        const n = Math.max(this._n || 1, 1);
        const arcRad = (RIBBON_ARC_DEG * Math.PI) / 180;
        const t = n === 1 ? 0.5 : index / (n - 1);
        const theta = (t - 0.5) * arcRad;
        const r = Math.max(RIBBON_RADIUS - ACTIVE_FORWARD - 0.04, 0.5);
        const x = r * Math.sin(theta);
        const z = -r * Math.cos(theta);
        const y = RIBBON_HEIGHT - CAPTION_DROP;
        this._captionMesh.position.set(x, y, z);
        this._captionMesh.lookAt(0, y, 0);

        // Redraw texture.
        const ctx = this._captionCtx;
        const w = this._captionCanvas.width;
        const h = this._captionCanvas.height;
        const MONO = '"JetBrains Mono", "Fira Code", "SF Mono", monospace';
        ctx.clearRect(0, 0, w, h);
        ctx.fillStyle = 'rgba(8, 14, 22, 0.92)';
        ctx.fillRect(0, 0, w, h);
        ctx.strokeStyle = '#4cd9ff';
        ctx.lineWidth = 2;
        ctx.strokeRect(2, 2, w - 4, h - 4);

        const idxStr = `#${String(index).padStart(2, '0')}`;
        ctx.fillStyle = '#ffe07a';
        ctx.font = `bold 36px ${MONO}`;
        ctx.fillText(idxStr, 16, 52);

        const tsStr = meta && meta.ts
            ? new Date(meta.ts * 1000).toLocaleTimeString()
            : '—';
        ctx.fillStyle = '#cfe0ff';
        ctx.font = `24px ${MONO}`;
        ctx.fillText(tsStr, 110, 50);

        this._captionTex.needsUpdate = true;
        this._captionTargetOpacity = 1.0;
    }

    _showPreview(texture) {
        this._previewMat.map = texture;
        this._previewMat.needsUpdate = true;
        this._previewTargetOpacity = 1.0;
    }

    _hidePreview() {
        this._previewTargetOpacity = 0.0;
    }

    setGlobalMap(jpegBytes, bounds) {
        // Decode the JPEG into a Texture and swap it into the minimap mesh.
        const blob = new Blob([jpegBytes], { type: 'image/jpeg' });
        const url = URL.createObjectURL(blob);
        const img = new Image();
        img.onload = () => {
            const tex = new THREE.Texture(img);
            tex.colorSpace = THREE.SRGBColorSpace;
            tex.needsUpdate = true;
            this._minimapMat.map = tex;
            this._minimapMat.color.setRGB(1, 1, 1);
            this._minimapMat.needsUpdate = true;
            URL.revokeObjectURL(url);
            this.diag('global_map_loaded', { w: img.width, h: img.height });
        };
        img.onerror = () => {
            URL.revokeObjectURL(url);
            this.diag('global_map_failed');
        };
        img.src = url;
        this._mapBounds = bounds;
    }

    setFrameMetaBatch(entries) {
        this._frameMeta = Array.isArray(entries) ? entries : [];
        this.diag('frame_meta_set', { count: this._frameMeta.length });
        // If we already have an active index, refresh side panes now.
        if (this._activeIndex >= 0) {
            const meta = this._frameMeta[this._activeIndex] || null;
            this._renderDashboard(meta);
            this._updateMarker(meta);
        }
    }

    _updateHandDisc(frame) {
        if (!this._handDiscSpinner || !frame) return;
        const refSpace = this.three.xr.getReferenceSpace();
        if (!refSpace) return;
        for (const inputSource of frame.session.inputSources) {
            if (inputSource.handedness !== 'right') continue;
            const space = inputSource.gripSpace || inputSource.targetRaySpace;
            if (!space) continue;
            const pose = frame.getPose(space, refSpace);
            if (!pose) continue;
            const gp = inputSource.gamepad;
            const grip = gp ? (gp.buttons[1]?.value ?? 0) : 0;
            const engaged = grip > 0.5;

            const o = pose.transform.orientation;
            const q = new THREE.Quaternion(o.x, o.y, o.z, o.w);
            const e = new THREE.Euler().setFromQuaternion(q, 'YXZ');
            // Sign chosen so the disc spins in the same direction as the
            // wrist (positive roll → CCW disc) — flipped from the previous
            // build, which read inverted.
            const rawRoll = e.z;

            if (engaged) {
                // Capture a baseline on engage so the disc starts at its
                // current angle and rotates relative to the wrist's pose at
                // grab time — mirrors how the timeline cursor tracks delta.
                if (this._wristRollBase === null) {
                    this._wristRollBase = rawRoll - this._wristRoll;
                }
                this._wristRoll = rawRoll - this._wristRollBase;
                this._handDiscSpinner.rotation.z = this._wristRoll;
            } else {
                // Release: drop the baseline so the next engage re-anchors.
                // The disc keeps its last angle (no snap-back) so the user
                // can see where the cursor sits.
                this._wristRollBase = null;
            }
            return;
        }
    }

    _updateMarker(meta) {
        if (!meta || !meta.pose || !this._mapBounds) {
            this._markerTargetOpacity = 0.0;
            return;
        }
        const [px, py] = meta.pose;
        const b = this._mapBounds;
        const u = (px - b.x_min) / (b.x_max - b.x_min);
        // Image rows were flipped so v=0 is the top (max world Y). Match.
        const v = 1.0 - (py - b.y_min) / (b.y_max - b.y_min);
        if (!Number.isFinite(u) || !Number.isFinite(v)) {
            this._markerTargetOpacity = 0.0;
            return;
        }
        // Local coords on the minimap plane: (-SIDE_W/2, -SIDE_H/2)..(+,+).
        const localX = (u - 0.5) * SIDE_W;
        const localY = (0.5 - v) * SIDE_H;  // Three plane Y is up
        this._markerMesh.position.set(localX, localY, 0.001);
        this._markerTargetOpacity = 1.0;
    }

    _renderDashboard(meta) {
        const ctx = this._dashCtx;
        const w = this._dashCanvas.width;
        const h = this._dashCanvas.height;
        const MONO = '"JetBrains Mono", "Fira Code", "SF Mono", "Consolas", monospace';
        const ACCENT = '#4cd9ff';
        const GRID = 'rgba(76, 217, 255, 0.08)';

        // Background
        ctx.fillStyle = 'rgba(8, 14, 22, 0.96)';
        ctx.fillRect(0, 0, w, h);
        // Faint grid (every 32 px) for the tech feel.
        ctx.strokeStyle = GRID;
        ctx.lineWidth = 1;
        for (let gx = 0; gx < w; gx += 32) {
            ctx.beginPath(); ctx.moveTo(gx, 0); ctx.lineTo(gx, h); ctx.stroke();
        }
        for (let gy = 0; gy < h; gy += 32) {
            ctx.beginPath(); ctx.moveTo(0, gy); ctx.lineTo(w, gy); ctx.stroke();
        }
        // Bracket-style corner accents.
        ctx.strokeStyle = ACCENT;
        ctx.lineWidth = 3;
        const C = 28;
        ctx.beginPath();
        ctx.moveTo(8, 8 + C); ctx.lineTo(8, 8); ctx.lineTo(8 + C, 8);
        ctx.moveTo(w - 8 - C, 8); ctx.lineTo(w - 8, 8); ctx.lineTo(w - 8, 8 + C);
        ctx.moveTo(8, h - 8 - C); ctx.lineTo(8, h - 8); ctx.lineTo(8 + C, h - 8);
        ctx.moveTo(w - 8 - C, h - 8); ctx.lineTo(w - 8, h - 8); ctx.lineTo(w - 8, h - 8 - C);
        ctx.stroke();

        // Title strip
        ctx.fillStyle = ACCENT;
        ctx.font = `bold 26px ${MONO}`;
        ctx.fillText('// FRAME META', 32, 56);

        if (!meta) {
            ctx.fillStyle = '#5a6478';
            ctx.font = `22px ${MONO}`;
            ctx.fillText('[ scrub to select ]', 32, 110);
            this._dashTex.needsUpdate = true;
            return;
        }

        // Big slot index
        ctx.fillStyle = '#fff';
        ctx.font = `bold 80px ${MONO}`;
        ctx.fillText(`#${String(meta.index ?? 0).padStart(2, '0')}`, 32, 140);

        ctx.fillStyle = '#9aa4bd';
        ctx.font = `20px ${MONO}`;
        const tsDate = meta.ts ? new Date(meta.ts * 1000).toLocaleString() : '—';
        ctx.fillText(tsDate, 32, 175);

        // Separator
        ctx.strokeStyle = ACCENT;
        ctx.globalAlpha = 0.5;
        ctx.beginPath(); ctx.moveTo(32, 200); ctx.lineTo(w - 32, 200); ctx.stroke();
        ctx.globalAlpha = 1;

        let y = 240;
        const row = (label, value, color = '#fff') => {
            ctx.fillStyle = '#5a6478';
            ctx.font = `22px ${MONO}`;
            ctx.fillText(label.toUpperCase().padEnd(8, ' '), 32, y);
            ctx.fillStyle = color;
            ctx.font = `bold 26px ${MONO}`;
            ctx.fillText(String(value), 220, y);
            y += 44;
        };

        row('id', meta.id ?? '—');
        const dims = meta.width && meta.height ? `${meta.width}x${meta.height}` : '—';
        row('dims', dims);
        const b = typeof meta.brightness === 'number' ? meta.brightness.toFixed(3) : '—';
        row('bright', b, '#a3e1ff');
        const s = typeof meta.sharpness === 'number' ? meta.sharpness.toFixed(3) : '—';
        row('sharp', s, '#a3e1ff');

        if (Array.isArray(meta.pose) && meta.pose.length >= 3) {
            const [px, py, pz] = meta.pose;
            row('x', px.toFixed(2), '#ffe07a');
            row('y', py.toFixed(2), '#ffe07a');
            row('z', pz.toFixed(2), '#ffe07a');
        }

        if (meta.tags && Object.keys(meta.tags).length) {
            y += 12;
            ctx.fillStyle = '#5a6478';
            ctx.font = `22px ${MONO}`;
            ctx.fillText('TAGS', 32, y);
            y += 32;
            ctx.fillStyle = '#cfe0ff';
            ctx.font = `18px ${MONO}`;
            for (const [k, v] of Object.entries(meta.tags)) {
                const text = `${k}: ${typeof v === 'object' ? JSON.stringify(v) : v}`;
                ctx.fillText(text.slice(0, 36), 36, y);
                y += 26;
                if (y > h - 32) break;
            }
        }
        this._dashTex.needsUpdate = true;
    }

    _tick(frame) {
        const now = performance.now();
        this._updateHandDisc(frame);

        // First-frame anchor: snapshot the user's XZ position and yaw, plant
        // the ribbon there, and never move it again. The user can then walk
        // around the workspace freely.
        if (!this._anchored && this._ribbonGroup) {
            const xrCam = this.three.xr.getCamera();
            if (xrCam) {
                const p = new THREE.Vector3();
                const q = new THREE.Quaternion();
                xrCam.getWorldPosition(p);
                xrCam.getWorldQuaternion(q);
                const yaw = Math.atan2(
                    2 * (q.w * q.y + q.x * q.z),
                    1 - 2 * (q.y * q.y + q.x * q.x),
                );
                if (Number.isFinite(yaw) && Number.isFinite(p.x)) {
                    this._ribbonGroup.position.set(p.x, 0, p.z);
                    this._ribbonGroup.rotation.y = yaw;
                    this._anchored = true;
                    this.diag('ribbon_anchored', { x: p.x, z: p.z, yaw });
                }
            }
        }

        // Smooth preview opacity toward target.
        if (this._previewMat) {
            const cur = this._previewMat.opacity;
            this._previewMat.opacity = cur + (this._previewTargetOpacity - cur) * PREVIEW_FADE_LERP;
        }
        if (this._markerMat) {
            const cur = this._markerMat.opacity;
            this._markerMat.opacity = cur + (this._markerTargetOpacity - cur) * PREVIEW_FADE_LERP;
        }
        if (this._timelineCursor) {
            const m = this._timelineCursor.material;
            const cur = m.opacity;
            const tgt = this._timelineCursorTargetOpacity ?? 0;
            m.opacity = cur + (tgt - cur) * PREVIEW_FADE_LERP;
        }
        if (this._captionMat) {
            const cur = this._captionMat.opacity;
            this._captionMat.opacity = cur + (this._captionTargetOpacity - cur) * PREVIEW_FADE_LERP;
        }

        for (const [idx, p] of this._panels) {
            const isActive = idx === this._activeIndex;
            const targetScale = isActive ? ACTIVE_SCALE : 1.0;
            const targetFwd = isActive ? ACTIVE_FORWARD : 0.0;
            const targetSlant = isActive ? 0.0 : PANEL_SLANT;
            p.currentScale += (targetScale - p.currentScale) * HOVER_LERP;
            p.forwardOffset += (targetFwd - p.forwardOffset) * HOVER_LERP;
            p.currentSlant += (targetSlant - p.currentSlant) * HOVER_LERP;
            p.mesh.scale.setScalar(p.currentScale);

            // Re-aim at the user, then apply the per-panel slant around its
            // own local Y so cards read as separate stacked panels.
            this._positionAtIndex(p.mesh, idx);
            if (Math.abs(p.currentSlant) > 1e-4) {
                p.mesh.rotateY(p.currentSlant);
            }
            // Active panel pops along the radial-inward direction so it
            // approaches the user without rotating away from face-on.
            const dir = new THREE.Vector3(0, RIBBON_HEIGHT, 0).sub(p.mesh.position).normalize();
            p.mesh.position.addScaledVector(dir, p.forwardOffset);

            if (p.dismissedAt !== null) {
                const t = Math.min(1, (now - p.dismissedAt) / DISMISS_DURATION_MS);
                const outward = new THREE.Vector3()
                    .subVectors(p.mesh.position, new THREE.Vector3(0, RIBBON_HEIGHT, 0))
                    .normalize();
                p.mesh.position.addScaledVector(outward, 0.3 * t);
                if (p.mesh.material) p.mesh.material.opacity = 1.0 - t;
                if (t >= 1) {
                    p.dismissedAt = null;
                    if (p.mesh.material) p.mesh.material.opacity = 1.0;
                }
            }
        }
    }
}
