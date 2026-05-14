import * as THREE from "three";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";

/**
 * AiAvatar (SimStudio)
 * - Vanilla THREE (no React)
 * - Uses Rapier KinematicCharacterController for collision-aware movement
 * - "Awareness" of tagged areas via `getTags()` and a sensing radius
 * - Persistent memory stored in localStorage per (worldKey, agentId)
 */
export class AiAvatar {
  constructor({
    id = null,
    scene,
    rapierWorld,
    RAPIER,
    getWorldKey,
    getTags,
    getPlayerPosition,
    avatarUrl = "/avatars/kai.glb",
    senseRadius = 3.0,
    walkSpeed = 2.0,
    vlm = null,
    headless = false,
  }) {
    this.id = id || `ai-${Math.random().toString(36).slice(2, 8)}`;
    this.scene = scene;
    this.rapierWorld = rapierWorld;
    this.RAPIER = RAPIER;
    this.getWorldKey = getWorldKey || (() => "default");
    this.getTags = getTags || (() => []);
    this.getPlayerPosition = getPlayerPosition || (() => [0, 0, 0]);
    this.avatarUrl = avatarUrl;
    this.senseRadius = senseRadius;
    this.walkSpeed = walkSpeed;
    this.vlm = vlm; // { enabled, endpoint, model, buildPrompt, actions, captureBase64, decideEverySteps, stepMeters }
    this.headless = headless; // Skip visual rendering in headless mode, keep colliders

    // AI dimensions (match the smaller player so the agent fits in tight interiors too).
    this.radius = 0.12;
    this.halfHeight = 0.25;

    // Look pitch (radians). Used for agent POV capture to allow looking up/down.
    // Kept separate from group rotation (yaw) so walking remains on XZ.
    this.pitch = 0;

    this.group = new THREE.Group();
    this.group.name = `AiAvatar:${this.id}`;
    this.scene.add(this.group);

    // Fallback visual (if GLB fails)
    this.fallback = new THREE.Mesh(
      new THREE.CapsuleGeometry(this.radius * 0.9, this.halfHeight * 2.0, 6, 10),
      new THREE.MeshStandardMaterial({ color: 0x8cffc1, roughness: 0.65 })
    );
    this.fallback.castShadow = false;
    this.fallback.receiveShadow = false;
    this.group.add(this.fallback);

    // Facing indicator (always present so direction is obvious even with capsule / GLB).
    // Cone points along +Z (our forward convention in this project).
    this._facing = new THREE.Mesh(
      new THREE.ConeGeometry(this.radius * 0.45, this.radius * 1.35, 14),
      new THREE.MeshStandardMaterial({ color: 0xffc36a, roughness: 0.45, metalness: 0.05 })
    );
    this._facing.rotation.x = Math.PI / 2;
    this._facing.position.set(0, this.halfHeight * 0.15, this.radius * 1.1);
    this._facing.castShadow = false;
    this._facing.receiveShadow = false;
    this._facing.renderOrder = 10;
    this.group.add(this._facing);

    // Thought label (simple sprite)
    this._labelCanvas = document.createElement("canvas");
    this._labelCanvas.width = 512;
    this._labelCanvas.height = 512;
    this._labelCtx = this._labelCanvas.getContext("2d");
    this._labelTex = new THREE.CanvasTexture(this._labelCanvas);
    this._labelSprite = new THREE.Sprite(
      new THREE.SpriteMaterial({
        map: this._labelTex,
        transparent: true,
        depthWrite: false,
        depthTest: false, // always visible (splats/voxels can otherwise cover it)
        toneMapped: false,
      })
    );
    this._labelSprite.scale.set(2.0, 1.0, 1);
    this._labelSprite.position.set(0, this.halfHeight * 2 + this.radius + 0.8, 0);
    this._labelSprite.renderOrder = 5000; // after SparkRenderer (999)
    this.group.add(this._labelSprite);
    this._setThought("");
    this._lastDecisionBubbleAt = 0;

    this._target = null; // THREE.Vector3
    this._state = "IDLE"; // IDLE | WALK | INSPECT
    this._stateUntil = 0;
    this._nearbyTagId = null;
    this._lastInspectAtByTagId = {};
    this._inspectCooldownMs = 8000; // don't repeatedly "stop" while standing inside a big tag radius

    // VLM action plan
    this._plan = null; // { type, ... }
    this._planRemaining = 0;
    this._stepCounter = 0;
    this._nextDecisionAt = 0;
    this._decisionJitterMs = (Math.random() * 350) | 0;
    this._vlmInFlight = null;
    this._pendingDecision = null;

    // Short-term trace for the current task run (fed back to the VLM).
    this._taskStartedAt = 0;
    this._trace = []; // [{t,type,msg,data?}]
    this._traceLimit = 20;
    // Prior model outputs for the current task run (fed back as assistant messages).
    this._vlmAssistantHistory = []; // string[]
    this._vlmAssistantHistoryLimit = 15;
    this._moveStepAcc = 0;
    this._turnAcc = 0;

    // Task state tracking
    this._startPosition = null; // {x,y,z} - where agent was when task started
    this._currentSubgoal = ""; // Current sub-goal from VLM reasoning
    this._discoveredItems = []; // Items/assets found during exploration

    // Rapier body/collider/controller
    const radius = this.radius;
    const halfHeight = this.halfHeight;
    this.body = this.rapierWorld.createRigidBody(
      this.RAPIER.RigidBodyDesc.kinematicPositionBased().setTranslation(0, 3, 0)
    );
    this.collider = this.rapierWorld.createCollider(
      this.RAPIER.ColliderDesc.capsule(halfHeight, radius).setFriction(0.8),
      this.body
    );
    // Fake robodog body volume: add a horizontal spine capsule to reduce rear clipping.
    // Movement still uses the vertical capsule for stable character-controller behavior.
    this._spineHalfLen = Math.max(radius * 1.2, 0.13);
    this._spineRadius = Math.max(radius * 0.62, 0.07);
    // Keep fake volume mostly behind body center so it doesn't "push from the air" in front.
    this._spineOffsetBack = Math.max(radius * 2.2, this._spineHalfLen + this._spineRadius + 0.02);
    this._spineOffsetY = Math.max(this.halfHeight * 0.35, 0.08);
    this.spineCollider = this.rapierWorld.createCollider(
      this.RAPIER.ColliderDesc.capsule(this._spineHalfLen, this._spineRadius)
        .setFriction(0.8)
        .setTranslation(0, this._spineOffsetY, -this._spineOffsetBack)
        // Rotate local +Y capsule axis to +Z (dog spine direction at yaw=0).
        .setRotation({ x: Math.SQRT1_2, y: 0, z: 0, w: Math.SQRT1_2 }),
      this.body
    );
    this.boxCollider = null; // Box collider created when GLB loads (matches model dimensions)
    this.controller = this.rapierWorld.createCharacterController(0.05);
    this.controller.enableAutostep(0.25, 0.15, true);
    this.controller.enableSnapToGround(0.5);
    this.controller.setSlideEnabled(true);
    this.controller.setMaxSlopeClimbAngle((45 * Math.PI) / 180);
    this.controller.setMinSlopeSlideAngle((75 * Math.PI) / 180);

    // Persistent memory
    this.memory = this._loadMemory();

    // Best-effort: load GLB visual
    this._loadGLB();
  }

  dispose() {
    try {
      this.scene.remove(this.group);
    } catch {}
    try {
      if (this.boxCollider) this.rapierWorld.removeCollider(this.boxCollider, true);
      if (this.spineCollider) this.rapierWorld.removeCollider(this.spineCollider, true);
      this.rapierWorld.removeCollider(this.collider, true);
      this.rapierWorld.removeRigidBody(this.body);
    } catch {}
  }

  setPosition(x, y, z) {
    this.body.setTranslation({ x, y, z }, true);
    this.body.setLinvel({ x: 0, y: 0, z: 0 }, true);
  }

  getPosition() {
    const p = this.body.translation();
    return [p.x, p.y, p.z];
  }

  update(dt, nowMs) {
    if (!this.rapierWorld || !this.body) return;
    const now = nowMs ?? Date.now();

    if (this.mixer) this.mixer.update(dt);

    // Sense tags in radius and update memory.
    const p = this.body.translation();
    const tags = this.getTags() || [];
    const nearby = [];
    for (const t of tags) {
      if (!t?.position) continue;
      const dx = t.position.x - p.x;
      const dy = t.position.y - p.y;
      const dz = t.position.z - p.z;
      const d = Math.sqrt(dx * dx + dy * dy + dz * dz);
      const r = Math.max(Number(t.radius ?? 1.5), this.senseRadius);
      if (d <= r) nearby.push({ tag: t, dist: d });
    }
    nearby.sort((a, b) => a.dist - b.dist);

    if (nearby.length > 0) {
      const top = nearby[0].tag;
      this._rememberTag(top);
      const topId = top.id || null;
      const title = top.title || "(untitled)";
      // Don't continuously overwrite model-output bubble while the VLM is active.
      const taskActive = !!this.vlm?.getTask?.()?.active;
      if (!this.vlm?.enabled && !taskActive && now - this._lastDecisionBubbleAt > 1200) {
        this._setThought(`I see: ${title}`);
      }

      // Only enter INSPECT when we newly encounter a tag OR the cooldown elapsed.
      // Without this, a large tag radius causes the agent to repeatedly re-enter INSPECT
      // and look "stuck" near the tag forever.
      const lastAt = (topId && this._lastInspectAtByTagId[topId]) || 0;
      const cooldownOk = !topId || now - lastAt > this._inspectCooldownMs;
      const newlyEntered = topId && topId !== this._nearbyTagId;
      this._nearbyTagId = topId;
      if (this._state !== "INSPECT" && (newlyEntered || cooldownOk)) {
        if (topId) this._lastInspectAtByTagId[topId] = now;
        this._state = "INSPECT";
        this._stateUntil = now + 900;
        this._target = null;
      }
    } else {
      this._nearbyTagId = null;
    }

    if (this._state === "INSPECT") {
      if (now > this._stateUntil) {
        this._state = "IDLE";
      }
      // When inspecting, gently look toward the closest tag in full 3D (pitch + yaw).
      try {
        const tags = this.getTags?.() || [];
        const p = this.body.translation();
        let best = null;
        let bestD = Infinity;
        for (const t of tags) {
          if (!t?.position) continue;
          const dx = t.position.x - p.x;
          const dy = t.position.y - (p.y + this.halfHeight); // approximate eye height
          const dz = t.position.z - p.z;
          const d = Math.hypot(dx, dy, dz);
          if (d < bestD) {
            bestD = d;
            best = { dx, dy, dz };
          }
        }
        if (best) {
          // Yaw toward target (XZ) and pitch toward target (Y).
          this.group.rotation.y = Math.atan2(best.dx, best.dz);
          const horiz = Math.hypot(best.dx, best.dz) || 1;
          const desiredPitch = Math.atan2(best.dy, horiz);
          const maxPitch = (85 * Math.PI) / 180;
          // Smooth pitch change to avoid snapping.
          const alpha = Math.min(1, dt * 8);
          const clamped = Math.max(-maxPitch, Math.min(maxPitch, desiredPitch));
          this.pitch = this.pitch + (clamped - this.pitch) * alpha;
        }
      } catch {
        // ignore
      }
      this._syncVisual();
      this._publishGlobals();
      return;
    }

    // If VLM mode is enabled, it drives the movement plan.
    if (this.vlm?.enabled) {
      this._vlmUpdate(dt, now, nearby);
      if (!this._plan) this._applyIdleGravity(dt);
      this._syncVisual();
      this._publishGlobals();
      return;
    }

    // Editor helper mode: hold position when not actively tasked.
    // This prevents idle spawned agents from random-walking before assignment.
    if (this.vlm?.holdPositionWhenIdle) {
      const task = this.vlm?.getTask?.();
      if (!task?.active || !this.vlm?.enabled) {
        this._state = "IDLE";
        this._target = null;
        this._setThought("");
        this._applyIdleGravity(dt);
        this._syncVisual();
        this._publishGlobals();
        return;
      }
    }

    // Pick a new wander target if needed.
    if (!this._target || this._state === "IDLE") {
      this._target = this._pickWanderTarget(p);
      this._state = "WALK";
    }

    // Move toward target on XZ plane with collision sliding.
    const dx = this._target.x - p.x;
    const dz = this._target.z - p.z;
    const dist = Math.sqrt(dx * dx + dz * dz);
    if (dist < 0.4) {
      this._state = "IDLE";
      this._target = null;
      this._setThought("");
      this._syncVisual();
      this._publishGlobals();
      return;
    }

    const vx = (dx / (dist || 1)) * this.walkSpeed;
    const vz = (dz / (dist || 1)) * this.walkSpeed;
    const desired = { x: vx * dt, y: -2.0 * dt, z: vz * dt }; // small down force to keep grounded

    // Query pipeline is updated by rapierWorld.step() in the main loop
    const m = this._computeConservativeMovement(desired);
    const mx = m.x, my = m.y, mz = m.z;
    this.body.setNextKinematicTranslation({ x: p.x + mx, y: p.y + my, z: p.z + mz });

    // Face direction.
    const yaw = Math.atan2(vx, vz);
    this.group.rotation.y = yaw;
    // While walking, ease pitch back to level.
    {
      const alpha = Math.min(1, dt * 6);
      this.pitch = this.pitch + (0 - this.pitch) * alpha;
    }

    this._syncVisual();
    this._publishGlobals();
  }

  // --- internals ---
  _vlmUpdate(dt, now, nearby) {
    const task = this.vlm?.getTask?.();
    // If there is no active instruction, do not call the VLM (keeps UI quiet and enforces one-task-at-a-time).
    if (!task?.active) return;

    // Reset per-task trace if a new task started.
    const startedAt = Number(task.startedAt || 0);
    if (startedAt && startedAt !== this._taskStartedAt) {
      this._taskStartedAt = startedAt;
      this._trace = [];
      this._vlmAssistantHistory = [];
      this._stepCounter = 0;
      this._plan = null;
      this._planRemaining = 0;
      this._moveStepAcc = 0;
      this._turnAcc = 0;
      this._currentSubgoal = "";
      this._discoveredItems = [];
      // Record start position for "return to start" navigation
      const [sx, sy, sz] = this.getPosition?.() || [0, 0, 0];
      this._startPosition = { x: sx, y: sy, z: sz };
      this._tracePush(now, "TASK_START", { instruction: String(task.instruction || ""), startPos: this._startPosition });
    }

    // Apply any resolved decision.
    if (this._pendingDecision) {
      const d = this._pendingDecision;
      this._pendingDecision = null;
      this._applyVlmDecision(d);
    }

    // Execute current plan if any.
    if (this._plan) {
      const done = this._stepPlan(dt);
      if (!done) return;
      // Plan completed.
      {
        const [x, y, z] = this.getPosition?.() || [0, 0, 0];
        const yaw = this.group?.rotation?.y ?? 0;
        this._tracePush(now, "PLAN_DONE", { plan: this._plan, pose: { x, y, z, yaw } });
      }
      this._plan = null;
      this._planRemaining = 0;
    }

    // Decide periodically (every N executed steps) or when idle.
    const decideEverySteps = Number(this.vlm.decideEverySteps ?? 4);
    if (now < this._nextDecisionAt) return;
    if (this._vlmInFlight) return;

    // Kick off async decision (non-blocking).
    this._vlmInFlight = this._requestVlmDecision(now, nearby)
      .then((d) => {
        this._pendingDecision = d;
      })
      .catch((e) => {
        // If it fails, wait a bit then try again.
        this._nextDecisionAt = now + 2000;
        this._setThought(`VLM error`);
        try {
          this.vlm?.onError?.(e);
        } catch {}
        console.warn("VLM decision failed:", e);
      })
      .finally(() => {
        this._vlmInFlight = null;
      });

    // Rate limit: even if we render at 60fps, don't spam.
    this._nextDecisionAt = now + Math.max(500, decideEverySteps * 250) + this._decisionJitterMs;
  }

  async _requestVlmDecision(now, nearby) {
    // Increment step counter once per VLM decision
    this._stepCounter++;

    const capture = this.vlm.captureBase64;
    const prompt = this.vlm.buildPrompt?.({ actions: this.vlm.actions }) ?? "";
    const model = this.vlm.getModel?.() || this.vlm.model;
    const endpoint = this.vlm.endpoint;

    const imageBase64 = await capture(this);
    if (!imageBase64) throw new Error("No image.");
    try {
      this.vlm.onCapture?.(imageBase64);
    } catch {}

    const [ax, ay, az] = this.getPosition?.() || [0, 0, 0];
    const yaw = this.group?.rotation?.y ?? 0;
    const pitch = typeof this.pitch === "number" ? this.pitch : 0;

    const pose = { x: ax.toFixed(2), y: ay.toFixed(2), z: az.toFixed(2), yaw: (yaw * 180 / Math.PI).toFixed(0) + "°", pitch: (pitch * 180 / Math.PI).toFixed(0) + "°" };

    // Get nearby assets with simplified info (filter out held items - they're not "nearby")
    const nearbyAssets = (this.vlm?.getNearbyAssets?.(this) || [])
      .filter(a => !a.isHeld) // Don't show held items in nearby list
      .map((a) => ({
        id: a.id,
        name: a.title || a.id,
        description: a.notes || "",
        distance: a.dist.toFixed(1) + "m",
        lookingAt: a.isLookedAt,
        state: a.currentStateName || a.currentState,
        canDo: (a.actions || []).map((x) => x.label).filter(Boolean),
        pickable: a.pickable || false,
      }));
    const nearbyPrimitives = (this.vlm?.getNearbyPrimitives?.(this) || []).map((p) => ({
      id: p.id,
      name: p.name || p.id,
      distance: p.dist.toFixed(1) + "m",
      lookingAt: !!p.isLookedAt,
      type: p.type || "primitive",
    }));
    const assetLibraryNames = (this.vlm?.getAssetLibraryNames?.() || []).slice(0, 12);
    const isEditorMode = !!this.vlm?.isEditorMode?.();

    // Get what the agent is currently holding
    const heldAsset = this.vlm?.getHeldAsset?.(this);
    const recentGeneratedAssets = (this.vlm?.getRecentGeneratedAssets?.(this) || []).slice(0, 8);

    // Get nearby tags/locations
    const nearbyLocations = (nearby || []).slice(0, 6).map((x) => ({
      id: x.tag?.id,
      name: x.tag?.title || "unknown",
      distance: x.dist.toFixed(1) + "m",
    }));

    // Build condensed context for user message
    const task = this.vlm?.getTask?.();
    const taskInstruction = String(task?.instruction || "No active task");

    // Chat-style message history: system prompt + prior assistant outputs + current user (context + image).
    const assistantMsgs = (this._vlmAssistantHistory || [])
      .slice(-this._vlmAssistantHistoryLimit)
      .filter((s) => typeof s === "string" && s.trim().length > 0)
      .map((s) => ({ role: "assistant", content: s }));

    // Build concise user message
    const lines = [
      `TASK: ${taskInstruction}`,
      // `${this._stepCounter}${this._stepCounter === 1 ? 'st' : this._stepCounter === 2 ? 'nd' : this._stepCounter === 3 ? 'rd' : 'th'} ACTION`,
      `POSITION: ${pose.x}, ${pose.y}, ${pose.z} | facing ${pose.yaw} yaw, ${pose.pitch} pitch`,
    ];

    if (this._startPosition) {
      lines.push(`START POSITION: ${this._startPosition.x.toFixed(1)}, ${this._startPosition.y.toFixed(1)}, ${this._startPosition.z.toFixed(1)}`);
    }

    // Show what the agent is currently holding
    if (heldAsset) {
      lines.push(`\nHOLDING: ${heldAsset.title || heldAsset.id} [id: ${heldAsset.id}]`);
      lines.push(`  → Use DROP to place it in front of you`);
    }

    if (nearbyAssets.length > 0) {
      lines.push(`\nNEARBY OBJECTS:`);
      for (const a of nearbyAssets) {
        const looking = a.lookingAt ? " [LOOKING AT]" : "";
        const pickable = a.pickable ? " [pickable]" : "";
        const portal = a.isPortal ? ` [PORTAL → ${a.destinationWorld || "?"}]` : "";
        const actions = a.canDo.length > 0 ? ` → can: ${a.canDo.join(", ")}` : "";
        // Include the asset ID so the VLM can use it for INTERACT/PICK_UP actions
        lines.push(`  • ${a.name} [id: ${a.id}] (${a.distance})${looking}${pickable}${portal} - ${a.state}${actions}`);
      }
    }
    if (nearbyPrimitives.length > 0) {
      lines.push(`\nNEARBY PRIMITIVES:`);
      for (const p of nearbyPrimitives) {
        const looking = p.lookingAt ? " [LOOKING AT]" : "";
        lines.push(`  • ${p.name} [id: ${p.id}] (${p.distance})${looking} - ${p.type}`);
      }
    }
    if (recentGeneratedAssets.length > 0) {
      lines.push(`\nRECENT GENERATED ASSETS (transform these by ID even if not in nearby list):`);
      for (const a of recentGeneratedAssets) {
        const name = String(a?.name || "generated-asset");
        const id = String(a?.id || "");
        if (!id) continue;
        lines.push(`  • ${name} [id: ${id}]`);
      }
    }

    if (nearbyLocations.length > 0) {
      lines.push(`\nNEARBY LOCATIONS:`);
      for (const loc of nearbyLocations) {
        lines.push(`  • ${loc.name} (${loc.distance}) [id: ${loc.id}]`);
      }
    }
    lines.push(`\nEDITOR MODE: ${isEditorMode ? "ON" : "OFF"}`);
    if (isEditorMode && assetLibraryNames.length > 0) {
      lines.push(`ASSET LIBRARY: ${assetLibraryNames.join(", ")}`);
    }

    const userText = lines.join("\n");

    // Keep context object for backwards compatibility but simplified
    const context = {
      task: { instruction: taskInstruction, active: !!task?.active },
      pose: { x: ax, y: ay, z: az, yaw, pitch },
      step: this._stepCounter,
      startPosition: this._startPosition,
      editorMode: isEditorMode,
      nearbyAssets,
      nearbyPrimitives,
      assetLibraryNames,
      nearbyLocations,
      heldAsset: heldAsset ? { id: heldAsset.id, title: heldAsset.title } : null,
    };

    const messages = [
      { role: "system", content: prompt },
      ...assistantMsgs,
      {
        role: "user",
        content: [
          { type: "text", text: userText },
          { type: "image_url", image_url: { url: `data:image/jpeg;base64,${imageBase64}` } },
        ],
      },
    ];

    try {
      this.vlm?.onRequest?.({ endpoint, model, prompt, context, imageBase64, messages });
    } catch {}

    const res = await this.vlm.request({
      endpoint,
      model,
      prompt,
      imageBase64,
      context,
      messages,
    });

    // Server returns {raw: "..."} or direct JSON object.
    const raw = res?.raw ?? res;
    // Persist raw model output into per-task assistant history for next turn context.
    try {
      const rawStr = typeof raw === "string" ? raw : JSON.stringify(raw);
      this._vlmAssistantHistory.push(rawStr);
      if (this._vlmAssistantHistory.length > this._vlmAssistantHistoryLimit) {
        this._vlmAssistantHistory.splice(0, this._vlmAssistantHistory.length - this._vlmAssistantHistoryLimit);
      }
    } catch {
      // ignore
    }
    const parsed = typeof raw === "string" ? safeParseJson(raw) : raw;
    if (!parsed || typeof parsed !== "object") throw new Error("Invalid VLM output.");
    // Update speech bubble immediately on each model turn (before action application).
    try {
      const responseBubble = this._extractBubbleTextFromModelOutput(parsed, raw);
      if (responseBubble) {
        this._setThought(responseBubble);
        this._lastDecisionBubbleAt = Date.now();
      }
    } catch {}
    try {
      this.vlm?.onResponse?.({ raw: typeof raw === "string" ? raw : JSON.stringify(raw), parsed });
    } catch {}
    return parsed;
  }

  _applyVlmDecision(decision) {
    // Extract reasoning/thinking for logs and bubble display.
    const paramsForBubble = decision?.params && typeof decision.params === "object" ? decision.params : {};
    const thinking = decision.thinking || decision.thought || decision.reasoning || "";
    const observation =
      decision.observation ||
      decision.obs ||
      decision.perception ||
      paramsForBubble.observation ||
      "";
    const subgoal = decision.currentSubgoal || "";

    // Update current sub-goal tracking
    if (subgoal) this._currentSubgoal = subgoal;

    // Display observation bubble (prioritize what the agent sees, not chain-of-thought).
    // If observation is absent, show a concise action status so the bubble still updates each turn.
    const actionPreview = String(decision?.action || "").trim();
    const paramPreview = Object.entries(paramsForBubble)
      .slice(0, 2)
      .map(([k, v]) => `${k}=${typeof v === "string" ? v : JSON.stringify(v)}`)
      .join(", ");
    const fallbackStatus = actionPreview
      ? `Action: ${actionPreview}${paramPreview ? ` (${paramPreview})` : ""}`
      : subgoal || "";
    const displayText = String(observation || fallbackStatus || "").trim();
    this._setThought(displayText);
    this._lastDecisionBubbleAt = Date.now();

    try {
      this.vlm?.onDecision?.(decision);
    } catch {}

    const action = String(decision.action || "").trim().toUpperCase();
    const params = decision.params && typeof decision.params === "object" ? decision.params : {};

    const [x, y, z] = this.getPosition?.() || [0, 0, 0];
    const yaw = this.group?.rotation?.y ?? 0;
    this._tracePush(Date.now(), "ACTION", {
      action,
      params,
      thinking: thinking.slice(0, 100),
      subgoal,
      pose: { x: x.toFixed(2), y: y.toFixed(2), z: z.toFixed(2), yaw: (yaw * 180 / Math.PI).toFixed(0) }
    });

    // Normalize bounds helpers
    const clampInt = (v, a, b, d) => {
      const n = Math.floor(Number(v));
      return Number.isFinite(n) ? Math.max(a, Math.min(b, n)) : d;
    };
    const clampNum = (v, a, b, d) => {
      const n = Number(v);
      return Number.isFinite(n) ? Math.max(a, Math.min(b, n)) : d;
    };

    // === DONE / FINISH_TASK ===
    if (action === "DONE" || action === "FINISH_TASK") {
      const summary = String(params.summary || "Task completed");
      const confidence = clampNum(params.confidence, 0, 1, 1);
      try {
        this.vlm?.onTaskFinished?.({ summary, confidence });
      } catch {}
      this._tracePush(Date.now(), "TASK_DONE", { summary, confidence });
      this._plan = { type: "WAIT" };
      this._planRemaining = 0.5;
      return;
    }

    // === THINK - pause to reason ===
    if (action === "THINK") {
      const thought = String(params.thought || thinking || "thinking...");
      // THINK action still surfaces as a visible status, but never truncated.
      this._setThought(thought);
      this._tracePush(Date.now(), "THINK", { thought });
      this._plan = { type: "WAIT" };
      this._planRemaining = 0.5;
      return;
    }

    // === INTERACT / INTERACT_ASSET ===
    if (action === "INTERACT" || action === "INTERACT_ASSET") {
      const assetId = String(params.assetId || "");
      // Support both actionLabel (new) and actionId (old)
      const actionLabel = String(params.actionLabel || params.actionId || "");

      console.log(`[AI INTERACT] Raw params: assetId="${assetId}", actionLabel="${actionLabel}"`);

      // Find matching action by label
      const nearbyAssets = this.vlm?.getNearbyAssets?.(this) || [];
      console.log(`[AI INTERACT] Nearby assets:`, nearbyAssets.map(a => ({
        id: a.id,
        title: a.title,
        dist: a.dist?.toFixed(2),
        isLookedAt: a.isLookedAt,
        currentState: a.currentState,
        actions: a.actions?.map(act => `${act.id}:${act.label}(${act.from}->${act.to})`)
      })));

      const targetAsset = nearbyAssets.find((a) => a.id === assetId);
      let actionId = actionLabel;

      if (targetAsset && targetAsset.actions) {
        console.log(`[AI INTERACT] Target asset found: "${targetAsset.title}", looking for action: "${actionLabel}"`);
        console.log(`[AI INTERACT] Available actions:`, targetAsset.actions);

        const matchingAction = targetAsset.actions.find(
          (a) => a.label?.toLowerCase() === actionLabel.toLowerCase() || a.id === actionLabel
        );
        if (matchingAction) {
          console.log(`[AI INTERACT] Matched action: "${matchingAction.id}" (label: "${matchingAction.label}")`);
          actionId = matchingAction.id;
        } else {
          console.warn(`[AI INTERACT] No matching action found for label "${actionLabel}"`);
        }
      } else {
        console.warn(`[AI INTERACT] Target asset "${assetId}" not found in nearby assets`);
      }

      console.log(`[AI INTERACT] Final actionId to use: "${actionId}"`);
      this._tracePush(Date.now(), "INTERACT", { assetId, actionLabel, actionId });

      Promise.resolve()
        .then(() => this.vlm?.interactAsset?.({ agent: this, assetId, actionId }))
        .then((res) => {
          if (res?.ok) {
            this._tracePush(Date.now(), "INTERACT_OK", { assetId, actionLabel });
            this._discoveredItems.push({ id: assetId, interacted: true, at: Date.now() });
          } else {
            this._tracePush(Date.now(), "INTERACT_FAIL", { assetId, actionLabel, reason: res?.reason });
          }
        })
        .catch((e) => {
          this._tracePush(Date.now(), "INTERACT_FAIL", { assetId, actionLabel, reason: String(e?.message || e) });
        });

      this._plan = { type: "WAIT" };
      this._planRemaining = 0.6;
      return;
    }

    // === PICK_UP ===
    if (action === "PICK_UP") {
      const assetId = String(params.assetId || "");
      console.log(`[AI PICK_UP] Attempting to pick up: assetId="${assetId}"`);

      this._tracePush(Date.now(), "PICK_UP", { assetId });

      Promise.resolve()
        .then(() => this.vlm?.pickUpAsset?.({ agent: this, assetId }))
        .then((res) => {
          if (res?.ok) {
            this._tracePush(Date.now(), "PICK_UP_OK", { assetId });
            console.log(`[AI PICK_UP] Successfully picked up: ${assetId}`);
          } else {
            this._tracePush(Date.now(), "PICK_UP_FAIL", { assetId, reason: res?.reason });
            console.warn(`[AI PICK_UP] Failed: ${res?.reason}`);
          }
        })
        .catch((e) => {
          this._tracePush(Date.now(), "PICK_UP_FAIL", { assetId, reason: String(e?.message || e) });
        });

      this._plan = { type: "WAIT" };
      this._planRemaining = 0.5;
      return;
    }

    // === DROP ===
    if (action === "DROP") {
      console.log(`[AI DROP] Attempting to drop held item`);

      this._tracePush(Date.now(), "DROP", {});

      Promise.resolve()
        .then(() => this.vlm?.dropAsset?.({ agent: this }))
        .then((res) => {
          if (res?.ok) {
            this._tracePush(Date.now(), "DROP_OK", { assetId: res.assetId });
            console.log(`[AI DROP] Successfully dropped: ${res.assetId}`);
          } else {
            this._tracePush(Date.now(), "DROP_FAIL", { reason: res?.reason });
            console.warn(`[AI DROP] Failed: ${res?.reason}`);
          }
        })
        .catch((e) => {
          this._tracePush(Date.now(), "DROP_FAIL", { reason: String(e?.message || e) });
        });

      this._plan = { type: "WAIT" };
      this._planRemaining = 0.5;
      return;
    }

    // === TURN_LEFT / TURN_RIGHT ===
    if (action === "TURN_LEFT" || action === "TURN_RIGHT") {
      const deg = clampNum(params.degrees, 15, 90, 30);
      this._plan = { type: "TURN", dir: action === "TURN_LEFT" ? 1 : -1, radians: (deg * Math.PI) / 180 };
      this._planRemaining = this._plan.radians;
      this._turnAcc = 0;
      return;
    }

    // === LOOK_UP / LOOK_DOWN / PITCH_UP / PITCH_DOWN ===
    if (action === "LOOK_UP" || action === "PITCH_UP") {
      const deg = clampNum(params.degrees, 10, 60, 20);
      this._plan = { type: "PITCH", dir: 1, radians: (deg * Math.PI) / 180 };
      this._planRemaining = this._plan.radians;
      return;
    }
    if (action === "LOOK_DOWN" || action === "PITCH_DOWN") {
      const deg = clampNum(params.degrees, 10, 60, 20);
      this._plan = { type: "PITCH", dir: -1, radians: (deg * Math.PI) / 180 };
      this._planRemaining = this._plan.radians;
      return;
    }

    // === EDITOR: CREATE_PRIMITIVE ===
    if (action === "CREATE_PRIMITIVE") {
      if (!this.vlm?.isEditorMode?.()) {
        this._tracePush(Date.now(), "EDIT_FAIL", { action, reason: "not-edit-mode" });
        this._plan = { type: "WAIT" };
        this._planRemaining = 0.5;
        return;
      }
      const shape = String(params.shape || "box").toLowerCase();
      this._tracePush(Date.now(), "EDIT_CREATE_PRIMITIVE", { shape });
      Promise.resolve()
        .then(() => this.vlm?.createPrimitiveInEditor?.({ agent: this, shape }))
        .then((res) => {
          if (!res?.ok) this._tracePush(Date.now(), "EDIT_FAIL", { action, reason: res?.reason || "create-failed" });
        })
        .catch((e) => {
          this._tracePush(Date.now(), "EDIT_FAIL", { action, reason: String(e?.message || e) });
        });
      this._plan = { type: "WAIT" };
      this._planRemaining = 0.45;
      return;
    }

    // === EDITOR: SPAWN_LIBRARY_ASSET ===
    if (action === "SPAWN_LIBRARY_ASSET") {
      if (!this.vlm?.isEditorMode?.()) {
        this._tracePush(Date.now(), "EDIT_FAIL", { action, reason: "not-edit-mode" });
        this._plan = { type: "WAIT" };
        this._planRemaining = 0.5;
        return;
      }
      const assetName = String(params.assetName || "").trim();
      this._tracePush(Date.now(), "EDIT_SPAWN_LIBRARY_ASSET", { assetName });
      Promise.resolve()
        .then(() => this.vlm?.spawnLibraryAssetInEditor?.({ agent: this, assetName }))
        .then((res) => {
          if (!res?.ok) this._tracePush(Date.now(), "EDIT_FAIL", { action, reason: res?.reason || "spawn-failed" });
        })
        .catch((e) => {
          this._tracePush(Date.now(), "EDIT_FAIL", { action, reason: String(e?.message || e) });
        });
      this._plan = { type: "WAIT" };
      this._planRemaining = 0.55;
      return;
    }

    // === EDITOR: TRANSFORM_OBJECT ===
    if (action === "TRANSFORM_OBJECT") {
      if (!this.vlm?.isEditorMode?.()) {
        this._tracePush(Date.now(), "EDIT_FAIL", { action, reason: "not-edit-mode" });
        this._plan = { type: "WAIT" };
        this._planRemaining = 0.5;
        return;
      }
      const targetType = String(params.targetType || "").toLowerCase();
      const targetId = String(params.targetId || "");
      const editParams = {
        targetType,
        targetId,
        moveX: Number(params.moveX) || 0,
        moveY: Number(params.moveY) || 0,
        moveZ: Number(params.moveZ) || 0,
        rotateYDeg: Number(params.rotateYDeg) || 0,
        scaleMul: Number(params.scaleMul) || 1,
        setPositionX: Number.isFinite(Number(params.setPositionX)) ? Number(params.setPositionX) : undefined,
        setPositionY: Number.isFinite(Number(params.setPositionY)) ? Number(params.setPositionY) : undefined,
        setPositionZ: Number.isFinite(Number(params.setPositionZ)) ? Number(params.setPositionZ) : undefined,
        setRotationYDeg: Number.isFinite(Number(params.setRotationYDeg)) ? Number(params.setRotationYDeg) : undefined,
        setScaleX: Number.isFinite(Number(params.setScaleX)) ? Number(params.setScaleX) : undefined,
        setScaleY: Number.isFinite(Number(params.setScaleY)) ? Number(params.setScaleY) : undefined,
        setScaleZ: Number.isFinite(Number(params.setScaleZ)) ? Number(params.setScaleZ) : undefined,
        snapToCrosshair: params.snapToCrosshair === true,
      };
      this._tracePush(Date.now(), "EDIT_TRANSFORM_OBJECT", editParams);
      Promise.resolve()
        .then(() => this.vlm?.transformObjectInEditor?.({ ...editParams, agent: this }))
        .then((res) => {
          if (!res?.ok) this._tracePush(Date.now(), "EDIT_FAIL", { action, reason: res?.reason || "transform-failed" });
        })
        .catch((e) => {
          this._tracePush(Date.now(), "EDIT_FAIL", { action, reason: String(e?.message || e) });
        });
      this._plan = { type: "WAIT" };
      this._planRemaining = 0.45;
      return;
    }

    // === EDITOR: GENERATE_ASSET ===
    if (action === "GENERATE_ASSET") {
      if (!this.vlm?.isEditorMode?.()) {
        this._tracePush(Date.now(), "EDIT_FAIL", { action, reason: "not-edit-mode" });
        this._plan = { type: "WAIT" };
        this._planRemaining = 0.5;
        return;
      }
      const prompt = String(params.prompt || "").trim();
      if (!prompt) {
        this._tracePush(Date.now(), "EDIT_FAIL", { action, reason: "missing-prompt" });
        this._plan = { type: "WAIT" };
        this._planRemaining = 0.5;
        return;
      }
      const placeNow = params.placeNow !== false;
      this._tracePush(Date.now(), "EDIT_GENERATE_ASSET", { prompt: prompt.slice(0, 120), placeNow });
      Promise.resolve()
        .then(() => this.vlm?.generateAssetInEditor?.({ agent: this, prompt, placeNow }))
        .then((res) => {
          if (!res?.ok) {
            this._tracePush(Date.now(), "EDIT_FAIL", { action, reason: res?.reason || "asset-generate-failed" });
            return;
          }
          this._tracePush(Date.now(), "EDIT_ASSET_READY", {
            action,
            assetName: String(res?.assetName || ""),
            assetId: String(res?.assetId || ""),
            reused: !!res?.reused,
            placed: !!res?.placed,
          });
          const labelId = res?.assetId ? ` [id: ${res.assetId}]` : "";
          const kind = res?.reused ? "Reused" : "Generated";
          this._setThought(`${kind}: ${String(res?.assetName || "asset")}${labelId}. Next: transform to fit.`);
        })
        .catch((e) => {
          this._tracePush(Date.now(), "EDIT_FAIL", { action, reason: String(e?.message || e) });
        });
      this._plan = { type: "WAIT" };
      this._planRemaining = 0.8;
      return;
    }

    // === GOTO_LOCATION / GOTO_TAG ===
    if (action === "GOTO_LOCATION" || action === "GOTO_TAG") {
      const locId = String(params.locationId || params.tagId || "");

      // Special case: "start" returns to start position
      if (locId === "start" && this._startPosition) {
        this._plan = { type: "GOTO", x: this._startPosition.x, z: this._startPosition.z };
        this._planRemaining = 999;
        this._tracePush(Date.now(), "GOTO_START", {});
        return;
      }

      // Find tag by ID
      const tags = this.getTags?.() || [];
      const t = tags.find((x) => x?.id === locId);
      if (t?.position) {
        this._plan = { type: "GOTO", x: t.position.x, z: t.position.z };
        this._planRemaining = 999;
        return;
      }

      // Tag not found - wait briefly
      this._tracePush(Date.now(), "GOTO_FAIL", { locId, reason: "not found" });
      this._plan = { type: "WAIT" };
      this._planRemaining = 0.5;
      return;
    }

    // === MOVEMENT ===
    const isMove = ["MOVE_FORWARD", "MOVE_BACKWARD", "STRAFE_LEFT", "STRAFE_RIGHT", "MOVE_UP", "MOVE_DOWN"].includes(action);
    if (isMove) {
      const steps = clampInt(params.steps, 1, 8, 2);
      this._plan = { type: "MOVE", dir: action, steps };
      this._planRemaining = steps;
      this._moveStepAcc = 0;
      return;
    }

    // === WAIT (explicit or fallback) ===
    const secs = clampNum(params.seconds, 0.3, 3, 0.5);
    this._plan = { type: "WAIT" };
    this._planRemaining = secs;
  }

  _stepPlan(dt) {
    if (!this._plan) return true;
    const p = this.body.translation();

    if (this._plan.type === "WAIT") {
      this._planRemaining -= dt;
      return this._planRemaining <= 0;
    }

    if (this._plan.type === "TURN") {
      const turnRate = 2.4; // rad/sec
      const dYaw = Math.min(this._planRemaining, turnRate * dt);
      this.group.rotation.y += dYaw * this._plan.dir;
      this._planRemaining -= dYaw;
      this._turnAcc += dYaw;
      if (this._turnAcc >= Math.PI / 6) {
        this._tracePush(Date.now(), "TURN_PROGRESS", { degrees: Math.round((this._turnAcc * 180) / Math.PI) });
        this._turnAcc = 0;
      }
      return this._planRemaining <= 0;
    }

    if (this._plan.type === "PITCH") {
      const pitchRate = 2.6; // rad/sec
      const d = Math.min(this._planRemaining, pitchRate * dt);
      const before = this.pitch || 0;
      const maxPitch = (85 * Math.PI) / 180;
      let after = before + d * this._plan.dir;
      after = Math.max(-maxPitch, Math.min(maxPitch, after));
      this.pitch = after;
      const applied = Math.abs(after - before);
      this._planRemaining -= applied;
      // If we hit the clamp and couldn't apply, consider the action done.
      if (applied <= 1e-6) this._planRemaining = 0;
      return this._planRemaining <= 0;
    }

    // Compute basis from yaw.
    const yaw = this.group.rotation.y;
    const forward = new THREE.Vector3(Math.sin(yaw), 0, Math.cos(yaw));
    const right = new THREE.Vector3(forward.z, 0, -forward.x);

    if (this._plan.type === "GOTO") {
      const dx = this._plan.x - p.x;
      const dz = this._plan.z - p.z;
      const dist = Math.sqrt(dx * dx + dz * dz);
      if (dist < 0.35) return true;
      const vx = (dx / (dist || 1)) * this.walkSpeed;
      const vz = (dz / (dist || 1)) * this.walkSpeed;
      const desired = { x: vx * dt, y: -2.0 * dt, z: vz * dt };
      const m = this._computeConservativeMovement(desired);
      const mx = m.x, my = m.y, mz = m.z;
      this.body.setNextKinematicTranslation({ x: p.x + mx, y: p.y + my, z: p.z + mz });
      this.group.rotation.y = Math.atan2(vx, vz);
      return false;
    }

    if (this._plan.type === "MOVE") {
      const stepMeters = Number(this.vlm.stepMeters ?? 0.35);
      const dir = this._plan.dir;
      let v = new THREE.Vector3();
      if (dir === "MOVE_FORWARD") v.copy(forward);
      if (dir === "MOVE_BACKWARD") v.copy(forward).multiplyScalar(-1);
      if (dir === "STRAFE_LEFT") v.copy(right).multiplyScalar(-1);
      if (dir === "STRAFE_RIGHT") v.copy(right);
      if (dir === "MOVE_UP") v.set(0, 1, 0);
      if (dir === "MOVE_DOWN") v.set(0, -1, 0);

      // Consume "steps" by distance traveled (approx).
      const speed = this.walkSpeed;
      const distThisFrame = speed * dt;
      const verticalMove = dir === "MOVE_UP" || dir === "MOVE_DOWN";
      const desired = verticalMove
        ? { x: 0, y: v.y * speed * dt, z: 0 }
        : { x: v.x * speed * dt, y: -2.0 * dt, z: v.z * speed * dt };
      const m = this._computeConservativeMovement(desired);
      const mx = m.x, my = m.y, mz = m.z;
      this.body.setNextKinematicTranslation({ x: p.x + mx, y: p.y + my, z: p.z + mz });

      // decrement steps by fraction of stepMeters
      this._moveStepAcc += distThisFrame / Math.max(0.05, stepMeters);
      if (this._moveStepAcc >= 1) {
        const whole = Math.floor(this._moveStepAcc);
        this._moveStepAcc -= whole;
        this._tracePush(Date.now(), "MOVE_PROGRESS", { steps: whole, dir });
      }
      this._planRemaining -= distThisFrame / Math.max(0.05, stepMeters);
      if (this._planRemaining <= 0) {
        // Plan completed - step counter is incremented per VLM decision, not here
        return true;
      }
      return false;
    }

    return true;
  }

  _tracePush(t, type, data) {
    const msg = (() => {
      if (type === "TASK_START") return `task: ${String(data?.instruction || "").slice(0, 120)}`;
      if (type === "DECISION") return `${data?.action || ""}`;
      if (type === "PLAN_SET") return `${data?.plan?.type || ""}`;
      if (type === "PLAN_DONE") return `${data?.plan?.type || ""}`;
      if (type === "MOVE_PROGRESS") return `${data?.dir || ""} +${data?.steps || 0} step`;
      if (type === "TURN_PROGRESS") return `turn ~${data?.degrees || 0}°`;
      if (type === "FINISH_TASK") return `finish: ${String(data?.summary || "").slice(0, 120)}`;
      if (type === "EDIT_ASSET_READY") return `asset ready: ${String(data?.assetName || data?.assetId || "").slice(0, 120)}`;
      return "";
    })();
    this._trace.push({ t, type, msg, data });
    if (this._trace.length > this._traceLimit) this._trace.splice(0, this._trace.length - this._traceLimit);
  }

  _pickWanderTarget(p) {
    // Simple random walk around current position; if world has tags, bias toward them sometimes.
    const tags = this.getTags() || [];
    if (tags.length > 0 && Math.random() < 0.35) {
      const t = tags[(Math.random() * tags.length) | 0];
      if (t?.position) return new THREE.Vector3(t.position.x, p.y, t.position.z);
    }
    const r = 6 + Math.random() * 10;
    const a = Math.random() * Math.PI * 2;
    return new THREE.Vector3(p.x + Math.cos(a) * r, p.y, p.z + Math.sin(a) * r);
  }

  _applyIdleGravity(dt) {
    try {
      const p = this.body.translation();
      const desired = { x: 0, y: -15.0 * dt, z: 0 };
      const m = this._computeConservativeMovement(desired);
      const my = m.y;
      this.body.setNextKinematicTranslation({ x: p.x, y: p.y + my, z: p.z });
    } catch {}
  }

  _computeConservativeMovement(desired) {
    const flags = this.RAPIER.QueryFilterFlags.EXCLUDE_SENSORS;
    this.controller.computeColliderMovement(this.collider, desired, flags);
    const mm = this.controller.computedMovement();
    const main = { x: mm.x, y: mm.y, z: mm.z };
    if (!this.spineCollider) return main;

    // Keep the rear fake body aligned before querying its allowed movement.
    this._syncSpineCollider();
    this.controller.computeColliderMovement(this.spineCollider, desired, flags);
    const ms = this.controller.computedMovement();
    const spine = { x: ms.x, y: ms.y, z: ms.z };

    // Conservative merge: use whichever collider allows less displacement per axis.
    const towardZero = (a, b) => (Math.abs(a) <= Math.abs(b) ? a : b);
    return {
      x: towardZero(main.x, spine.x),
      y: towardZero(main.y, spine.y),
      z: towardZero(main.z, spine.z),
    };
  }

  _syncVisual() {
    const p = this.body.translation();
    this.group.position.set(p.x, p.y, p.z);
    this._syncSpineCollider();
  }

  _syncSpineCollider() {
    if (!this.spineCollider) return;
    const p = this.body?.translation?.();
    if (!p) return;
    const yaw = this.group?.rotation?.y ?? 0;
    // Keep horizontal capsule aligned with model yaw and shifted toward rear torso.
    const xOff = -Math.sin(yaw) * this._spineOffsetBack;
    const zOff = -Math.cos(yaw) * this._spineOffsetBack;
    const q = new THREE.Quaternion().setFromEuler(new THREE.Euler(Math.PI / 2, yaw, 0, "YXZ"));
    try {
      // Prefer parent-relative APIs when available (stable for colliders attached to a rigid body).
      if (typeof this.spineCollider.setTranslationWrtParent === "function") {
        this.spineCollider.setTranslationWrtParent({ x: xOff, y: this._spineOffsetY, z: zOff });
      } else {
        this.spineCollider.setTranslation({ x: p.x + xOff, y: p.y + this._spineOffsetY, z: p.z + zOff });
      }
      if (typeof this.spineCollider.setRotationWrtParent === "function") {
        this.spineCollider.setRotationWrtParent({ x: q.x, y: q.y, z: q.z, w: q.w });
      } else {
        this.spineCollider.setRotation({ x: q.x, y: q.y, z: q.z, w: q.w });
      }
    } catch {
      // ignore physics updates if collider is unavailable this frame
    }
  }

  _loadGLB() {
    if (!this.avatarUrl) return;
    const loader = new GLTFLoader();
    const urls = Array.isArray(this.avatarUrl) ? this.avatarUrl : [this.avatarUrl];
    const tryLoad = (index) => {
      if (index >= urls.length) {
        console.log(`[AiAvatar] No avatar model found, using capsule fallback.`);
        return;
      }
      loader.load(
        urls[index],
        (gltf) => { this._applyGLB(gltf); },
        undefined,
        () => { tryLoad(index + 1); }
      );
    };
    tryLoad(0);
  }

  _applyGLB(gltf) {
    {
        // Replace fallback with GLB
        try {
          this.group.remove(this.fallback);
        } catch {}
        // Also hide the facing indicator since the model shows direction
        try {
          this._facing.visible = false;
        } catch {}

        this.model = gltf.scene;

        // Auto-fit the model to our agent dimensions
        const bbox = new THREE.Box3().setFromObject(this.model);
        const size = bbox.getSize(new THREE.Vector3());
        const center = bbox.getCenter(new THREE.Vector3());

        // Target height: roughly capsule height (halfHeight * 2 + radius * 2)
        const targetHeight = this.halfHeight * 2 + this.radius * 2;
        const scaleFactor = targetHeight / (size.y || 1);
        // Y-squash: the current GLB has no rig, so we can't pose it into a
        // proper crouch. Compress Y instead so the robot reads as a low-slung
        // quadruped rather than a tall upright one. Purely cosmetic — camera
        // POV (GO2_CAMERA_HEIGHT in engine.js) is the accurate signal.
        this.model.scale.set(scaleFactor, scaleFactor * 0.6, scaleFactor);

        // Re-center: the group origin is at the physics body center,
        // which sits at (halfHeight + radius) above the ground.
        // We need to offset the model down so its feet touch the ground.
        bbox.setFromObject(this.model);
        const newCenter = bbox.getCenter(new THREE.Vector3());
        const newMin = bbox.min;
        this.model.position.x -= newCenter.x;
        this.model.position.z -= newCenter.z;
        // Offset down by the body's center height so feet are on the floor
        this.model.position.y = -newMin.y - (this.halfHeight + this.radius);
        // Keep the model centered on the group origin so yaw rotation pivots
        // around the body's geometric center (between the legs), not the head.

        // The agent's forward convention is +Z (yaw=0 looks along +Z).
        // This robot model already faces +Z, so no rotation needed.

        // Create box collider matching the scaled and positioned model dimensions
        bbox.setFromObject(this.model);
        const finalSize = bbox.getSize(new THREE.Vector3());
        const finalCenter = bbox.getCenter(new THREE.Vector3());

        // Remove old box collider if it exists
        if (this.boxCollider) {
          this.rapierWorld.removeCollider(this.boxCollider, true);
          this.boxCollider = null;
        }

        // Create box collider centered on the model's actual bounding box
        const boxHalfExtents = {
          x: finalSize.x / 2,
          y: finalSize.y / 2,
          z: finalSize.z / 2
        };
        this.boxCollider = this.rapierWorld.createCollider(
          this.RAPIER.ColliderDesc.cuboid(boxHalfExtents.x, boxHalfExtents.y, boxHalfExtents.z)
            .setFriction(0.8)
            .setTranslation(finalCenter.x, finalCenter.y, finalCenter.z),
          this.body
        );

        console.log(`[AI] Created box collider: ${finalSize.x.toFixed(2)}x${finalSize.y.toFixed(2)}x${finalSize.z.toFixed(2)} at offset (${finalCenter.x.toFixed(2)}, ${finalCenter.y.toFixed(2)}, ${finalCenter.z.toFixed(2)})`);

        // Headless mode: skip visual rendering (save memory/GPU), keep collider
        if (this.headless) {
          console.log(`[AI] Headless mode: skipping visual model rendering`);
          // Don't add model to scene, don't set up animations
          // Model is loaded only to calculate box collider dimensions
          this.model = null; // Release reference to free memory
          return;
        }

        // Normal mode: add visual model
        // Real meshes receive shadows but don't cast (too expensive)
        this.model.traverse((m) => {
          if (m.isMesh) {
            m.castShadow = false;
            m.receiveShadow = true;
          }
        });

        // No shadow proxy box: avoid cube-like shadow blockers around the robot.

        this.group.add(this.model);

        if (gltf.animations?.length) {
          this.mixer = new THREE.AnimationMixer(this.model);
          this._actions = {};
          for (const clip of gltf.animations) {
            this._actions[clip.name] = this.mixer.clipAction(clip);
          }
          // Try common idle animation names
          const idle = this._actions["idle"] || this._actions["Idle"] || this._actions["Idle_A"];
          if (idle) idle.play();
          else this.mixer.clipAction(gltf.animations[0]).play();
        }

        console.log(`[AI] Loaded robot model: ${size.x.toFixed(2)}x${size.y.toFixed(2)}x${size.z.toFixed(2)}, scale=${scaleFactor.toFixed(3)}`);
    }
  }

  _setThought(text) {
    const ctx = this._labelCtx;
    if (!ctx) return;
    if (this.vlm?.showSpeechBubbleInScene === false) {
      ctx.clearRect(0, 0, this._labelCanvas.width, this._labelCanvas.height);
      this._labelTex.needsUpdate = true;
      this._labelSprite.visible = false;
      return;
    }
    if (!text) {
      ctx.clearRect(0, 0, this._labelCanvas.width, this._labelCanvas.height);
      this._labelSprite.scale.set(2.0, 1.0, 1);
      this._labelTex.needsUpdate = true;
      this._labelSprite.visible = false;
      return;
    }

    const bubbleX = 20;
    const bubbleY = 20;
    const bubbleW = 472;
    const bubbleH = 472;
    const padX = 24;
    const padY = 20;
    const maxTextH = bubbleH - padY * 2;

    // Keep canvas size fixed to avoid stale visual remnants from resizing.
    if (this._labelCanvas.width !== 512) this._labelCanvas.width = 512;
    if (this._labelCanvas.height !== 512) this._labelCanvas.height = 512;

    // Fit all text within fixed bubble by reducing font size as needed.
    let fontSize = 30;
    let lineHeight = 36;
    let lines = [];
    while (fontSize >= 12) {
      ctx.font = `bold ${fontSize}px ui-sans-serif, system-ui, -apple-system, Segoe UI, Roboto`;
      lineHeight = Math.round(fontSize * 1.18);
      lines = wrapTextLines(ctx, text, bubbleW - padX * 2);
      const usedH = lines.length * lineHeight;
      if (usedH <= maxTextH) break;
      fontSize -= 2;
    }

    ctx.clearRect(0, 0, this._labelCanvas.width, this._labelCanvas.height);
    this._labelSprite.visible = true;
    // bubble
    ctx.fillStyle = "rgba(0,0,0,0.65)";
    ctx.strokeStyle = "rgba(255,255,255,0.25)";
    roundRect(ctx, bubbleX, bubbleY, bubbleW, bubbleH, 18);
    ctx.fill();
    ctx.stroke();
    ctx.fillStyle = "rgba(255,255,255,0.92)";
    ctx.font = `bold ${fontSize}px ui-sans-serif, system-ui, -apple-system, Segoe UI, Roboto`;
    ctx.textBaseline = "top";
    const textX = bubbleX + padX;
    let textY = bubbleY + padY;
    for (const line of lines) {
      ctx.fillText(line, textX, textY);
      textY += lineHeight;
    }
    this._labelSprite.scale.set(2.0, 2.0, 1);
    this._labelTex.needsUpdate = true;
  }

  _extractBubbleTextFromModelOutput(parsed, raw) {
    const p = parsed && typeof parsed === "object" ? parsed : {};
    const obs =
      p.observation ||
      p.obs ||
      p.perception ||
      p.sceneObservation ||
      p.visualObservation ||
      p.params?.observation ||
      "";
    if (typeof obs === "string" && obs.trim()) return obs.trim();
    const action = typeof p.action === "string" ? p.action.trim() : "";
    if (action) return `Action: ${action}`;

    const rawText = typeof raw === "string" ? raw : "";
    if (rawText) {
      const m = rawText.match(/"observation"\s*:\s*"([^"]+)"/i);
      if (m?.[1]) return m[1];
    }
    return "";
  }

  _memoryKey() {
    return `sparkWorldAiMemory:${this.getWorldKey()}:${this.id}`;
  }

  _loadMemory() {
    try {
      const raw = localStorage.getItem(this._memoryKey());
      return raw ? JSON.parse(raw) : { seenTags: {} };
    } catch {
      return { seenTags: {} };
    }
  }

  _saveMemory() {
    try {
      localStorage.setItem(this._memoryKey(), JSON.stringify(this.memory));
    } catch {
      // ignore
    }
  }

  _rememberTag(tag) {
    if (!tag?.id) return;
    const now = Date.now();
    if (!this.memory.seenTags) this.memory.seenTags = {};
    const entry = this.memory.seenTags[tag.id] || {
      firstSeen: now,
      lastSeen: now,
      count: 0,
      title: tag.title || "",
      notes: tag.notes || "",
    };
    entry.lastSeen = now;
    entry.count = (entry.count || 0) + 1;
    entry.title = tag.title || entry.title;
    entry.notes = tag.notes || entry.notes;
    this.memory.seenTags[tag.id] = entry;
    // Throttle writes a bit
    if (!this._nextMemSave || now > this._nextMemSave) {
      this._nextMemSave = now + 1200;
      this._saveMemory();
    }
  }

  _publishGlobals() {
    if (typeof window === "undefined") return;
    if (!window.__aiAgentPositions) window.__aiAgentPositions = {};
    const [x, y, z] = this.getPosition();
    window.__aiAgentPositions[this.id] = { x, y, z };
  }
}

function safeParseJson(text) {
  try {
    return JSON.parse(text);
  } catch {
    // Attempt to extract the first JSON object.
    const s = String(text || "");
    const i = s.indexOf("{");
    const j = s.lastIndexOf("}");
    if (i !== -1 && j !== -1 && j > i) {
      try {
        return JSON.parse(s.slice(i, j + 1));
      } catch {}
    }
    return null;
  }
}

function roundRect(ctx, x, y, w, h, r) {
  const rr = Math.min(r, w / 2, h / 2);
  ctx.beginPath();
  ctx.moveTo(x + rr, y);
  ctx.arcTo(x + w, y, x + w, y + h, rr);
  ctx.arcTo(x + w, y + h, x, y + h, rr);
  ctx.arcTo(x, y + h, x, y, rr);
  ctx.arcTo(x, y, x + w, y, rr);
  ctx.closePath();
}

function wrapTextLines(ctx, text, maxWidth) {
  const words = String(text).replace(/\s+/g, " ").trim().split(" ");
  const lines = [];
  let line = "";
  for (const rawWord of words) {
    const word = rawWord || "";
    const test = line ? `${line} ${word}` : word;
    if (ctx.measureText(test).width <= maxWidth || !line) {
      // If single word is too long, hard-break it.
      if (!line && ctx.measureText(word).width > maxWidth) {
        let chunk = "";
        for (const ch of word) {
          const next = chunk + ch;
          if (ctx.measureText(next).width > maxWidth && chunk) {
            lines.push(chunk);
            chunk = ch;
          } else {
            chunk = next;
          }
        }
        line = chunk;
      } else {
        line = test;
      }
    } else {
      lines.push(line);
      line = word;
    }
  }
  if (line) lines.push(line);
  return lines.length ? lines : [""];
}
