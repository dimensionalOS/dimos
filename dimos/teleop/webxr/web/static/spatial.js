// Copyright 2026 Dimensional Inc. Licensed under the Apache License, Version 2.0.
import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {PinchGrab, ToolPinch} from './spatial_input.mjs';
import {decodeSurface, SurfaceGrid} from './spatial_surface.mjs';
import {SpatialUI} from './spatial_ui.mjs';
import {WorkspaceTransform, workspaceHands} from './spatial_workspace.mjs';
import {VoxelMap} from './spatial_voxels.mjs';
import {interactionMode, sampleXRInputs, xrSessionOptions} from './xr_input.mjs';
import {SpatialOperator} from './spatial_operator.mjs';
import {KeyboardClutch} from './spatial_keyboard.mjs';
import {BodyClutchFrame} from './body_clutch.mjs';
import {BodyFollow} from './body_follow.mjs';
import {captureBody} from './webxr_body.mjs';
import {HandFollowSession} from './hand_follow.mjs';
import {RobotEyesView} from './spatial_view.mjs';

const embedded = new URLSearchParams(location.search).has('embedded');
const boundSession = new URLSearchParams(location.search).get('operator_session');
const desktop = embedded || new URLSearchParams(location.search).has('desktop');
if (embedded) document.body.classList.add('embedded');
const apiBase = new URL('./spatial/', location.href);
const endpoint = path => new URL(path, apiBase).href;
const status = document.querySelector('#status');
const enter = document.querySelector('#enter');
const renderer = new THREE.WebGLRenderer({antialias: true, alpha: true});
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.setSize(innerWidth, innerHeight);
renderer.xr.enabled = !desktop;
renderer.xr.setReferenceSpaceType('local-floor');
document.body.prepend(renderer.domElement);
const scene = new THREE.Scene();
scene.background = new THREE.Color('#11181d');
scene.add(new THREE.HemisphereLight(0xffffff, 0x536976, 2));
const light = new THREE.DirectionalLight(0xffffff, 2);
light.position.set(1, 3, 2); scene.add(light);
const camera = new THREE.PerspectiveCamera(55, innerWidth / innerHeight, 0.03, 15);
camera.position.set(1.2, 1.6, 1.5);
const orbit = new OrbitControls(camera, renderer.domElement);
orbit.target.set(0, 1, -0.4); orbit.update();

// World X forward / Y left / Z up -> XR X right / Y up / Z back.
// Scene selection uses this transform. Body-relative clutch input bypasses it.
const basis = new THREE.Matrix4().set(0,-1,0,0, 0,0,1,0, -1,0,0,0, 0,0,0,1);
const workspace = new WorkspaceTransform();
const stage = new THREE.Group(); stage.matrixAutoUpdate = false;
scene.add(stage); stage.matrix.copy(basis);
const inverseStage = new THREE.Matrix4();
const body = new THREE.Group(); stage.add(body);
const arms = {}, handles = {}, gestures = {}, handDots = {}, openingControls = {};
const clutches = {}, sources = {}, rays = {}, menuPinches = {};
const colours = {left: 0x76e2c1, right: 0xfab972};
for (const side of ['left', 'right']) {
  arms[side] = new THREE.Group(); stage.add(arms[side]);
  const handle = new THREE.Mesh(new THREE.SphereGeometry(0.035, 16, 12),
    new THREE.MeshBasicMaterial({color: colours[side], transparent: true, opacity: 0.7, wireframe: true}));
  handles[side] = handle; stage.add(handle); handle.visible = false;
  gestures[side] = new ToolPinch();
  clutches[side] = new PinchGrab(); menuPinches[side]=new PinchGrab();
  rays[side]=new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(),new THREE.Vector3()]),
    new THREE.LineBasicMaterial({color:colours[side],transparent:true,opacity:0.6}));
  rays[side].visible=false; scene.add(rays[side]);
  const control = new THREE.Group(); stage.add(control); control.visible = false;
  const material = new THREE.MeshBasicMaterial({color: colours[side], transparent:true, opacity:0.65});
  const rail = new THREE.Mesh(new THREE.BoxGeometry(1,0.003,0.003), material);
  control.add(rail);
  const measured = new THREE.Mesh(new THREE.SphereGeometry(0.012,16,12),material);
  const requested = new THREE.Mesh(new THREE.SphereGeometry(0.018,16,12),
    new THREE.MeshBasicMaterial({color:0xffffff, wireframe:true}));
  control.add(measured,requested); requested.visible=false;
  const canvas = document.createElement('canvas'); canvas.width=256; canvas.height=64;
  const texture = new THREE.CanvasTexture(canvas); texture.colorSpace=THREE.SRGBColorSpace;
  const label = new THREE.Sprite(new THREE.SpriteMaterial({map:texture,depthTest:false}));
  label.scale.set(0.075,0.01875,1); label.position.set(0,0.025,0); control.add(label);
  openingControls[side] = {group:control,rail,measured,requested,canvas,texture,label,lastText:''};
  handDots[side] = [];
  for (let i = 0; i < 25; i++) {
    const dot = new THREE.Mesh(new THREE.SphereGeometry(0.005, 8, 6), new THREE.MeshBasicMaterial({color: colours[side]}));
    dot.visible = false; scene.add(dot); handDots[side].push(dot);
  }
}
let showSurface=true;
const mapCloud=new VoxelMap(); stage.add(mapCloud);
const detections = new THREE.Group(); stage.add(detections);
const focusMarker = new THREE.Mesh(new THREE.SphereGeometry(0.018, 12, 8), new THREE.MeshBasicMaterial({color: 0xffffff, wireframe: true}));
focusMarker.visible = false; stage.add(focusMarker);
let feedback = null, feedbackAt = 0, sequence = 0, socket = null;
let placeAhead = true, lastSend = 0, stopped = false, controlRequested = false;
let entryError = '';
let paused = false, inputGeneration=0, controlEpoch=null;
let modePreference='auto', activeMode='tools';
let driveAxes=[0,0,0], torsoAxis=0;
let viewMode='workspace',latestViewer=null,lastRawHands={},nativeHandSelection=false,menuButtonHeld=false;
let savedDesktopView=null;
const robotEyes=new RobotEyesView();
const handFollow=new HandFollowSession();
let desktopFramed=false, selectedArm='left';
let keyboardEnabled=false, keyboardSide='left', keyboardFrame='world', lastKeyboardSend=0;
let objectsTS=0, selectedObject=null;
const keyboard=new KeyboardClutch();
const bodyClutch=new BodyClutchFrame();
const bodyFollow=new BodyFollow();
let calibrateBodyAt=0;
function requestBodyCalibration() {
  if(!renderer.xr.isPresenting) {entryError='Enter XR before calibrating your body.';return;}
  handFollow.hold(); bodyFollow.reset(); calibrateBodyAt=performance.now()+3000; resetHands(); send({});
}
const ui = new SpatialUI(stage,endpoint,{embedded,xrScene:scene,onAction:(action,state)=>{
  if(action==='follow_hands') { if(handFollow.enabled)handFollow.hold();else handFollow.enable();resetTools();send({});return; }
  if(action==='menu_open') {handFollow.hold();resetHands();send({});return;}
  if(action.startsWith('view:')) {setViewMode(action.slice(5));return;}
  if(action==='calibrate_body') { requestBodyCalibration(); return; }
  if(action==='toggle_map') { mapCloud.visible=!mapCloud.visible; return; }
  if(action==='hold'||action==='resume') {bodyClutch.reset();handFollow.hold();}
  if(state) { feedback=state; paused=!!state.control?.paused; }
  if(action==='actual_size') { workspace.resize(1,workspacePivot().applyMatrix4(stage.matrixWorld).toArray()); applyPlacement(); }
  if(action==='scale_down'||action==='scale_up') {
    workspace.resize(workspace.scale*(action==='scale_up'?1.25:0.8),workspacePivot().applyMatrix4(stage.matrixWorld).toArray()); applyPlacement();
  }
  if(action==='bring_here') placeAhead=true;
  if(action.startsWith('mode:')) setInteractionMode(action.slice(5));
  if(action==='exit_xr') renderer.xr.getSession()?.end().catch(error=>{entryError=error.message;});
  resetHands(); send({});
}});
const uiPointer=new THREE.Mesh(new THREE.SphereGeometry(0.007,8,6),new THREE.MeshBasicMaterial({color:0xffffff,depthTest:false}));
scene.add(uiPointer); uiPointer.visible=false;
const geometry = await fetch(endpoint('model')).then(check).then(r => r.json()).catch(error => {
  status.textContent = `Cannot load station: ${error.message}. Reload after the blueprint is running.`;
  throw error;
});
if(geometry.absolute_body_targets) {
  modePreference='clutch'; activeMode='clutch';
  const choice=document.querySelector('#interaction-mode'); choice.value='clutch'; choice.disabled=true;
  choice.querySelector('[value="clutch"]').textContent='Body follow';
  document.querySelector('#body-calibration').hidden=false;
  document.querySelector('#tool-help').hidden=true;
  document.querySelector('#control-help').textContent='Calibrate with arms forward, palms down and hands open. Choose Follow hands: open hands open the jaws, fists close them. Raise your left palm and point/pinch Controls to hold and open the menu. Controllers: both grips enable arms, triggers close jaws, Y opens controls.';
}
const meshes = {};
if(!geometry.finger_geometry) {
  const note=document.querySelector('#model-note'); note.hidden=false;
  note.textContent='Measured jaw opening appears on each tool. Fingertips are visible through the depth camera.';
}
for (const [id, mesh] of Object.entries(geometry.meshes)) {
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(mesh.vertices, 3));
  g.setIndex(mesh.faces); g.computeVertexNormals(); g.computeBoundingBox(); meshes[id] = g;
}
for (const geom of geometry.body || []) {
 const mesh = new THREE.Mesh(meshes[geom.mesh],new THREE.MeshStandardMaterial({color:new THREE.Color(...geom.rgba.slice(0,3)),roughness:0.65}));
 mesh.matrixAutoUpdate=false; body.add(mesh);
}
const surfaces = Object.fromEntries(Object.keys(geometry.cameras).map(slot => {
  const mesh = new THREE.Mesh(new THREE.BufferGeometry(), new THREE.MeshBasicMaterial({side:THREE.DoubleSide}));
  mesh.matrixAutoUpdate = false; mesh.visible = false; stage.add(mesh);
  return [slot, {mesh, grid:null, ts:0, received:0}];
}));
ui.configureCameras(geometry.cameras || {});
document.querySelector('h1').textContent=geometry.title || 'Robot workspace';
if(geometry.body_relative_clutch) document.querySelector('#place').textContent='Place workspace';
for (const side of geometry.sides) {
  for (const geom of geometry.arms[side]) {
    const mesh = new THREE.Mesh(meshes[geom.mesh], new THREE.MeshStandardMaterial({color: new THREE.Color(...geom.rgba.slice(0,3)), roughness: 0.65}));
    mesh.matrixAutoUpdate = false; arms[side].add(mesh);
  }
  arms[side].visible = false;
}
const operator=embedded?new SpatialOperator({scene,stage,camera,renderer,orbit,arms,endpoint,
  onTarget:pose=>parent.postMessage({type:'dimos.spatial.target',value:pose},location.origin)}):null;

function check(response) { if (!response.ok) throw new Error(`Station HTTP ${response.status}`); return response; }
async function poll(path, interval, accept) {
  while (!stopped) {
    try { await accept(await fetch(path, {cache: 'no-store'}).then(check)); }
    catch (error) { console.warn(path, error.message); }
    await new Promise(resolve => setTimeout(resolve, interval));
  }
}
function connect() {
  if (!controlRequested) return;
  const url=embedded?new URL(endpoint('ws')):new URL('/ws',location.href);
  url.protocol=url.protocol==='https:'?'wss:':'ws:';
  socket = new WebSocket(url.href);
  socket.onopen = () => { handFollow.hold(); sequence = 0; controlEpoch=null; entryError=''; bodyClutch.reset(); resetHands(); };
  socket.onclose = event => { handFollow.hold(); resetHands(); if(event.code===1008) entryError='Another operator currently owns control.'; if (!stopped && controlRequested) setTimeout(connect, 1500); };
}
poll(endpoint('state'), 35, async response => {
  const next = await response.json();
  if(boundSession && next.control?.session !== boundSession) {
    stopped=true; controlRequested=false; controlEpoch=null; socket?.close();
    document.body.replaceChildren(Object.assign(document.createElement('p'), {
      textContent:'This blueprint session ended. Reopen the current workspace.'
    }));
    throw new Error('Spatial session changed');
  }
  if(!Number.isInteger(next.control?.epoch) || !next.control.modes?.includes('clutch')) {
    entryError='Station controls were updated. Restart the blueprint and reload this page.';
    throw new Error(entryError);
  }
  feedback = next; feedbackAt = performance.now(); paused=!!next.control.paused;
  if(controlEpoch!==feedback.control.epoch) {
    bodyClutch.reset(); handFollow.hold();
    resetTools(); keyboard.reset(); controlEpoch=feedback.control.epoch;
  }
  (feedback.body || []).forEach((matrix,i)=>body.children[i]?.matrix.fromArray(matrix));
  for (const side of geometry.sides) {
    const arm = feedback.arms?.[side]; if (!arm) continue;
    arm.transforms.forEach((matrix, i) => arms[side].children[i].matrix.fromArray(matrix));
    handles[side].position.fromArray(arm.grasp);
    const c=openingControls[side], q=new THREE.Quaternion().fromArray(arm.orientation);
    c.group.position.copy(handles[side].position).add(new THREE.Vector3().fromArray(feedback.opening_handle_offset).applyQuaternion(q));
    c.group.quaternion.copy(q);
    c.rail.scale.x=feedback.opening_handle_travel_m;
    c.measured.position.x=(arm.opening-0.5)*feedback.opening_handle_travel_m;
    c.label.position.x=c.measured.position.x;
    const text=`${Math.round(arm.opening*100)}%`;
    if(text!==c.lastText) {
      c.lastText=text; const ctx=c.canvas.getContext('2d'); ctx.clearRect(0,0,256,64);
      ctx.fillStyle=side==='left'?'#76e2c1':'#fab972'; ctx.font='bold 44px system-ui';
      ctx.strokeStyle='#10202b'; ctx.lineWidth=6; ctx.lineJoin='round'; ctx.textAlign='center';
      ctx.strokeText(text,128,48); ctx.fillText(text,128,48);
      c.texture.needsUpdate=true;
    }
  }
  document.querySelector('#perception').textContent = feedback.objects_ts
    ? `${feedback.objects.length} observed objects · last scan ${Math.max(0,Math.round(feedback.server_time-feedback.objects_ts))} s ago`
    : 'No object scan yet.';
  if(objectsTS!==feedback.objects_ts){
    objectsTS=feedback.objects_ts;
    for (const child of [...detections.children]) { child.geometry.dispose(); child.material.dispose(); detections.remove(child); }
    for (const object of feedback.objects) {
      const solid=new THREE.BoxGeometry(...object.size), edges=new THREE.EdgesGeometry(solid);solid.dispose();
      const box = new THREE.LineSegments(edges, new THREE.LineBasicMaterial({color:object.id===selectedObject?0xffcf70:0x88ceff}));
      edges.computeBoundingBox();
      box.userData.id=object.id;
      box.position.fromArray(object.position); box.quaternion.fromArray(object.orientation); detections.add(box);
    }
  }
});
// Each request asks only for a newer acquisition. One in flight; no frame queue.
async function pollSurface(slot, surface) {
  const cloud=surface.mesh;
  let surfaceGrid=null;
  while(!stopped) {
    try {
      const response=check(await fetch(endpoint(`surface?camera=${encodeURIComponent(slot)}&after=${surface.ts}`),{cache:'no-store'}));
      if(response.status!==204) {
        const {header:h,depth,jpeg}=decodeSurface(await response.arrayBuffer());
        const bitmap=await createImageBitmap(new Blob([jpeg],{type:'image/jpeg'}),{imageOrientation:'flipY'});
        if(!surfaceGrid || surfaceGrid.width!==h.width || surfaceGrid.height!==h.height) {
          surfaceGrid=new SurfaceGrid(h.width,h.height);
          cloud.geometry.dispose(); cloud.geometry=new THREE.BufferGeometry();
          cloud.geometry.setAttribute('position',new THREE.BufferAttribute(surfaceGrid.positions,3).setUsage(THREE.DynamicDrawUsage));
          cloud.geometry.setAttribute('uv',new THREE.BufferAttribute(surfaceGrid.uv,2));
          cloud.geometry.setIndex(new THREE.BufferAttribute(surfaceGrid.indices,1).setUsage(THREE.DynamicDrawUsage));
        }
        surfaceGrid.update(h,depth);
        for(const attribute of Object.values(cloud.geometry.attributes)) attribute.needsUpdate=true;
        cloud.geometry.index.needsUpdate=true; cloud.geometry.setDrawRange(0,surfaceGrid.count);
        cloud.geometry.computeBoundingSphere(); cloud.matrix.fromArray(h.world_from_optical);
        if(!cloud.material.map) {
          cloud.material.map=new THREE.Texture(); cloud.material.map.colorSpace=THREE.SRGBColorSpace;
          cloud.material.map.generateMipmaps=false; cloud.material.map.minFilter=THREE.LinearFilter;
          cloud.material.needsUpdate=true;
        }
        cloud.material.map.image?.close(); cloud.material.map.image=bitmap; cloud.material.map.needsUpdate=true;
        surface.ts=h.ts; surface.received=performance.now(); surface.grid=surfaceGrid;
      }
    } catch(error) { console.warn('RGB-D:',error.message); }
    await new Promise(resolve=>setTimeout(resolve,25));
  }
}
for (const [slot, surface] of Object.entries(surfaces)) pollSurface(slot, surface);
let mapStamp=0;
poll(endpoint('map'),1000,async response=>{if(response.status===204)return;const stamp=Number(response.headers.get('X-Frame-Timestamp'));if(stamp===mapStamp)return;mapCloud.update(new Float32Array(await response.arrayBuffer()),geometry.map_voxel_size_m??0.05);mapStamp=stamp;});
function resetTools() {
  inputGeneration++;
  for (const state of [...Object.values(gestures),...Object.values(clutches),...Object.values(menuPinches)]) state.reset();
}
function resetHands() {
  torsoAxis=0;
  driveAxes=[0,0,0];
  resetTools(); workspace.cancel(); ui.hidePalm(); uiPointer.visible=false;
  for(const side of ['left','right']) {
    rays[side].visible=false;
    for(const dot of handDots[side]) dot.visible=false;
  }
}
function setInteractionMode(preference) {
  modePreference=preference; document.querySelector('#interaction-mode').value=preference;
  if(preference!=='auto') activeMode=preference;
  resetHands(); send({});
}
document.querySelector('#interaction-mode').onchange=event=>setInteractionMode(event.target.value);
function workspacePivot() {
  const known=Object.values(feedback?.arms??{});
  return known.reduce((center,arm)=>center.add(new THREE.Vector3().fromArray(arm.grasp)),new THREE.Vector3())
    .multiplyScalar(known.length?1/known.length:0);
}
function setViewMode(mode) {
  if(mode==='ego' && (!geometry.egocentric_available || !feedback?.eye_pose)) {entryError='Waiting for the robot head pose.';return;}
  if(mode===viewMode)return;
  handFollow.hold();resetHands();send({});
  if(mode==='ego') {
    savedDesktopView={position:camera.position.clone(),quaternion:camera.quaternion.clone(),target:orbit.target.clone()};
    if(latestViewer)robotEyes.enter(latestViewer);
  }else if(savedDesktopView) {
    camera.position.copy(savedDesktopView.position);camera.quaternion.copy(savedDesktopView.quaternion);orbit.target.copy(savedDesktopView.target);
  }
  viewMode=mode;orbit.enabled=mode==='workspace'&&!renderer.xr.isPresenting;applyPlacement();
}
function applyPlacement() {
  if(viewMode==='ego') {
    if(renderer.xr.isPresenting && robotEyes.anchor && feedback?.eye_pose)stage.matrix.copy(robotEyes.placement(feedback.eye_pose));
    else if(!renderer.xr.isPresenting)stage.matrix.copy(basis);
    stage.updateMatrixWorld(true);return;
  }
  stage.matrix.makeRotationY(workspace.yaw).multiply(basis)
    .scale(new THREE.Vector3().setScalar(workspace.scale)).setPosition(...workspace.position);
  stage.updateMatrixWorld(true);
}
function surfaceFresh(slot, surface) {
  const timeout=(feedback?.observation_timeout_s || 0.5)*1000;
  return surface.ts>0 && performance.now()-surface.received<timeout
    && feedback?.surfaces?.[slot]?.age*1000<timeout;
}
function observationsFresh() {
  const now=feedback?.server_time+(performance.now()-feedbackAt)/1000;
  return feedback?.arms && performance.now()-feedbackAt < 350
    && now-feedback.ts < feedback.observation_timeout_s
    && Object.entries(surfaces).some(([slot,surface])=>surfaceFresh(slot,surface));
}
function place(viewer) {
  if(viewMode==='ego'){placeAhead=false;return;}
  const position = new THREE.Vector3().copy(viewer.transform.position);
  const heading = new THREE.Vector3(0,0,-1).applyQuaternion(new THREE.Quaternion().copy(viewer.transform.orientation));
  heading.y = 0; if (heading.length() < 0.1) return;
  heading.normalize();
  const yaw = Math.atan2(-heading.x, -heading.z);
  const target=new THREE.Vector3(position.x+heading.x*0.5,Math.max(0.3,position.y-0.4),position.z+heading.z*0.5);
  const offset=workspacePivot().applyMatrix4(new THREE.Matrix4().makeRotationY(yaw).multiply(basis)
    .scale(new THREE.Vector3().setScalar(workspace.scale)));
  workspace.place(target.sub(offset).toArray(),yaw);
  applyPlacement(); resetHands(); send({}); placeAhead = false;
}
document.querySelector('#calibrate-body').onclick=requestBodyCalibration;
document.querySelector('#place').onclick = () => { placeAhead=true; resetHands(); send({}); };
enter.onclick = async () => {
  entryError = '';
  let session;
  try {
    session = await navigator.xr.requestSession(document.querySelector('#xr-mode').value, xrSessionOptions({bodyTracking:geometry.absolute_body_targets}));
    controlRequested = true; paused=false;
    scene.background=session.environmentBlendMode==='opaque'?new THREE.Color('#11181d'):null;
    renderer.setClearAlpha(scene.background?1:0);
    session.addEventListener('end', () => {
      handFollow.hold(); bodyFollow.reset(); calibrateBodyAt=0;
      controlRequested = false; resetHands(); send({}); socket?.close(); enter.disabled = false; orbit.enabled = true;
      document.querySelector('#xr-mode').disabled=false;
      scene.background=new THREE.Color('#11181d'); renderer.setClearAlpha(1);
    });
    session.addEventListener('visibilitychange', () => { if (session.visibilityState !== 'visible') { handFollow.hold(); resetHands(); send({}); } });
    nativeHandSelection=false;
    session.addEventListener('selectstart',event=>{
      if(event.inputSource.targetRayMode!=='transient-pointer')return;nativeHandSelection=true;
      if(nativeMenuHit(event)){handFollow.hold();resetHands();send({});}
    });
    session.addEventListener('select',event=>{if(event.inputSource.targetRayMode==='transient-pointer'){const hit=nativeMenuHit(event);if(hit)ui.activate(hit.action);}});
    await renderer.xr.setSession(session);
    renderer.xr.getReferenceSpace().addEventListener('reset',()=>{handFollow.hold();bodyFollow.reset();calibrateBodyAt=0;resetHands();robotEyes.anchor=null;send({});});
    handFollow.hold();robotEyes.anchor=null;connect(); placeAhead = true; orbit.enabled = false; enter.disabled = true;
    document.querySelector('#xr-mode').disabled=true;
  } catch (error) {
    controlRequested = false; socket?.close(); entryError = `Cannot enter WebXR: ${error.message}`;
    if(session) await session.end().catch(endError=>console.warn('Ending failed XR session:',endError.message));
  }
};
const xrMode=document.querySelector('#xr-mode');
const supportedModes=await Promise.all(['immersive-vr','immersive-ar'].map(async mode=>{
  try { return [mode,!desktop && !!navigator.xr && await navigator.xr.isSessionSupported(mode)]; }
  catch { return [mode,false]; }
}));
for(const [mode,supported] of supportedModes) xrMode.querySelector(`[value="${mode}"]`).disabled=!supported;
const firstMode=supportedModes.find(([,supported])=>supported)?.[0];
if(firstMode) { xrMode.value=firstMode; enter.disabled=false; }
else enter.textContent = 'WebXR unavailable in this browser';
document.querySelector('#entry-hint').textContent=supportedModes[1][1]
  ? 'Mixed reality shows passthrough. Exit XR to change VR / MR; HUD pins follow your view.'
  : 'This browser does not advertise mixed reality. Hand tracking is optional.';

function send(hands, focus = null) {
  if (socket?.readyState !== WebSocket.OPEN || controlEpoch===null) return;
  if (socket.bufferedAmount > 16384) { socket.close(); resetHands(); return; }
  const bodyInput=renderer.xr.isPresenting && activeMode==='clutch' && geometry.body_relative_clutch;
  socket.send(JSON.stringify({type: 'spatial_input', session: boundSession || feedback?.control?.session, mode: activeMode, sequence: sequence++, control_epoch: controlEpoch, observed_at: feedback?.ts || 0, hands, focus, drive: paused ? [0,0,0] : driveAxes,
    pose_frame:bodyInput?'body':'world',torso_height:paused?0:torsoAxis}));
}
function nativeMenuHit(event) {
  const pose=event.frame.getPose(event.inputSource.targetRaySpace,renderer.xr.getReferenceSpace());
  if(!pose)return null;
  const ray=new THREE.Raycaster(new THREE.Vector3().copy(pose.transform.position),
    new THREE.Vector3(0,0,-1).applyQuaternion(new THREE.Quaternion().copy(pose.transform.orientation)),0,3);
  return ui.hit(ray,event.inputSource.handedness);
}
function track(frame, reference, observationsFresh, viewer) {
  const raw=sampleXRInputs(frame,reference,{bodyFollow:geometry.absolute_body_targets}), samples={};
  lastRawHands=raw;
  const menuPressed=!!raw.left?.source?.gamepad?.buttons?.[5]?.pressed;
  if(menuPressed&&!menuButtonHeld)ui.activate('toggle_menu');menuButtonHeld=menuPressed;
  const axis=(side,index)=> {const value=raw[side]?.source?.gamepad?.axes?.[index]||0;return Math.abs(value)<0.15?0:Math.max(-1,Math.min(1,value));};
  driveAxes=geometry.base_drive?[-axis('left',3),-axis('left',2),-axis('right',2)]:[0,0,0];
  torsoAxis=geometry.torso_height_control?-axis('right',3):0;
  if(ui.open){driveAxes=[0,0,0];torsoAxis=0;}
  const mode=Object.keys(raw).length?interactionMode(modePreference,raw):activeMode;
  if(mode!==activeMode) { activeMode=mode; resetHands(); send({}); }
  const robotReady=feedback?.control?.robot_ready && feedback.control.mode===activeMode;
  const bodyInput=activeMode==='clutch' && geometry.body_relative_clutch;
  const viewerPose=viewer?{position:[viewer.transform.position.x,viewer.transform.position.y,viewer.transform.position.z],
    orientation:[viewer.transform.orientation.x,viewer.transform.orientation.y,viewer.transform.orientation.z,viewer.transform.orientation.w]}:null;
  const trackedBody=geometry.absolute_body_targets?captureBody(frame,reference):null;
  if(geometry.absolute_body_targets) {
    if(calibrateBodyAt && performance.now()>=calibrateBodyAt) {
      bodyFollow.calibrate(viewerPose,raw,trackedBody); calibrateBodyAt=0;
      resetHands(); send({});
    }
    document.querySelector('#body-calibration-status').textContent=(calibrateBodyAt?`Calibration in ${Math.ceil((calibrateBodyAt-performance.now())/1000)}s: release grips, extend both arms forward.`:bodyFollow.error) ||
      `Calibrated · ${bodyFollow.calibration?.source}. ${handFollow.reason}`;
  }
  if(bodyInput && !paused && robotReady && viewer) {
    const p=viewer.transform.position,q=viewer.transform.orientation;
    bodyClutch.calibrate([p.x,p.y,p.z],[q.x,q.y,q.z,q.w]);
  }
  ui.hovered=null; uiPointer.visible=false;
  for(const side of ['left','right']) {
    const sample=raw[side];
    for(const dot of handDots[side]) dot.visible=false;
    rays[side].visible=false;
    if(sources[side]!==sample?.source) {
      gestures[side].reset(); clutches[side].reset(); sources[side]=sample?.source;
    }
    if(!sample) continue;
    const position=new THREE.Vector3().fromArray(sample.position);
    const orientation=new THREE.Quaternion().fromArray(sample.orientation);
    const hand={...sample,position,orientation,pose:new THREE.Matrix4().compose(position,orientation,new THREE.Vector3(1,1,1))};
    if(sample.kind==='hand') {
      Object.values(sample.poses).forEach((pose,i)=>{
        const dot=handDots[side][i]; if(dot && pose) { dot.visible=true; dot.position.copy(pose.transform.position); }
      });
      hand.wrist=sample.poses.wrist;
      hand.indexPosition=new THREE.Vector3().copy(sample.poses['index-finger-tip'].transform.position);
      const knuckle=sample.poses['index-finger-metacarpal'];
      const direction=knuckle?hand.indexPosition.clone().sub(new THREE.Vector3().copy(knuckle.transform.position)):null;
      hand.ray=direction?.length()>0.01?new THREE.Raycaster(hand.indexPosition,direction.normalize(),0,3):null;
    } else hand.ray=sample.ray?new THREE.Raycaster(new THREE.Vector3().fromArray(sample.ray.position),
      new THREE.Vector3(0,0,-1).applyQuaternion(new THREE.Quaternion().fromArray(sample.ray.orientation)),0,3):null;
    samples[side]=hand;
  }
  stage.updateMatrixWorld(true);
  inverseStage.copy(stage.matrixWorld).invert();
  const owned=[...Object.values(gestures),...Object.values(clutches)].some(g=>g.grabbed);
  const now=performance.now();
  ui.updatePalm(samples,viewer?.transform.position,false,now);
  // Resolve every tool/UI target before considering a two-hand air grab.
  for(const [side,hand] of Object.entries(samples)) {
    hand.local=hand.position.clone().applyMatrix4(inverseStage);
    hand.uiHit=ui.hit(hand.ray,side,hand.position);
    if(hand.ray && (hand.uiHit || hand.kind==='controller')) {
      const points=rays[side].geometry.attributes.position;
      const end=hand.uiHit?.point??hand.ray.ray.at(1,new THREE.Vector3());
      points.setXYZ(0,...hand.ray.ray.origin.toArray()); points.setXYZ(1,...end.toArray());
      points.needsUpdate=true; rays[side].geometry.computeBoundingSphere(); rays[side].visible=true;
    }
    const targets=[];
    // Known tools keep their hit space even while held or observations lag.
    // Otherwise a pinch at a delayed tool can become a workspace gesture.
    for(const arm of activeMode==='tools'?Object.keys(feedback?.arms??{}):[]) {
      const c=openingControls[arm];
      const knob=c.measured.position.clone().applyQuaternion(c.group.quaternion).add(c.group.position);
      for(const [action,position,radius] of [...(geometry.groups?.[arm]?.gripper_joint?[['opening',knob,0.045]]:[]),['move',handles[arm].position,0.10]]) {
        const distance=hand.local.distanceTo(position);
        if(distance<radius) targets.push({arm,action,distance});
      }
    }
    hand.tool=targets.sort((a,b)=>a.distance-b.distance)[0]??null;
    hand.target=hand.tool?.action || (hand.uiHit?`ui:${hand.uiHit.action}`:null);
  }
  const arranging=viewMode==='workspace'&&!ui.open&&workspace.update(workspaceHands(Object.fromEntries(Object.entries(samples).map(([side,hand])=>[
    side,{...hand,position:hand.position.toArray()},
  ])),activeMode),workspacePivot().applyMatrix4(stage.matrixWorld).toArray(),now,owned);
  if(arranging) {
    driveAxes=[0,0,0]; torsoAxis=0; applyPlacement(); resetTools(); send({}); return;
  }
  for(const [side,hand] of Object.entries(samples)) {
    const click=menuPinches[side].update(hand.distance,!!hand.uiHit);
    if(hand.uiHit){ui.hovered=hand.uiHit.action;uiPointer.position.copy(hand.uiHit.point);uiPointer.visible=true;}
    if(click.pressed&&click.grab&&!(hand.kind==='hand'&&nativeHandSelection)) {
      handFollow.hold();ui.activate(hand.uiHit.action);
    }
  }
  const handsFollowing=handFollow.update(raw,{ready:robotReady&&!paused,calibrated:!!bodyFollow.calibration,blocked:ui.open});
  const inputs = {}; let focus = null;
  const generation=inputGeneration;
  // A control has one owner. Keep previous owners reserved through their release
  // frame so another held pinch cannot silently take over.
  const owners=new Map(Object.entries(gestures).filter(([,g])=>g.arm!==null)
    .map(([side,g])=>[`${g.arm}:${g.action}`,side]));
  // Uniform view scale affects positions, never the hand's orientation quaternion.
  const rotation = new THREE.Quaternion().setFromRotationMatrix(new THREE.Matrix4().extractRotation(inverseStage));
  for(const [side,hand] of Object.entries(samples)) {
    const {local:pinchPosition,ray,uiHit}=hand;
    const orientation=rotation.clone().multiply(hand.orientation);
    if(geometry.absolute_body_targets && hand.kind==='hand' && handsFollowing) {
      const pose=bodyFollow.pose(side,{position:hand.position.toArray(),orientation:hand.orientation.toArray()},viewerPose,trackedBody);
      if(!pose){handFollow.hold();send({});return;}
      inputs[side]={...pose,pinched:true,arm:side,action:'move',trigger:hand.trigger};continue;
    }
    if(activeMode==='clutch' && !(geometry.absolute_body_targets && hand.kind==='hand')) {
      const clutch=clutches[side].update(hand.clutchDistance,
        !ui.open && !paused && robotReady && (!bodyInput || (geometry.absolute_body_targets?bodyFollow.calibration!==null:bodyClutch.rotation!==null)) && !gestures[side].grabbed && !(hand.kind==='hand' && uiHit));
      if(clutch.grab) {
        gestures[side].reset();
        const pose=bodyInput?(geometry.absolute_body_targets?bodyFollow.pose(side,{position:hand.position.toArray(),orientation:hand.orientation.toArray()},viewerPose,trackedBody):bodyClutch.pose(hand.position.toArray(),hand.orientation.toArray()))
          :{position:pinchPosition.toArray(),orientation:orientation.toArray()};
        if(!pose) { clutches[side].reset(); continue; }
        inputs[side]={...pose,
          pinched:true,arm:side,action:'move',trigger:hand.trigger};
        continue;
      }
    }
    const state=gestures[side], armSide=state.arm??hand.tool?.arm;
    const arm=feedback?.arms?.[armSide], c=openingControls[armSide];
    const axis=new THREE.Vector3(1,0,0); if(c) axis.applyQuaternion(c.group.quaternion);
    const key=hand.tool?`${hand.tool.arm}:${hand.tool.action}`:null;
    const target=key && owners.has(key) && owners.get(key)!==side?null:hand.target;
    if(target?.startsWith('ui:')) { ui.hovered=uiHit.action; uiPointer.position.copy(uiHit.point); uiPointer.visible=true; }
    const selectable=hand.tool && (paused || !robotReady)?null:target;
    const gesture=state.update(hand.distance,selectable,
      pinchPosition.toArray(),axis.toArray(),arm?.opening??0,feedback?.opening_handle_travel_m??1,hand.tool?.arm??null);
    if(state.arm!==null) owners.set(`${state.arm}:${state.action}`,side);
    if(feedback?.arms && !paused) inputs[side] = {position: pinchPosition.toArray(), orientation: orientation.toArray(),
      pinched:activeMode==='tools'&&gesture.pinched, arm:state.arm, action:['move','opening'].includes(gesture.action)?gesture.action:null};
    if (activeMode==='tools' && observationsFresh && gesture.releasedFree && target===null) {
      if (ray) {
        const hit = ray.intersectObjects(Object.entries(surfaces).filter(([slot,surface])=>surfaceFresh(slot,surface)).map(([,surface])=>surface.mesh))[0];
        if (hit) { focus = hit.point.clone().applyMatrix4(inverseStage).toArray(); focusMarker.position.fromArray(focus); focusMarker.visible = true; }
      }
    }
  }
  send(generation===inputGeneration?inputs:{}, focus);
}
renderer.setAnimationLoop((time, frame) => {
  const immersive=renderer.xr.isPresenting;
  ui.setImmersive(immersive);
  body.visible=viewMode!=='ego';
  const fresh = !!observationsFresh();
  const ready = !!feedback?.control?.robot_ready && controlEpoch!==null;
  for (const side of ['left', 'right']) {
    const moving=Object.values(gestures).find(g=>g.arm===side && g.action==='move');
    const opening=Object.values(gestures).find(g=>g.arm===side && g.action==='opening');
    const known=!!feedback?.arms?.[side];
    arms[side].visible = known; handles[side].visible = immersive && known && activeMode==='tools';
    const c=openingControls[side]; c.group.visible=immersive && known && activeMode==='tools' && !!geometry.groups?.[side]?.gripper_joint;
    for(const mesh of arms[side].children) mesh.material.emissive.setHex(embedded && !immersive && side===selectedArm ? (side==='left'?0x14352b:0x392514) : 0);
    handles[side].material.opacity=workspace.active||paused||!ready?0.12:moving?1:0.45;
    c.rail.material.opacity=workspace.active?0.12:0.65; c.label.material.opacity=workspace.active?0.25:1;
    c.requested.visible=known && !!opening;
    if(c.requested.visible) c.requested.position.x=(opening.opening-0.5)*feedback.opening_handle_travel_m;
  }
  // Delayed observations change presentation, never an active tool's ownership.
  for (const [slot,surface] of Object.entries(surfaces)) {
    // A stale wrist surface must not linger as the arm changes pose.
    surface.mesh.visible=showSurface && surfaceFresh(slot,surface);
  }
  if(embedded && !desktopFramed && ready) {
    desktopFramed=true; frameWorkspace('fit');
    parent.postMessage({type:'dimos.spatial.ready'},location.origin);
  }
  detections.visible=fresh;
  if(!fresh) focusMarker.visible=false;
  const controlStatus=paused?'teleop paused':!controlRequested?'desktop controls':socket?.readyState!==WebSocket.OPEN?'control disconnected':feedback?.control?.mode!==activeMode?'changing control mode':feedback?.control?.hold_reason || (ready?'control connected':'waiting for robot joint feedback');
  const liveSurfaces=Object.entries(surfaces).filter(([slot,surface])=>surfaceFresh(slot,surface));
  const depthSamples=liveSurfaces.reduce((sum,[,surface])=>sum+(surface.grid?.valid || 0),0);
  const viewStatus=fresh?`Live · ${liveSurfaces.length}/${Object.keys(surfaces).length} RGB-D views · ${depthSamples} depth samples`:feedback?.observation_error || 'Waiting for fresh RGB-D observations';
  status.textContent = entryError || `${viewStatus} · ${controlStatus}`;
  ui.update({now:feedback?feedback.server_time+(performance.now()-feedbackAt)/1000:Date.now()/1000,
    ready,observationsFresh:fresh,holdReason:feedback?.control?.hold_reason,surfaceAvailable:Object.values(surfaces).some(surface=>surface.ts>0),connected:socket?.readyState===WebSocket.OPEN,paused:!!feedback?.control?.paused,actions:feedback?.actions,preview:!controlRequested,control:feedback?.control,
    workspaceScale:workspace.scale,workspaceActive:workspace.active,mode:activeMode,
    calibrated:!!bodyFollow.calibration,handsAvailable:['left','right'].every(side=>lastRawHands[side]?.kind==='hand'&&Number.isFinite(lastRawHands[side].trigger)),following:handFollow.enabled,viewMode,egoAvailable:geometry.egocentric_available&&!!feedback?.eye_pose,
    absoluteBody:geometry.absolute_body_targets,bodyStatus:geometry.absolute_body_targets?document.querySelector('#body-calibration-status').textContent:null,torsoHeightControl:geometry.torso_height_control,mapVisible:mapCloud.visible});
  if (frame) {
    const reference = renderer.xr.getReferenceSpace();
    const viewer = frame.getViewerPose(reference);
    if(viewer) {
      latestViewer={position:new THREE.Vector3().copy(viewer.transform.position).toArray(),orientation:new THREE.Quaternion().copy(viewer.transform.orientation).toArray()};
      ui.updateHead(latestViewer.position,latestViewer.orientation);
      if(viewMode==='ego' && !robotEyes.anchor)robotEyes.enter(latestViewer);
    }
    if(viewMode==='ego')applyPlacement();
    if (placeAhead && viewer) place(viewer);
    if (time-lastSend > 20) {
      lastSend = time;
      if (frame.session.visibilityState === 'visible') track(frame, reference, fresh, viewer);
      else { handFollow.hold();resetHands(); uiPointer.visible=false; send({}); }
    }
  } else {
    if(viewMode==='ego'&&feedback?.eye_pose) {
      applyPlacement();robotEyes.desktopCamera(stage.matrix,feedback.eye_pose).decompose(camera.position,camera.quaternion,new THREE.Vector3());
    }else orbit.update();
    if(keyboardEnabled && time-lastKeyboardSend>20) {
      const measured=feedback?.arms?.[keyboardSide];
      if(measured){
        const sample=keyboard.sample(measured,!paused && ready && feedback.control.mode==='clutch',time,(time-lastKeyboardSend)/1000);
        send({[keyboardSide]:{...sample,arm:keyboardSide,action:'move'}});
      }
      lastKeyboardSend=time;
    }
  }
  renderer.render(scene, camera);
});
addEventListener('resize', () => { camera.aspect = innerWidth/innerHeight; camera.updateProjectionMatrix(); renderer.setSize(innerWidth, innerHeight); });
addEventListener('pagehide', () => { stopped = true; resetHands(); send({}); socket?.close(); ui.dispose(); operator?.dispose(); mapCloud.dispose(); });

function frameWorkspace(view) {
  stage.updateMatrixWorld(true);
  const bounds=new THREE.Box3();
  bounds.union(new THREE.Box3().setFromObject(body));
  for(const arm of Object.values(arms)) if(arm.visible) bounds.union(new THREE.Box3().setFromObject(arm));
  if(bounds.isEmpty()) return;
  // Frame the known manipulation workspace; distant depth fragments must not
  // make the arms tiny. This changes the camera, not the sensor observations.
  bounds.expandByScalar(0.12);
  const center=bounds.getCenter(new THREE.Vector3());
  const size=bounds.getSize(new THREE.Vector3());
  const distance=Math.max(size.y,size.x/camera.aspect,size.z)*0.65/Math.tan(THREE.MathUtils.degToRad(camera.fov/2));
  const direction=new THREE.Vector3(...(view==='top'?[0,1,0.001]:view==='front'?[0,0.35,1]:[0.8,0.65,1])).normalize();
  orbit.target.copy(center); camera.position.copy(center).addScaledVector(direction,Math.max(0.7,distance)); orbit.update();
}
if(embedded) {
  addEventListener('message',event=>{
    if(event.source!==parent || event.origin!==location.origin) return;
    if(event.data?.type==='dimos.spatial.layer'){if(event.data.value.name==='surface')showSurface=event.data.value.visible;if(event.data.value.name==='map')mapCloud.visible=event.data.value.visible;}
    if(event.data?.type==='dimos.spatial.selection' && geometry.sides.includes(event.data.value)) selectedArm=event.data.value;
    if(event.data?.type==='dimos.spatial.view') {if(event.data.value==='ego')setViewMode('ego');else if(['fit','top','front'].includes(event.data.value)){setViewMode('workspace');frameWorkspace(event.data.value);}}
    if(event.data?.type==='dimos.spatial.target')operator?.setTarget(event.data.value);
    if(event.data?.type==='dimos.spatial.preview')void operator?.preview(event.data.value);
    if(event.data?.type==='dimos.spatial.object'){
      selectedObject=event.data.value;
      for(const box of detections.children)box.material.color.setHex(box.userData.id===selectedObject?0xffcf70:0x88ceff);
    }
    if(event.data?.type==='dimos.spatial.keyboard'){
      keyboardEnabled=!!event.data.value?.enabled;
      keyboardSide=event.data.value?.side??selectedArm;keyboardFrame=event.data.value?.frame??'world';
      keyboard.update([],keyboardFrame,performance.now());keyboard.reset();
      if(keyboardEnabled){activeMode='clutch';controlRequested=true;connect();}
      else {send({});controlRequested=false;socket?.close();}
    }
    if(event.data?.type==='dimos.spatial.keys' && keyboardEnabled)keyboard.update(event.data.value,keyboardFrame,performance.now());
  });
  let down=null;
  renderer.domElement.addEventListener('pointerdown',event=>{down=[event.clientX,event.clientY];});
  renderer.domElement.addEventListener('pointerup',event=>{
    if(operator?.controls.dragging || !down || Math.hypot(event.clientX-down[0],event.clientY-down[1])>5) {down=null;return;}
    down=null;
    const box=renderer.domElement.getBoundingClientRect();
    const ray=new THREE.Raycaster();
    ray.setFromCamera(new THREE.Vector2((event.clientX-box.left)/box.width*2-1,1-(event.clientY-box.top)/box.height*2),camera);
    const hit=ray.intersectObjects(Object.values(arms),true)[0];
    let objectHit=null, nearest=hit?.distance??Infinity;
    for(const box of detections.children) {
      const localRay=ray.ray.clone().applyMatrix4(box.matrixWorld.clone().invert());
      const point=localRay.intersectBox(box.geometry.boundingBox,new THREE.Vector3());
      if(!point) continue;
      const distance=point.applyMatrix4(box.matrixWorld).distanceTo(ray.ray.origin);
      if(distance<nearest){nearest=distance;objectHit=box.userData.id;}
    }
    if(objectHit){parent.postMessage({type:'dimos.spatial.selectObject',value:objectHit},location.origin);return;}
    const side=geometry.sides.find(side=>arms[side].children.includes(hit?.object));
    if(side) parent.postMessage({type:'dimos.spatial.select',value:side},location.origin);
  });
}
