// Copyright 2026 Dimensional Inc. Licensed under the Apache License, Version 2.0.
import * as THREE from 'three';
import {XRPanelLayout} from './xr_panels.mjs';

const panelRotation = new THREE.Matrix4().makeBasis(
  new THREE.Vector3(0,-1,0), new THREE.Vector3(0,0,1), new THREE.Vector3(-1,0,0));

const PALM_REVEAL_MS = 250;
const PALM_WIDTH = 0.24, PALM_HEIGHT = 0.075;

function panel(parent, width, height, position, pixels) {
  const canvas=document.createElement('canvas'); [canvas.width,canvas.height]=pixels;
  const texture=new THREE.CanvasTexture(canvas); texture.colorSpace=THREE.SRGBColorSpace;
  texture.minFilter=THREE.LinearFilter; texture.generateMipmaps=false;
  const mesh=new THREE.Mesh(new THREE.PlaneGeometry(width,height),new THREE.MeshBasicMaterial({map:texture,side:THREE.DoubleSide}));
  mesh.position.set(...position); mesh.quaternion.setFromRotationMatrix(panelRotation); parent.add(mesh);
  return {canvas,texture,mesh,context:canvas.getContext('2d')};
}

function wrapped(context, text, x, y, width, lineHeight) {
  let line='';
  for(const word of text.split(' ')) {
    if(line && context.measureText(`${line} ${word}`).width>width) {
      context.fillText(line,x,y); line=word; y+=lineHeight;
    } else line+=`${line?' ':''}${word}`;
  }
  context.fillText(line,x,y);
}

// Browser and XR share session actions; tracked arm input stays in spatial.js.
export class SpatialUI {
  constructor(parent, endpoint, {embedded, onAction, xrScene=parent}) {
    this.endpoint=endpoint; this.embedded=embedded; this.onAction=onAction;
    this.abort=new AbortController(); this.stopped=false; this.pending=null; this.version=0;
    this.collection=null; this.collectionAt=0; this.error=''; this.pollError=''; this.hovered=null; this.drawn='';
    this.cameras=new Map(); this.buttons=[];
    this.workspaceButtons=[];
    this.navigationButtons=[];
    this.layout=new XRPanelLayout(parent,xrScene);
    this.hud=panel(parent,0.84,0.4725,[0.82,0,1.1],[1024,576]);
    this.hud.mesh.material.depthTest=false; this.hud.mesh.material.depthWrite=false;
    this.hud.mesh.renderOrder=10;
    this.hud.id='session'; this.layout.add(this.hud.id,this.hud.mesh);
    this.outline=new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.BoxGeometry(1,1,1)),
      new THREE.LineBasicMaterial({color:0x76e2c1,transparent:true,opacity:0.8,depthTest:false,depthWrite:false}));
    this.outline.renderOrder=8; this.outline.visible=false; parent.add(this.outline);
    this.palm=panel(xrScene,PALM_WIDTH,PALM_HEIGHT,[0,0,0],[512,160]);
    this.palm.mesh.material.depthTest=false; this.palm.mesh.material.depthWrite=false;
    this.palm.mesh.renderOrder=20; this.palm.mesh.visible=false;
    this.palmCandidate=null; this.palmOwner=null;
    const c=this.palm.context;
    c.fillStyle='#10242f'; c.fillRect(0,0,512,160);
    c.strokeStyle='#76e2c1'; c.lineWidth=5; c.strokeRect(3,3,506,154);
    c.textAlign='center'; c.fillStyle='#d6fff1'; c.font='bold 32px system-ui';
    c.fillText('Bring workspace here',256,70);
    c.fillStyle='#b9cecf'; c.font='20px system-ui'; c.fillText('Point and pinch with your other hand',256,116);
    this.palm.texture.needsUpdate=true;
    if(!embedded) this.poll(()=>this.loadCollection(),500);
    for(const button of document.querySelectorAll('[data-action]')) {
      button.onclick=()=>this.activate(button.dataset.action);
    }
  }

  configureCameras(cameras) {
    const entries=Object.entries(cameras);
    entries.forEach(([name,title],i)=>{
      const view=panel(this.layout.parent || this.hud.mesh.parent,0.42,0.3465,[0.78,(i-(entries.length-1)/2)*0.48,0.68],[640,528]);
      Object.assign(view,{id:`camera-${name}`,name,title,ts:0,url:null,stale:true});this.cameras.set(name,view);
      this.layout.add(view.id,view.mesh);
      const figure=document.createElement('figure');
      const img=document.createElement('img');img.id=`${name}-camera`;img.alt=title;
      const caption=document.createElement('figcaption');caption.id=`${name}-caption`;caption.textContent=title;
      figure.append(img,caption);document.querySelector('#cameras').append(figure);this.cameraStatus(view,false);
      if(!this.embedded)this.poll(()=>this.loadCamera(view),150);
    });
  }

  async request(path, options={}) {
    const response=await fetch(this.endpoint(path),{cache:'no-store',signal:this.abort.signal,...options});
    if(!response.ok) {
      const data=await response.json().catch(()=>({}));
      throw new Error(typeof data.detail==='string'?data.detail:`Station HTTP ${response.status}`);
    }
    return response;
  }

  async poll(load, delay) {
    while(!this.stopped) {
      try { await load(); }
      catch(error) { if(!this.stopped) this.pollError=error.message; }
      if(!this.stopped) await new Promise(resolve=>setTimeout(resolve,delay));
    }
  }

  async loadCamera(view) {
    let response;
    try { response=await this.request(`camera/${view.name}`); }
    catch(error) { if(!this.stopped) this.cameraStatus(view,false); return; }
    const ts=Number(response.headers.get('X-Frame-Timestamp'));
    if(!Number.isFinite(ts) || ts<=view.ts) return;
    const blob=await response.blob(), bitmap=await createImageBitmap(blob);
    if(this.stopped) { bitmap.close(); return; }
    const c=view.context;
    c.fillStyle='#0c141b'; c.fillRect(0,48,640,480);
    const scale=Math.min(640/bitmap.width,480/bitmap.height);
    const w=bitmap.width*scale,h=bitmap.height*scale;
    c.drawImage(bitmap,(640-w)/2,48+(480-h)/2,w,h); bitmap.close();
    view.ts=ts; this.cameraStatus(view,true);
    if(!this.embedded) {
      const img=document.querySelector(`#${view.name}-camera`);
      const url=URL.createObjectURL(blob); img.src=url; img.hidden=false;
      if(view.url) URL.revokeObjectURL(view.url); view.url=url;
    }
  }

  cameraStatus(view, live) {
    const c=view.context; view.stale=!live;
    c.fillStyle='#13232d'; c.fillRect(0,0,640,48);
    c.fillStyle=view.name==='left'?'#76e2c1':view.name==='right'?'#fab972':'#c4dcf2';
    c.font='bold 23px system-ui'; c.textAlign='left'; c.fillText(view.title,18,32);
    const label=live?'Live':view.ts?'Delayed':'Unavailable';
    c.fillStyle=live?'#76e2c1':'#f3bf77'; c.textAlign='right'; c.fillText(label.toUpperCase(),510,32);
    this.drawPin(view);
    if(!live && !view.ts) {
      c.fillStyle='#0c141b'; c.fillRect(0,48,640,480);
      c.textAlign='center'; c.fillStyle='#9db0b8'; c.font='24px system-ui';
      c.fillText('Waiting for camera',320,290);
      document.querySelector(`#${view.name}-camera`).hidden=true;
    }
    document.querySelector(`#${view.name}-caption`).textContent=`${view.title} · ${label}`;
    view.texture.needsUpdate=true;
  }

  async loadCollection() {
    if(this.pending) return;
    const version=this.version;
    const state=await this.request('collection').then(r=>r.json());
    if(version===this.version && !this.pending) {
      this.collection=state; this.collectionAt=performance.now(); this.pollError='';
    }
  }

  async activate(action) {
    if(action.startsWith('pin:')) {
      const id=action.slice(4); this.layout.togglePin(id); this.drawn='';
      const view=[this.hud,...this.cameras.values()].find(view=>view.id===id);
      this.drawPin(view); return;
    }
    if(action==='bring_here') {
      this.onAction(action); this.hidePalm(); return;
    }
    if(![...this.buttons,...this.workspaceButtons,...this.navigationButtons].some(button=>button.action===action && button.enabled)) return;
    if(['actual_size','scale_down','scale_up','exit_xr'].includes(action) || action.startsWith('mode:')) { this.onAction(action); return; }
    if(this.pending && action!=='hold') return;
    this.pending=action; const version=++this.version; this.error='';
    try {
      if(['home','hold','resume'].includes(action)) {
        const result=await this.request('control',{
          method:'POST',headers:{'Content-Type':'application/json'},
          body:JSON.stringify({action,session:this.control.session,epoch:this.control.epoch}),
        }).then(r=>r.json());
        if(version===this.version) this.onAction(action,result);
      } else {
        const collection=await this.request('collection',{
        method:'POST',headers:{'Content-Type':'application/json'},
        body:JSON.stringify({action,session:this.control.session,task:document.querySelector('#task').value.trim()}),
        }).then(r=>r.json());
        if(version===this.version) { this.collection=collection; this.collectionAt=performance.now(); }
      }
    } catch(error) { if(version===this.version) this.error=error.message; }
    finally { if(version===this.version) this.pending=null; }
  }

  update({now,ready,observationsFresh=true,holdReason=null,surfaceAvailable=false,connected,paused,actions,preview,workspaceScale=1,workspaceActive=false,mode='tools',control={},torsoHeightControl=false}) {
    this.control=control;
    this.outline.visible=workspaceActive;
    this.workspaceButtons=[
      {action:'scale_down',label:'−',x:550,w:96},
      {action:'scale_up',label:'+',x:662,w:96},
      {action:'actual_size',label:'Actual size',x:780,w:220},
    ].map(button=>({...button,y:424,h:48,enabled:!preview&&!workspaceActive}));
    this.navigationButtons=[
      ['mode:tools','Tool handles'],['mode:clutch','Classic teleop'],['bring_here','Bring here'],['exit_xr','Exit XR'],
    ].map(([action,label],i)=>({action,label,x:24+i*250,y:512,w:232,h:48,enabled:!preview}));
    for(const view of this.cameras.values()) {
      if(!view.stale && now-view.ts>0.5) this.cameraStatus(view,false);
    }
    const state=this.collection;
    const available=state && performance.now()-this.collectionAt<2000;
    const recording=available && !!state.recording;
    const elapsed=recording?Math.max(0,Math.floor(now-state.started_at)):0;
    const timer=`${Math.floor(elapsed/60).toString().padStart(2,'0')}:${(elapsed%60).toString().padStart(2,'0')}`;
    this.buttons=[
      {action:'home',label:'Home',enabled:ready && available && !recording && !!control.session && control.home_available},
      {action:'record',label:'Record',enabled:ready && available && state.collection_ready && !recording && !!document.querySelector('#task').value.trim()},
      {action:'save',label:'Stop & save',enabled:recording},
      {action:'discard',label:'Discard',enabled:recording},
      {action:'hold',label:'Hold',enabled:!!control.session},
      {action:'resume',label:'Resume',enabled:!!control.session && paused && ready},
    ].filter(button=>!['record','save','discard'].includes(button.action)||state?.available!==false).map((button,i)=>({...button,x:24+i*166,y:336,w:152,h:68,enabled:button.enabled && (!this.pending || button.action==='hold')}));
    const controlLabel=!ready?'ROBOT FEEDBACK UNAVAILABLE':paused?'TELEOP PAUSED':preview?'DESKTOP · STATION CONTROLS':workspaceActive?'ARRANGING WORKSPACE · ARMS HELD':!connected?'XR INPUT DISCONNECTED':'XR INPUT CONNECTED';
    const take=this.pending?({record:'STARTING',save:'SAVING',discard:'DISCARDING',home:'PLANNING HOME',hold:'HOLDING',resume:'RESUMING'}[this.pending])
      :recording?`REC ${timer}`:!available?'RECORDER UNAVAILABLE':state.available===false?'NO RECORDER':state.last_event==='save'?'SAVED':state.last_event==='discard'?'DISCARDED':'READY';
    const hint=this.error || (paused&&holdReason) || (!observationsFresh&&surfaceAvailable?'View delayed · waiting for fresh sensor surfaces.':null) || this.pollError || state?.collection_error || (recording?state.task:state?.collection_waiting)
      || (torsoHeightControl?'Face forward on XR entry. Hold → Resume recalibrates. Right stick ↑/↓ changes height.':'Point at a button and pinch. Tool handles move only their own arm.');
    const arms=['left','right'].map(side=>{
      const owned=ready&&!paused&&!workspaceActive?actions?.[side]:null;
      return `${side.toUpperCase()} ${owned?.move&&owned?.opening?'MOVING + JAWS':owned?.move?'MOVING':owned?.opening?'JAWS':'HELD'}`;
    }).join('   ·   ');
    const summary=`${take} · ${state?.episodes_saved??0} saved · ${state?.episodes_discarded??0} discarded`;
    document.querySelector('#episode-state').textContent=summary;
    document.querySelector('#episode-hint').textContent=hint;
    document.querySelector('#task').disabled=recording || !!this.pending;
    for(const element of document.querySelectorAll('[data-action="record"],[data-action="save"],[data-action="discard"],#task,label[for="task"]'))element.hidden=state?.available===false;
    for(const button of this.buttons) {
      const element=document.querySelector(`[data-action="${button.action}"]`);
      element.disabled=!button.enabled; element.textContent=button.label;
    }
    const signature=JSON.stringify([controlLabel,summary,arms,hint,this.buttons,this.workspaceButtons,this.hovered,Math.round(workspaceScale*100),mode]);
    if(signature===this.drawn) return; this.drawn=signature;
    const c=this.hud.context;
    c.fillStyle='#13232d'; c.fillRect(0,0,1024,576); c.textAlign='left';
    c.fillStyle='#223a46'; c.fillRect(0,0,1024,48);
    c.fillStyle='#edf5f4'; c.font='bold 24px system-ui'; c.fillText('SESSION CONTROLS · drag this bar',24,33);
    this.drawPin(this.hud);
    c.textAlign='left';
    c.fillStyle=ready&&!paused?'#76e2c1':'#f3bf77'; c.font='bold 28px system-ui'; c.fillText(controlLabel,24,90);
    c.fillStyle=recording?'#ff8888':'#edf5f4'; c.font='bold 35px system-ui'; c.fillText(summary,24,145);
    c.fillStyle='#c4dcf2'; c.font='25px system-ui'; c.fillText(arms,24,192);
    c.fillStyle='#b9cecf'; c.font='24px system-ui'; wrapped(c,hint,24,239,976,32);
    c.font='21px system-ui'; c.fillStyle='#76e2c1';
    c.fillText(`${mode==='clutch'?'Classic teleop':'Tool handles'} · Workspace ${Math.round(workspaceScale*100)}%`,24,440);
    c.fillStyle='#b9cecf'; c.fillText(mode==='clutch'?'Grip: move arm · trigger: jaws · −/+ changes view only':'Pinch / trigger at a tool · free hand pinches arrange workspace',24,480);
    for(const button of [...this.buttons,...this.workspaceButtons,...this.navigationButtons]) {
      c.fillStyle=!button.enabled?'#25333e':button.action===this.hovered?'#c0f4e7':button.action==='discard'?'#f1b38f':'#76e2c1';
      c.fillRect(button.x,button.y,button.w,button.h);
      c.fillStyle=button.enabled?'#10202b':'#78909b'; c.font=`bold ${button.w<180?22:27}px system-ui`; c.textAlign='center';
      c.fillText(button.label,button.x+button.w/2,button.y+button.h/2+10);
    }
    this.hud.texture.needsUpdate=true;
  }

  hit(ray, side, point) {
    const palmAvailable=this.palm.mesh.visible && side!==this.palmOwner;
    if(palmAvailable && point) {
      const local=this.palm.mesh.worldToLocal(point.clone());
      if(Math.abs(local.x)<PALM_WIDTH/2 && Math.abs(local.y)<PALM_HEIGHT/2 && Math.abs(local.z)<0.05)
        return {action:'bring_here',point:point.clone()};
    }
    const views=[this.hud,...this.cameras.values()];
    const meshes=views.map(view=>view.mesh);
    const hit=ray?.intersectObjects(palmAvailable?[...meshes,this.palm.mesh]:meshes)[0];
    if(!hit) return null;
    if(hit.object===this.palm.mesh) return {action:'bring_here',point:hit.point};
    const view=views.find(view=>view.mesh===hit.object);
    const x=hit.uv.x*view.canvas.width,y=(1-hit.uv.y)*view.canvas.height;
    if(y<48) return {action:`${x>view.canvas.width-104?'pin':'drag'}:${view.id}`,point:hit.point};
    const button=view===this.hud?[...this.buttons,...this.workspaceButtons,...this.navigationButtons]
      .find(b=>b.enabled && x>=b.x && x<=b.x+b.w && y>=b.y && y<=b.y+b.h):null;
    return {action:button?.action??'panel_body',point:hit.point};
  }

  drawPin(view) {
    const c=view.context, width=view.canvas.width;
    c.fillStyle=this.layout.pinned(view.id)?'#76e2c1':'#344e5b'; c.fillRect(width-104,0,104,48);
    c.fillStyle=this.layout.pinned(view.id)?'#10202b':'#edf5f4'; c.textAlign='center'; c.font='bold 22px system-ui';
    c.fillText(this.layout.pinned(view.id)?'UNPIN':'HUD',width-52,32); view.texture.needsUpdate=true;
  }

  drag(action, side, pose, pressed) {
    if(!action?.startsWith('ui:drag:')) { this.layout.end(side); return; }
    if(pressed) this.layout.begin(action.slice(8),side,pose);
    this.layout.move(side,pose);
  }

  setWorkspaceBounds(robotBounds) {
    const bounds=robotBounds.clone();
    for(const view of [this.hud,...this.cameras.values()]) {
      if(!view.mesh.visible || view.mesh.parent!==this.layout.workspace) continue;
      view.mesh.updateMatrix(); view.mesh.geometry.computeBoundingBox();
      bounds.union(view.mesh.geometry.boundingBox.clone().applyMatrix4(view.mesh.matrix));
    }
    bounds.expandByScalar(0.04); bounds.getCenter(this.outline.position); bounds.getSize(this.outline.scale);
  }

  hidePalm() { this.palm.mesh.visible=false; this.palmOwner=null; this.palmCandidate=null; }

  recenter() {
    this.layout.recenter(); this.drawn='';
    for(const view of [this.hud,...this.cameras.values()]) this.drawPin(view);
  }

  setImmersive(active) {
    for(const view of [this.hud,...this.cameras.values()]) view.mesh.visible=active;
  }

  updatePalm(hands, eye, blocked, now) {
    if(blocked || !eye) { this.hidePalm(); return; }
    const candidates=[];
    for(const [side,hand] of Object.entries(hands)) {
      if(hand.kind!=='hand') continue;
      const middle=hand.poses['middle-finger-phalanx-proximal'];
      const tip=hand.poses['middle-finger-tip'];
      if(!middle || !tip || hand.distance<0.04) continue;
      const wrist=new THREE.Vector3().copy(hand.wrist.transform.position);
      const knuckle=new THREE.Vector3().copy(middle.transform.position);
      // WebXR joint -Y points out of the palm: https://www.w3.org/TR/webxr-hand-input-1/#xrjointspace
      const normal=new THREE.Vector3(0,-1,0).applyQuaternion(new THREE.Quaternion().copy(hand.wrist.transform.orientation));
      if(normal.y<0.65 || new THREE.Vector3().copy(tip.transform.position).distanceTo(wrist)<1.5*knuckle.distanceTo(wrist)) continue;
      candidates.push({side,position:wrist.add(knuckle).multiplyScalar(0.5).addScaledVector(normal,0.1)});
    }
    const palm=candidates.find(c=>c.side===this.palmCandidate?.side) || candidates[0];
    if(!palm) { this.hidePalm(); return; }
    if(this.palmCandidate?.side!==palm.side) this.palmCandidate={side:palm.side,since:now};
    this.palm.mesh.visible=now-this.palmCandidate.since>=PALM_REVEAL_MS;
    this.palmOwner=this.palm.mesh.visible?palm.side:null;
    this.palm.mesh.position.copy(palm.position); this.palm.mesh.lookAt(new THREE.Vector3().copy(eye));
    this.palm.mesh.updateMatrixWorld(true);
  }

  dispose() {
    this.stopped=true; this.abort.abort();
    this.layout.dispose();
    for(const view of [this.hud,this.palm,...this.cameras.values()]) {
      view.texture.dispose(); view.mesh.geometry.dispose(); view.mesh.material.dispose(); view.mesh.removeFromParent();
      if(view.url) URL.revokeObjectURL(view.url);
    }
    this.outline.geometry.dispose(); this.outline.material.dispose(); this.outline.removeFromParent();
    for(const button of document.querySelectorAll('[data-action]')) button.onclick=null;
  }
}
