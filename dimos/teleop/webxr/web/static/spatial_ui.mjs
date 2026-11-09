// Copyright 2026 Dimensional Inc. Licensed under the Apache License, Version 2.0.
import * as THREE from 'three';

function panel(parent,width,height,pixels) {
  const canvas=document.createElement('canvas');[canvas.width,canvas.height]=pixels;
  const texture=new THREE.CanvasTexture(canvas);texture.colorSpace=THREE.SRGBColorSpace;
  texture.minFilter=THREE.LinearFilter;texture.generateMipmaps=false;
  const mesh=new THREE.Mesh(new THREE.PlaneGeometry(width,height),new THREE.MeshBasicMaterial({map:texture,side:THREE.DoubleSide,depthTest:false,depthWrite:false}));
  mesh.renderOrder=20;parent.add(mesh);mesh.visible=false;
  return {canvas,texture,mesh,context:canvas.getContext('2d')};
}
function wrapped(c,text,x,y,width,lineHeight) {
  let line='';for(const word of text.split(' ')) {
    if(line && c.measureText(`${line} ${word}`).width>width) {c.fillText(line,x,y);line=word;y+=lineHeight;}
    else line+=`${line?' ':''}${word}`;
  }c.fillText(line,x,y);
}

// One fixed-size, world-anchored menu. No UI is parented to the scalable robot scene.
export class SpatialUI {
  constructor(parent,endpoint,{embedded,onAction,xrScene=parent}) {
    this.endpoint=endpoint;this.embedded=embedded;this.onAction=onAction;
    this.abort=new AbortController();this.stopped=false;this.pending=null;this.version=0;
    this.collection=null;this.collectionAt=0;this.error='';this.pollError='';this.hovered=null;
    this.cameras=new Map();this.buttons=[];this.control={};this.tab='control';this.selectedCamera='workspace';
    this.open=true;this.immersive=false;this.head=null;this.needsPlacement=true;
    this.menu=panel(xrScene,.72,.62,[1000,860]);
    this.palm=panel(xrScene,.22,.065,[512,150]);this.palmOwner=null;this.palmCandidate=null;
    const c=this.palm.context;c.fillStyle='#142a34';c.fillRect(0,0,512,150);
    c.fillStyle='#d9fff1';c.font='bold 38px system-ui';c.textAlign='center';c.fillText('Controls',256,67);
    c.font='23px system-ui';c.fillText('Point and pinch with the other hand',256,113);this.palm.texture.needsUpdate=true;
    this.lastDraw=0;
    if(!embedded)this.poll(()=>this.loadCollection(),500);
    for(const button of document.querySelectorAll('[data-action]'))button.onclick=()=>this.activate(button.dataset.action);
  }
  setOpen(open) {
    if(open && !this.open) {this.needsPlacement=true;this.onAction('menu_open');}
    this.open=open;this.menu.mesh.visible=this.immersive&&open;
    if(open)this.hidePalm();
  }
  updateHead(position,orientation) {
    this.head={position,orientation};if(this.needsPlacement)this.recenter();
  }
  recenter() {
    if(!this.head)return;
    const q=new THREE.Quaternion().fromArray(this.head.orientation);
    const forward=new THREE.Vector3(0,0,-1).applyQuaternion(q);forward.y=0;
    if(forward.length()<.1)return;forward.normalize();
    this.menu.mesh.position.fromArray(this.head.position).addScaledVector(forward,.95).add(new THREE.Vector3(0,-.13,0));
    this.menu.mesh.quaternion.setFromAxisAngle(new THREE.Vector3(0,1,0),Math.atan2(-forward.x,-forward.z));
    this.menu.mesh.updateMatrixWorld(true);this.needsPlacement=false;
  }
  setImmersive(active) {
    if(active!==this.immersive) {this.immersive=active;this.needsPlacement=true;this.open=true;}
    this.menu.mesh.visible=active&&this.open;if(!active)this.hidePalm();
  }
  configureCameras(cameras) {
    for(const [name,title] of Object.entries(cameras)) {
      const canvas=document.createElement('canvas');canvas.width=640;canvas.height=480;
      const view={name,title,canvas,context:canvas.getContext('2d'),ts:0,url:null,stale:true};this.cameras.set(name,view);
      const figure=document.createElement('figure');figure.dataset.cameraSlot=name;
      const img=document.createElement('img');img.id=`${name}-camera`;img.alt=title;
      const caption=document.createElement('figcaption');caption.id=`${name}-caption`;caption.textContent=title;
      figure.append(img,caption);document.querySelector('#cameras').append(figure);this.cameraStatus(view,false);
      if(!this.embedded)this.poll(async()=>{
        if(this.selectedCamera===name && (!this.immersive || (this.open&&this.tab==='cameras')))await this.loadCamera(view);
      },150);
    }
  }
  async request(path,options={}) {
    const response=await fetch(this.endpoint(path),{cache:'no-store',signal:this.abort.signal,...options});
    if(!response.ok){const data=await response.json().catch(()=>({}));throw new Error(typeof data.detail==='string'?data.detail:`Station HTTP ${response.status}`);}return response;
  }
  async poll(load,delay) {
    while(!this.stopped){try{await load();}catch(error){if(!this.stopped)this.pollError=error.message;}
      if(!this.stopped)await new Promise(resolve=>setTimeout(resolve,delay));}
  }
  async loadCamera(view) {
    let response;try{response=await this.request(`camera/${view.name}`);}catch{this.cameraStatus(view,false);return;}
    const ts=Number(response.headers.get('X-Frame-Timestamp'));if(!Number.isFinite(ts)||ts<=view.ts)return;
    const blob=await response.blob(),bitmap=await createImageBitmap(blob);
    if(this.stopped){bitmap.close();return;}
    const c=view.context;c.fillStyle='#0c141b';c.fillRect(0,0,640,480);
    const scale=Math.min(640/bitmap.width,480/bitmap.height),w=bitmap.width*scale,h=bitmap.height*scale;
    c.drawImage(bitmap,(640-w)/2,(480-h)/2,w,h);bitmap.close();view.ts=ts;this.cameraStatus(view,true);
    const img=document.querySelector(`#${view.name}-camera`);const url=URL.createObjectURL(blob);img.src=url;img.hidden=false;
    if(view.url)URL.revokeObjectURL(view.url);view.url=url;
  }
  cameraStatus(view,live) {
    view.stale=!live;document.querySelector(`#${view.name}-caption`).textContent=`${view.title} · ${live?'Live':view.ts?'Delayed':'Waiting'}`;
    document.querySelector(`#${view.name}-camera`).hidden=!view.ts;
  }
  async loadCollection() {
    if(this.pending)return;const version=this.version;const state=await this.request('collection').then(r=>r.json());
    if(version===this.version&&!this.pending){this.collection=state;this.collectionAt=performance.now();this.pollError='';}
  }
  async activate(action) {
    if(action==='summon_menu'){this.setOpen(true);this.needsPlacement=true;return;}
    if(action==='toggle_menu'){this.setOpen(!this.open);return;}
    if(action==='close_menu'){this.setOpen(false);return;}
    if(action.startsWith('tab:')){this.tab=action.slice(4);return;}
    if(action.startsWith('camera:')){this.selectedCamera=action.slice(7);return;}
    if(!this.actions?.some(b=>b.action===action&&b.enabled))return;
    if(['follow_hands','calibrate_body','actual_size','scale_down','scale_up','toggle_map','bring_here','exit_xr'].includes(action)||action.startsWith('view:')||action.startsWith('mode:')) {
      this.onAction(action);if(action==='follow_hands')this.setOpen(false);return;
    }
    if(this.pending&&action!=='hold')return;
    this.pending=action;const version=++this.version;this.error='';
    try {
      if(['home','hold','resume'].includes(action)) {
        const state=await this.request('control',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action,session:this.control.session,epoch:this.control.epoch})}).then(r=>r.json());
        if(version===this.version)this.onAction(action,state);
      }else{
        const state=await this.request('collection',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action,session:this.control.session,task:document.querySelector('#task').value.trim()})}).then(r=>r.json());
        if(version===this.version){this.collection=state;this.collectionAt=performance.now();}
      }
    }catch(error){if(version===this.version)this.error=error.message;}finally{if(version===this.version)this.pending=null;}
  }
  update({now,ready,paused,preview,control={},workspaceScale=1,mapVisible=true,absoluteBody=false,bodyStatus=null,calibrated=false,handsAvailable=false,following=false,viewMode='workspace',egoAvailable=false,holdReason=null}) {
    this.control=control;
    const state=this.collection,available=state&&performance.now()-this.collectionAt<2000,recording=available&&!!state.recording;
    const hint=this.error||(paused?holdReason:null)||bodyStatus||holdReason||'Raise your left palm to summon controls. The menu stays where you place it.';
    const actions=[
      {action:'hold',label:'Hold robot',enabled:!!control.session},
      {action:'resume',label:'Resume',enabled:!!control.session&&paused&&ready},
      {action:'home',label:'Home',enabled:ready&&!recording&&control.home_available},
      {action:'record',label:'Record',enabled:ready&&available&&state.collection_ready&&!recording},
      {action:'save',label:'Save',enabled:recording},
      {action:'discard',label:'Discard',enabled:recording},
    ];
    for(const b of actions){const el=document.querySelector(`[data-action="${b.action}"]`);if(el){el.disabled=!b.enabled;el.textContent=b.label;}}
    document.querySelector('#episode-state').textContent=recording?'Recording':state?.available===false?'':'Session';
    document.querySelector('#episode-hint').textContent=hint;
    for(const el of document.querySelectorAll('[data-action="record"],[data-action="save"],[data-action="discard"],#task,label[for="task"]'))el.hidden=state?.available===false;
    for(const [name,view] of this.cameras){if(now-view.ts>.5)view.stale=true;document.querySelector(`[data-camera-slot="${name}"]`).hidden=name!==this.selectedCamera;}
    const buttons=[];
    const add=(action,label,x,y,w=440,h=76,enabled=true)=>buttons.push({action,label,x,y,w,h,enabled});
    ['control','view','cameras',...(state?.available?['record']:[])].forEach((tab,i)=>add(`tab:${tab}`,tab[0].toUpperCase()+tab.slice(1),28+i*236,86,218,58));
    if(this.tab==='control') {
      add('follow_hands',following?'Hold hands':'Follow hands',28,240,944,90,!preview&&ready&&!paused&&absoluteBody&&calibrated&&handsAvailable);
      add('calibrate_body','Calibrate body',28,350,944,76,!preview&&absoluteBody);
      add('hold','Hold robot',28,450,452,76,!!control.session);
      add('resume','Resume robot',520,450,452,76,ready&&paused&&!!control.session);
      if(!absoluteBody){add('mode:clutch','Controller teleop',28,550);add('mode:tools','Tool handles',520,550);}
    }else if(this.tab==='view') {
      add('view:workspace',viewMode==='workspace'?'✓ Workspace':'Workspace',28,180,452);
      add('view:ego',viewMode==='ego'?'✓ Robot eyes':'Robot eyes',520,180,452,76,egoAvailable);
      add('scale_down','Smaller',28,290,290,76,viewMode==='workspace');add('actual_size','Actual size',354,290,290,76,viewMode==='workspace');add('scale_up','Larger',680,290,290,76,viewMode==='workspace');
      add('bring_here','Bring workspace here',28,400,944,76,viewMode==='workspace'&&!preview);
      add('toggle_map',mapVisible?'Hide voxel map':'Show voxel map',28,510,944);
    }else if(this.tab==='cameras') {
      [...this.cameras.values()].forEach((view,i)=>add(`camera:${view.name}`,view.name==='workspace'?'Head':view.name==='left'?'Left wrist':'Right wrist',28+i*326,180,290,64));
    }else if(this.tab==='record')actions.filter(b=>['record','save','discard','home'].includes(b.action)).forEach((b,i)=>add(b.action,b.label,28+(i%2)*492,200+Math.floor(i/2)*110,452,76,b.enabled));
    add('close_menu','Close controls',28,754,690,76);add('exit_xr','Exit XR',746,754,226,76,!preview);
    this.buttons=buttons;
    this.actions=[...buttons,...actions,
      {action:'toggle_map',enabled:true},{action:'view:workspace',enabled:true},{action:'view:ego',enabled:egoAvailable},
      {action:'follow_hands',enabled:!preview&&ready&&!paused&&absoluteBody&&calibrated&&handsAvailable}];
    for(const action of ['follow_hands','toggle_map','view:workspace','view:ego']) {
      const el=document.querySelector(`[data-action="${action}"]`);
      if(el){el.disabled=action==='follow_hands'?(!ready||paused||!calibrated||!handsAvailable):action==='view:ego'&&!egoAvailable;
        if(action==='follow_hands')el.textContent=following?'Hold hands':'Follow hands';
        if(action==='toggle_map')el.textContent=mapVisible?'Hide map':'Show map';}
    }
    // Cameras only redraw at camera cadence; no panels are coupled to scene scale.
    if(!this.menu.mesh.visible || performance.now()-this.lastDraw<80)return;this.lastDraw=performance.now();
    const c=this.menu.context;c.fillStyle='#13232d';c.fillRect(0,0,1000,860);c.textAlign='left';
    c.fillStyle='#edf5f4';c.font='bold 36px system-ui';c.fillText('Robot controls',28,52);
    if(this.tab==='control'){
      c.fillStyle=following?'#76e2c1':'#b9cecf';c.font='28px system-ui';c.fillText(following?'Following your hands':paused?'Robot held':!ready?'Waiting for robot feedback':'Hands held',28,202);
      c.fillStyle='#b9cecf';c.font='26px system-ui';wrapped(c,hint,28,630,944,38);
    }else if(this.tab==='view'){
      c.fillStyle='#b9cecf';c.font='27px system-ui';wrapped(c,viewMode==='ego'?'Life-size view from the robot’s head. Look around within the observed depth scene.':`Workspace scale ${Math.round(workspaceScale*100)}%. Controls keep their own size.`,28,650,944,38);
    }else if(this.tab==='cameras'){
      const view=this.cameras.get(this.selectedCamera);c.fillStyle='#0c141b';c.fillRect(180,270,640,480);
      if(view?.ts)c.drawImage(view.canvas,180,270,640,480);
      c.fillStyle=view?.stale?'#f3bf77':'#76e2c1';c.font='24px system-ui';c.fillText(view?.stale?'Camera delayed / unavailable':'Live camera',28,263);
    }
    for(const b of buttons){c.fillStyle=!b.enabled?'#263640':b.action===this.hovered?'#d6fff1':b.action==='hold'?'#f3bf77':b.action.startsWith('tab:')?'#344c57':'#76e2c1';c.fillRect(b.x,b.y,b.w,b.h);
      c.fillStyle=!b.enabled?'#78909b':b.action.startsWith('tab:')?'#edf5f4':'#10202b';c.font='bold 27px system-ui';c.textAlign='center';c.fillText(b.label,b.x+b.w/2,b.y+b.h/2+10);}
    this.menu.texture.needsUpdate=true;
  }
  hit(ray,side,point) {
    const meshes=[];if(this.menu.mesh.visible)meshes.push(this.menu.mesh);
    if(this.palm.mesh.visible&&side!==this.palmOwner)meshes.push(this.palm.mesh);
    const hit=ray?.intersectObjects(meshes)[0];if(!hit)return null;
    if(hit.object===this.palm.mesh)return {action:'summon_menu',point:hit.point};
    const x=hit.uv.x*1000,y=(1-hit.uv.y)*860;
    const b=this.buttons.find(b=>b.enabled&&x>=b.x&&x<=b.x+b.w&&y>=b.y&&y<=b.y+b.h);
    return {action:b?.action??'panel_body',point:hit.point};
  }
  hidePalm(){this.palm.mesh.visible=false;this.palmOwner=null;this.palmCandidate=null;}
  updatePalm(hands,eye,blocked,now) {
    const hand=hands.left;
    if(!this.immersive||!eye||hand?.kind!=='hand'||hand.trigger>.15){this.hidePalm();return;}
    const wrist=hand.poses.wrist,knuckle=hand.poses['middle-finger-phalanx-proximal'];
    if(!wrist||!knuckle){this.hidePalm();return;}
    const normal=new THREE.Vector3(0,-1,0).applyQuaternion(new THREE.Quaternion().copy(wrist.transform.orientation));
    if(normal.y<.65){this.hidePalm();return;}
    if(!this.palmCandidate)this.palmCandidate=now;
    this.palm.mesh.visible=now-this.palmCandidate>350;this.palmOwner='left';
    this.palm.mesh.position.copy(wrist.transform.position).lerp(new THREE.Vector3().copy(knuckle.transform.position),.5).addScaledVector(normal,.09);
    this.palm.mesh.lookAt(new THREE.Vector3().copy(eye));this.palm.mesh.updateMatrixWorld(true);
  }
  dispose(){this.stopped=true;this.abort.abort();for(const view of [this.menu,this.palm]){view.texture.dispose();view.mesh.geometry.dispose();view.mesh.material.dispose();view.mesh.removeFromParent();}for(const view of this.cameras.values())if(view.url)URL.revokeObjectURL(view.url);}
}
