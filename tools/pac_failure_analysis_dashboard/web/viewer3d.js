/* PAC2026 V4.6 READ-ONLY 3D viewer, no Gazebo commands, no synthetic animation.
 * Background meshes from actual local V4.6 SDF <visual> primitives;
 * positions shown only when a fresh dedicated Gazebo pose/info observation exists.
 * Plotly.js is bundled locally (no API key, CDN, WebSocket, or internet needed).
 */
(function(){
'use strict';
const MAX_DYNAMIC=80;
const UPDATE_MS=650;  // Complex Plotly STL meshes: ~1.5 FPS; live file updates faster.
let lastDrawAt=0, redrawInFlight=false;
const $=id=>document.getElementById(id);
const txt=(id,value)=>{const e=$(id);if(e)e.textContent=value;};
const qnorm=a=>{
 const n=Math.hypot(...a);return n>0.1?a.map(v=>v/n):[0,0,0,1];
};
function qrot(a,v){
 const [x,y,z,w]=qnorm(a),[vx,vy,vz]=v;
 const tx=2*(y*vz-z*vy),ty=2*(z*vx-x*vz),tz=2*(x*vy-y*vx);
 return [vx+w*tx+y*tz-z*ty,vy+w*ty+z*tx-x*tz,vz+w*tz+x*ty-y*tx];
}
function newGeo(){return {x:[],y:[],z:[],i:[],j:[],k:[]};}
function put(geo,v,tri){
 const base=geo.x.length;
 for(const p of v){geo.x.push(p[0]);geo.y.push(p[1]);geo.z.push(p[2]);}
 for(const [a,b,c] of tri){geo.i.push(base+a);geo.j.push(base+b);geo.k.push(base+c);}
}
function meshOne(geo,shape,size,position,quaternion){
 if(!Array.isArray(position)||position.length!==3||!position.every(Number.isFinite))return;
 if(!Array.isArray(quaternion)||quaternion.length!==4||!quaternion.every(Number.isFinite))return;
 if(!Array.isArray(size)||!size.every(s=>Number.isFinite(s)&&s>0))return;
 let vertices=[],faces=[];
 if(shape==='box'){
   if(size.length!==3)return;
   const [hx,hy,hz]=size.map(x=>x/2);
   vertices=[[-hx,-hy,-hz],[hx,-hy,-hz],[hx,hy,-hz],[-hx,hy,-hz],[-hx,-hy,hz],[hx,-hy,hz],[hx,hy,hz],[-hx,hy,hz]];
   faces=[[0,2,1],[0,3,2],[4,5,6],[4,6,7],[0,1,5],[0,5,4],[1,2,6],[1,6,5],[2,3,7],[2,7,6],[3,0,4],[3,4,7]];
 }else if(shape==='cylinder'){
   if(size.length!==2)return;
   const [r,h]=size,n=10;
   for(let j=0;j<2;j++)for(let k=0;k<n;k++){
       const a=2*Math.PI*k/n;vertices.push([r*Math.cos(a),r*Math.sin(a), (j-.5)*h]);}
   vertices.push([0,0,-h/2],[0,0,h/2]);
   for(let k=0;k<n;k++){
     const b=(k+1)%n;
     faces.push([k,b,n+k],[b,n+b,n+k],[2*n,b,k],[2*n+1,n+k,n+b]);
   }
 }else if(shape==='sphere'){
    if(size.length!==1)return;
    const r=size[0];vertices=[[0,0,r],[0,0,-r]];
    const n=12,levels=5;
    for(let l=1;l<=levels;l++)for(let k=0;k<n;k++){
      const theta=Math.PI*l/(levels+1),a=Math.PI*2*k/n;
      vertices.push([r*Math.sin(theta)*Math.cos(a),r*Math.sin(theta)*Math.sin(a),r*Math.cos(theta)]);
    }
    for(let k=0;k<n;k++)faces.push([0,2+k,2+(k+1)%n]);
    for(let l=0;l<levels-1;l++)for(let k=0;k<n;k++){
      const u=2+l*n+k,v=2+l*n+(k+1)%n,w=u+n,t=v+n;faces.push([u,w,v],[v,w,t]);
    }
    const end=2+(levels-1)*n;
    for(let k=0;k<n;k++)faces.push([1,end+(k+1)%n,end+k]);
 }else{return;}
 put(geo,vertices.map(v=>qrot(quaternion,v).map((x,i)=>x+position[i])),faces);
}
function trace(geo,color,name){return {
 type:'mesh3d',name,x:geo.x,y:geo.y,z:geo.z,i:geo.i,j:geo.j,k:geo.k,
 flatshading:true,color,opacity:1,hoverinfo:'skip',showscale:false,
 lighting:{ambient:0.7,diffuse:0.85,roughness:0.8,specular:0.1},
 showlegend:false};}
function initialCamera(){return {eye:{x:1.45,y:-1.75,z:1.08},up:{x:0,y:0,z:1},projection:{type:'perspective'}};}
function layout(){return {
 uirevision:'pac3d-user-camera-v2',paper_bgcolor:'rgba(0,0,0,0)',plot_bgcolor:'rgba(0,0,0,0)',
 margin:{l:0,r:0,t:4,b:0},showlegend:false,
 scene:{uirevision:'pac3d-user-camera-v2',aspectmode:'data',dragmode:'orbit',
  bgcolor:'rgba(0,0,0,0)',
  xaxis:{visible:false,showbackground:false,showgrid:false,zeroline:false,showticklabels:false},
  yaxis:{visible:false,showbackground:false,showgrid:false,zeroline:false,showticklabels:false},
  zaxis:{visible:false,showbackground:false,showgrid:false,zeroline:false,showticklabels:false},
  camera:initialCamera()}}
}
let base=[],shown=false,initDone=false,lastFrame='',scene=null,robotModel=null;
let fallback=false,fallbackLive=null,fallbackCanvas=null;
const orbit={azimuth:.6,elevation:.55,zoom:1.0,center:null};
function qmul(a,b){
 const [x,y,z,w]=qnorm(a),[xx,yy,zz,ww]=qnorm(b);
 return qnorm([w*xx+x*ww+y*zz-z*yy,w*yy-x*zz+y*ww+z*xx,w*zz+x*yy-y*xx+z*ww,w*ww-x*xx-y*yy-z*zz]);
}
function poseCompose(parent,local){
 const d=qrot(parent.quaternion,local.position);
 return {position:parent.position.map((n,i)=>n+d[i]),quaternion:qmul(parent.quaternion,local.quaternion)};
}
function qAxis(axis,angle){
 const norm=Math.hypot(...axis)||1;const s=Math.sin(angle/2)/norm;
 return [axis[0]*s,axis[1]*s,axis[2]*s,Math.cos(angle/2)];
}
function robotFK(robot,joints){
 if(!robot||!robot.joints?.length)return null;
 const angles=new Map((joints||[]).map(j=>[j.name,j.position]));
 const required=robot.joints.filter(j=>j.type==='revolute' || j.type==='continuous');
 if(!required.length||required.some(j=>!angles.has(j.name)))return null;
 const frames=new Map([['world',{position:[0,0,0],quaternion:[0,0,0,1]}]]);
 for(let iter=0;iter<10;iter++){
    let changed=false;
    for(const joint of robot.joints){
      if(frames.has(joint.child)||!frames.has(joint.parent))continue;
      const local={position:joint.origin,quaternion:joint.quaternion};
      let frame=poseCompose(frames.get(joint.parent),local);
      if(joint.type==='revolute' || joint.type==='continuous'){
         frame.quaternion=qmul(frame.quaternion,qAxis(joint.axis,angles.get(joint.name)));
      }
      frames.set(joint.child,frame);changed=true;
    }
    if(!changed)break;
 }
 return frames;
}
function modelFromJoints(robot,joints,reference=false){
 const frames=robotFK(robot,joints);if(!frames)return [];
 const groups=new Map();
 for(const mesh of (robot.meshes||[])){
    const frame=frames.get(mesh.link);if(!frame)continue;
    let geo=groups.get(mesh.color);
    if(!geo){geo=newGeo();groups.set(mesh.color,geo);}
    const visualPose=poseCompose(frame,{position:mesh.origin,quaternion:mesh.quaternion});
    for(const tri of mesh.triangles){
       if(!Array.isArray(tri)||tri.length!==9)continue;
       const pts=[];
       for(let i=0;i<9;i+=3){
          const local=[tri[i]*mesh.scale[0],tri[i+1]*mesh.scale[1],tri[i+2]*mesh.scale[2]];
          const r=qrot(visualPose.quaternion,local);
          pts.push(r.map((v,k)=>v+visualPose.position[k]));
       }
       put(geo,pts,[[0,1,2]]);
    }
 }
 const result=[...groups].filter(([c,g])=>g.x.length).map(([color,g])=>{
   const m=trace(g,reference?'#8295ad':color,reference?'SRDF home 참고 자세 (실측 아님)':'HDR50-22 STL + ROS 실측 관절');
   if(reference)m.opacity=.38;return m;});
 if(!result.length){
    const names=['base_link','lower_frame_link','upper_frame_link','arm_link','wrist_body_link','wrist_holder_link','flange_link'];
    const nodes=names.map(n=>frames.get(n)).filter(Boolean);
    if(nodes.length>1)result.push({type:'scatter3d',mode:'lines+markers',
      x:nodes.map(n=>n.position[0]),y:nodes.map(n=>n.position[1]),z:nodes.map(n=>n.position[2]),
      line:{color:reference?'#8595a9':'#80c0e9',width:reference?9:16},
      marker:{size:5,color:reference?'#9cabba':'#96ddff'},
      name:reference?'SRDF home 참고 자세 (실측 아님)':'ROS 관절 + URDF 링크 중심 간략형',showlegend:false});
 }
 return result;
}
function pointsTrace(entities){return {type:'scatter3d',mode:'markers+text',
 x:entities.map(e=>e.x),y:entities.map(e=>e.y),z:entities.map(e=>e.z),
 text:entities.map(e=>(e.name||'').slice(0,36)),textposition:'top center',
 textfont:{color:'#f3d59e',size:10},
 marker:{size:5,color:entities.map(e=>/box/i.test(e.name)?'#edc876':'#80dacf'),opacity:0.97},
 name:'Gazebo 실측 위치',customdata:entities.map(e=>e.name),hovertemplate:'%{customdata}<br>x=%{x:.3f} y=%{y:.3f} z=%{z:.3f} m<extra>Gazebo</extra>',showlegend:false};}
function robotTrace(entities){
 // Link frames are not full mesh; use actual Gazebo coordinate samples only.
 const robot=entities.filter(e=>/hdr50_22|hdr_robot/i.test(e.name) && /base_link|lower_frame|upper_frame|arm_link|wrist_body|flange_link|suction_cup_link|suction_tcp/i.test(e.name));
 if(robot.length<2)return null;
 const ordered=['base_link','lower_frame_link','upper_frame_link','arm_link','wrist_body_link','flange_link','suction_cup_link','suction_tcp'];
 function rank(s){const i=ordered.findIndex(a=>s.endsWith(a));return i<0?999:i;}
 robot.sort((a,b)=>rank(a.name)-rank(b.name));
 return {type:'scatter3d',mode:'lines+markers',x:robot.map(e=>e.x),y:robot.map(e=>e.y),z:robot.map(e=>e.z),
 line:{color:'#77c4f1',width:14},marker:{color:'#9ddbff',size:4},
 customdata:robot.map(e=>e.name),hovertemplate:'%{customdata}<extra>관측 링크 중심</extra>',
 name:'관측된 로봇 링크 중심선 (메시 아님)',showlegend:false};
}
function actualBoxes(entities){
 const groups=new Map();
 for(const e of entities){
    if(!(/^box[_-]\d+$/i.test(e.name)||(integration?.boxes||[]).some(b=>b.id===e.name))||!Array.isArray(e.size))continue;
    const g=groups.get('box')||newGeo();
    meshOne(g,'box',e.size,[e.x,e.y,e.z],e.quaternion||[0,0,Math.sin((e.yaw||0)/2),Math.cos((e.yaw||0)/2)]);
    groups.set('box',g);
 }
 return [...groups.values()].filter(g=>g.x.length).map(g=>trace(g,'#dfb773','Gazebo에서 치수까지 확인한 박스'));
}
function modelPrimitives(parsed){
 const groups=new Map();
 for(const o of (parsed.primitives||[])){
    const color=o.color||'#8492a2';let g=groups.get(color);
    if(!g){g=newGeo();groups.set(color,g);}
    meshOne(g,o.shape,o.size,o.position,o.quaternion);
 }
 return [...groups.entries()].filter(([c,g])=>g.x.length).map(([color,g])=>trace(g,color,'V4.6 SDF 고정 형상'));
}
function freshEntities(live){
 if(!live||!live.available||!live.data||!live.data.gazebo)return [];
 const age=live.source_age_s?live.source_age_s.gazebo:null;
 if(age===null||age>5||live.age_s===null||live.age_s>5)return [];
 return (live.data.gazebo.entities||[]).filter(e=>[e.x,e.y,e.z].every(Number.isFinite)).slice(0,MAX_DYNAMIC);
}
function makeKey(entities){return JSON.stringify(entities.map(e=>[e.name,e.x,e.y,e.z,e.yaw]));}
async function load(){
 if(initDone)return;initDone=true;
 const dom=$('viewer3d');if(!dom)return;
 if(typeof Plotly==='undefined'){txt('viewer3d-status','3D 라이브러리를 불러올 수 없습니다.');return;}
 try{
  const [worldResp,robotResp]=await Promise.all([
   fetch('/api/scene3d',{cache:'no-store'}),fetch('/api/robot3d',{cache:'no-store'})]);
  if(!worldResp.ok)throw Error('SDF request failed');scene=await worldResp.json();base=modelPrimitives(scene);
  if(robotResp.ok)robotModel=await robotResp.json();
  const count=scene.primitives?.length||0;
  txt('viewer3d-source', count?
    `배경: 로컬 V4.6 SDF visual ${count}개 (지원하지 않는 형상 ${scene.skipped_mesh_or_unsupported}개 제외) · `+
    `로봇: ${robotModel?.meshes?.length||0}개 로컬 STL 형상 · SRDF 참고 자세: ${robotModel?.reference_joints?.length||0}관절 · `+
    `${robotModel?.error||'로봇 STL 정상'} · 움직임은 ROS 실측 관절만 사용 · API 키 불필요.`:
    `월드 배경을 읽지 못했습니다. ${scene.error||'형상 없음'}`);
  if(!webGLAvailable()){
    fallback=true; setupFallback(dom); renderFallback();
  }else{
    const ref=robotModel?.reference_joints||[];
    const initial=ref.length?modelFromJoints(robotModel,ref,true):[];
    await Plotly.newPlot(dom,[...base,...initial],layout(),{responsive:true,displaylogo:false,scrollZoom:true});
  }
  if(!fallback)dom.on('plotly_click',ev=>{
    const point=ev.points&&ev.points[0];
    if(point&&point.customdata)txt('viewer3d-selection','선택: '+String(point.customdata).slice(0,120));
  });
  shown=true;txt('viewer3d-status',(robotModel?.reference_joints?.length?'참고 로봇 표시 (실측 아님) · ':'')+
       (fallback?'WebGL 대체 · 관측 대기':'Gazebo 실측 관측 대기'));
 }catch(e){console.error('PAC3D viewer initialization failed:',e);txt('viewer3d-status','3D 초기화 실패 · 브라우저 콘솔 확인');}
}
function setView(name){
 const div=$('viewer3d');if(!shown||!scene)return;
 if(fallback){
   if(name==='all'){orbit.azimuth=.6;orbit.elevation=.55;orbit.zoom=1;orbit.center=null;}
   else if(name==='robot'){orbit.zoom=2.0;orbit.center=[0,0,1.3];}
   else if(name==='pallet'){orbit.zoom=2.0;orbit.center=[0,1.2,.9];}
   else{orbit.zoom=1.4;orbit.center=[-3,1.2,.85];}
   renderFallback();return;
 }
 if(name==='all'){
  Plotly.relayout(div,{'scene.camera':initialCamera(),
    'scene.xaxis.autorange':true,'scene.yaxis.autorange':true,'scene.zaxis.autorange':true});
  return;
 }
 if(name==='robot'){
   Plotly.relayout(div,{'scene.xaxis.range':[-1.8,1.8],
    'scene.yaxis.range':[-1.5,1.9],'scene.zaxis.range':[-.1,3.5],
    'scene.camera':initialCamera()});
   txt('viewer3d-selection','로봇 영역 확대 · 실측 값이 없으면 SRDF home 참고 자세만 표시합니다.');
   return;
 }
 const matched=(scene.primitives||[]).filter(p=>name==='pallet'?/pallet/i.test(p.model):
                                          /conveyor|camera|scale/i.test(p.model));
 if(!matched.length){txt('viewer3d-selection','선택 구역 형상을 찾지 못했습니다.');return;}
 const wanted=name==='pallet'?'팔레트':'컨베이어';
 const center=[0,1,2].map(i=>matched.reduce((s,p)=>s+p.position[i],0)/matched.length);
 const horizontal=name==='pallet'?1.8:4.2;
 Plotly.relayout(div,{'scene.xaxis.range':[center[0]-horizontal,center[0]+horizontal],
  'scene.yaxis.range':[center[1]-horizontal/2,center[1]+horizontal/2],
  'scene.zaxis.range':[-0.15,3], 'scene.camera':initialCamera()});
 txt('viewer3d-selection',wanted+' 구역을 표시합니다.');
}
async function update(live,integration){
 if(!initDone)await load();if(!shown)return;
 if(fallback){fallbackLive=live;renderFallback();return;}
 if(redrawInFlight || Date.now()-lastDrawAt<UPDATE_MS)return;
 lastDrawAt=Date.now();redrawInFlight=true;
 const entities=freshEntities(live),liveOK=entities.length>0;
 txt('viewer3d-status',liveOK?`실측 관측 ${entities.length}개 · ${live.source_age_s.gazebo.toFixed(1)}초 전`:
    '관측 없음 / 지연 · 배경만 표시 (실제 움직임 아님)');
 const age=live?.source_age_s?.joint_states;
 const joints=(live?.available&&age!==null&&age!==undefined&&age<=5&&live.age_s<=5)?
    (live.data.joint_states?.joints||[]):[];
 const key=makeKey(entities)+'|'+JSON.stringify(joints.map(j=>[j.name,j.position]));
 if(key===lastFrame){redrawInFlight=false;return;}
 lastFrame=key;
 const traces=[...base];
 const liveRobot=joints.length&&robotFK(robotModel,joints)!==null;
 if(liveRobot)traces.push(...modelFromJoints(robotModel,joints));
 else if(robotModel?.reference_joints?.length)
   traces.push(...modelFromJoints(robotModel,robotModel.reference_joints,true));
 if(liveRobot)txt('viewer3d-status',`로봇 ROS 실측 ${joints.length}개 관절 (${age.toFixed(1)}초 전)`+(liveOK?` · Gazebo 엔티티 ${entities.length}개`:' · Gazebo 위치 미수신'));
 else txt('viewer3d-status',robotModel?.reference_joints?.length?'참고 로봇 자세만 표시 중 (실측 아님)':'로봇 실측/참고 자세 모두 없음');
 if(liveOK){
   traces.push(...actualBoxes(entities));
   const robot=robotTrace(entities);if(robot&&!liveRobot)traces.push(robot);
   const names=entities.filter(e=>!e.name.includes('::')||/suction_tcp/.test(e.name));
   if(names.length)traces.push(pointsTrace(names));
 }
 try{await Plotly.react($('viewer3d'),traces,layout(),{responsive:true,displaylogo:false,scrollZoom:true});}
 catch(e){txt('viewer3d-status','3D 갱신 실패 (새로고침하여 확인)');}
 finally{redrawInFlight=false;}
}

function webGLAvailable(){
 try{
   const c=document.createElement('canvas');
   return !!(c.getContext('webgl2')||c.getContext('webgl')||c.getContext('experimental-webgl'));
 }catch(e){return false;}
}
function setupFallback(dom){
 dom.replaceChildren();
 fallbackCanvas=document.createElement('canvas');
 fallbackCanvas.id='canvas3d-fallback';fallbackCanvas.width=1400;fallbackCanvas.height=700;
 fallbackCanvas.style.cssText='width:100%;height:100%;display:block;cursor:grab;touch-action:none;background:#244357;';
 dom.appendChild(fallbackCanvas);
 let drag=null;
 fallbackCanvas.onpointerdown=e=>{drag=[e.clientX,e.clientY];fallbackCanvas.setPointerCapture(e.pointerId);fallbackCanvas.style.cursor='grabbing';};
 fallbackCanvas.onpointermove=e=>{
   if(!drag)return;
   orbit.azimuth+=(e.clientX-drag[0])*0.008;
   orbit.elevation=Math.max(-1.25,Math.min(1.25,orbit.elevation+(e.clientY-drag[1])*0.006));
   drag=[e.clientX,e.clientY];renderFallback();
 };
 fallbackCanvas.onpointerup=()=>{drag=null;fallbackCanvas.style.cursor='grab';};
 fallbackCanvas.addEventListener('wheel',e=>{
    e.preventDefault();orbit.zoom=Math.max(.32,Math.min(6,orbit.zoom*Math.exp(-e.deltaY*.0014)));
    renderFallback();
 },{passive:false});
}
function parsedColor(s,shade=1){
 const m=/^#([0-9a-f]{6})$/i.exec(s||'');
 if(!m)return '#66849a';const n=parseInt(m[1],16);
 const rgb=[(n>>16)&255,(n>>8)&255,n&255];
 return 'rgb('+rgb.map(x=>Math.max(0,Math.min(255,Math.round(x*shade)))).join(',')+')';
}
function fallbackBox(o,putFace){
 const [dx,dy,dz]=o.size;
 const half=[dx/2,dy/2,dz/2];
 const local=[[-1,-1,-1],[1,-1,-1],[1,1,-1],[-1,1,-1],[-1,-1,1],[1,-1,1],[1,1,1],[-1,1,1]];
 const points=local.map(a=>qrot(o.quaternion,a.map((v,i)=>v*half[i])).map((v,i)=>v+o.position[i]));
 [[0,1,2,3],[4,5,6,7],[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7]].forEach(indices=>putFace(indices.map(i=>points[i]),o.color));
}
function fallbackCylinder(o,putFace){
 const [radius,len]=o.size,n=10;
 const points=[];
 for(let z of [-len/2,len/2])for(let k=0;k<n;k++){
   const a=2*Math.PI*k/n;
   points.push(qrot(o.quaternion,[radius*Math.cos(a),radius*Math.sin(a),z]).map((v,i)=>v+o.position[i]));
 }
 putFace(points.slice(0,n),o.color);putFace(points.slice(n),o.color);
 for(let k=0;k<n;k++)putFace([points[k],points[(k+1)%n],points[n+(k+1)%n],points[n+k]],o.color);
}
function renderFallback(){
 if(!fallbackCanvas||!scene)return;
 const ctx=fallbackCanvas.getContext('2d'),w=fallbackCanvas.width,h=fallbackCanvas.height;
 ctx.fillStyle='#244357';ctx.fillRect(0,0,w,h);
 const staticItems=(scene.primitives||[]).slice(0,1400);
 const bounds=staticItems.length?staticItems:[];
 const center=orbit.center||[0,1,2].map(i=>bounds.length?bounds.reduce((n,o)=>n+o.position[i],0)/bounds.length:0);
 if(!orbit.center)center[2]=0.7;
 const span=orbit.center?5:Math.max(5,...bounds.map(o=>Math.max(Math.abs(o.position[0]-center[0])*2,
   Math.abs(o.position[1]-center[1])*2)),0);
 const scale=Math.min(w*.8,h*.72)/span*orbit.zoom;
 const ca=Math.cos(orbit.azimuth),sa=Math.sin(orbit.azimuth),cp=Math.cos(orbit.elevation),sp=Math.sin(orbit.elevation);
 const proj=pt=>{
   const [x,y,z]=pt.map((v,i)=>v-center[i]);
   const u=ca*x-sa*y,v=sa*x+ca*y;
   return [w/2+scale*u,h*.55-scale*(sp*v+cp*z),cp*v-sp*z];
 };
 const faces=[];
 const addFace=(pts,color)=>{
  if(pts.length<3)return;
  const p=pts.map(proj),depth=p.reduce((a,v)=>a+v[2],0)/p.length;
  const shade=.72+Math.max(0,Math.min(.35,(p[0][1]-p[1][1])*.0005+.17));
  faces.push({p,depth,color:parsedColor(color,shade)});
 };
 for(const o of staticItems){
  if(o.shape==='box')fallbackBox(o,addFace);
  if(o.shape==='cylinder')fallbackCylinder(o,addFace);
 }
 const entities=freshEntities(fallbackLive);
 const actual=entities.filter(e=>(/^box[_-]\d+$/i.test(e.name)||(integration?.boxes||[]).some(b=>b.id===e.name))&&Array.isArray(e.size));
 for(const e of actual)fallbackBox({size:e.size,quaternion:e.quaternion||[0,0,0,1],
   position:[e.x,e.y,e.z],color:'#edbe79'},addFace);
 faces.sort((a,b)=>b.depth-a.depth);
 for(const f of faces){
  ctx.beginPath();f.p.forEach((p,i)=>i?ctx.lineTo(p[0],p[1]):ctx.moveTo(p[0],p[1]));
  ctx.closePath();ctx.fillStyle=f.color;ctx.fill();
  ctx.lineWidth=.5;ctx.strokeStyle='#172e42';ctx.stroke();
 }
 const axes=[[1.2,0,0],[0,1.2,0],[0,0,1.2]],axesColors=['#e89181','#87c89a','#91bafa'];
 for(let i=0;i<3;i++){
  const a=proj([0,0,0]),b=proj(axes[i]);ctx.beginPath();ctx.moveTo(a[0],a[1]);ctx.lineTo(b[0],b[1]);
  ctx.strokeStyle=axesColors[i];ctx.lineWidth=3;ctx.stroke();
 }
 const age=fallbackLive?.source_age_s?.joint_states;
 const joints=fallbackLive?.available&&age!==null&&age!==undefined&&age<=5&&fallbackLive?.age_s<=5?
   fallbackLive.data.joint_states.joints:[];
 const activeJoints=joints.length?joints:(robotModel?.reference_joints||[]);
 if(activeJoints.length){
  const frame=robotFK(robotModel,activeJoints);
  if(frame){
   const chain=['base_link','lower_frame_link','upper_frame_link','arm_link','wrist_body_link','wrist_holder_link','flange_link','suction_tcp'];
   const vertices=chain.map(n=>frame.get(n)).filter(Boolean);
   if(vertices.length>1){
    ctx.strokeStyle=joints.length?'#83c5f0':'#8194a9';ctx.lineCap='round';ctx.lineWidth=10;
    ctx.beginPath();vertices.forEach((v,i)=>{const p=proj(v.position);i?ctx.lineTo(p[0],p[1]):ctx.moveTo(p[0],p[1]);});ctx.stroke();
    ctx.fillStyle=joints.length?'#aadcf7':'#9eafc0';for(const v of vertices){const p=proj(v.position);ctx.beginPath();ctx.arc(p[0],p[1],5,0,Math.PI*2);ctx.fill();}
   }
  }
 }
 ctx.font='17px system-ui';ctx.fillStyle='#f3d59a';
 for(const e of entities.slice(0,65)){
  if(e.name.includes('::')&&!/suction_tcp/.test(e.name))continue;
  const p=proj([e.x,e.y,e.z]);ctx.beginPath();ctx.arc(p[0],p[1],6,0,Math.PI*2);ctx.fill();
  ctx.fillText(e.name.slice(0,30),p[0]+10,p[1]-8);
 }
 const status=(joints.length?'로봇 실측 관절 표시':'SRDF home 참고 자세 (실측 아님)')+
   (entities.length?` · Gazebo 관측 ${entities.length}개`:' · Gazebo 위치 미수신')+
   ' · Canvas 간략형 (STL 외형 생략)';
 txt('viewer3d-status',status);
}
window.PACPlotlyViewer3D={load,update,setView};
})();
