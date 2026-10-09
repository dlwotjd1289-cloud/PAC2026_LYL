/* PAC2026 V5: Three.js real STL renderer, Gazebo/ROS READ-ONLY. The original
 * HDR50-22 STL files are served by a strictly bounded local endpoint. No
 * position values are fabricated; SRDF reference pose is explicitly marked.
 */
(function(){
'use strict';
const $=id=>document.getElementById(id);
const notice=s=>{const e=$('viewer3d-status');if(e)e.textContent=s;};
let renderer,scene,camera,controls,robot,initialized=false,linkGroups=new Map(), boxModels=new Map(),
    refs=[],activeReadings=false,lastPoseTime=0,staticCount=0,meshCount=0;
let selectedView='all';
const ident=[0,0,0,1];
function normalize(q){const n=Math.hypot(...q);return n>0.1?q.map(x=>x/n):[0,0,0,1];}
function rotate(q,v){const [x,y,z,w]=normalize(q),[a,b,c]=v;
 const tx=2*(y*c-z*b),ty=2*(z*a-x*c),tz=2*(x*b-y*a);
 return [a+w*tx+y*tz-z*ty,b+w*ty+z*tx-x*tz,c+w*tz+x*ty-y*tx];}
function multiply(a,b){const [x,y,z,w]=a,[i,j,k,m]=b;
 return normalize([w*i+x*m+y*k-z*j,w*j-x*k+y*m+z*i,w*k+x*j-y*i+z*m,w*m-x*i-y*j-z*k]);}
function compose(p,l){const delta=rotate(p.q,l.p);return {p:p.p.map((x,i)=>x+delta[i]),q:multiply(p.q,l.q)};}
function axisQuat(axis,angle){const len=Math.hypot(...axis)||1,k=Math.sin(angle/2)/len;
 return [axis[0]*k,axis[1]*k,axis[2]*k,Math.cos(angle/2)];}
function robotFrames(data,joints){
 const allowed=new Map(joints.map(j=>[j.name,j.position]));
 const essential=data.joints.filter(j=>j.type==='revolute'||j.type==='continuous');
 if(!essential.length||essential.some(j=>!allowed.has(j.name)))return null;
 const frames=new Map([['world',{p:[0,0,0],q:ident}]]);
 for(let iteration=0;iteration<12;iteration++){
  let changed=false;
  for(const j of data.joints){
   if(frames.has(j.child)||!frames.has(j.parent))continue;
   let f=compose(frames.get(j.parent),{p:j.origin,q:j.quaternion});
   if(j.type==='revolute'||j.type==='continuous')f.q=multiply(f.q,axisQuat(j.axis,allowed.get(j.name)));
   frames.set(j.child,f);changed=true;
  }
  if(!changed)break;
 }
 return frames;
}
function pose(node,p,q){node.position.set(...p);node.quaternion.set(...normalize(q));}
function color(hex){return new THREE.Color(/^#[a-f0-9]{6}$/i.test(hex||'')?hex:'#8da6ba');}
function material(hex,opt={}){return new THREE.MeshStandardMaterial({color:color(hex),roughness:.52,metalness:.13,...opt});}
function shapeGeometry(shape,size){
 if(shape==='box'&&size.length===3)return new THREE.BoxGeometry(...size);
 if(shape==='cylinder'&&size.length===2){const g=new THREE.CylinderGeometry(size[0],size[0],size[1],24);g.rotateX(Math.PI/2);return g;}
 if(shape==='sphere'&&size.length===1)return new THREE.SphereGeometry(size[0],20,12);
 return null;
}
function addPrimitive(o){
 const geometry=shapeGeometry(o.shape,o.size);
 if(!geometry)return;
 const mesh=new THREE.Mesh(geometry,material(o.color));pose(mesh,o.position,o.quaternion);
 if(!/floor/i.test(o.model)){mesh.castShadow=true;}
 mesh.receiveShadow=true;scene.add(mesh);staticCount++;
}
function smoothSTL(geo){
 // The upstream STL is triangulated; average normals only for *shared positions*.
 // This changes lighting, not the geometry or pose of the model.
 const p=geo.getAttribute('position'),n=p.count;
 if(!n||n>2500000)return geo;  // avoid unbounded computation
 const sums=new Map(), keys=new Array(n);
 for(let i=0;i+2<n;i+=3){
  const a=[p.getX(i),p.getY(i),p.getZ(i)],b=[p.getX(i+1),p.getY(i+1),p.getZ(i+1)],c=[p.getX(i+2),p.getY(i+2),p.getZ(i+2)];
  const u=[b[0]-a[0],b[1]-a[1],b[2]-a[2]],v=[c[0]-a[0],c[1]-a[1],c[2]-a[2]];
  const face=[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]];
  for(let j=0;j<3;j++){
   const k=i+j,xyz=j===0?a:j===1?b:c;
   const key=xyz.map(x=>Math.round(x*100)).join(':');keys[k]=key;
   const old=sums.get(key)||[0,0,0];sums.set(key,[old[0]+face[0],old[1]+face[1],old[2]+face[2]]);
  }
 }
 const norm=new Float32Array(n*3);
 for(let i=0;i<n;i++){
  const v=sums.get(keys[i])||[0,0,1],len=Math.hypot(...v)||1;
  norm[i*3]=v[0]/len;norm[i*3+1]=v[1]/len;norm[i*3+2]=v[2]/len;
 }
 geo.setAttribute('normal',new THREE.BufferAttribute(norm,3));return geo;
}
async function buildRobot(data){
 robot=data;
 if(!data||!Array.isArray(data.joints)||!Array.isArray(data.meshes))return;
 refs=data.reference_joints||[];
 const stlLoader=new THREE.STLLoader();
 const cache=new Map();
 const loadSTL=filename=>{
  if(cache.has(filename))return cache.get(filename);
  const promise=new Promise((resolve,reject)=>stlLoader.load(
    '/assets/robot-mesh/'+encodeURIComponent(filename),g=>resolve(smoothSTL(g)),undefined,reject));
  cache.set(filename,promise);return promise;
 };
 const fetchers=[];
 for(const v of data.meshes){
  const link=linkGroups.get(v.link)||new THREE.Group();
  if(!linkGroups.has(v.link)){linkGroups.set(v.link,link);scene.add(link);}
  const visual=new THREE.Group();pose(visual,v.origin,v.quaternion);link.add(visual);
  const job=loadSTL(v.source).then(g=>{
   const mesh=new THREE.Mesh(g,material(v.color,{side:THREE.DoubleSide}));
   mesh.scale.set(...v.scale);mesh.castShadow=true;mesh.receiveShadow=true;
   visual.add(mesh);meshCount++;
  }).catch(()=>{
   // A missing STL doesn't cause a fake robot geometry to be substituted.
  });
  fetchers.push(job);
 }
 // Show the genuine kinematic reference even while meshes stream in.
 if(refs.length)applyRobot(refs,false);
 Promise.all(fetchers).then(()=>{
   if(meshCount===0)notice('로봇 원본 STL을 읽지 못했습니다. 메시 경로/콘솔을 확인하세요.');
   else notice(`원본 HDR50-22 STL ${meshCount}개 로드 · 관절은 ROS2 읽기 전용`);
 });
}
function applyRobot(joints,actual){
 const frames=robotFrames(robot,joints);if(!frames)return false;
 for(const [name,group] of linkGroups){
   const v=frames.get(name);if(v){pose(group,v.p,v.q);group.visible=true;}else group.visible=false;
   group.traverse(ch=>{if(ch.material?.opacity!==undefined){
     ch.material.transparent=!actual;
     ch.material.opacity=actual?1:.62;
     ch.material.needsUpdate=true;
   }});
 }
 activeReadings=actual;return true;
}
function updateBoxes(entities,integration){
 const known=new Set((integration?.boxes||[]).map(b=>b.id));
 const current=new Set();
 for(const e of entities){
  if(!/^box[_-]\d+$/i.test(e.name)&&!known.has(e.name))continue;
  current.add(e.name);
  let item=boxModels.get(e.name);
  const key=Array.isArray(e.size)?e.size.join(','):'observed_point';
  if(!item||item.geometryKey!==key){
   if(item){scene.remove(item.mesh);item.mesh.geometry.dispose();item.mesh.material.dispose();}
   const known=Array.isArray(e.size)&&e.size.every(n=>n>0);
   const shape=known?new THREE.BoxGeometry(...e.size):new THREE.SphereGeometry(.047,14,10);
   const mesh=new THREE.Mesh(shape,material(known?'#e6b879':'#f5da93',{metalness:0,roughness:.82}));
   mesh.castShadow=true;mesh.receiveShadow=true;scene.add(mesh);
   item={mesh,geometryKey:key};boxModels.set(e.name,item);
  }
  const v=item.mesh;
  pose(v,[e.x,e.y,e.z],e.quaternion||[0,0,Math.sin((e.yaw||0)/2),Math.cos((e.yaw||0)/2)]);
 }
 for(const [k,v] of boxModels)if(!current.has(k)){
  scene.remove(v.mesh);v.mesh.geometry.dispose();v.mesh.material.dispose();boxModels.delete(k);
 }
}
function fresh(live,source){return live&&live.available&&live.age_s<=5
 &&live.source_age_s&&live.source_age_s[source]!==null&&live.source_age_s[source]<=5;}
function update(live,integration){
 if(!initialized||!robot)return;
 const observed=fresh(live,'joint_states')?(live.data.joint_states.joints||[]):[];
 const names=new Set(observed.map(j=>j.name));
 const actual=[1,2,3,4,5,6].every(i=>names.has('j'+i));
 if(actual)applyRobot(observed,true);
 else if(refs.length)applyRobot(refs,false);
 const entities=fresh(live,'gazebo')?(live.data.gazebo.entities||[]):[];
 updateBoxes(entities,integration);
 if(actual){notice(`ROS 관절 6/6 최신 (${live.source_age_s.joint_states.toFixed(1)}초 전)`+
 (entities.length?` · Gazebo 추적 엔티티 ${entities.length}개`:' · 박스 위치 미확인'));}
 else notice('로봇 SRDF home 참고 자세 (실측 아님) · ROS 관절 수신 대기');
 const source=$('viewer3d-source');if(source&&staticCount)
  source.textContent=`배경: 활성 SDF ${staticCount}개 시각 형상 · 로봇: 원본 HDR50-22 STL ${meshCount}개 · `+
    (actual?'관절: 최신 ROS /joint_states':'관절: SRDF home 참고')+
    ` · 박스: Gazebo pose/info (치수는 box SDF 있을 때만) · 독립 PyBullet 없음`;
}
function setView(which){if(!initialized)return;selectedView=which;
 let position,target;
 if(which==='robot'){position=[2.8,-3.9,2.8];target=[0,0,1.1];}
 else if(which==='pallet'){position=[1.9,-2.7,2.6];target=[0,1.2,.65];}
 else if(which==='pick'){position=[-1.7,-4.6,2.9];target=[-3.1,1.2,.9];}
 else{position=[0.8,-9.7,5.5];target=[-3.0,.7,.75];}
 camera.position.set(...position);controls.target.set(...target);controls.update();
 const status=$('viewer3d-selection');if(status)status.textContent=`${({robot:'HDR50-22',pallet:'팔레트',pick:'컨베이어',all:'전체'})[which]||'전체'} 확대 · 카메라 시점만 이동 (명령 없음)`;
}
async function load(){
 if(initialized)return true;
 if(!window.THREE||!THREE.OrbitControls||!THREE.STLLoader)return false;
 const dom=$('viewer3d');if(!dom)return false;
 try{
  const response=await Promise.all([fetch('/api/scene3d',{cache:'no-store'}),fetch('/api/robot3d',{cache:'no-store'})]);
  if(response.some(r=>!r.ok))return false;
  const world=await response[0].json(),links=await response[1].json();
  scene=new THREE.Scene();scene.background=new THREE.Color('#294758');
  scene.fog=new THREE.Fog('#294758',12,28);
  camera=new THREE.PerspectiveCamera(43,dom.clientWidth/Math.max(1,dom.clientHeight),.05,90);
  camera.up.set(0,0,1);camera.position.set(.8,-9.7,5.5);
  renderer=new THREE.WebGLRenderer({antialias:true,alpha:false,powerPreference:'high-performance'});
  renderer.setPixelRatio(Math.min(1.6,window.devicePixelRatio||1));
  renderer.setSize(dom.clientWidth,dom.clientHeight,false);
  renderer.outputEncoding=THREE.sRGBEncoding;
  renderer.toneMapping=THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure=1.35;
  renderer.shadowMap.enabled=true;renderer.shadowMap.type=THREE.PCFSoftShadowMap;
  dom.replaceChildren(renderer.domElement);
  renderer.domElement.style.cssText='width:100%;height:100%;display:block;touch-action:none;';
  controls=new THREE.OrbitControls(camera,renderer.domElement);
  controls.enableDamping=true;controls.dampingFactor=.085;controls.target.set(-3,.7,.75);
  controls.maxDistance=28;controls.minDistance=.55;controls.screenSpacePanning=true;
  scene.add(new THREE.HemisphereLight(0xe9f7ff,0x243f53,1.65));
  const key=new THREE.DirectionalLight(0xffffff,1.6);key.position.set(1.5,-3,8);
  key.castShadow=true;key.shadow.mapSize.set(2048,2048);key.shadow.camera.left=-10;key.shadow.camera.right=10;
  key.shadow.camera.top=10;key.shadow.camera.bottom=-10;scene.add(key);
  const fill=new THREE.DirectionalLight(0xc6e3ff,.65);fill.position.set(-7,4,5);scene.add(fill);
  for(const primitive of (world.primitives||[]))addPrimitive(primitive);
  // Three.js axes: Z-up (Gazebo world), exact SDF coords.
  await buildRobot(links);
  initialized=true;
  const loop=()=>{
   if(!initialized)return;
   requestAnimationFrame(loop);
   controls.update();renderer.render(scene,camera);
  };loop();
  window.addEventListener('resize',()=>{if(!initialized)return;
    const w=dom.clientWidth,h=Math.max(1,dom.clientHeight);
    camera.aspect=w/h;camera.updateProjectionMatrix();renderer.setSize(w,h,false);});
  notice(`Three.js 원본 STL 로딩 · 배경 SDF ${staticCount}개`);
  return true;
 }catch(err){console.warn('PAC Three renderer unavailable:',err);
  try{renderer?.dispose()}catch(_){}
  return false;
 }
}
window.PACThreeViewer3D={load,update,setView};
})();
