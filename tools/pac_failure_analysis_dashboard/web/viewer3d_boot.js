/* Prefer the smooth Three.js renderer used in the team's PyBullet viewer.
 * It loads from the same public CDNs as the old viewer. If external JS or
 * WebGL is unavailable, fall back to the already-bundled local Plotly viewer.
 */
(function(){
'use strict';
let engine=null, latest=null,latestIntegration=null,requestedView='all',boundKey=null;
const $=x=>document.getElementById(x);
function script(src,timeoutMs=2600){
 return new Promise((resolve,reject)=>{
  const el=document.createElement('script');
  let done=false;
  const timer=setTimeout(()=>{if(done)return;done=true;el.remove();reject(Error('library timeout'));},timeoutMs);
  el.onload=()=>{if(done)return;done=true;clearTimeout(timer);resolve();};
  el.onerror=()=>{if(done)return;done=true;clearTimeout(timer);reject(Error('library unavailable'));};
  el.src=src;el.async=false;document.head.append(el);
 });
}
async function boot(){
 const status=$('viewer3d-status');
 const fallback=async()=>{engine=window.PACPlotlyViewer3D;
  if(!engine){status.textContent='로컬 Plotly 대체 3D 로더가 없습니다.';return;}
  await engine.load();
  if(latest)await engine.update(latest,latestIntegration);
  status.title='원본 STL 전체 형상이 아닌 삼각형 축소판을 표시합니다.';
 };
 const hasGL=()=>{try{const c=document.createElement('canvas');return Boolean(c.getContext('webgl2')||c.getContext('webgl'));}catch(_){return false;}};
 if(hasGL()){
  try{
   status.textContent='원본 STL을 지원하는 Three.js 렌더러 준비 중…';
   if(!window.THREE)await script('https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js');
   if(!THREE.OrbitControls)await script('https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js');
   if(!THREE.STLLoader)await script('https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/loaders/STLLoader.js');
   if(await window.PACThreeViewer3D.load()){
     engine=window.PACThreeViewer3D;
     if(latest)engine.update(latest,latestIntegration);
     return;
   }
  }catch(_){/* no network or WebGL: fall back to bundled Plotly */}
 }
 status.textContent='로컬 3D 대체 렌더러 사용 중 · 원본 STL 전체 렌더링은 Three.js 필요';
 await fallback();
}
window.PACViewer3D={
 load:()=>engine?Promise.resolve(true):Promise.resolve(false),
 update:(live,integration)=>{latest=live;latestIntegration=integration;if(engine)engine.update(live,integration);},
 runKey:key=>{
  if(!key)return;
  if(boundKey===null){boundKey=key;return;}
  if(boundKey!==key){
   // A world changed. Do not leave static primitives from the old world on screen.
   // Reload after the user finishes an in-flight question; the server and Gazebo stay up.
   const t=document.getElementById('question');
   const status=$('viewer3d-status');if(status)status.textContent='실행 월드 변경 감지 · 3D 배경 재연결 필요';
   if(!document.body.dataset.rebinding){document.body.dataset.rebinding='1';
    const reload=()=>{if(document.getElementById('send')?.disabled || t?.value.trim()){
         setTimeout(reload,1500);return;}location.reload();};
    setTimeout(reload,700);}
  }
 },
 setView:name=>{requestedView=name;if(engine)engine.setView(name);}
};
function ready(){
 for(const mode of ['all','robot','pallet','pick']){
  $('viewer3d-'+mode)?.addEventListener('click',()=>window.PACViewer3D.setView(mode));
 }
 boot().catch(()=>{$('viewer3d-status').textContent='3D 렌더러 초기화 실패 · 브라우저 콘솔 확인';});
}
if(document.readyState==='loading')document.addEventListener('DOMContentLoaded',ready);else ready();
})();
