const {test}=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const vm=require('node:vm');
const path=require('node:path');
const source=fs.readFileSync(path.join(__dirname,'../web/viewer3d.js'),'utf8');
function setup(gl){
 const calls={fills:0,plots:[]};
 const ctx=new Proxy({}, {get:(o,k)=>k==='fill'?()=>calls.fills++:(o[k]||(()=>{}))});
 const elements=new Map();
 const element=()=>({style:{},textContent:'',width:1400,height:700,replaceChildren(){},appendChild(){},on(){},addEventListener(){},getContext:k=>k==='2d'?ctx:(gl?{}:null)});
 const window={};
 const world={primitives:[{shape:'box',model:'pallet',size:[1,1,.2],position:[0,0,.1],quaternion:[0,0,0,1],color:'#aaaaaa'}]};
 const document={getElementById:id=>{if(!elements.has(id))elements.set(id,element());return elements.get(id);},createElement:element};
 let now=10000;
 vm.runInNewContext(source,{window,document,console,Date:{now:()=>now+=1000},
  Plotly:{newPlot:async()=>{},relayout:()=>{},react:async(_,traces)=>calls.plots.push(traces)},
  fetch:async url=>({ok:true,json:async()=>url==='/api/scene3d'?world:{joints:[],meshes:[],reference_joints:[]}})});
 return {api:window.PACPlotlyViewer3D,calls,elements};
}
const live={available:true,age_s:0,source_age_s:{gazebo:0,joint_states:0},data:{joint_states:{joints:[]},gazebo:{entities:[
 {name:'hdr50_22::suction_tcp',x:0,y:0,z:1},
 {name:'custom_box_A',x:1,y:0,z:.5,size:[.3,.4,.5]}
]}}};
for(const gl of [false,true])test(gl?'Plotly: robot entity and registered box render repeatedly':'Canvas: robot entity cannot clear the scene with ReferenceError',async()=>{
 const {api,calls,elements}=setup(gl);
 await api.load();
 await api.update(live,{boxes:[{id:'custom_box_A'}]});
 api.setView('all');
 await api.update({...live,data:{...live.data,gazebo:{entities:live.data.gazebo.entities.map(e=>({...e,x:e.x+.1}))}}},{boxes:[{id:'custom_box_A'}]});
 if(gl){assert.equal(calls.plots.length,2);assert(calls.plots[0].some(t=>t.name==='Gazebo에서 치수까지 확인한 박스'));}
 else {assert(calls.fills>=30);assert.match(elements.get('viewer3d-status').textContent,/Canvas/);}
});
