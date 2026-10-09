"""Browser visual smoke: ALL scene/telemetry/replies are MOCK, not real Gazebo."""
import re,json
from pathlib import Path
from playwright.sync_api import sync_playwright
ROOT=Path(__file__).resolve().parents[1]
html=(ROOT/'web/dashboard.html').read_text()
inline=html.split('<script>')[-1].split('</script>')[0]
html=re.sub(r'<script(?:\s[^>]*)?>[\s\S]*?</script>','',html)
world={'world':'mock','primitives':[
 {'model':'floor','shape':'box','size':[11.2,7,.1],'position':[-2.1,-.5,-.05],'quaternion':[0,0,0,1],'color':'#566a7e'},
 {'model':'conveyor_main','shape':'box','size':[6.19,.83,.2],'position':[-3.9,1.2,.8],'quaternion':[0,0,0,1],'color':'#96a9b9'},
 {'model':'pallet_main','shape':'box','size':[1.1,1.1,.15],'position':[0,1.2,.075],'quaternion':[0,0,0,1],'color':'#b88f64'},
 {'model':'buffer_table','shape':'box','size':[.9,.66,.06],'position':[-1,.44,.92],'quaternion':[0,0,0,1],'color':'#567f8a'},
 {'model':'camera','shape':'box','size':[.16,.1,.09],'position':[-.6,1.95,2.2],'quaternion':[0,0,0,1],'color':'#8c9da9'}],
 'skipped_mesh_or_unsupported':0,'error':''}
links=['world','base_link','lower_frame_link','upper_frame_link','arm_link','wrist_body_link','wrist_holder_link','flange_link']
parents=links[:-1]
joints=[{'name':'world_joint' if i==0 else f'j{i}','type':'fixed' if i==0 else 'revolute','parent':parents[i],'child':links[i+1],
  'origin':[0,0,.4] if i==0 else ([.7,0,0] if i>3 else [0,0,.6]),'quaternion':[0,0,0,1],'axis':[0,0,1] if i==1 else [0,1,0]} for i in range(7)]
ref=[{'name':f'j{i}','position':.18*i,'velocity':None} for i in range(1,7)]
robot={'joints':joints,'meshes':[],'reference_joints':ref,'skipped_meshes':0,'source':'mock_urdf','error':'mock geometry'}
live={'available':True,'fresh':True,'age_s':.1,'source_age_s':{'gazebo':.2,'joint_states':.1,'scale':.1,'cctv':None,'controller':None,'suction':None},
 'camera_jpeg_ok':{'scale':False,'cctv':False},'camera_jpeg_age_s':{'scale':None,'cctv':None},
 'data':{'gazebo':{'topic':'/world/mock/dynamic_pose/info','error':'','entities':[{'name':'box_01','x':-1.4,'y':1.2,'z':1.0,'size':[.42,.3,.3],'quaternion':[0,0,0,1]}]},
 'joint_states':{'topic':'/joint_states','joints':ref},'controller':{'max_abs_position_error':None,'topic':''},'suction':{'value':''},
 'cameras':{'scale':{'topic':'/pac/scale_camera/image','observed_epoch_s':1},'cctv':{'topic':'','observed_epoch_s':None}}}}
context={'run_id':'T_BUFFER_SWAP_20261010_044913','last_log_age_s':2953.8,'observation':'failure_logged',
'alert':{'id':'abc','text':'MOVEIT PICK&PLACE FAIL: vertical pre-alignment exceeds 0.5 mm edge budget; no descent','source':'logs/box_13_moveit.log'},
'evidence':[], 'collected_at':'2026-10-09T20:40:53Z','stop_confirmed':None,'suction':'unknown'}
state={'schema_version':'1.0','telemetry':live,'context':context,'failure_stage':{'description':'수직 하강 전에 정렬 실패'},
       'api_configured':False,'offline_questions':True,'model':'gpt-4o-mini'}
reply={'schema_version':'1.0','request_id':'abc','session_id':'','created_at':'2026-10-10T06:20:00Z',
 'status':'ok','source':'local','answer':'이전 실행에서 수직 정렬 허용 범위를 초과한 기록이 있습니다. 실제 원인은 추가 확인이 필요합니다.',
 'confirmed_facts':[],'hypotheses':[],'checks':[],'actions':[],'missing_information':['실패 단계 상세 로그'],
 'context':context,'control':{'mode':'read_only','command_executed':False}}
with sync_playwright() as p:
 browser=p.chromium.launch(headless=True,executable_path='/usr/bin/chromium',args=['--no-sandbox','--disable-webgl'])
 page=browser.new_page(viewport={'width':1650,'height':1000},device_scale_factor=1)
 errors=[];page.on('pageerror',lambda e:errors.append(str(e)))
 page.set_content(html, wait_until='domcontentloaded')
 page.evaluate('''m=>{window.fetch=async(path,options)=>({ok:true,json:async()=>
    path.includes('/api/robot3d')?m.robot:path.includes('/api/scene3d')?m.world:
    path.includes('/api/ask')?m.reply:m.state});}''',{'world':world,'robot':robot,'state':state,'reply':reply})
 for file in ['vendor/plotly.min.js','viewer3d.js','viewer3d_three.js','viewer3d_boot.js']:
  page.add_script_tag(path=str(ROOT/'web'/file))
 page.add_script_tag(content=inline)
 page.wait_for_timeout(1600)
 print('V5 scene status:',page.locator('#viewer3d-status').inner_text())
 print('joints:',page.locator('#metric-joints').inner_text())
 print('camera:',page.locator('#metric-camera').inner_text())
 print('offline send enabled:',page.locator('#send').is_enabled())
 assert page.locator('#canvas3d-fallback').count()==1
 assert page.locator('#metric-joints').inner_text()=='6 / 6'
 assert page.locator('#metric-camera').inner_text()=='0 / 2', 'ROS Image alone must not claim JPEG'
 assert page.locator('#send').is_enabled()
 page.get_by_role('button',name='마지막 실패 원인').click()
 page.wait_for_timeout(160)
 assert '수직 정렬' in page.locator('.chat-assistant .chat-answer').inner_text()
 assert page.locator('#new').is_enabled()
 page.locator('#question').fill('현재 로봇 관절은?')
 page.locator('#question').press('Enter')
 page.wait_for_timeout(200)
 assert page.locator('.chat-turn').count()==2
 page.screenshot(path=str(ROOT/'preview_v5_mock.png'),full_page=False)
 page.get_by_role('button',name='상태 패널 접기').click()
 assert 'side-collapsed' in page.locator('body').get_attribute('class')
 assert not errors,errors
 browser.close()
print('MOCK browser smoke PASS; screenshot',ROOT/'preview_v5_mock.png')
