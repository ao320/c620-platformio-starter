#pragma once
#include <Arduino.h>
static const char WEB_UI[] PROGMEM = R"HTML(<!doctype html>
<html lang="ja"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>C620 Motor Control</title><style>
:root{color-scheme:dark;--bg:#10151e;--panel:#1a2230;--line:#303c50;--muted:#a8b6ca;--text:#f1f5fb;--blue:#77baff;--green:#80e0b7;--red:#ff9c9c}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font-size:15px;font-family:system-ui,-apple-system,sans-serif;line-height:1.65}main{max-width:1040px;margin:auto;padding:32px 22px 50px}.eyebrow{color:var(--blue);font-size:12px;letter-spacing:.18em;font-weight:700}header{display:flex;justify-content:space-between;align-items:start;gap:20px}h1{font-size:clamp(26px,5vw,38px);line-height:1.2;margin:10px 0}p{margin:8px 0}h2{margin:0;font-size:18px}.muted{color:var(--muted)}.badge{padding:7px 13px;border-radius:30px;border:1px solid var(--line);white-space:nowrap;color:var(--muted)}.badge.good{color:var(--green);border-color:#3a7960}.badge.bad{color:var(--red);border-color:#945252}.strip{display:flex;gap:22px;flex-wrap:wrap;border-block:1px solid var(--line);padding:17px 0;margin:25px 0}.strip strong{color:var(--blue)}.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:16px}.card{padding:22px;background:var(--panel);border:1px solid var(--line);border-radius:16px}.card-top{display:flex;justify-content:space-between;align-items:center;gap:8px}.online{font-size:12px;color:var(--green)}.offline{color:var(--red)}.rpm{font-size:42px;font-weight:650;line-height:1.25;margin:21px 0 0;font-variant-numeric:tabular-nums}.rpm small{font-size:14px;font-weight:400;color:var(--muted)}.facts{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin:18px 0}.facts span{display:block;color:var(--muted);font-size:12px}.facts b{font-weight:500}.button{font:inherit;font-weight:650;border-radius:10px;padding:13px 18px;cursor:pointer;border:1px solid #8ac6ff;background:#94ccff;color:#102235;min-height:50px;width:100%}.button:hover:enabled{filter:brightness(1.1)}.button:disabled{opacity:.35;cursor:not-allowed}.button:focus-visible,input:focus-visible{outline:3px solid white;outline-offset:4px}.settings-row{display:flex;gap:12px;align-items:end;flex-wrap:wrap;margin:14px 0}.settings-field{flex:1;min-width:165px}.settings-field label{display:block;font-size:13px;color:var(--muted);margin-bottom:4px}.settings-field input{width:100%;padding:12px;border:1px solid var(--line);border-radius:10px;background:var(--bg);color:var(--text);font:inherit;min-height:50px}.settings-row .button{width:auto;min-width:140px}.settings-message{min-height:1.6em;color:#ffd5a0}.high-current{color:#ffcf9b}.control{margin-top:20px;border:1px solid var(--line);border-radius:16px;padding:24px}.confirm{display:flex;align-items:start;gap:12px;padding:18px 0;cursor:pointer}.confirm input{width:22px;height:22px;margin-top:3px;accent-color:#94ccff;flex-shrink:0}.actions{display:grid;grid-template-columns:1fr 1fr;gap:14px}.stop{background:#622e36;border-color:#e78c98;color:#fff}.status{margin-top:20px;display:flex;align-items:center;gap:12px}.dot{width:9px;height:9px;background:var(--blue);border-radius:50%;flex-shrink:0}progress{width:100%;height:7px;accent-color:var(--blue);margin-top:16px}.note{font-size:13px;color:var(--muted)}.errors{font-variant-numeric:tabular-nums}footer{margin-top:24px;color:var(--muted);font-size:12px}#message{min-height:1.6em;color:#ffd5a0}.running{border-color:var(--blue)}@media(max-width:700px){main{padding:24px 16px}.grid{grid-template-columns:1fr}.card{padding:18px}.rpm{margin-top:14px}.facts{margin:14px 0}.actions{grid-template-columns:1fr}header{flex-direction:column}.control{padding:18px}}
</style></head><body><main>
<header><div><div class="eyebrow">ESP32 / C620</div><h1>3つのモーターを操作</h1><p class="muted">個別に、または3台同時に、1秒だけ動かせます。</p></div><div id="connection" class="badge" role="status">接続を確認中</div></header>
<div class="strip"><span>1秒駆動の電流指令 <strong id="pulse-current">—</strong></span><span>駆動時間 <strong>最大1秒</strong></span><span>終了後 <strong>0指令を1秒</strong></span></div>
<div class="grid">
<section class="card" id="card1"><div class="card-top"><h2>モーター 1</h2><span id="link1" class="online offline">未確認</span></div><div class="rpm"><span id="rpm1">—</span> <small>RPM</small></div><div class="facts"><div><span>温度</span><b id="temp1">—</b></div><div><span>今回の最大RPM</span><b id="peak1">—</b></div></div><button class="button start" data-mask="1" disabled>1台目を1秒動かす</button></section>
<section class="card" id="card2"><div class="card-top"><h2>モーター 2</h2><span id="link2" class="online offline">未確認</span></div><div class="rpm"><span id="rpm2">—</span> <small>RPM</small></div><div class="facts"><div><span>温度</span><b id="temp2">—</b></div><div><span>今回の最大RPM</span><b id="peak2">—</b></div></div><button class="button start" data-mask="2" disabled>2台目を1秒動かす</button></section>
<section class="card" id="card3"><div class="card-top"><h2>モーター 3</h2><span id="link3" class="online offline">未確認</span></div><div class="rpm"><span id="rpm3">—</span> <small>RPM</small></div><div class="facts"><div><span>温度</span><b id="temp3">—</b></div><div><span>今回の最大RPM</span><b id="peak3">—</b></div></div><button class="button start" data-mask="4" disabled>3台目を1秒動かす</button></section>
</div>
<p class="note">RPMはC620のモーターフィードバック値です。タイヤの回転数ではありません。同じ電流指令でも回転数はそろいません。</p>
<section class="control"><h2>電流指令の上限設定</h2>
<p>現在の設定 <strong id="current-live">—</strong> raw / 設定範囲 <span id="current-range">1000〜6000</span> raw</p>
<form id="current-form"><div class="settings-row"><div class="settings-field"><label for="current-input">電流指令の上限（raw値）</label><input id="current-input" type="number" min="1000" max="6000" step="100" inputmode="numeric" required></div><button class="button" id="current-save" type="submit" disabled>停止中に設定する</button></div></form>
<p class="note">速度PI制御の上限と、この画面の1秒駆動の電流指令に反映します。設定は再起動すると1000に戻ります。</p>
<p class="note high-current">大きい値では急に強い力が出ます。最初は1000で確認し、機体を固定して段階的に上げてください。</p>
<p class="settings-message" id="current-message" role="status" aria-live="polite"></p></section>
<section class="control"><h2>実行前の確認</h2><label class="confirm"><input id="armed" type="checkbox"><span>タイヤを床から浮かせ、機体を固定しました。手・配線を回転部から離し、すぐ電源を切れる状態です。</span></label><div class="actions"><button id="all" class="button start" data-mask="7" disabled>3台を同時に1秒動かす</button><button id="stop" class="button stop">全台の電流を0にする</button></div><p class="note">0指令はブレーキではありません。惰性で回るため、完全に止まるまで触れないでください。</p><div class="status"><span class="dot"></span><strong id="phase" role="status">ESP32に接続しています</strong></div><progress id="progress" max="1000" value="0" aria-label="駆動の残り時間"></progress><p id="message" role="alert"></p><p class="note errors" id="errors">CAN診断を取得中</p></section>
<footer>専用Wi-Fi：C620-Control · この画面はESP32内で動作します。再接続・再読み込みでは駆動しません。</footer>
</main><script>
'use strict';
const byId=id=>document.getElementById(id);
const starts=Array.from(document.querySelectorAll('.start'));
let state=null,token='',connected=false,busy=false,activeRun=0,heartbeatBusy=false;
let limitDirty=false,pendingLimit=null,configRequestId=Math.floor(Math.random()*4294967294)+1,knownBootId=null;
const reasons={ready:'実行できます',running:'駆動中',complete:'試験終了',user_stop:'0指令に切り替えました',browser_lost:'画面との通信が途切れ、0指令へ切り替えました',feedback_lost:'フィードバックが途切れました。配線を確認してください',can_fault:'CAN異常です。原因を確認してからESP32を再起動してください',tx_failed:'CAN送信失敗です。配線・電源を確認してください'};
function buttons(){
  const enabled=connected&&state&&state.ready&&!state.observationOnly&&byId('armed').checked&&!busy&&!pendingLimit&&!document.hidden;
  starts.forEach(b=>b.disabled=!enabled);
  const settings=state&&state.settings;
  const input=byId('current-input');
  if(settings){input.min=String(settings.minCurrentLimitRaw);input.max=String(settings.maxCurrentLimitRaw)}
  const value=input.value.trim(),num=Number(value);
  const valid=/^\d+$/.test(value)&&Number.isSafeInteger(num)&&settings&&num>=settings.minCurrentLimitRaw&&num<=settings.maxCurrentLimitRaw&&num%100===0;
  byId('current-save').disabled=!(connected&&state&&state.ready&&!state.observationOnly&&!busy&&!pendingLimit&&!document.hidden&&valid&&num!==settings.currentLimitRaw);
}
async function request(path,body){const controller=new AbortController();const timeout=setTimeout(()=>controller.abort(),700);try{const options={cache:'no-store',signal:controller.signal};if(body!==undefined){options.method='POST';options.headers={'X-Control-Token':token,'Content-Type':'application/x-www-form-urlencoded'};options.body=new URLSearchParams(body)}const response=await fetch(path,options);const value=await response.json();if(!response.ok)throw new Error(value.error||'request_failed');return value}finally{clearTimeout(timeout)}}
function lost(){connected=false;byId('connection').textContent='接続が途切れました';byId('connection').className='badge bad';byId('armed').checked=false;byId('phase').textContent='再接続を待っています';buttons()}
function render(s){
  if(knownBootId!==null&&s.bootId!==knownBootId){
    activeRun=0;byId('armed').checked=false;pendingLimit=null;limitDirty=false;
    byId('current-message').textContent='ESP32が再起動しました。設定値を再確認してください。';
  }
  knownBootId=s.bootId;
  state=s;token=s.token;connected=true;
  byId('connection').textContent=s.observationOnly?'観測専用':s.fault?'CAN異常':s.canReady?'ESP32 接続済み':'CAN初期化失敗';
  byId('connection').className='badge '+(s.observationOnly||s.fault||!s.canReady?'bad':'good');
  const settings=s.settings;
  if(settings){
    byId('current-live').textContent=settings.currentLimitRaw.toLocaleString();
    byId('pulse-current').textContent='＋'+settings.currentLimitRaw.toLocaleString()+' raw';
    byId('current-range').textContent=settings.minCurrentLimitRaw+'〜'+settings.maxCurrentLimitRaw;
    if(pendingLimit&&settings.ack===pendingLimit.id){
      const accepted=settings.accepted&&settings.currentLimitRaw===pendingLimit.value;
      byId('current-message').textContent=accepted?'電流指令の上限を '+pendingLimit.value+' raw に変更しました。':'設定が反映されませんでした。停止状態を確認してください。';
      pendingLimit=null;limitDirty=false;
    }
    if(pendingLimit&&Date.now()>pendingLimit.deadline){
      byId('current-message').textContent='設定の確認がタイムアウトしました。現在の表示値を確認してください。';
      pendingLimit=null;limitDirty=false;
    }
    if(!limitDirty&&!pendingLimit)byId('current-input').value=String(settings.currentLimitRaw);
  }
for(const m of s.motors){const fresh=m.seen&&m.ageMs<=100;byId('link'+m.id).textContent=fresh?'受信中':'受信なし';byId('link'+m.id).className='online'+(fresh?'':' offline');byId('rpm'+m.id).textContent=fresh?m.rpm.toLocaleString():'—';byId('temp'+m.id).textContent=fresh?m.temperature+' ℃':'—';byId('peak'+m.id).textContent=m.peakRpm.toLocaleString();byId('card'+m.id).classList.toggle('running',s.phase==='running'&&!!(s.mask&(1<<(m.id-1))))}
const moving=s.motors.some(m=>m.rpm!==0&&m.seen&&m.ageMs<=100);
let label=s.observationOnly?'観測専用モード（駆動無効）':s.phase==='running'?'駆動中 · 残り '+(s.remainingMs/1000).toFixed(1)+'秒':s.phase==='zeroing'?'全台へ0指令を送信中':s.fault?(reasons[s.reason]||'異常を確認してください'):s.ready?'実行できます':moving?'惰性回転中 · 完全停止を待っています':'3台の正常な受信を待っています';byId('phase').textContent=label;byId('progress').value=s.phase==='running'?s.remainingMs:0;byId('errors').textContent='CANエラー '+s.errors+' / 送信失敗 '+s.failed+' / TEC '+s.tec+' / REC '+s.rec;
if(activeRun===s.run&&s.phase!=='running')activeRun=0;buttons()}
async function poll(){try{render(await request('/api/status'))}catch(e){lost()}finally{setTimeout(poll,200)}}
async function start(mask){if(!connected||!state||!state.ready||state.observationOnly||pendingLimit||!byId('armed').checked||busy||document.hidden)return;busy=true;byId('message').textContent='';buttons();try{const result=await request('/api/start',{mask:String(mask),confirm:'yes'});activeRun=result.run;byId('armed').checked=false;byId('message').textContent='実行を受け付けました。完全に止まるまで触れないでください。'}catch(e){byId('message').textContent=e.message==='not_ready'?'まだ実行できません。停止・受信状態を確認してください。':'実行結果を確認できません。再実行せず、停止状態を確認してください。';byId('armed').checked=false;stopBeacon()}finally{busy=false;buttons()}}
async function saveCurrentLimit(){
  buttons();if(byId('current-save').disabled)return;
  const value=Number(byId('current-input').value);
  configRequestId=configRequestId>=4294967295?1:configRequestId+1;
  const id=configRequestId;
  pendingLimit={id,value,deadline:Date.now()+3000};limitDirty=false;
  byId('current-message').textContent='ESP32への反映を確認中…';buttons();
  try{
    await request('/api/current-limit',{request:String(id),limit:String(value)});
  }catch(e){
    if(pendingLimit&&pendingLimit.id===id){
      pendingLimit=null;limitDirty=true;
      byId('current-message').textContent='設定要求を確認できませんでした（'+e.message+'）。現在値を確認してください。';
    }
  }finally{buttons()}
}
async function stop(){byId('armed').checked=false;buttons();try{await request('/api/stop',{});activeRun=0;byId('message').textContent='全台への0指令を要求しました。完全停止を確認してください。'}catch(e){stopBeacon();byId('message').textContent='停止要求の到達を確認できません。回転が続く場合は電源を切ってください。'}}
function stopBeacon(){if(token&&activeRun){navigator.sendBeacon('/api/stop',new URLSearchParams({token}));activeRun=0}}
starts.forEach(button=>button.addEventListener('click',()=>start(Number(button.dataset.mask))));
byId('current-input').addEventListener('input',()=>{limitDirty=true;buttons()});
byId('current-form').addEventListener('submit',event=>{event.preventDefault();saveCurrentLimit()});
byId('armed').addEventListener('change',buttons);byId('stop').addEventListener('click',stop);
document.addEventListener('visibilitychange',()=>{if(document.hidden){byId('armed').checked=false;stopBeacon();buttons()}});window.addEventListener('pagehide',stopBeacon);
setInterval(async()=>{if(!activeRun||document.hidden||heartbeatBusy)return;heartbeatBusy=true;try{await request('/api/heartbeat',{run:String(activeRun)})}catch(e){/* ESP32 deadline and lease remain authoritative; never restart. */}finally{heartbeatBusy=false}},100);
poll();
</script></body></html>)HTML";
