"""Dedicated bounded ID4 arm UI. USB initial validation; no chassis commands."""
import atexit
import importlib.util
import json
from pathlib import Path
import secrets
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

BASE=Path(__file__).with_name('controller_input')
sys.path.insert(0,str(BASE))
from serial_client import SerialClient
from recovering_serial import RecoveringSerialClient
from hid_decode import decode_report

class ArmBridge:
    def __init__(self,client,hid,clock=time.monotonic):
        self.client=client;self.hid=hid;self.clock=clock;self.lock=threading.RLock()
        self.enabled=False;self.run=0;self.ui_at=0.;self.started=0.;self.state={};self.error='受信待ち'
        self.comparison=False
        self.standalone_mode=False
        self.controller_mode=False
        self.auto_mode=False
        self.needs_home=False;self.transport_error=False
        self.requested_at=0.
        self.wait_release=True;self.token=secrets.token_hex(24)
    def stop(self,why):
        self.enabled=False;self.run=0;self.auto_mode=False;self.error=why
        self.client._write(b'ARM,STOP\n')
    def start_auto(self):
        with self.lock:
            s=self.client.status();m=s.get('id4',{});p=s.get('positionCounts',-1)
            if self.enabled or self.run or s.get('active') or s.get('zeroing'):raise ValueError('停止完了後に開始してください')
            if self.needs_home or not s.get('homed') or s.get('fault') or not s.get('healthy') or not s.get('canReady'):
                raise ValueError('原点・CAN・故障状態を確認してください')
            if not s.get('actuationEnabled') or 'autoActive' not in s or s.get('autoLatched') or s.get('overspeedRpm')!=1000:
                raise ValueError('自動試験対応版と停止条件を確認してください')
            if s.get('manualCurrentLimitRaw')!=6000 or not 15000<=p<=35000 or m.get('ageMs',999)>20 or abs(m.get('rpm',999))>5 or m.get('temperature',999)>=60:
                raise ValueError('6000 raw・範囲内・停止・温度を確認してください')
            self.comparison=False;self.standalone_mode=False;self.controller_mode=False;self.auto_mode=True
            self.enabled=True;self.requested_at=self.ui_at=self.started=self.clock();self.run=secrets.randbits(31)+1
            self.error='自動往復の開始確認待ち';self.client._write(f'ARM,AUTO,{self.run}\n'.encode())
    def home(self):
        with self.lock:
            if self.enabled:raise ValueError('先に無効化してください')
            self.client._write(b'ARM,HOME\n')
            self.needs_home=False
    def configure_current(self,value):
        with self.lock:
            if type(value)!=int or value<100 or value>6000 or value%100:raise ValueError('100〜6000 rawを100刻みで指定してください')
            s=self.client.status()
            if self.enabled or self.run or s.get('active') or s.get('zeroing'):raise ValueError('操作無効・停止完了後に変更してください')
            if 'manualCurrentLimitRaw' not in s:raise ValueError('電流設定対応ファームウェアが必要です')
            request=secrets.randbits(31)+1
            self.client._write(f'ARM,LIMIT,{request},{value}\n'.encode())
            deadline=self.clock()+2
            while self.clock()<deadline:
                s=self.client.status()
                if s.get('limitAck')==request:
                    if not s.get('limitAccepted') or s.get('manualCurrentLimitRaw')!=value:raise ValueError('ESPが設定を拒否しました。停止と受信を確認してください')
                    self.state=s;self.error=f'電流上限{value} rawをESPへ反映済み';return
                time.sleep(.03)
            raise TimeoutError('設定の反映未確認です。状態表示を確認してください')
    def arm(self):
        with self.lock:
            s=self.client.status();g,age=self.hid()
            if self.needs_home:raise ValueError('USB再接続後は実際にAへ合わせて原点を確定してください')
            if s.get('overspeedRpm')!=1000:raise ValueError('停止速度1000 RPMのファームウェアを確認してください')
            if not s.get('actuationEnabled'):raise ValueError('駆動禁止ビルドです。承認後の書き込みが必要です')
            if not s.get('armFirmware') or not s.get('homed') or s.get('active') or s.get('fault') or not s.get('healthy'):raise ValueError('原点・CAN状態を確認してください')
            if not g or age>.15:raise ValueError('コントローラーの受信待ち')
            if any(g['buttons'][i]['pressed'] for i in (4,5,12,13)):raise ValueError('L・R・十字キーを離してください')
            p=s.get('positionCounts',-1);m=s['marginCounts']
            if not 0<=p<=s['spanCounts']-m:raise ValueError('A位置からB側95％境界の内側へ移してください')
            self.comparison=False;self.standalone_mode=False;self.controller_mode=False
            self.enabled=True;self.run=0;self.wait_release=True;self.ui_at=self.started=self.clock();self.error="操作有効：Lを保持している間だけ動作・位置保持します。時間上限なし"
    def compare(self):
        with self.lock:
            s=self.client.status()
            if not s.get('comparisonSupported') or s.get('positionCounts',0)+2000>s.get('spanCounts',0)-s.get('marginCounts',0):
                raise ValueError('比較試験用ファームウェアまたはB側の余裕を確認してください')
            self.arm()
            self.comparison=True;self.requested_at=self.clock();self.run=secrets.randbits(31)+1
            self.error=f"比較試験：B側最大2000カウント・{s.get('manualCurrentLimitRaw',500)} raw・2秒"
            self.client._write(f'ARM,COMPARE,{self.run}\n'.encode())
    def standalone(self,controller_mode=False):
        with self.lock:
            s=self.client.status();m=s.get('id4',{})
            if self.enabled or s.get('active') or s.get('zeroing'):raise ValueError('すでに実行中または停止処理中です')
            if not s.get('standaloneSupported') or not s.get('actuationEnabled'):raise ValueError('単体試験用ファームウェアが必要です')
            if s.get('standalone') and s.get('session') and not s.get('standaloneRepeatable'):raise ValueError('今回の単体試験は終了しています。結果を確認してください')
            if not s.get('healthy') or not s.get('canReady') or m.get('ageMs',999)>20 or abs(m.get('rpm',999))>5 or m.get('temperature',999)>=60:
                raise ValueError('CAN・受信・停止・温度を確認してください')
            self.controller_mode=controller_mode
            self.enabled=True;self.comparison=True;self.standalone_mode=True
            self.requested_at=self.ui_at=self.started=self.clock();self.run=secrets.randbits(31)+1
            self.error='単体試験：最大2000カウント・1000 raw・2秒'
            self.client._write(f'ARM,FREE,{self.run}\n'.encode())
    def arm_standalone_controller(self):
        with self.lock:
            s=self.client.status();g,age=self.hid();m=s.get('id4',{})
            if self.enabled or s.get('active') or s.get('zeroing'):raise ValueError('停止完了を待ってください')
            if not s.get('standaloneSupported') or not s.get('healthy') or not s.get('canReady') or m.get('ageMs',999)>20 or abs(m.get('rpm',999))>5:raise ValueError('単体モード・CAN・停止状態を確認してください')
            if not g or age>.15 or any(g['buttons'][i]['pressed'] for i in (4,5,12,13)):raise ValueError('コントローラーを接続しL・R・十字キーを離してください')
            self.controller_mode=True;self.standalone_mode=True;self.comparison=True
            self.enabled=True;self.run=0;self.wait_release=True;self.ui_at=self.started=self.clock()
            self.error='単体コントローラー待機：Lを保持し十字キー上で1回実行'
    def tick(self):
        with self.lock:
            try:
                self.state=self.client.status()
                if self.transport_error:
                    self.error="USB受信復帰・操作無効。Aで原点を確認し直してください"
                    self.transport_error=False
                if self.error in ("受信待ち","serial_status_stale"):
                    self.error="受信正常・操作は未有効"
            except Exception as e:
                if self.enabled:
                    try:self.stop('シリアル受信停止')
                    except Exception:pass
                self.enabled=False;self.run=0;self.wait_release=True;self.needs_home=True;self.transport_error=True
                self.state={};self.error='USB受信停止・再接続中: '+str(e)
                if hasattr(self.client,'reconnect'):
                    self.client.reconnect()
                return
            if not self.enabled:return
            now=self.clock();g,age=self.hid()
            if self.auto_mode:
                if now-self.ui_at>.4:self.stop('画面の通信停止');return
                if self.state.get('session')==self.run and not self.state.get('active'):
                    self.stop('ESP側終了: '+self.state.get('reason',''));return
                if self.state.get('session')!=self.run and now-self.requested_at>1.8:
                    self.stop('自動往復の開始未確認');return
                self.client._write(f'ARM,AUTOLEASE,{self.run}\n'.encode());return
            if now-self.ui_at>.4 or (not self.run and now-self.started>30) or ((not self.standalone_mode or self.controller_mode) and (not g or age>.15)):
                self.stop('入力または画面の通信停止');return
            buttons=g['buttons'] if g else [dict(pressed=False) for _ in range(18)]
            if buttons[5]['pressed']:self.stop('Rで電流0');return
            if self.controller_mode:
                if self.run and (not buttons[4]['pressed'] or buttons[13]['pressed']):self.stop('L解放または十字下で電流0');return
                if not self.run:
                    if not any(buttons[i]['pressed'] for i in (4,12,13)):self.wait_release=False
                    if self.wait_release or not buttons[4]['pressed'] or not buttons[12]['pressed'] or buttons[13]['pressed']:return
                    self.enabled=False
                    self.standalone(controller_mode=True)
            if self.run and not self.comparison and not buttons[4]['pressed']:self.stop('Lを離したため電流0');return
            if self.run and self.state.get('session')==self.run and not self.state.get('active'):
                self.stop('ESP側終了: '+self.state.get('reason',''));return
            if not self.run:
                if not buttons[4]['pressed']:self.wait_release=False;return
                if self.wait_release:return
                if buttons[12]['pressed'] or buttons[13]['pressed']:return
                self.requested_at=now
                self.run=secrets.randbits(31)+1
                self.client._write(f'ARM,ENABLE1000,{self.run}\n'.encode())
            if self.state.get('session')!=self.run and now-self.requested_at>1.8:
                self.stop('ESPが開始を確認できません。位置・受信状態を確認してください');return
            direction=0 if self.comparison else int(buttons[12]['pressed'])-int(buttons[13]['pressed'])
            self.client._write(f'ARM,MOVE,{self.run},{direction}\n'.encode())
    def view(self):
        with self.lock:
            g,age=self.hid();e=dict(self.state)
            if self.needs_home:e['homed']=False;e['referenceNeedsConfirmation']=True
            keys={name:bool(g and g['buttons'][i]['pressed']) for name,i in [('L',4),('R',5),('up',12),('down',13)]}
            serial_ok=False
            try:self.client.status();serial_ok=True
            except Exception:pass
            p=e.get('positionCounts',-1);margin=e.get('marginCounts',2838)
            checks=[dict(label='ESP受信',ok=serial_ok),
                    dict(label='CAN正常',ok=bool(e.get('healthy') and e.get('canReady') and not e.get('fault'))),
                    dict(label='Aで原点確定',ok=bool(e.get('homed'))),
                    dict(label='開始位置がA〜B側95％境界の内側',ok=bool(e.get('homed') and 0<=p<=e.get('spanCounts',0)-margin)),
                    dict(label='コントローラー受信',ok=bool(g and age<=.15)),
                    dict(label='駆動有効版',ok=bool(e.get('actuationEnabled')))]
            checks.append(dict(label='過速度停止1000 motor RPM',ok=e.get('overspeedRpm')==1000))
            ready=all(c['ok'] for c in checks)
            ack=bool(self.run and e.get('session')==self.run and e.get('active') and serial_ok)
            m=e.get('id4',{})
            standalone_ready=bool(serial_ok and e.get('standaloneSupported') and e.get('actuationEnabled') and e.get('canReady') and e.get('healthy') and not e.get('active') and not self.enabled and not e.get('zeroing') and (e.get('standaloneRepeatable') or not (e.get('standalone') and e.get('session'))) and m.get('ageMs',999)<=20 and abs(m.get('rpm',999))<=5 and m.get('temperature',999)<60)
            if ack and self.auto_mode:stage='moving';title='自動往復中：第'+str(e.get('autoLeg',0))+'区間'
            elif ack and self.standalone_mode:stage='moving';title='単体試験実行中：軸・歯車に触れないでください'
            elif ack and self.comparison:stage='moving';title='比較試験中：B側へ最大2000カウント・最大2秒'
            elif ack:stage='moving' if keys['up']!=keys['down'] else 'holding';title='操作中：B側へ' if keys['up'] and not keys['down'] else ('操作中：A側へ' if keys['down'] and not keys['up'] else '位置保持中')
            elif self.run:stage='pending';title='ESPの開始確認待ち'
            elif self.enabled and self.controller_mode:stage='armed';title='単体操作待機：Lを保持して十字キー上を押すと1回回転'
            elif self.enabled:
                stage='armed';title='L・十字キーを一度離してください' if self.wait_release else ('十字キーを離してからLを押してください' if keys['up'] or keys['down'] else '操作待機：Lを押すと保持開始')
            elif standalone_ready:stage='ready';title='単体試験の準備完了：原点確定・L操作は不要です'
            elif e.get('standaloneSupported') and e.get('standalone') and not self.enabled:stage='ready';title='単体試験終了：下の結果を確認してください'
            elif ready:stage='ready';title='準備完了：操作待機を有効化し、L＋方向キーで操作'
            else:stage='blocked';title='準備が必要：'+next(c['label'] for c in checks if not c['ok'])
            limit=1000 if self.standalone_mode else e.get('manualCurrentLimitRaw',500)
            seconds=2 if self.comparison else None
            return dict(token=self.token,enabled=self.enabled,error=self.error,esp=e,controllerConnected=bool(g and age<=.15),
                        controllerAgeMs=round(age*1000),limits=dict(maxMotorRpm=e.get('maxJogUpRpm',60) if not self.comparison else 60,maxDescentRpm=e.get('maxDescentRpm',60),gravityAware=bool(e.get('gravityAware')),currentRaw=limit,seconds=seconds,maxSessionCounts=2000 if self.comparison or (limit>500 and e.get('jogLimitEnabled',True)) else None),otherMotorCommands=[0,0,0],
                        stage=stage,title=title,checks=checks,buttons=keys,canStandalone=standalone_ready,controllerMode=self.controller_mode,autoMode=self.auto_mode,canAuto=bool(serial_ok and not self.enabled and not self.needs_home and e.get('homed') and not e.get('active') and not e.get('fault') and e.get('healthy') and e.get('canReady') and e.get('actuationEnabled') and 'autoActive' in e and not e.get('autoLatched') and e.get('overspeedRpm')==1000 and e.get('manualCurrentLimitRaw')==6000 and 15000<=p<=35000 and m.get('ageMs',999)<=20 and abs(m.get('rpm',999))<=5 and m.get('temperature',999)<60),canController=bool(standalone_ready and g and age<=.15),canArm=ready and not self.enabled,canCompare=bool(ready and not self.enabled and e.get('comparisonSupported') and p+2000<=e.get('spanCounts',0)-margin),
                        espConfirmedActive=ack,remainingSeconds=max(0,seconds-(self.clock()-self.requested_at)) if self.run and seconds is not None else None)

PAGE=Path(__file__).with_name('arm_controller.html').read_text()


def main():
    client=RecoveringSerialClient(sys.argv[1] if len(sys.argv)>1 else '/dev/cu.usbserial-10')
    binary=Path('/private/tmp/c620_hid_reader');source=BASE/'hid_reader.m'
    if not binary.exists() or binary.stat().st_mtime<source.stat().st_mtime:
        subprocess.run(['clang','-fobjc-arc','-framework','Foundation','-framework','IOKit',str(source),'-o',str(binary)],check=True)
    reader=subprocess.Popen([str(binary)],stdout=subprocess.PIPE,text=True,bufsize=1)
    latest=[None,0.];hid_lock=threading.Lock()
    def consume():
        for line in reader.stdout:
            try:
                event=json.loads(line);g=decode_report(event['reportId'],bytes.fromhex(event['hex']))
                if g:
                    with hid_lock:latest[:]=[g,time.monotonic()]
            except (ValueError,KeyError):pass
    def hid():
        with hid_lock:return latest[0],time.monotonic()-latest[1]
    bridge=ArmBridge(client,hid)
    log_path=Path(__file__).resolve().parents[1]/'diagnostics'/('arm_control_'+time.strftime('%Y%m%d_%H%M%S')+'.jsonl')
    def control_loop():
        previous=None
        stream=log_path.open('a')
        while True:
            try:
                bridge.tick()
                state=bridge.view()
                uptime=state['esp'].get('uptimeMs')
                if uptime!=previous:
                    state.pop('token',None);state['wallTime']=time.strftime('%Y-%m-%dT%H:%M:%S');stream.write(json.dumps(state,ensure_ascii=False)+'\n');stream.flush();previous=uptime
            except Exception as e:
                with bridge.lock:bridge.enabled=False;bridge.run=0;bridge.error=str(e)
            time.sleep(.01)
    threading.Thread(target=consume,daemon=True).start();threading.Thread(target=control_loop,daemon=True).start()
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def reply(self,code,value,html=False):
            data=value.encode() if html else json.dumps(value,ensure_ascii=False).encode()
            self.send_response(code);self.send_header('Content-Type','text/html; charset=utf-8' if html else 'application/json; charset=utf-8');self.send_header('Cache-Control','no-store');self.send_header('X-Frame-Options','DENY');self.send_header('Content-Length',str(len(data)));self.end_headers();self.wfile.write(data)
        def do_GET(self):
            if self.path in ('/','/arm'):self.reply(200,Path(__file__).with_name('arm_controller.html').read_text(),True)
            elif self.path=='/status':self.reply(200,bridge.view())
            else:self.reply(404,{})
        def do_POST(self):
            if self.headers.get('X-Control-Token')!=bridge.token:return self.reply(403,dict(error='画面を再読み込みしてください'))
            try:
                with bridge.lock:
                    if self.path in ('/arm','/compare','/auto'):
                        setting=bridge.client.status().get('manualCurrentLimitRaw',500)
                        if self.headers.get('X-Arm-Limit')!=str(setting):raise ValueError('電流上限が変わりました。表示条件を確認し直してください')
                        if self.path in ('/compare','/auto') and setting>2000 and self.headers.get('X-High-Current-Confirm')!='yes':raise ValueError('高トルクの試験条件と支持固定の確認が必要です')
                    if self.path=='/home':bridge.home()
                    elif self.path=='/auto':
                        if self.headers.get('X-Arm-Confirm')!='yes':raise ValueError('表示条件と支持固定を確認してください')
                        bridge.start_auto()
                    elif self.path=='/arm':
                        bridge.arm()
                    elif self.path=='/current-limit':
                        size=int(self.headers.get('Content-Length','0'))
                        if not 0<size<100:raise ValueError('invalid_request')
                        bridge.configure_current(json.loads(self.rfile.read(size))['limit'])
                    elif self.path=='/standalone-controller':
                        if self.headers.get('X-Standalone-Safe')!='yes':raise ValueError('単体固定の確認が必要です')
                        bridge.arm_standalone_controller()
                    elif self.path=='/standalone':
                        if self.headers.get('X-Standalone-Safe')!='yes':raise ValueError('単体固定の確認が必要です')
                        bridge.standalone()
                    elif self.path=='/compare':bridge.compare()
                    elif self.path=='/stop':bridge.stop('画面から電流0')
                    elif self.path=='/lease':bridge.ui_at=time.monotonic()
                    else:return self.reply(404,{})
                self.reply(200,dict(ok=True))
            except Exception as e:self.reply(409,dict(error=str(e)))
    print('Arm controller: http://127.0.0.1:8766/arm (DISABLED)',flush=True)
    try:ThreadingHTTPServer(('127.0.0.1',8766),Handler).serve_forever()
    finally:
        if bridge.run:bridge.stop('サーバー終了')
        client.close();reader.terminate()
if __name__=='__main__':main()
