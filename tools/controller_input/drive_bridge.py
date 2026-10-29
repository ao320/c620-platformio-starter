"""Wi-Fi bridge: Mac HID -> 120-degree omni targets -> ESP32 UDP.
HTTP is used only for stopped configuration. USB is opt-in diagnostic only.
"""
import json,secrets,threading,time,urllib.request,urllib.parse
from udp_link import UdpLink
from omni_mix import mix
from runtime_settings import apply as apply_settings, validate as validate_settings
from runtime_settings import apply_limit

class EspClient:
    transport='Wi-Fi UDP'
    def __init__(self):
        self.opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
        self.token=''
        self.link=UdpLink()
    def request(self,path,fields=None):
        body=None if fields is None else urllib.parse.urlencode(fields).encode()
        headers={} if fields is None else {'X-Control-Token':self.token,'Content-Type':'application/x-www-form-urlencoded'}
        req=urllib.request.Request('http://192.168.4.1'+path,data=body,headers=headers)
        with self.opener.open(req,timeout=.18) as r:return json.load(r)
    def status(self):
        status=self.link.status();self.token=status['token'];return status
    def velocity(self,run,targets):
        return self.link.velocity(run,targets)
    def stop(self):return self.link.stop()
    def position(self,run,angle_cdeg):return self.link.position(run,angle_cdeg)
    def position_heartbeat(self,run):return self.link.position_heartbeat(run)
    def close(self):self.link.close()
    def configure(self,motor,kp,ki,low_kp=0):
        return apply_settings(self,motor,kp,ki,lambda request:self.request('/api/config',
            dict(request=request,motor=motor,kpMilli=kp,kiMilli=ki,lowKpMilli=low_kp)),low_kp)
    def configure_current(self,limit):
        return apply_limit(self,limit,lambda request:self.request('/api/current-limit',dict(request=request,limit=limit)))

class DriveBridge:
    def __init__(self,hid_provider,client=None,clock=time.monotonic,event_sink=None):
        self.hid_provider=hid_provider;self.client=client or EspClient();self.clock=clock
        self.lock=threading.Lock();self.token=secrets.token_hex(16)
        self.armed=False;self.owner='';self.armed_at=0.;self.ui_at=0.;self.run=0;self.wait_release=True
        self.signs=[1,1,1];self.message='未有効 · モーターへ送信していません'
        self.esp=None;self.esp_at=0.;self.last_poll=-10.;self.last_send=-10.
        self.targets=[0,0,0];self.connected=False;self.force_stop=False
        self.connection_error=''
        self.mode='drive';self.angle_cdeg=9000
        self.event_sink=event_sink;self.last_stop=None;self.stop_events=[]
        self.max_send_gap_ms=0.;self.max_tick_gap_ms=0.;self.last_tick=None;self.hid_age_ms=None
    def view(self):
        with self.lock:
            return dict(token=self.token,transport=getattr(self.client,'transport','Wi-Fi'),armed=self.armed,owner=self.owner,run=self.run,message=self.message,
                        mode=self.mode,angleCdeg=self.angle_cdeg,
                        signs=list(self.signs),targets=list(self.targets),connected=self.connected,
                        esp=self.esp,espAgeMs=int((self.clock()-self.esp_at)*1000) if self.esp else None,connectionError=self.connection_error,
                        lastStop=self.last_stop,stopEvents=list(self.stop_events),maxSendGapMs=round(self.max_send_gap_ms,1),maxTickGapMs=round(self.max_tick_gap_ms,1),
                        secondsLeft=max(0,int(60-(self.clock()-self.armed_at))) if self.armed else 0)
    def _record_stop_locked(self,message):
        if not (self.armed or self.run):return
        now=self.clock();esp=self.esp or {}
        event=dict(reason=message,monotonic=now,run=self.run,armedForMs=round((now-self.armed_at)*1000),
            uiAgeMs=round((now-self.ui_at)*1000),hidAgeMs=self.hid_age_ms,
            sendGapMs=round((now-self.last_send)*1000) if self.run else None,maxSendGapMs=round(self.max_send_gap_ms,1),
            maxTickGapMs=round(self.max_tick_gap_ms,1),connectionError=self.connection_error,
            espReason=esp.get('reason'),link=esp.get('link'),canErrors=esp.get('errors'))
        self.last_stop=event;self.stop_events.append(event);self.stop_events=self.stop_events[-30:]
        if self.event_sink:self.event_sink(event)
    def lease(self,owner='test'):
        with self.lock:
            if self.armed and owner==self.owner:
                self.ui_at=self.clock();return True
            return False
    def disarm(self,message='画面から停止要求'):
        with self.lock:
            self._record_stop_locked(message)
            self.armed=False;self.owner='';self.message=message;self.wait_release=True;self.force_stop=True
    def arm(self,signs,owner='test',mode='drive',angle_cdeg=9000):
        if mode not in ('drive','angle') or type(angle_cdeg)!=int or not angle_cdeg or abs(angle_cdeg)>18000:raise ValueError('invalid_angle_mode')
        if len(signs)!=3 or any(type(s)!=int or s not in (-1,1) for s in signs):raise ValueError('invalid_signs')
        hid=self.hid_provider();g=hid.get('gamepad')
        if not g or hid.get('ageMs',10000)>150:raise ValueError('controller_not_ready')
        if not mix(g['axes'])['neutral'] or g['buttons'][4]['pressed'] or g['buttons'][5]['pressed']:
            raise ValueError('release_sticks_and_shoulders')
        with self.lock:
            if self.armed or self.run:raise ValueError('already_active')
            if not self.connected or not self.esp or self.clock()-self.esp_at>.8 or not self.esp.get('ready'):
                raise ValueError('esp_not_ready')
            if 'velocity' not in self.esp:raise ValueError('firmware_update_required')
            if mode=='angle' and ('position' not in self.esp or self.esp.get('settings',{}).get('currentLimitRaw')!=1000):raise ValueError('position_firmware_or_current_limit')
            self.mode=mode;self.angle_cdeg=angle_cdeg
            self.signs=list(signs);self.armed=True;self.owner=owner;self.armed_at=self.ui_at=self.clock()
            self.max_send_gap_ms=0.;self.max_tick_gap_ms=0.
            self.wait_release=True;self.message='有効 · スティックを中央にしてLを押し続けてください'
    def configure(self,motor,kp,ki,low_kp=0):
        validate_settings(motor,kp,ki,low_kp)
        # Keep arm() blocked until the stopped-only transaction is acknowledged.
        with self.lock:
            if self.armed or self.run or self.force_stop:raise ValueError('disable_controls_and_wait_for_stop')
            state=self.client.configure(motor,kp,ki,low_kp) if low_kp else self.client.configure(motor,kp,ki)
            self.esp=state;self.esp_at=self.clock();self.connected=True
            self.message='PI設定を反映しました（停止中・再起動で初期値に戻ります）'
    def _stop(self,message,disarm=True,force=False):
        with self.lock:
            self._record_stop_locked(message)
            run=self.run;self.run=0;self.targets=[0,0,0];self.wait_release=True
            if disarm:self.armed=False;self.owner=''
            self.message=message
        if run or force:
            try:self.client.stop()
            except Exception:
                with self.lock:self.message=message+'（停止要求の到達未確認・ESP側250ms制限）'
    def tick(self):
        now=self.clock();hid=self.hid_provider();g=hid.get('gamepad')
        with self.lock:
            self.hid_age_ms=hid.get('ageMs')
            if self.last_tick is not None and (self.armed or self.run):self.max_tick_gap_ms=max(self.max_tick_gap_ms,(now-self.last_tick)*1000)
            self.last_tick=now
        fresh=g is not None and hid.get('ageMs',10000)<=150
        with self.lock:armed=self.armed;run=self.run;ui_at=self.ui_at;armed_at=self.armed_at
        with self.lock:
            force=self.force_stop;self.force_stop=False;message=self.message
        if force:self._stop(message,force=True);return
        if run or armed:
            if not armed:self._stop('操作を無効にしました');return
            if not fresh:self._stop('コントローラー切断・入力途絶');return
            if now-ui_at>.4:self._stop('操作画面との通信が途絶えました');return
            if now-armed_at>60:self._stop('60秒の操作枠が終了しました。再確認してください');return
            if g['buttons'][5]['pressed']:self._stop('Rボタンで停止しました',force=True);return
            if run and not g['buttons'][4]['pressed']:self._stop('Lを離したため0指令へ切り替えました',False);return
        if now-self.last_poll>=.25:
            self.last_poll=now
            try:
                esp=self.client.status()
                with self.lock:self.esp=esp;self.esp_at=self.clock();self.connected=True;self.connection_error=''
                if (armed or run) and (esp.get('fault') or not esp.get('canReady')):
                    if esp.get('reason')=='position_fault':
                        self._stop('角度制御停止: '+str((esp.get('position') or {}).get('reason','unknown')))
                    elif esp.get('reason')=='overspeed':
                        threshold=esp.get('settings',{}).get('feedbackStopRpm','設定値')
                        self._stop('速度超過停止（実測'+str(threshold)+' RPM超）')
                    else:self._stop('ESP32 / CAN異常')
                    return
                if run and self.mode=='angle' and not (esp.get('position') or {}).get('active'):
                    p=esp.get('position') or {}
                    if p.get('reason')=='complete':self._stop('角度制御完了 · 実測 '+str(p.get('actualCdeg',0)/100)+'°')
                    else:self._stop('角度制御終了: '+str(p.get('reason','unknown')))
                    return
                if run and (esp.get('run')!=run or esp.get('phase')!='running' or not esp.get('velocity')):
                    self._stop('ESP32側で操作が終了しました');return
            except Exception as error:
                with self.lock:self.connected=False;self.connection_error=type(error).__name__+': '+str(error)
                if armed or run:self._stop('ESP32との通信が途絶えました')
                return
        if not armed:return
        with self.lock:
            if not self.armed:return
            run=self.run;wait=self.wait_release;signs=list(self.signs);esp=self.esp
        l=g['buttons'][4]['pressed']
        preview=mix(g['axes'],signs)
        if not l:
            with self.lock:self.wait_release=False;self.targets=[0,0,0]
            return
        if not run:
            if wait:return
            with self.lock:self.wait_release=True
            if not preview['neutral']:
                with self.lock:self.message='スティックを中央に戻し、Lを離してから押してください'
                return
            if not esp or not esp.get('ready'):return
            try:
                run=self.client.velocity(0,[0,0,0])
                with self.lock:self.run=run;self.last_send=self.clock()
                if self.mode=='angle':self.client.position(run,self.angle_cdeg)
                with self.lock:self.run=run;self.last_send=self.clock();self.message='Lを保持中 · 左スティックで移動、右で旋回'
                if self.mode=='angle':
                    with self.lock:self.message='ID1角度制御中 · L保持、Rで停止。ID2〜4の指令は0'
            except Exception:self._stop('操作開始を確認できません。再有効化してください')
            return
        if now-self.last_send>=.05:
            try:
                if self.mode=='angle':
                    if not preview['neutral']:self._stop('角度試験中のスティック入力で停止');return
                    self.client.position_heartbeat(run)
                else:self.client.velocity(run,preview['targets'])
                with self.lock:
                    sent=self.clock();self.max_send_gap_ms=max(self.max_send_gap_ms,(sent-self.last_send)*1000)
                    self.targets=[(self.esp.get('position') or {}).get('targetRpm',0),0,0] if self.mode=='angle' else preview['targets'];self.last_send=sent
            except Exception:self._stop('目標値送信に失敗しました。自動再送しません')
    def serve(self):
        while True:
            try:self.tick()
            except Exception:self._stop('入力処理で異常を検出しました')
            time.sleep(.01)
