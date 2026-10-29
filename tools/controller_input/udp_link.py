"""UDP control with independent read-only telemetry receiver.

Motor targets are never retransmitted. Start/stop require confirmation;
telemetry waits never block the regular command sender. ESP lease stays 250 ms.
"""
import json,secrets,socket,threading,time
from omni_mix import MAX_RPM

class UdpLink:
    def __init__(self,address=('192.168.4.1',4210),clock=time.monotonic):
        self.clock=clock;self.lock=threading.Condition(threading.RLock())
        self.address=(socket.gethostbyname(address[0]),address[1])
        self.sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0',0));self.sock.settimeout(.02)
        self.latest=None;self.received=0.;self.token='';self.run=0;self.seq=0
        self.last_rtt_ms=None;self.max_rtt_ms=0.;self.status_misses=0
        self.pending={};self.stop_ack=0;self.restarted=False;self.closed=False;self.last_error=''
        self.route_ip=''
        self.receiver=threading.Thread(target=self._receive,daemon=True);self.receiver.start()
    def _receive(self):
        next_query=0.;next_route=0.
        while not self.closed:
            now=self.clock()
            if now>=next_route:
                next_route=now+1.
                try:
                    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as probe:
                        probe.connect(self.address);self.route_ip=probe.getsockname()[0]
                except OSError:self.route_ip='unreachable'
            with self.lock:
                for nonce,issued in list(self.pending.items()):
                    if now-issued>.35:
                        del self.pending[nonce];self.status_misses+=1;self.last_error='udp_response_timeout'
                if now>=next_query:
                    nonce=secrets.randbits(31) or 1;self.pending[nonce]=now;next_query=now+.1
                    try:self.sock.sendto(f'Q,{nonce}'.encode('ascii'),self.address)
                    except OSError as error:
                        self.pending.pop(nonce,None);self.status_misses+=1;self.last_error=str(error)
            try:
                packet,source=self.sock.recvfrom(4096)
                if source!=self.address:continue
                data=json.loads(packet)
            except socket.timeout:continue
            except (ValueError,UnicodeError):continue
            except OSError as error:
                if self.closed:return
                with self.lock:self.last_error=str(error)
                time.sleep(.02);continue
            if not isinstance(data,dict):continue
            now=self.clock()
            with self.lock:
                if data.get('stopAccepted') is True and type(data.get('seq')) is int:
                    self.stop_ack=data['seq'];self.lock.notify_all();continue
                nonce=data.get('nonce')
                if type(nonce)!=int:continue
                issued=self.pending.pop(nonce,None)
                if issued is None or now-issued>.35:continue
                token=data.get('token');uptime=data.get('uptimeMs')
                if data.get('udpVersion')!=1 or type(uptime)!=int or not 0<=uptime<=0xffffffff or not isinstance(token,str) or len(token)!=32 or any(c not in '0123456789abcdef' for c in token):
                    self.last_error='udp_firmware_required';continue
                if self.token and self.token!=token and self.run:
                    self.restarted=True;self.run=0
                self.latest=data;self.received=now;self.token=token;self.last_error=''
                self.last_rtt_ms=round((now-issued)*1000,1);self.max_rtt_ms=max(self.max_rtt_ms,self.last_rtt_ms)
                self.lock.notify_all()
    def status(self):
        with self.lock:
            deadline=self.clock()+.3
            while self.latest is None and self.clock()<deadline and not self.closed:
                self.lock.wait(max(.001,deadline-self.clock()))
            if self.restarted:
                self.restarted=False;raise RuntimeError('esp_restarted')
            if self.latest is None or self.clock()-self.received>.5:
                raise TimeoutError((self.last_error or 'udp_status_stale')+'; routeSource='+self.route_ip)
            state=self.latest.copy()
            state['link']=dict(protocol='UDP',ageMs=round((self.clock()-self.received)*1000),rttMs=self.last_rtt_ms,
                maxRttMs=self.max_rtt_ms,statusMisses=self.status_misses,localBind=self.sock.getsockname()[0],routeSourceIp=self.route_ip)
            return state
    def _packet(self,kind,run,targets=None):
        if not self.latest or self.clock()-self.received>.5:raise TimeoutError('udp_status_stale')
        self.seq+=1
        expires=(self.latest['uptimeMs']+int((self.clock()-self.received)*1000)+120)&0xffffffff
        fields=[kind,self.token,str(run),str(self.seq),str(expires)]
        if targets is not None:fields.extend(map(str,targets))
        return ','.join(fields)
    def velocity(self,run,targets):
        if len(targets)!=3 or any(type(v)!=int or abs(v)>MAX_RPM for v in targets):raise ValueError('invalid_target')
        with self.lock:
            if run==0:
                if any(targets):raise ValueError('neutral_start_required')
                if not self.latest or not self.latest.get('ready'):raise ValueError('esp_not_ready')
                run=secrets.randbits(31)|0x80000000;self.run=run
                self.sock.sendto(self._packet('V',run,[0,0,0]).encode('ascii'),self.address)
                deadline=self.clock()+.22
                while self.clock()<deadline:
                    state=self.latest or {}
                    if state.get('run')==run and state.get('phase')=='running' and state.get('velocity'):return run
                    self.lock.wait(max(.001,deadline-self.clock()))
                raise TimeoutError('udp_neutral_start_unconfirmed')
            if run!=self.run:raise ValueError('invalid_udp_run')
            self.sock.sendto(self._packet('V',run,targets).encode('ascii'),self.address)
            return run
    def stop(self):
        with self.lock:
            if not self.run:return
            packet=self._packet('S',self.run);sequence=self.seq;self.stop_ack=0
            self.sock.sendto(packet.encode('ascii'),self.address)
            deadline=self.clock()+.15
            while self.clock()<deadline:
                if self.stop_ack==sequence:self.run=0;return
                self.lock.wait(max(.001,deadline-self.clock()))
            raise TimeoutError('udp_stop_unconfirmed')
    def position(self,run,angle_cdeg):
        if type(angle_cdeg)!=int or not angle_cdeg or abs(angle_cdeg)>18000:raise ValueError('invalid_angle')
        with self.lock:
            if run!=self.run or not self.latest or 'position' not in self.latest:raise ValueError('position_firmware_required')
            self.sock.sendto(self._packet('A',run,[angle_cdeg]).encode('ascii'),self.address)
            deadline=self.clock()+.22
            while self.clock()<deadline:
                state=self.latest or {};p=state.get('position') or {}
                if state.get('run')==run and p.get('active') and p.get('requestedCdeg')==angle_cdeg:return
                self.lock.wait(max(.001,deadline-self.clock()))
            raise TimeoutError('position_start_unconfirmed')
    def position_heartbeat(self,run):
        with self.lock:
            if run!=self.run:raise ValueError('invalid_udp_run')
            self.sock.sendto(self._packet('H',run).encode('ascii'),self.address)
    def close(self):
        self.closed=True
        with self.lock:self.lock.notify_all()
        self.sock.close();self.receiver.join(timeout=.1)
