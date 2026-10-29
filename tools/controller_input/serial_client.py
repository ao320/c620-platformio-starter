"""Opt-in USB serial diagnostic backend. Same bounded ESP32 speed controller."""
import json,secrets,threading,time
from omni_mix import MAX_RPM
from runtime_settings import apply as apply_settings
from runtime_settings import apply_limit

class SerialClient:
    transport='USBシリアル（診断）'
    def __init__(self,path):
        try:import serial
        except ImportError as e:raise RuntimeError('シリアル診断はPlatformIOのPythonで起動してください（pyserialが必要です）') from e
        self.port=serial.Serial(port=None,baudrate=115200,timeout=.1,write_timeout=.2,exclusive=True)
        self.port.dtr=False;self.port.rts=False;self.port.port=path;self.port.open()
        self.lock=threading.Lock();self.write_lock=threading.Lock()
        self.latest=None;self.updated=0.;self.error=None;self.closed=False
        threading.Thread(target=self._read,daemon=True).start()
    def _read(self):
        buffer=b''
        try:
            while not self.closed:
                buffer+=self.port.read(4096)
                if len(buffer)>16384:buffer=b''
                while b'\n' in buffer:
                    line,buffer=buffer.split(b'\n',1)
                    if not line.startswith(b'STATUS '):continue
                    try:status=json.loads(line[7:])
                    except (ValueError,UnicodeError):continue
                    with self.lock:self.latest=status;self.updated=time.monotonic()
        except Exception as e:
            with self.lock:self.error=f"{type(e).__name__}: {e}"
    def status(self):
        with self.lock:
            if self.error:raise ConnectionError(self.error)
            if self.latest is None or time.monotonic()-self.updated>1.8:raise TimeoutError('serial_status_stale')
            return self.latest.copy()
    def _write(self,data):
        with self.write_lock:
            if self.closed:raise ConnectionError('serial_closed')
            count=self.port.write(data)
            if count!=len(data):raise ConnectionError('serial_short_write')
    def velocity(self,run,targets):
        if len(targets)!=3 or any(type(v)!=int or abs(v)>MAX_RPM for v in targets):raise ValueError('invalid_target')
        if run==0:
            if any(targets):raise ValueError('neutral_start_required')
            run=secrets.randbits(31)|0x80000000
            deadline=time.monotonic()+1.4
            # Repeat ONLY neutral while waiting for once-per-second telemetry.
            # Never send a nonzero target until the ESP32 acknowledges the run.
            while time.monotonic()<deadline:
                self._send(run,[0,0,0])
                status=self.status()
                if status.get('fault'):raise RuntimeError('serial_can_fault')
                if status.get('run')==run and status.get('velocity') and status.get('phase')=='running':return run
                time.sleep(.05)
            self.stop();raise TimeoutError('serial_neutral_start_unconfirmed')
        self._send(run,targets)
        return run
    def _send(self,run,targets):
        if type(run)!=int or not 0<run<=0xffffffff:raise ValueError('invalid_run')
        self._write(('V,'+str(run)+','+','.join(map(str,targets))+'\n').encode('ascii'))
    def stop(self):self._write(b'z\n')
    def position(self,run,angle_cdeg):
        if type(angle_cdeg)!=int or not angle_cdeg or abs(angle_cdeg)>18000:raise ValueError('invalid_angle')
        self._write(f'A,{run},{angle_cdeg}\n'.encode('ascii'))
        deadline=time.monotonic()+1.4
        while time.monotonic()<deadline:
            self.position_heartbeat(run)
            state=self.status();p=state.get('position') or {}
            if state.get('run')==run and p.get('requestedCdeg')==angle_cdeg and (p.get('active') or p.get('reason')=='complete'):return
            if state.get('fault'):raise RuntimeError('position_start_fault')
            time.sleep(.05)
        self.stop();raise TimeoutError('position_start_unconfirmed')
    def position_heartbeat(self,run):self._write(f'H,{run}\n'.encode('ascii'))
    def configure(self,motor,kp,ki,low_kp=0):
        return apply_settings(self,motor,kp,ki,
            lambda request:self._write((f'P,{request},{motor},{kp},{ki}'+(f',{low_kp}' if low_kp else '')+'\n').encode('ascii')),low_kp)
    def configure_current(self,limit):
        return apply_limit(self,limit,lambda request:self._write(f'C,{request},{limit}\n'.encode('ascii')))
    def close(self):
        if not self.closed:
            self.closed=True;self.port.close()
