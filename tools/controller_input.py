"""Local Mac HID display; --drive uses Wi-Fi, --drive --serial PORT enables diagnostic USB."""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import importlib.util,json,subprocess,threading,time,atexit,sys,urllib.parse,queue,datetime
BASE=Path(__file__).with_name('controller_input')
HTML=BASE/'index.html'
sys.path.insert(0,str(BASE))
from drive_bridge import DriveBridge
DRIVE_MODE='--drive' in sys.argv
SERIAL_PORT=None
if '--serial' in sys.argv:
    i=sys.argv.index('--serial')
    if not DRIVE_MODE or i+1>=len(sys.argv):raise SystemExit('Usage: controller_input.py --drive --serial /dev/cu.usbserial-10')
    SERIAL_PORT=sys.argv[i+1]
    from serial_client import SerialClient
spec=importlib.util.spec_from_file_location('hid_decode',BASE/'hid_decode.py')
decoder=importlib.util.module_from_spec(spec);spec.loader.exec_module(decoder)
lock=threading.Lock()
state=dict(available=False,openResult=None,gamepad=None,reports=0,changes=0,updated=0.0)
reader=None

def consume(process):
    previous=None
    for line in process.stdout:
        try:
            event=json.loads(line)
            if 'openResult' in event:
                with lock: state['openResult']=event['openResult'];state['available']=event['openResult']==0
                continue
            gamepad=decoder.decode_report(event['reportId'],bytes.fromhex(event['hex']))
            if gamepad is None:continue
            signature=(tuple(gamepad['rawAxes']),tuple(b['pressed'] for b in gamepad['buttons']))
            with lock:
                state['reports']+=1
                if previous is not None and signature!=previous:state['changes']+=1
                state['gamepad']=gamepad;state['updated']=time.monotonic()
            previous=signature
        except (ValueError,KeyError,TypeError):continue
    with lock:state['available']=False

def get_hid_state():
    with lock: result=state.copy()
    age=time.monotonic()-result.pop('updated')
    result['ageMs']=int(age*1000) if result['reports'] else None
    if age>0.15:result['gamepad']=None
    return result
event_queue=queue.Queue(maxsize=256)
event_path=Path(__file__).resolve().parents[1]/'diagnostics'/('controller_events_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'.jsonl')
def queue_event(event):
    try:event_queue.put_nowait(dict(event,wallTime=datetime.datetime.now().isoformat()))
    except queue.Full:pass
def write_events():
    event_path.parent.mkdir(parents=True,exist_ok=True)
    with event_path.open('a') as stream:
        while True:
            event=event_queue.get();stream.write(json.dumps(event,ensure_ascii=False)+'\n');stream.flush()
bridge=DriveBridge(get_hid_state,SerialClient(SERIAL_PORT) if SERIAL_PORT else None,event_sink=queue_event) if DRIVE_MODE else None

class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        path=self.path.split('?',1)[0]
        if path=='/api/drive-state' and bridge is not None:
            result=bridge.view();result['hid']=get_hid_state()
            data=json.dumps(result,ensure_ascii=False).encode();mime='application/json'
        elif path=='/drive' and bridge is not None:
            data=(BASE/'drive.html').read_bytes();mime='text/html; charset=utf-8'
        elif path=='/api/hid-state':
            with lock:
                result=state.copy()
            age=time.monotonic()-result.pop('updated')
            result['ageMs']=int(age*1000) if result['reports'] else None
            if age>0.5:result['gamepad']=None
            data=json.dumps(result,ensure_ascii=False).encode();mime='application/json'
        elif path in ('/','/index.html'):
            data=HTML.read_bytes();mime='text/html; charset=utf-8'
        else:self.send_error(404);return
        self.send_response(200)
        self.send_header('Content-Type',mime)
        self.send_header('Content-Length',str(len(data)))
        self.send_header('Cache-Control','no-store')
        self.send_header('Content-Security-Policy',"default-src 'self'; script-src 'self' 'unsafe-inline'; style-src 'self' 'unsafe-inline'; connect-src 'self'; object-src 'none'; frame-ancestors 'none'")
        self.end_headers();self.wfile.write(data)
    def do_POST(self):
        if bridge is None:self.send_error(405);return
        try:
            length=int(self.headers.get('Content-Length','0'))
            if length<0 or length>512:raise ValueError('invalid_request')
            form=urllib.parse.parse_qs(self.rfile.read(length).decode(),keep_blank_values=True)
            token=self.headers.get('X-Control-Token','') or (form.get('token') or [''])[0]
            if token!=bridge.token:self.send_error(403);return
            origin=self.headers.get('Origin')
            if origin not in (None,'http://127.0.0.1:8766','http://localhost:8766'):
                self.send_error(403);return
            if self.path=='/api/control/arm':
                if (form.get('confirm') or [''])[0]!='yes':raise ValueError('confirmation_required')
                signs=[int((form.get('s'+str(i)) or ['1'])[0]) for i in (1,2,3)]
                owner=(form.get('owner') or [''])[0]
                if len(owner)!=32 or any(c not in '0123456789abcdef' for c in owner):raise ValueError('invalid_owner')
                mode=(form.get('mode') or ['drive'])[0]
                angle_cdeg=int((form.get('angleCdeg') or ['9000'])[0])
                bridge.arm(signs,owner,mode,angle_cdeg)
            elif self.path=='/api/control/lease':
                if not bridge.lease((form.get('owner') or [''])[0]):raise ValueError('inactive_owner')
            elif self.path=='/api/control/stop':
                reason=(form.get('reason') or ['manual'])[0]
                bridge.disarm({'tab_hidden':'操作タブが非表示になったため停止','page_exit':'操作画面を閉じたため停止','lease_failed':'操作画面の維持通信が失敗したため停止'}.get(reason,'画面の停止ボタンで停止'))
            elif self.path=='/api/control/config':
                values=[int((form.get(k) or [''])[0]) for k in ('motor','kpMilli','kiMilli')]
                bridge.configure(*values,int((form.get('lowKpMilli') or ['0'])[0]))
            else:self.send_error(404);return
            data=b'{"ok":true}';code=200
        except (ValueError,UnicodeDecodeError) as e:
            data=json.dumps(dict(error=str(e))).encode();code=409
        except (OSError,TimeoutError,RuntimeError) as e:
            data=json.dumps(dict(error=str(e))).encode();code=503
        self.send_response(code);self.send_header('Content-Type','application/json')
        self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(data)))
        self.end_headers();self.wfile.write(data)
    def log_message(self,*args):pass

if __name__=='__main__':
    binary=Path('/private/tmp/c620_hid_reader')
    source=BASE/'hid_reader.m'
    if not binary.exists() or binary.stat().st_mtime<source.stat().st_mtime:
        subprocess.run(['clang','-fobjc-arc','-framework','Foundation','-framework','IOKit',str(source),'-o',str(binary)],check=True)
    reader=subprocess.Popen([str(binary)],stdout=subprocess.PIPE,text=True,bufsize=1)
    atexit.register(reader.terminate)
    threading.Thread(target=consume,args=(reader,),daemon=True).start()
    if bridge is not None:
        threading.Thread(target=write_events,daemon=True).start()
        threading.Thread(target=bridge.serve,daemon=True).start()
    print('Controller: http://127.0.0.1:8766/'+('drive ('+getattr(bridge.client,'transport','Wi-Fi')+' DISARMED)' if bridge else ' (display only)'),flush=True)
    try:ThreadingHTTPServer(('127.0.0.1',8766),Handler).serve_forever()
    finally:
        if bridge is not None:
            bridge._stop('ローカルサーバー終了')
            if hasattr(bridge.client,'close'):bridge.client.close()
        reader.terminate()
