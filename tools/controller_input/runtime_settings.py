"""Stopped-only PI settings protocol. Does not send motor targets."""
import secrets,time

def validate(motor,kp,ki,low_kp=0):
    if type(motor)!=int or motor not in (1,2,3):raise ValueError('invalid_motor')
    if any(type(v)!=int or not 0<=v<=10000 for v in (kp,ki,low_kp)):raise ValueError('invalid_gain')

def initial(client):
    state=client.status()
    if not state.get('ready') or state.get('phase')!='idle' or state.get('fault'):raise ValueError('esp_not_stopped_ready')
    if 'settings' not in state:raise ValueError('firmware_update_required')
    return state

def wait_ack(client,request):
    deadline=time.monotonic()+2.5
    while time.monotonic()<deadline:
        try:
            state=client.status()
        except TimeoutError:
            # Wait for telemetry; never retransmit an unconfirmed setting.
            time.sleep(.05)
            continue
        if not state.get('ready') or state.get('phase')!='idle' or state.get('fault'):
            raise ValueError('esp_not_stopped_ready')
        if state.get('settings',{}).get('ack')==request:
            if not state['settings'].get('accepted'):raise ValueError('esp_settings_rejected')
            return state
        time.sleep(.05)
    raise TimeoutError('settings_result_unknown_read_status_before_retry')

def request_id(state):
    request=secrets.randbits(32) or 1
    if request==state['settings'].get('ack'):request=request%0xffffffff+1
    return request

def apply(client,motor,kp,ki,send,low_kp=0):
    validate(motor,kp,ki,low_kp)
    state=initial(client);request=request_id(state)
    send(request)  # Exactly once. A timeout means unknown outcome, never retry.
    state=wait_ack(client,request);settings=state['settings']
    actual=next((m for m in settings.get('motors',[]) if m['id']==motor),None)
    if not actual or actual.get('kpMilli')!=kp or actual.get('kiMilli')!=ki or actual.get('lowKpMilli',0)!=low_kp:raise ValueError('settings_readback_mismatch')
    return state

def apply_limit(client,limit,send):
    if type(limit)!=int or not 1000<=limit<=1500:raise ValueError('invalid_current_limit')
    state=initial(client)
    if state['settings'].get('maxCurrentLimitRaw',1000)<limit:raise ValueError('firmware_update_required')
    request=request_id(state);send(request)
    state=wait_ack(client,request)
    if state['settings'].get('currentLimitRaw')!=limit:raise ValueError('settings_readback_mismatch')
    return state
