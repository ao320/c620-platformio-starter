"""Read-only decoding of observed Switch-compatible 0x30 input reports.
Layout reference: Chromium device/gamepad/nintendo_controller.cc ControllerData.
No configuration/calibration/output reports are sent.
"""
BUTTON_LABELS = ['B（下）','A（右）','Y（左）','X（上）','L','R','ZL','ZR','−','＋','左スティック押込','右スティック押込','十字 上','十字 下','十字 左','十字 右','HOME','Capture']
def decode_report(report_id, data):
    # IOHIDManager includes the report-ID byte in the observed 49-byte reports.
    if report_id != 0x30 or len(data) != 49 or data[0] != report_id:
        return None
    right, middle, left = data[3:6]
    flags = [(right,2),(right,3),(right,0),(right,1),(left,6),(right,6),(left,7),(right,7),
             (middle,0),(middle,1),(middle,3),(middle,2),(left,1),(left,0),(left,3),(left,2),(middle,4),(middle,5)]
    buttons = [dict(pressed=bool(value & (1<<bit)), value=1.0 if value & (1<<bit) else 0.0) for value,bit in flags]
    raw=[]
    for i in (6,9):
        raw.extend((data[i] | ((data[i+1]&15)<<8), (data[i+1]>>4) | (data[i+2]<<4)))
    axes = [max(-1.0,min(1.0,(value-2048)/2048 * (-1 if i%2 else 1))) for i,value in enumerate(raw)]
    return dict(index=100, id='Pro Controller / Mac HID', mapping='switch-hid', connected=True,
                buttons=buttons, buttonLabels=BUTTON_LABELS, axes=axes, rawAxes=raw)
