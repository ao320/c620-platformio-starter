"""Reconnect USB transport while leaving arming/reference policy to the bridge."""
import time
from serial_client import SerialClient

class RecoveringSerialClient:
    transport='USBシリアル（自動再接続）'
    def __init__(self,path,factory=SerialClient,clock=time.monotonic):
        self.path=path;self.factory=factory;self.clock=clock
        self.inner=None;self.attempted=-100.;self.connected_at=-100.;self.last_error='USB接続待ち'
        self.reconnect()
    def reconnect(self):
        now=self.clock()
        # Give a newly opened ESP32 time to boot and emit its first telemetry.
        if self.inner is not None and not self.inner.error:
            # A late 1 Hz status should stop control, but reopening the USB
            # port can reset some ESP32 boards. Give an intact reader time to
            # recover before closing it. Hard reader errors still reconnect.
            last_update=getattr(self.inner,'updated',0.)
            last_seen=last_update if last_update>0 else self.connected_at
            if now-self.connected_at<5 or now-last_seen<8:return False
        if now-self.attempted<3:return False
        self.attempted=now
        if self.inner is not None:self.inner.close();self.inner=None
        try:
            self.inner=self.factory(self.path);self.connected_at=now;self.last_error=''
            return True
        except Exception as e:
            self.last_error=f'{type(e).__name__}: {e}'
            return False
    def status(self):
        if self.inner is None:raise ConnectionError('USB再接続待ち: '+self.last_error)
        return self.inner.status()
    def _write(self,data):
        if self.inner is None:raise ConnectionError('USB未接続')
        return self.inner._write(data)
    def close(self):
        if self.inner is not None:self.inner.close();self.inner=None
