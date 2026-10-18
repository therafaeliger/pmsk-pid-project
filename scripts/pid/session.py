"""Serial/PID worker independent of Tk and Matplotlib rendering.

UI reads queued snapshots; no Tk callbacks may be called from the worker.
"""
from collections import deque
import threading
import time
from .servo_control import ServoYawController


class HardwareSession:
    def __init__(self,hardware):
        self.hardware=hardware
        self._lock=threading.RLock()
        self._rows=deque(maxlen=5000)
        self._samples=deque(maxlen=5000)
        self._notices=deque(maxlen=100)
        self._commands=deque(maxlen=10)
        self._quit=threading.Event()
        self._fatal=None
        self._enabled=False
        self._pid=None
        self._start=None
        self._previous=None
        self._target=0.0
        self._received_at=None
        self._latest_yaw=None
        self._strategy='pid'
        self._origin_yaw=None
        self._thread=threading.Thread(target=self._run,name='serial-pid',daemon=True)
        self._thread.start()

    @property
    def serial(self): return self.hardware.serial

    @property
    def enabled(self):
        with self._lock: return self._enabled

    @property
    def ready(self):
        with self._lock:
            return (self._fatal is None and self._received_at is not None
                    and time.monotonic()-self._received_at<.5)

    @property
    def compensation_target(self):
        with self._lock: return self._target

    @property
    def origin_yaw(self):
        with self._lock: return self._origin_yaw

    def set_control(self,config,target=0.0,strategy='pid'):
        config.validate()
        if strategy not in ('pid','base_compensation'): raise ValueError('Unknown control strategy')
        with self._lock:
            if strategy=='base_compensation' and not self.ready:
                raise ValueError('Menunggu telemetry untuk menyimpan yaw awal')
            self._strategy=strategy
            self._origin_yaw=self._latest_yaw if strategy=='base_compensation' else None
            self._pid=ServoYawController(config); self._target=target
            if strategy=='base_compensation': self._target=0.0
            self._start=self._previous=None
            self._deadline=time.monotonic()+.5
            self._samples.clear(); self._commands.clear()
            self._notices.clear()
            self._enabled=True

    def set_target(self,target):
        with self._lock:
            if self._strategy=='pid': self._target=target

    def stop_control(self):
        with self._lock:
            self._enabled=False
            self._commands.clear(); self._commands.append(0.0)

    def command(self,value):
        with self._lock: self._commands.append(value)

    def read(self):
        with self._lock:
            if self._fatal is not None: raise self._fatal
            rows=list(self._rows); self._rows.clear()
            return rows

    def drain_samples(self):
        with self._lock:
            rows=list(self._samples); self._samples.clear()
            return rows

    def drain_notices(self):
        with self._lock:
            notices=list(self._notices); self._notices.clear()
            return notices

    def _stop(self,message):
        self._enabled=False
        self._notices.append(message)
        self._commands.clear(); self._commands.append(0.0)

    def _process(self,rows):
        with self._lock:
            self._rows.extend(rows)
            if rows:
                self._received_at=time.monotonic()
                self._latest_yaw=rows[-1][3]
            if not self._enabled: return
            now=time.monotonic()
            if not rows:
                if now>self._deadline: self._stop('Telemetry yaw hilang > 0.5 s')
                return
            self._deadline=now+.5
            for stamp,roll,pitch,yaw in rows:
                if self._start is None: self._start=stamp
                t=stamp-self._start
                if self._previous is not None and stamp-self._previous < self._pid.c.dt*.9:
                    if stamp<self._previous: self._stop('Timestamp Arduino mundur; mulai ulang run')
                    continue
                dt=self._pid.c.dt if self._previous is None else stamp-self._previous
                if dt<=0 or dt>.2:
                    self._stop('Jeda timestamp Arduino > 0.2 s; kendali dihentikan')
                    break
                if self._strategy=='base_compensation':
                    delta=yaw-self._origin_yaw
                    self._target=-delta
                    u=max(-self._pid.c.limit,min(self._pid.c.limit,self._target))
                    sample=(t,self._target,delta,yaw,u,roll,pitch)
                else:
                    u=self._pid.update(self._target,yaw,dt)
                    sample=(t,self._target,yaw,yaw,u,roll,pitch)
                self._previous=stamp
                self._samples.append(sample)
                # Actual serial output occurs on this worker, never on the UI.
                self.hardware.command(u)
                # Hardware holds heading continuously; simulation duration must
                # never release the correction and return the servo to neutral.

    def _run(self):
        try:
            while not self._quit.is_set():
                with self._lock:
                    pending=list(self._commands); self._commands.clear()
                for value in pending: self.hardware.command(value)
                self._process(self.hardware.read())
                self._quit.wait(.003)
        except Exception as exc:
            with self._lock: self._fatal=exc; self._enabled=False
        finally:
            try: self.hardware.close()
            except Exception: pass

    def close(self):
        self._quit.set()
        self._thread.join(timeout=.5)
