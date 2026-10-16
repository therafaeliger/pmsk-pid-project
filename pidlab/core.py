"""Deterministic, UI-independent discrete PID and servo model (degrees, seconds)."""
from dataclasses import dataclass
import math
import numpy as np


@dataclass
class Config:
    kp: float = 2.0
    ki: float = 5.0
    kd: float = 0.12
    dt: float = 0.01
    duration: float = 8.0
    amplitude: float = 20.0
    wn: float = 10.0
    zeta: float = 0.7
    limit: float = 60.0
    noise: float = 0.0
    disturbance: float = 0.0
    signal: str = 'Step'

    def validate(self):
        for name, value in vars(self).items():
            if name != 'signal' and not math.isfinite(value):
                raise ValueError(f'{name} harus berupa angka hingga.')
        if min(self.kp, self.ki, self.kd, self.noise) < 0:
            raise ValueError('Gain dan noise harus >= 0.')
        if max(self.kp, self.ki, self.kd) > 100 or self.noise > 10 or abs(self.disturbance) > 60:
            raise ValueError('Gain <= 100, noise <= 10°, |gangguan| <= 60°.')
        if not 0.001 <= self.dt <= 0.05 or not 1 <= self.duration <= 60:
            raise ValueError('dt: 0.001–0.05 s; durasi: 1–60 s.')
        if not 1 <= self.wn <= 30 or not 0.1 <= self.zeta <= 3:
            raise ValueError('wn: 1–30 rad/s; zeta: 0.1–3.')
        if not 1 <= self.limit <= 80 or abs(self.amplitude) > 60:
            raise ValueError('Batas servo: 1–80°; amplitudo: -60–60°.')
        if self.signal not in ('Step', 'Impulse', 'Sine', 'Manual'):
            raise ValueError('Sinyal tidak dikenal.')


class PID:
    def __init__(self, config):
        self.c = config
        self.integral = 0.0
        self.previous = None
        self.derivative = 0.0

    def update(self, reference, measured, dt=None):
        c = self.c
        dt = c.dt if dt is None else dt
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError('dt harus positif.')
        error = reference - measured
        raw_d = 0 if self.previous is None else -(measured - self.previous) / dt
        self.derivative += dt / (0.03 + dt) * (raw_d - self.derivative)
        candidate = self.integral + error * dt
        trial = c.kp * error + c.ki * candidate + c.kd * self.derivative
        # Conditional integration: permit integration only out of saturation.
        if abs(trial) <= c.limit or trial * error < 0:
            self.integral = candidate
        output = c.kp * error + c.ki * self.integral + c.kd * self.derivative
        self.previous = measured
        return float(np.clip(output, -c.limit, c.limit))


def reference(t, c):
    if c.signal == 'Manual':
        return c.amplitude
    if t < 0.5:
        return 0.0
    if c.signal == 'Step':
        return c.amplitude
    if c.signal == 'Impulse':
        # Finite pulse, width 0.10 s; area = amplitude * 0.10 degree.second.
        return c.amplitude if t < 0.6 - 1e-10 else 0.0
    return c.amplitude * math.sin(2 * math.pi * 0.5 * (t - 0.5))


class Simulation:
    def __init__(self, c):
        c.validate()
        self.c = c
        self.pid = PID(c)
        self.rng = np.random.default_rng(42)
        self.theta = self.velocity = 0.0
        self.index = 0
        self.rows = []

    def step(self, target=None):
        c = self.c
        t = self.index * c.dt
        r = reference(t, c) if target is None else float(target)
        if not math.isfinite(r) or abs(r) > 60:
            raise ValueError('Target yaw harus hingga dan berada pada +/-60°.')
        y = self.theta + self.rng.normal(0, c.noise)
        u = self.pid.update(r, y)
        self.rows.append((t, r, self.theta, y, u))
        d = c.disturbance if t >= c.duration / 2 else 0.0
        # RK4 with zero-order hold command and disturbance over this sample.
        def f(x):
            return np.array([x[1], c.wn**2 * (u + d - x[0]) - 2*c.zeta*c.wn*x[1]])
        x = np.array([self.theta, self.velocity])
        k1 = f(x); k2 = f(x + c.dt*k1/2)
        k3 = f(x + c.dt*k2/2); k4 = f(x + c.dt*k3)
        self.theta, self.velocity = x + c.dt*(k1 + 2*k2 + 2*k3 + k4)/6
        self.index += 1
        return self.rows[-1]

    def run(self):
        while self.index <= round(self.c.duration / self.c.dt):
            self.step()
        return np.array(self.rows)


def metrics(data, c):
    t, r, y = data[:, 0], data[:, 1], data[:, 2]
    error = r - y
    result = {'RMSE (deg)': float(np.sqrt(np.mean(error**2))),
              'IAE (deg.s)': float(np.trapezoid(abs(error), t)),
              'Final error (deg)': float(error[-1])}
    if c.signal == 'Step' and abs(c.amplitude) > 1e-9 and np.any(t >= 0.5):
        mask = t >= 0.5
        ts, normalized = t[mask], y[mask] / c.amplitude
        result['Overshoot (%)'] = max(0.0, float((max(normalized)-1)*100))
        crossings = [np.flatnonzero(normalized >= p) for p in (0.1, 0.9)]
        result['Rise 10-90% (s)'] = (float(ts[crossings[1][0]]-ts[crossings[0][0]])
                                             if all(len(a) for a in crossings) else None)
        outside = np.flatnonzero(abs(normalized - 1) > 0.02)
        result['Settling 2% (s)'] = (None if len(outside) and outside[-1] == len(ts)-1
                                   else float(ts[outside[-1]+1]-0.5) if len(outside) else 0.0)
    return result
