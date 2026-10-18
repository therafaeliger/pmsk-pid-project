"""Output-mounted IMU: yaw PID produces rate, integrated into servo position.

Position is relative to the 90 degree mechanical centre. Simulation retains
its original position-output PID; these gains therefore have different units.
"""
from dataclasses import replace
import math
from .core import PID


class ServoYawController:
    def __init__(self, config, max_rate=60.0):
        if not math.isfinite(max_rate) or max_rate <= 0:
            raise ValueError('Laju servo harus positif dan hingga.')
        self.c = config
        self.position = 0.0
        self.max_rate = max_rate
        self.pid = PID(replace(config, limit=max_rate))

    def update(self, reference, measured, dt):
        previous_integral = self.pid.integral
        rate = self.pid.update(reference, measured, dt)
        candidate = self.position + rate * dt
        # Also prevent windup at mechanical position limits, not just rate limits.
        if abs(candidate) > self.c.limit and rate * (reference - measured) > 0:
            self.pid.integral = previous_integral
        self.position = max(-self.c.limit, min(self.c.limit, candidate))
        return self.position
