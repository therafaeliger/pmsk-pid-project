"""SET,relative_yaw_command; telemetry IMU,ms,roll,pitch,yaw (degrees)."""
import math
import time


def parse_telemetry(line):
    fields = line.strip().split(',')
    if len(fields) != 5 or fields[0] != 'IMU':
        raise ValueError('Telemetry tidak valid')
    ms, roll, pitch, yaw = map(float, fields[1:])
    if (not all(math.isfinite(x) for x in (ms,roll,pitch,yaw)) or ms < 0
            or abs(roll)>180 or abs(pitch)>90 or abs(yaw)>3600):
        raise ValueError('Telemetry di luar rentang')
    return ms / 1000, roll, pitch, yaw


class Hardware:
    def __init__(self, port):
        import serial
        self.serial = serial.Serial(port, 115200, timeout=0, write_timeout=0.1)
        self.buffer = b''
        self.last_rx = time.monotonic()

    def read(self):
        self.buffer += self.serial.read(min(self.serial.in_waiting, 8192))
        rows = []
        while b'\n' in self.buffer:
            line, self.buffer = self.buffer.split(b'\n', 1)
            try:
                rows.append(parse_telemetry(line.decode('ascii')))
                self.last_rx = time.monotonic()
            except (ValueError, UnicodeError):
                pass
        if len(self.buffer) > 8192:
            self.buffer = b''
        return rows

    def command(self, angle):
        if not math.isfinite(angle):
            raise ValueError('Sudut tidak valid')
        self.serial.write(f'SET,{max(-60, min(60, angle)):.3f}\n'.encode())

    def close(self):
        try:
            self.command(0)
        finally:
            self.serial.close()
