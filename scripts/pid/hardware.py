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
    def __init__(self, port, on_message=None, on_error=None):
        import serial
        self.serial = serial.Serial(port, 115200, timeout=0, write_timeout=0.1)
        self.buffer = b''
        self.last_rx = time.monotonic()
        self.on_message = on_message
        self.on_error = on_error   # called with the text of "ERR,..." lines only

    def _notify(self, direction, message):
        if self.on_message is not None:
            self.on_message(direction,message)

    def read(self):
        self.buffer += self.serial.read(min(self.serial.in_waiting, 8192))
        rows = []
        while b'\n' in self.buffer:
            line, self.buffer = self.buffer.split(b'\n', 1)
            text = line.decode('ascii', errors='replace').rstrip('\r')
            self._notify('RX', text)
            if self.on_error is not None and text.startswith('ERR,'):
                self.on_error(text)
            try:
                rows.append(parse_telemetry(text))
                self.last_rx = time.monotonic()
            except ValueError:
                pass
        if len(self.buffer) > 8192:
            self.buffer = b''
        return rows

    def command(self, angle):
        if not math.isfinite(angle):
            raise ValueError('Sudut tidak valid')
        command=f'SET,{max(-60, min(60, angle)):.3f}\n'
        self.serial.write(command.encode())
        self._notify('TX',command.rstrip('\n'))

    def close(self):
        try:
            self.command(0)
        finally:
            self.serial.close()
