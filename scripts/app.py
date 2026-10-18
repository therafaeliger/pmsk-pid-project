"""Launch with python app_v2.py. Lean PID lab UI; all controller calculations run in Python (pidlab)."""
import csv
import queue
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
from serial import SerialException
import numpy as np
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from pid.core import Config, Simulation, metrics
from pid.hardware import Hardware
from pid.session import HardwareSession
from pid.orientation import imu_rotation

TICK_MS = 40
# key, label, label in Arduino mode (or None), simulation-only
FIELDS = [('kp', 'Kp', 'Kp (1/s)', False), ('ki', 'Ki (1/s)', 'Ki (1/s²)', False),
          ('kd', 'Kd (s)', 'Kd', False), ('amplitude', 'Target / amplitudo (°)', None, False),
          ('duration', 'Durasi (s)', None, True), ('dt', 'Sampling (s)', None, False),
          ('wn', 'wn (rad/s)', None, True), ('zeta', 'Redaman zeta', None, True),
          ('limit', 'Batas output (°)', None, False), ('noise', 'Noise std (°)', None, True),
          ('disturbance', 'Gangguan @ T/2 (°)', None, True)]
HW_DEFAULTS = {'duration': 8, 'wn': 10, 'zeta': .7, 'noise': 0, 'disturbance': 0}
CSV_COLUMNS = ['time_s', 'reference_deg', 'yaw_deg', 'measured_deg', 'command_deg', 'roll_deg', 'pitch_deg']


class App:
    def __init__(self, root, auto_tick=True):
        self.root, self.auto_tick = root, auto_tick
        root.title('SIMULASI PID')
        root.geometry(f'{min(1400, root.winfo_screenwidth() - 60)}x{min(920, root.winfo_screenheight() - 100)}')
        root.minsize(1000, 640)
        ttk.Style().theme_use('clam')
        self.hw = None
        self.running = False
        self.active_mode = None
        self.errors = queue.SimpleQueue()            # ERR lines from the serial thread
        self.data, self.pose_samples = [], []        # rows: time, ref, yaw, measured, command | roll, pitch
        self.target = 0.0
        self.attitude = (0.0, 0.0, 0.0)
        self.pose_key = None
        self.pose_artists = []
        self.mouse_down = False
        self.c = Config()
        self.sim = Simulation(self.c)
        self.gain_mode = 'Simulation'
        self.gain_presets = {'Arduino': ('2', '0', '0')}
        self.build_ui()
        self.build_plots()
        self.draw()
        root.protocol('WM_DELETE_WINDOW', self.close)
        if auto_tick: root.after(TICK_MS, self.tick)

    @property
    def hardware(self):
        return self.mode.get() == 'Arduino'

    # ---------------------------------------------------------------- UI
    def build_ui(self):
        root = self.root
        self.mode, self.signal = tk.StringVar(value='Simulation'), tk.StringVar(value='Step')
        self.port, self.live = tk.StringVar(value='COM3'), tk.StringVar()
        self.status = tk.StringVar(value='Siap')
        header = ttk.Frame(root, padding=(18, 12)); header.pack(fill='x')
        ttk.Label(header, text='SIMULASI PID: Servo dan MPU6050', font=('Segoe UI', 20, 'bold')).pack(side='left')
        ttk.Label(header, textvariable=self.live, font=('Segoe UI', 12)).pack(side='right')
        body = ttk.Frame(root, padding=(18, 0)); body.pack(fill='both', expand=True)
        side = ttk.Frame(body, width=295); side.pack(side='left', fill='y', padx=(0, 16))
        side.pack_propagate(False)

        mode_box = ttk.Combobox(side, textvariable=self.mode, values=['Simulation', 'Arduino'], state='readonly')
        mode_box.pack(fill='x', pady=5); mode_box.bind('<<ComboboxSelected>>', self.mode_changed)
        self.signal_box = ttk.Combobox(side, textvariable=self.signal, state='readonly',
                                       values=['Step', 'Impulse', 'Sine', 'Manual'])
        self.signal_box.pack(fill='x'); self.signal_box.bind('<<ComboboxSelected>>', lambda e: self.reset())

        grid = ttk.Frame(side); grid.pack(fill='x', pady=8)
        self.fields, self.labels, self.sim_only = {}, {}, []
        for i, (key, label, _, sim_only) in enumerate(FIELDS):
            self.labels[key] = ttk.Label(grid, text=label); self.labels[key].grid(row=i, column=0, sticky='w', pady=3)
            self.fields[key] = tk.StringVar(value=str(getattr(self.c, key)))
            entry = ttk.Entry(grid, textvariable=self.fields[key], width=9); entry.grid(row=i, column=1, padx=8)
            if sim_only: self.sim_only.append(entry)

        row = ttk.Frame(side); row.pack(fill='x')
        for text, cmd in (('▶ Jalankan', self.start), ('Jeda', self.pause), ('Reset', self.reset)):
            ttk.Button(row, text=text, command=cmd).pack(side='left', expand=True, fill='x')
        ttk.Button(side, text='Ekspor CSV + PNG', command=self.export).pack(fill='x', pady=8)
        ttk.Separator(side).pack(fill='x', pady=4)
        ttk.Label(side, text='Port Arduino (115200 baud)').pack(anchor='w')
        ttk.Entry(side, textvariable=self.port).pack(fill='x', pady=4)
        self.connect_button = ttk.Button(side, text='Hubungkan', command=self.connect)
        self.connect_button.pack(fill='x')

        self.fig = Figure(figsize=(9, 7), dpi=100, facecolor='#f5f7fb')
        self.canvas = FigureCanvasTkAgg(self.fig, master=body)
        self.canvas.get_tk_widget().pack(side='left', fill='both', expand=True)
        self.canvas.mpl_connect('button_press_event', self.mouse_press)
        self.canvas.mpl_connect('motion_notify_event', self.mouse_move)
        self.canvas.mpl_connect('button_release_event', lambda e: setattr(self, 'mouse_down', False))
        ttk.Label(root, textvariable=self.status, padding=(18, 8)).pack(fill='x')

    def build_plots(self):
        """Create all artists once; draw() only updates their data."""
        gs = self.fig.add_gridspec(3, 3, height_ratios=[1.15, 1, .65], width_ratios=[1, 1, 1.3])
        self.ax3 = self.fig.add_subplot(gs[0, 0], projection='3d')
        self.dial = self.fig.add_subplot(gs[0, 1], projection='polar')
        info_ax = self.fig.add_subplot(gs[0, 2]); info_ax.axis('off')
        self.ax, self.au = self.fig.add_subplot(gs[1, :]), self.fig.add_subplot(gs[2, :])

        self.ax3.set(xlim=(-1.5, 1.5), ylim=(-1.5, 1.5), zlim=(-1.5, 1.5), title='Orientasi MPU6050',
                     xlabel='X', ylabel='Y', zlabel='Z')
        self.ax3.set_box_aspect((1, 1, 1))
        self.dial.set_title('Target yaw', fontsize=10, pad=15)
        self.dial.set_ylim(0, 1.1); self.dial.set_yticks([])
        self.dial.set_thetagrids([0, 30, 60, 90, 180, 270, 300, 330], fontsize=8,
                                 labels=['0°', '+30°', '+60°', '+90°', '180°', '-90°', '-60°', '-30°'])
        self.dial_target, = self.dial.plot([0, 0], [0, 1], ls='--', color='#64748b', lw=2)
        self.dial_yaw, = self.dial.plot([0, 0], [0, .9], color='#0d9488', lw=3, marker='o', markersize=4)
        self.info = info_ax.text(.02, .95, '', va='top', fontsize=9, linespacing=1.65)
        self.ref_line, = self.ax.plot([], [], '--', color='#64748b', label='Referensi')
        self.yaw_line, = self.ax.plot([], [], color='#0d9488', label='Yaw')
        self.ax.set_ylabel('Yaw (°)'); self.ax.grid(alpha=.2); self.ax.legend(loc='upper right', ncol=2)
        self.u_line, = self.au.plot([], [], color='#6366f1')
        self.limit_lines = [self.au.axhline(0, color='#ef4444', ls=':', lw=1) for _ in range(2)]
        self.au.set(xlabel='Waktu (s)', ylabel='Perintah servo u (°)'); self.au.grid(alpha=.2)
        self.fig.tight_layout(pad=1.8)

    # ------------------------------------------------------------ connection
    def connect(self):
        self.pause()
        if self.hw: self.disconnect('Arduino terputus'); return
        try:
            port = self.port.get().strip()
            self.hw = HardwareSession(Hardware(port, on_error=self.errors.put))
        except Exception as e:
            self.hw = None
            messagebox.showerror('Koneksi gagal', str(e)); return
        self.connect_button.configure(text='Putuskan')
        self.mode.set('Arduino'); self.mode_changed()
        self.status.set(f'Terhubung {port} • tunggu kalibrasi IMU (~3 s), sensor harus diam')

    def disconnect(self, reason):
        self.running = False
        hardware, self.hw = self.hw, None
        if hardware:
            try: hardware.close()
            except Exception: pass
        self.connect_button.configure(text='Hubungkan')
        self.status.set(reason)

    # --------------------------------------------------------------- control
    def mode_changed(self, event=None):
        self.pause()
        hardware, mode = self.hardware, self.mode.get()
        if mode != self.gain_mode:   # remember Kp/Ki/Kd separately for each mode
            self.gain_presets[self.gain_mode] = tuple(self.fields[k].get() for k in ('kp', 'ki', 'kd'))
            for key, value in zip(('kp', 'ki', 'kd'), self.gain_presets[mode]): self.fields[key].set(value)
            self.gain_mode = mode
        for key, sim_label, hw_label, _ in FIELDS:
            if hw_label: self.labels[key].configure(text=hw_label if hardware else sim_label)
        for entry in self.sim_only: entry.configure(state='disabled' if hardware else 'normal')
        self.signal_box.configure(state='disabled' if hardware else 'readonly')
        if hardware:
            self.signal.set('Manual')
            self.fields['limit'].set('60'); self.fields['amplitude'].set('0')
        self.fields['dt'].set('0.02' if hardware else '0.01')
        self.reset()

    def config(self):
        values = {k: HW_DEFAULTS[k] if self.hardware and k in HW_DEFAULTS else float(v.get())
                  for k, v in self.fields.items()}
        c = Config(**values, signal='Manual' if self.hardware else self.signal.get())
        c.validate()
        if self.hardware and (c.dt < .02 or c.limit > 60):
            raise ValueError('Arduino: dt >= 0.02 s dan batas <= 60°. Gunakan limit awal 10°.')
        return c

    def reset(self):
        try: c = self.config()
        except ValueError as e:
            messagebox.showerror('Parameter tidak valid', str(e)); return False
        self.pause()
        self.c, self.sim = c, Simulation(c)
        self.data, self.pose_samples = [], []
        self.target = c.amplitude
        self.draw(); self.status.set('Siap')
        return True

    def start(self):
        if self.hardware and not self.hw:
            messagebox.showinfo('Arduino', 'Hubungkan Arduino terlebih dahulu.'); return
        if self.hardware and not self.hw.ready:
            self.status.set('Menunggu telemetry IMU • kalibrasi sensor harus selesai'); return
        try: requested = self.config()
        except ValueError as e:
            messagebox.showerror('Parameter tidak valid', str(e)); return
        desired = requested.amplitude if requested.amplitude != self.c.amplitude else self.target
        finished = self.data and self.data[-1][0] >= self.c.duration
        if (requested != self.c or not self.data or finished or self.hardware
                or self.mode.get() != self.active_mode) and not self.reset(): return
        if self.c.signal == 'Manual': self.target = desired
        self.active_mode = self.mode.get()
        if self.hardware: self.hw.set_control(self.c, self.target, strategy='pid')
        self.running = True
        self.status.set(f'Berjalan • {self.active_mode}')

    def pause(self):
        self.running = False
        if self.hw:
            try: self.hw.stop_control()
            except Exception: pass
        self.status.set('Dijeda')

    def set_target(self, value):
        if self.c.signal != 'Manual': return
        self.target = float(np.clip(value, -60, 60))
        if self.hw and self.hardware: self.hw.set_target(self.target)
        if not self.running: self.draw()

    def mouse_press(self, event):
        if event.inaxes == self.dial and event.button == 1:
            self.mouse_down = True; self.canvas.get_tk_widget().focus_set(); self.mouse_move(event)

    def mouse_move(self, event):
        if self.mouse_down and event.inaxes == self.dial and event.xdata is not None:
            self.set_target((np.degrees(event.xdata) + 180) % 360 - 180)

    # ------------------------------------------------------------------ loop
    def tick(self):
        redraw = False
        try:
            incoming = self.hw.read() if self.hw else []
            if incoming:
                self.attitude = tuple(incoming[-1][1:]); redraw = True
            if self.running:
                redraw = True
                c = self.c
                if self.active_mode == 'Simulation':
                    for _ in range(max(1, round(.04 / c.dt))):
                        if self.sim.index * c.dt > c.duration + 1e-9: break
                        self.data.append(self.sim.step(self.target if c.signal == 'Manual' else None))
                        self.pose_samples.append((0.0, 0.0))
                    if self.data and self.data[-1][0] >= c.duration - 1e-8:
                        self.pause(); self.status.set('Selesai')
                else:
                    for sample in self.hw.drain_samples():
                        self.data.append(sample[:5]); self.pose_samples.append(sample[5:])
                    notices = self.hw.drain_notices()
                    if notices:
                        self.running = False; self.status.set(notices[-1])
            if redraw: self.draw()
        except (OSError, SerialException) as e:
            self.disconnect(f'Koneksi terputus: {e}')
        except Exception as e:   # includes TimeoutError
            self.pause(); self.status.set(f'Berhenti: {e}')
        while not self.errors.empty():
            self.status.set(self.errors.get_nowait())
        if self.auto_tick: self.root.after(TICK_MS, self.tick)

    # ------------------------------------------------------------------ plots
    def draw(self):
        c, hardware, manual = self.c, self.hardware, self.c.signal == 'Manual'
        data = np.array(self.data) if self.data else np.zeros((1, 5))
        roll, pitch, yaw = self.attitude if self.hw and hardware else (0, 0, data[-1, 2])
        target = self.target if manual else data[-1, 1]
        self.live.set(f'R {roll:.1f}°   P {pitch:.1f}°   Y {yaw:.1f}°   |   Target {target:.1f}°')

        key = (round(roll, 2), round(pitch, 2), round(yaw, 2))
        if key != self.pose_key:   # 3D pose is rebuilt only when the attitude changes
            self.pose_key = key
            for artist in self.pose_artists: artist.remove()
            # Match mounting: swap tilt axes and reverse visual roll (X rotation).
            # Telemetry/CSV retain Arduino roll,pitch,yaw; yaw PID is unchanged.
            rotation = imu_rotation(-pitch, roll, yaw)
            vertices = np.array([[-1, -.6, 0], [1, -.6, 0], [1, .6, 0], [-1, .6, 0]]) @ rotation.T
            self.pose_artists = [self.ax3.add_collection3d(
                Poly3DCollection([vertices], facecolors='#0d9488', edgecolors='#134e4a', alpha=.8))]
            self.pose_artists += [self.ax3.quiver(0, 0, 0, *rotation[:, i], color=color, length=1.3)
                                  for i, color in enumerate(['#ef4444', '#22c55e', '#3b82f6'])]

        self.dial_target.set_data([np.radians(target)] * 2, [0, 1])
        self.dial_yaw.set_data([np.radians(yaw)] * 2, [0, .9])

        lines = ['PERFORMA', f'Kp {c.kp:g}   Ki {c.ki:g}   Kd {c.kd:g}']
        if len(data) > 2:
            lines += [f'{k}: {v:.3f}' if v is not None else f'{k}: belum tercapai' for k, v in metrics(data, c).items()]
        if self.data:
            saturated = np.mean(np.abs(data[:, 4]) >= c.limit - 1e-9) * 100
            lines += [f'Output: {data[-1, 4]:.2f}° / ±{c.limit:g}°', f'Saturasi: {saturated:.1f}%']
        self.info.set_text('\n'.join(lines))

        t = data[:, 0]
        self.ref_line.set_data(t, data[:, 1]); self.yaw_line.set_data(t, data[:, 2])
        self.u_line.set_data(t, data[:, 4])
        for line, sign in zip(self.limit_lines, (1, -1)): line.set_ydata([sign * c.limit] * 2)

        xmax = (max(5, data[-1, 0] + 1) if hardware else
                max(5, min(c.duration, data[-1, 0] + 1)) if manual else c.duration)
        xmin = max(0, xmax - 12) if manual else 0
        for axis in (self.ax, self.au):
            axis.relim(); axis.autoscale_view(scalex=False); axis.set_xlim(xmin, xmax)
        self.canvas.draw_idle()

    # ----------------------------------------------------------------- output
    def export(self):
        if not self.data: return
        path = filedialog.asksaveasfilename(defaultextension='.csv', filetypes=[('CSV', '*.csv')])
        if not path: return
        try:
            with open(path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f); writer.writerow(CSV_COLUMNS)
                writer.writerows((*row, *pose) for row, pose in zip(self.data, self.pose_samples))
            self.fig.savefig(path.rsplit('.', 1)[0] + '.png', dpi=160)
            self.status.set(f'Tersimpan: {path} (+ PNG)')
        except OSError as e:
            messagebox.showerror('Ekspor gagal', str(e))

    def close(self):
        self.pause()
        if self.hw: self.disconnect('')
        self.root.destroy()


if __name__ == '__main__':
    App(tk.Tk()).root.mainloop()
