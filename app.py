"""Launch with python app.py. All controller calculations run in Python."""
import csv
from pathlib import Path
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import numpy as np
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from pidlab.core import Config, Simulation, PID, metrics
from pidlab.hardware import Hardware
from pidlab.orientation import imu_rotation


class App:
    def __init__(self, root, auto_tick=True):
        self.root = root
        self.auto_tick = auto_tick
        root.title('SIMULASI PID')
        root.geometry(f'{min(1400,root.winfo_screenwidth()-60)}x{min(920,root.winfo_screenheight()-100)}')
        root.minsize(1000, 640)
        ttk.Style().theme_use('clam')
        self.running = False
        self.hw = None
        self.data = []
        self.baseline = None
        self.c = Config()
        self.sim = Simulation(self.c)
        self.status = tk.StringVar(value='Siap')
        self.live = tk.StringVar(value='Yaw 0.00° | Target 0.00°')
        self.manual_target = tk.DoubleVar(value=0)
        self.events = []
        self.mouse_down = False
        self.attitude = (0.0,0.0,0.0)
        self.attitude_rows = []
        self.pose_samples = []
        self.fields = {}
        header = ttk.Frame(root, padding=(18, 12)); header.pack(fill='x')
        ttk.Label(header, text='SIMULASI PID: Servo dan MPU6050', font=('Segoe UI', 20, 'bold')).pack(side='left')
        ttk.Label(header, textvariable=self.live, font=('Segoe UI', 12)).pack(side='right')
        body = ttk.Frame(root, padding=(18, 0)); body.pack(fill='both', expand=True)
        control_shell = ttk.Frame(body,width=295); control_shell.pack(side='left',fill='y',padx=(0,16))
        control_shell.pack_propagate(False)
        controls=ttk.Frame(control_shell)
        controls.pack(fill='both',expand=True)
        ttk.Label(controls, text='EKSPERIMEN', font=('Segoe UI', 11, 'bold')).pack(anchor='w')
        self.mode = tk.StringVar(value='Simulation')
        self.mode_box=ttk.Combobox(controls, textvariable=self.mode, values=['Simulation', 'Arduino'], state='readonly', width=25)
        self.mode_box.pack(fill='x', pady=5)
        self.mode_box.bind('<<ComboboxSelected>>',self.mode_changed)
        self.signal = tk.StringVar(value='Step')
        self.signal_box=ttk.Combobox(controls, textvariable=self.signal, values=['Step', 'Impulse', 'Sine', 'Manual'], state='readonly')
        self.signal_box.pack(fill='x')
        self.signal_box.bind('<<ComboboxSelected>>',self.signal_changed)
        self.sim_only_widgets=[]
        grid = ttk.Frame(controls); grid.pack(fill='x', pady=8)
        labels = [('kp','Kp'), ('ki','Ki (1/s)'), ('kd','Kd (s)'), ('amplitude','Target / amplitudo (°)'),
                  ('duration','Durasi (s)'), ('dt','Sampling (s)'), ('wn','wn (rad/s)'),
                  ('zeta','Redaman zeta'), ('limit','Batas output (°)'), ('noise','Noise std (°)'),
                  ('disturbance','Gangguan @ T/2 (°)')]
        for i, (key, label) in enumerate(labels):
            caption=ttk.Label(grid, text=label); caption.grid(row=i,column=0,sticky='w',pady=3)
            v = tk.StringVar(value=str(getattr(self.c, key))); self.fields[key] = v
            entry=ttk.Entry(grid,textvariable=v,width=9); entry.grid(row=i,column=1,padx=8)
            if key in ('wn','zeta','noise','disturbance'): self.sim_only_widgets.extend([caption,entry])
        ttk.Button(controls, text='Terapkan & reset', command=self.reset).pack(fill='x', pady=4)
        row = ttk.Frame(controls); row.pack(fill='x')
        self.start_button = ttk.Button(row, text='▶ Jalankan', command=self.start); self.start_button.pack(side='left', expand=True, fill='x')
        ttk.Button(row, text='Jeda', command=self.pause).pack(side='left', expand=True, fill='x')
        ttk.Button(controls, text='Simpan sebagai pembanding', command=self.compare).pack(fill='x', pady=4)
        ttk.Button(controls, text='Ekspor CSV + PNG', command=self.export).pack(fill='x', pady=4)
        ttk.Separator(controls).pack(fill='x', pady=9)
        self.port = tk.StringVar(value='COM3')
        ttk.Label(controls, text='Port Arduino (115200 baud)').pack(anchor='w')
        ttk.Entry(controls, textvariable=self.port).pack(fill='x', pady=4)
        ttk.Button(controls, text='Hubungkan / putuskan', command=self.connect).pack(fill='x')
        panel = ttk.Frame(body); panel.pack(side='left', fill='both', expand=True)
        self.fig = Figure(figsize=(9, 7), dpi=100, facecolor='#f5f7fb')
        gs = self.fig.add_gridspec(3, 3, height_ratios=[1.15, 1, .65], width_ratios=[1,1,1.3])
        self.ax3 = self.fig.add_subplot(gs[0, 0], projection='3d')
        self.dial = self.fig.add_subplot(gs[0, 1], projection='polar')
        self.info_ax = self.fig.add_subplot(gs[0, 2]); self.info_ax.axis('off')
        self.ax = self.fig.add_subplot(gs[1, :]); self.au = self.fig.add_subplot(gs[2, :])
        self.canvas = FigureCanvasTkAgg(self.fig, master=panel); self.canvas.get_tk_widget().pack(fill='both', expand=True)
        self.canvas.mpl_connect('button_press_event', self.mouse_press)
        self.canvas.mpl_connect('motion_notify_event', self.mouse_move)
        self.canvas.mpl_connect('button_release_event', self.mouse_release)
        ttk.Label(root, textvariable=self.status, padding=(18, 8)).pack(fill='x')
        self.draw()
        for value in self.fields.values():
            value.trace_add('write',self.parameters_edited)
        root.protocol('WM_DELETE_WINDOW', self.close)
        if self.auto_tick: root.after(40, self.tick)

    def parameters_edited(self,*args):
        self.status.set('Parameter diubah • tekan Jalankan untuk menerapkan')

    def mode_changed(self, event=None):
        self.pause()
        hardware=self.mode.get()=='Arduino'
        if hardware: self.signal.set('Manual')
        self.signal_box.configure(state='disabled' if hardware else 'readonly')
        for widget in self.sim_only_widgets:
            widget.grid_remove() if hardware else widget.grid()
        self.fields['dt'].set('0.02' if hardware else '0.01')
        if hardware: self.fields['limit'].set('10')
        self.baseline=None
        self.reset()

    def signal_changed(self, event=None):
        self.baseline=None
        self.reset()

    def set_target(self, value):
        if self.c.signal != 'Manual': return
        target=float(np.clip(value,-60,60))
        self.manual_target.set(target)
        stamp=(self.data[-1][0] if self.data else 0.0) if self.mode.get()=='Arduino' else self.sim.index*self.c.dt
        self.events.append({'time_s':stamp,'type':'target','value_deg':target})
        if not self.running: self.draw()

    def mouse_press(self,event):
        if event.inaxes == self.dial and event.button == 1:
            self.mouse_down=True; self.canvas.get_tk_widget().focus_set(); self.mouse_move(event)

    def mouse_move(self,event):
        if self.mouse_down and event.inaxes == self.dial and event.xdata is not None:
            self.set_target((np.degrees(event.xdata)+180)%360-180)

    def mouse_release(self,event): self.mouse_down=False

    def config(self):
        if self.mode.get()=='Arduino': self.signal.set('Manual')
        defaults={'wn':10,'zeta':.7,'noise':0,'disturbance':0} if self.mode.get()=='Arduino' else {}
        values={k:defaults[k] if k in defaults else float(v.get()) for k,v in self.fields.items()}
        c = Config(**values,signal=self.signal.get())
        c.validate()
        if self.mode.get() == 'Arduino' and (c.dt < .02 or c.limit > 60):
            raise ValueError('Arduino: dt >= 0.02 s dan batas <= 60°. Gunakan limit awal 10°.')
        return c

    def reset(self):
        try:
            c = self.config()
        except ValueError as e:
            messagebox.showerror('Parameter tidak valid', str(e)); return False
        self.pause()
        self.c = c; self.sim = Simulation(c); self.pid = PID(c)
        self.data = []; self.hw_start = self.hw_previous = None
        self.manual_target.set(c.amplitude); self.events=[]; self.attitude_rows=[]; self.pose_samples=[]
        self.draw(); self.status.set('Siap')
        return True

    def start(self):
        if self.mode.get() == 'Arduino' and not self.hw:
            messagebox.showinfo('Arduino', 'Hubungkan Arduino terlebih dahulu.'); return
        try:
            requested=self.config()
        except ValueError as e:
            messagebox.showerror('Parameter tidak valid',str(e)); return
        changed=requested != self.c
        desired_target=requested.amplitude if requested.amplitude != self.c.amplitude else self.manual_target.get()
        if (changed or not self.data or self.c.signal != self.signal.get() or self.mode.get() != getattr(self, 'active_mode', None)
                or self.mode.get() == 'Arduino') and not self.reset(): return
        if self.data and self.data[-1][0] >= self.c.duration:
            if not self.reset(): return
        if self.c.signal=='Manual':
            self.manual_target.set(desired_target)
            if not self.data:
                self.events.append({'time_s':0.0,'type':'target','value_deg':desired_target})
        self.active_mode = self.mode.get()
        if self.hw and self.active_mode == 'Arduino':
            import time
            self.hw.read()  # Drop stale queued telemetry before starting a new clock.
            self.hw.last_rx = time.monotonic()
        self.running = True
        self.status.set(f'Berjalan • {self.active_mode}'+(' • parameter diterapkan, run baru' if changed else ''))

    def pause(self):
        self.running = False
        if self.hw:
            try: self.hw.command(0)
            except Exception: pass
        self.status.set('Dijeda')

    def connect(self):
        self.pause()
        if self.hw:
            try: self.hw.close()
            except Exception: pass
            self.hw = None; self.status.set('Arduino terputus'); return
        try:
            self.hw = Hardware(self.port.get())
            self.mode.set('Arduino'); self.mode_changed()
            self.status.set('Terhubung • kalibrasi 3 s')
        except Exception as e:
            messagebox.showerror('Koneksi gagal', str(e))

    def tick(self):
        try:
            incoming = self.hw.read() if self.hw else []
            if incoming:
                self.attitude=tuple(incoming[-1][1:])
                self.attitude_rows.extend(incoming)
            if self.running:
                if self.active_mode == 'Simulation':
                    for _ in range(max(1, round(.04/self.c.dt))):
                        if self.sim.index*self.c.dt > self.c.duration + 1e-9: break
                        self.data.append(self.sim.step(self.manual_target.get() if self.c.signal == 'Manual' else None))
                        self.pose_samples.append((0.0,0.0))
                else:
                    import time
                    if time.monotonic() - self.hw.last_rx > .5:
                        raise TimeoutError('Telemetry yaw hilang > 0.5 s; periksa firmware IMU dan koneksi.')
                    for stamp, roll, pitch, yaw in incoming:
                        if self.hw_start is None: self.hw_start = stamp
                        t = stamp - self.hw_start
                        if self.hw_previous is not None and stamp - self.hw_previous < self.c.dt*.9: continue
                        dt = self.c.dt if self.hw_previous is None else stamp - self.hw_previous
                        if dt <= 0 or dt > .2: raise ValueError('Timestamp Arduino tidak kontinu; reset eksperimen.')
                        r = self.manual_target.get(); u = self.pid.update(r, yaw, dt)
                        self.hw.command(u); self.hw_previous = stamp
                        self.data.append((t, r, yaw, yaw, u))
                        self.pose_samples.append((roll,pitch))
                        if t >= self.c.duration: break
                if self.data and self.data[-1][0] >= self.c.duration - 1e-8:
                    self.pause(); self.status.set('Selesai')
                self.draw()
            elif incoming:
                self.draw()
        except Exception as e:
            self.pause(); self.status.set(f'Berhenti: {e}')
        if self.auto_tick: self.root.after(40, self.tick)

    def draw(self):
        data = np.array(self.data) if self.data else np.zeros((1,5))
        roll,pitch,yaw=self.attitude if self.hw and self.mode.get()=='Arduino' else (0,0,data[-1,2])
        target=self.manual_target.get() if self.c.signal == 'Manual' else data[-1,1]
        self.live.set(f'R {roll:.1f}°   P {pitch:.1f}°   Y {yaw:.1f}°   |   Target {target:.1f}°')
        camera=(self.ax3.elev,self.ax3.azim)
        self.ax3.clear(); self.dial.clear(); self.ax.clear(); self.au.clear(); self.info_ax.clear(); self.info_ax.axis('off')
        rotation = imu_rotation(roll,pitch,yaw)
        self.display_rotation = rotation
        vertices = np.array([[-1,-.6,0],[1,-.6,0],[1,.6,0],[-1,.6,0]]) @ rotation.T
        self.ax3.add_collection3d(Poly3DCollection([vertices], facecolors='#0d9488', edgecolors='#134e4a', alpha=.8))
        for i, color in enumerate(['#ef4444','#22c55e','#3b82f6']):
            axis = rotation[:,i]; self.ax3.quiver(0,0,0,*axis,color=color,length=1.3)
        self.ax3.set(xlim=(-1.5,1.5),ylim=(-1.5,1.5),zlim=(-1.5,1.5),title='Orientasi MPU6050',xlabel='X',ylabel='Y',zlabel='Z')
        self.ax3.view_init(*camera)
        self.ax3.set_box_aspect((1,1,1))
        self.dial.set_title('Target yaw',fontsize=10,pad=15)
        self.dial.set_ylim(0,1.1); self.dial.set_yticks([])
        self.dial.set_thetagrids([0,30,60,90,180,270,300,330],labels=['0°','+30°','+60°','+90°','180°','-90°','-60°','-30°'],fontsize=8)
        self.dial.plot([np.radians(target)]*2,[0,1],ls='--',color='#64748b',lw=2)
        self.dial.plot([np.radians(yaw)]*2,[0,.9],color='#0d9488',lw=3,marker='o',markersize=4)
        lines = ['PERFORMA', f'Kp {self.c.kp:g}   Ki {self.c.ki:g}   Kd {self.c.kd:g}']
        if len(data)>2:
            lines += [f'{k}: {v:.3f}' if v is not None else f'{k}: belum tercapai' for k,v in metrics(data,self.c).items()]
        if self.data:
            saturated=np.mean(np.abs(data[:,4])>=self.c.limit-1e-9)*100
            lines += [f'Output: {data[-1,4]:.2f}° / ±{self.c.limit:g}°',
                      f'Saturasi: {saturated:.1f}%']
        self.info_ax.text(.02,.95,'\n'.join(lines),va='top',fontsize=9,linespacing=1.65)
        self.ax.plot(data[:,0],data[:,1],'--',color='#64748b',label='Referensi')
        self.ax.plot(data[:,0],data[:,2],color='#0d9488',label='Yaw')
        if self.baseline is not None: self.ax.plot(self.baseline[:,0],self.baseline[:,2],color='#f59e0b',alpha=.8,label='Pembanding')
        xmax=max(5,min(self.c.duration,data[-1,0]+1)) if self.c.signal == 'Manual' else self.c.duration
        xmin=max(0,xmax-12) if self.c.signal == 'Manual' else 0
        self.ax.set(ylabel='Yaw (°)', xlim=(xmin,xmax)); self.ax.legend(loc='upper right',ncol=3); self.ax.grid(alpha=.2)
        self.au.plot(data[:,0],data[:,4],color='#6366f1'); self.au.axhline(self.c.limit,color='#ef4444',ls=':',lw=1)
        self.au.axhline(-self.c.limit,color='#ef4444',ls=':',lw=1)
        self.au.set(xlabel='Waktu (s)',ylabel='Perintah servo u (°)',xlim=(xmin,xmax)); self.au.grid(alpha=.2)
        self.fig.tight_layout(pad=1.8); self.canvas.draw_idle()

    def compare(self):
        if self.data:
            self.baseline = np.array(self.data).copy(); self.draw()
            self.status.set('Pembanding disimpan')

    def export(self):
        if not self.data: return
        target = filedialog.asksaveasfilename(defaultextension='.csv',filetypes=[('CSV','*.csv')])
        if not target: return
        try:
            with open(target,'w',newline='',encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(['time_s','reference_deg','yaw_deg','measured_deg','command_deg','roll_deg','pitch_deg'])
                writer.writerows([(*row,*pose) for row,pose in zip(self.data,self.pose_samples)])
            if self.attitude_rows:
                with open(Path(target).with_name(Path(target).stem+'_imu.csv'),'w',newline='',encoding='utf-8') as f:
                    writer=csv.writer(f); writer.writerow(['arduino_time_s','roll_deg','pitch_deg','yaw_deg'])
                    writer.writerows(self.attitude_rows)
            self.fig.savefig(Path(target).with_suffix('.png'),dpi=160)
            import json
            from dataclasses import asdict
            Path(target).with_suffix('.json').write_text(json.dumps({'axis':'yaw','mode':getattr(self,'active_mode','Simulation'),'config':asdict(self.c),'events':self.events},indent=2),encoding='utf-8')
            self.status.set(f'Tersimpan: {target} (+ PNG, JSON parameter)')
        except OSError as e: messagebox.showerror('Ekspor gagal',str(e))

    def close(self):
        self.pause()
        if self.hw:
            try: self.hw.close()
            except Exception: pass
        self.root.destroy()


if __name__ == '__main__':
    App(tk.Tk()).root.mainloop()
