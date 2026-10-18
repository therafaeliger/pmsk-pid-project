"""Reproducible ZN experiments; no changes to application gain limits/defaults.

The research runner allows raw ZN gains above the UI's 100 gain bound. Its
discrete dynamics and PID implementation are checked against core.Simulation.
"""
from dataclasses import replace
import math
import numpy as np
from .core import Config, metrics


def plant_matrices(wn=10.0, zeta=0.7, dt=0.01):
    """RK4 transition with held input, identical to core.Simulation.step."""
    a = np.array([[0.0, 1.0], [-wn**2, -2*zeta*wn]])
    eye = np.eye(2)
    ad = eye + a*dt + a@a*dt**2/2 + np.linalg.matrix_power(a,3)*dt**3/6 + np.linalg.matrix_power(a,4)*dt**4/24
    bd = np.linalg.solve(a, ad-eye) @ np.array([0.0, wn**2])
    return ad, bd


def reaction_curve(wn=10.0, zeta=0.7):
    """First-inflection tangent of underdamped unit-step response."""
    if not 0 < zeta < 1:
        raise ValueError('Reaction-curve expression requires 0 < zeta < 1.')
    wd = wn*math.sqrt(1-zeta*zeta)
    ti = math.atan(wd/(zeta*wn))/wd
    yi = 1-math.exp(-zeta*wn*ti)*(math.cos(wd*ti)+zeta*wn/wd*math.sin(wd*ti))
    slope = wn**2/wd*math.exp(-zeta*wn*ti)*math.sin(wd*ti)
    delay = ti-yi/slope
    time_constant = 1/slope
    kp = 1.2*time_constant/delay  # DC gain K = 1.
    return {'K':1.0, 'inflection_s':ti, 'inflection_y':yi, 'slope':slope,
            'L_s':delay, 'T_s':time_constant,
            'gains':(kp, kp/(2*delay), kp*delay/2)}


def ultimate_sampled(dt=0.01):
    """Find first stability boundary of P-only sampled loop by bisection."""
    ad, bd = plant_matrices(dt=dt)
    def poles(gain):
        closed = ad.copy(); closed[:,0] -= bd*gain
        return np.linalg.eigvals(closed)
    low, high = 0.0, 1.0
    while max(abs(poles(high))) < 1:
        high *= 2
        if high > 1e6: raise ValueError('No critical gain in search range.')
    for _ in range(70):
        middle = (low+high)/2
        if max(abs(poles(middle))) < 1: low=middle
        else: high=middle
    eig = poles(high)
    angle = abs(np.angle(eig[0]))
    period = 2*math.pi*dt/angle
    return {'Ku':high, 'Pu_s':period, 'pole_real':float(eig[0].real),
            'pole_imag':float(abs(eig[0].imag)),
            'gains':(.6*high, 1.2*high/period, .075*high*period)}


def run(gains, *, dt=.01, duration=8.0, amplitude=20.0, wn=10.0,
        zeta=.7, noise=0.0, disturbance=0.0, seed=42, limit=60.0):
    """Scalar research runner, including derivative filter and anti-windup."""
    kp,ki,kd=map(float,gains)
    if not all(math.isfinite(x) and x>=0 for x in (kp,ki,kd)):
        raise ValueError('Gains must be finite and nonnegative.')
    c=Config(kp=0,ki=0,kd=0,dt=dt,duration=duration,amplitude=amplitude,
             wn=wn,zeta=zeta,noise=noise,disturbance=disturbance,limit=limit)
    c.validate()  # Research gains are validated separately, not clamped.
    c=replace(c,kp=kp,ki=ki,kd=kd)
    ad,bd=plant_matrices(wn,zeta,dt)
    a,b,d,e=ad.ravel(); f,g=bd
    rng=np.random.default_rng(seed)
    noises=rng.normal(0,noise,round(duration/dt)+1)
    data=np.empty((len(noises),5))
    theta=velocity=integral=derivative=0.0
    previous=None
    for index,n in enumerate(noises):
        t=index*dt; ref=amplitude if t>=.5 else 0.0
        measured=theta+n; error=ref-measured
        q=0 if previous is None else -(measured-previous)/dt
        derivative += dt/(.03+dt)*(q-derivative)
        candidate=integral+error*dt
        trial=kp*error+ki*candidate+kd*derivative
        if abs(trial)<=limit or trial*error<0: integral=candidate
        u=max(-limit,min(limit,kp*error+ki*integral+kd*derivative))
        previous=measured
        data[index]=(t,ref,theta,measured,u)
        applied=u+(disturbance if t>=duration/2 else 0)
        theta,velocity=a*theta+b*velocity+f*applied,d*theta+e*velocity+g*applied
    return data,c


def evaluate(gains, **kwargs):
    data,c=run(gains,**kwargs)
    m=metrics(data,c)
    steady=data[:,0]>=6
    m['TV_u_6_8 (deg)']=float(np.sum(abs(np.diff(data[steady,4]))))
    m['Peak |u| (deg)']=float(max(abs(data[:,4])))
    m['Saturation (%)']=float(np.mean(abs(data[:,4])>=c.limit-1e-9)*100)
    m['RMSE_6_8 (deg)']=float(np.sqrt(np.mean((data[steady,1]-data[steady,2])**2)))
    return data,m


def score(gains):
    _,nom=evaluate(gains)
    _,dist=evaluate(gains,disturbance=5)
    _,noisy=evaluate(gains,noise=.2)
    settling=nom['Settling 2% (s)']
    # Dimensionless weighted score. Nominal & disturbance IAE use A*1 second.
    value=nom['IAE (deg.s)']/20+dist['IAE (deg.s)']/20+.02*noisy['TV_u_6_8 (deg)']/20
    value += 2*max(0,nom['Overshoot (%)']-5)/100
    value += max(0,(8 if settling is None else settling)-2)
    value += 5*max(0,abs(nom['Final error (deg)'])-.1)
    return float(value)


def robust_score(gains):
    value=score(gains)
    corner_metrics=[evaluate(gains,wn=wn,zeta=zeta)[1]
                    for wn in [8,12] for zeta in [.5,.9]]
    value += 10*max(0,max(m['Overshoot (%)'] for m in corner_metrics)-5)/100
    value += max(0,max(8 if m['Settling 2% (s)'] is None else m['Settling 2% (s)']
                       for m in corner_metrics)-2)
    return value


def fine_tune(seeds, objective=score):
    """Bounded coordinate search in log gains, deterministic and local."""
    bounds=np.array([[.2,20],[.2,400],[.005,1]])
    history=[]; count=0; cache={}
    def cost(g):
        nonlocal count
        key=tuple(np.round(g,10))
        if key not in cache: cache[key]=objective(g); count+=1
        return cache[key]
    best=None
    for name,initial in seeds.items():
        current=np.clip(np.array(initial,dtype=float),bounds[:,0],bounds[:,1])
        value=cost(current)
        history.append({'seed':name,'stage':'start','gains':current.tolist(),'J':value})
        for step in [.5,.25,.12,.06,.03]:
            for _ in range(12):
                candidates=[]
                for axis in range(3):
                    for sign in [-1,1]:
                        trial=current.copy(); trial[axis]*=math.exp(sign*step)
                        trial=np.clip(trial,bounds[:,0],bounds[:,1])
                        candidates.append((cost(trial),trial))
                new_value,new=min(candidates,key=lambda pair:pair[0])
                if new_value>=value-1e-10: break
                value,current=new_value,new
            history.append({'seed':name,'stage':f'log step {step:g}','gains':current.tolist(),'J':value})
        if best is None or value<best[0]: best=(value,current,name)
    return {'gains':best[1].tolist(),'J':best[0],'winning_seed':best[2],
            'evaluations':count,'history':history,'bounds':bounds.tolist()}
