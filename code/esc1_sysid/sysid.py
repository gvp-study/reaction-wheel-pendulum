#!/usr/bin/env python3
"""sysid.py - record and fit the reaction-wheel pendulum (hanging), then design the upright LQR.

  python3 sysid.py rec H              # zero = hanging (let it hang dead still)
  python3 sysid.py rec S              # swing test -> swing_1.csv, swing_2.csv ...
  python3 sysid.py rec K              # kick test  -> kick_1.csv ...
  python3 sysid.py fit                # fits every swing_*.csv and kick_*.csv here, prints the model + LQR gains,
                                      # and plots data (dots) against the fitted model (lines) -> sysid_plots.png
  python3 sysid.py plot               # just plot the raw recordings, no fitting
  options: --port /dev/ttyACM0   (close the Arduino Serial Monitor first)

Model (theta = pendulum angle from HANGING, psi = wheel angle relative to the pendulum, u = Uq volts):
  theta'' = -lam2*sin(theta) - c*theta' - fc*sign(theta') - a*u + b*psi'
  psi''   = (a*u - b*psi')/r - theta''          r = I_wheel / J_pendulum
Physics behind it: motor torque tau = k1*u - k2*psi' (volts minus back-EMF), J*theta'' = -mgl sin - tau,
I_w*(theta''+psi'') = tau  ->  a = k1/J, b = k2/J, a/r = k1/I_w.  Upright: theta_up = theta - pi flips sin.
"""
import sys, glob, re, argparse
import numpy as np
from scipy.optimize import least_squares
from scipy.linalg import solve_continuous_are, expm

def load(fn):
    meta, rows = {}, []
    for line in open(fn):
        if line.startswith('#'):
            meta.update(dict(re.findall(r'(\w+)=(\S+)', line)))
        elif line[0].isdigit() or line[0] == '-':
            rows.append([float(v) for v in line.split(',')])
    d = np.array(rows)
    return meta, d[:, 0] / 1000, np.radians(d[:, 1]), d[:, 2], d[:, 3]

# ---------- recording ----------
def rec(cmd, port):
    import serial, time
    s = serial.Serial(port, 115200, timeout=1)
    time.sleep(0.3); s.reset_input_buffer()
    s.write((cmd + '\n').encode())
    if cmd.upper()[0] not in 'SK':
        t0 = time.time()
        while time.time() - t0 < 1.5:
            l = s.readline().decode(errors='ignore').strip()
            if l: print(l)
        return
    name = 'swing' if cmd.upper()[0] == 'S' else 'kick'
    lines = []
    while True:
        l = s.readline().decode(errors='ignore').strip()
        if not l: continue
        if l.startswith('# SWING') or l.startswith('# KICK') or l.startswith('# busy'): print(l); continue
        lines.append(l)
        if len(lines) % 300 == 0: print('.', end='', flush=True)
        if l == '# end': break
    n = len(glob.glob(name + '_*.csv')) + 1
    fn = f'{name}_{n}.csv'
    open(fn, 'w').write('\n'.join(lines) + '\n')
    print(f'\nsaved {fn} ({len(lines) - 3} samples)')

# ---------- simulation (RK4, u held over each 5 ms sample) ----------
def f(x, u, p, up=False):
    th, w, ps = x
    lam2, c, fc, a, b, r = p
    g = lam2 * np.sin(th) if not up else -lam2 * np.sin(th)
    thdd = -g - c * w - fc * np.tanh(w / 0.05) - a * u + b * ps
    psdd = (a * u - b * ps) / r - thdd
    return np.array([w, thdd, psdd])

def sim(p, x0, u, dt, sub=5):
    x = np.array(x0, float); out = np.empty((len(u), 3)); h = dt / sub
    for i, ui in enumerate(u):
        out[i] = x
        for _ in range(sub):
            k1 = f(x, ui, p); k2 = f(x + h/2*k1, ui, p); k3 = f(x + h/2*k2, ui, p); k4 = f(x + h*k3, ui, p)
            x = x + h/6*(k1 + 2*k2 + 2*k3 + k4)
    return out

def deriv(y, dt, k=3):
    return (y[2*k:] - y[:-2*k]) / (2*k*dt)

# ---------- swing fit: lam2, c, fc ----------
def fit_swing(fn):
    meta, t, th, ps, u = load(fn); dt = t[1] - t[0]
    # start just after the release: the first extreme once it is moving
    off = np.median(th[-200:])
    k = np.argmax(np.abs(th[:int(3 / dt)] - off))           # the held, most-deflected point
    i0 = k
    while i0 + 1 < len(th) and abs(th[i0 + 1] - th[k]) < np.radians(0.3): i0 += 1
    i0 = max(i0 - 1, 0)                                      # last sample before it gets away
    th, t = th[i0:], t[i0:] - t[i0]
    off0 = np.median(th[-200:])
    def res(q):
        lam2, c, fc, th0, w0, off = q
        y = sim((lam2, c, fc, 1, 0, 1), (th0 - off, w0, 0), np.zeros_like(t), dt)[:, 0] + off
        return y - th
    w0 = 0.0
    q0 = [60, 0.2, 0.01, th[0], w0, off0]
    sol = least_squares(res, q0, bounds=([1, 0, 0, -3, -50, -0.3], [400, 20, 5, 3, 50, 0.3]))
    lam2, c, fc, *_ = sol.x
    rms = np.degrees(np.sqrt(np.mean(sol.fun**2)))
    amp = np.degrees(np.max(np.abs(th - sol.x[5])))
    print(f'{fn}: lambda = {np.sqrt(lam2):.3f} rad/s  (hanging period {2*np.pi/np.sqrt(lam2):.3f} s)  '
          f'viscous c = {c:.3f} 1/s  coulomb = {fc:.3f} rad/s^2  offset {np.degrees(sol.x[5]):+.2f} deg  '
          f'| swing {amp:.1f} deg, fit rms {rms:.3f} deg')
    PLOTS.append(('swing', fn, t, th, sol.fun + th, None, None, None))
    return sol.x[:3], sol.x[5]

# ---------- kick fit: a, b, r (lam2, c, fc from the swings) ----------
def fit_kick(fn, base):
    meta, t, th, ps, u = load(fn); dt = t[1] - t[0]
    off = np.median(th[:40]); th = th - off
    sth = max(np.std(th), 1e-3); sps = max(np.std(ps), 1e-3)
    def res(q):
        a, b, r = q
        y = sim((*base, a, b, r), (th[0], 0, 0), u, dt)
        yps = np.concatenate([[0], np.cumsum(y[:-1, 2] * dt)])        # wheel angle from the rate
        return np.concatenate([(y[:, 0] - th) / sth, (yps - ps) / sps])
    best = None
    for a0 in (-10, 10):                                             # sign of a depends on the wiring
        sol = least_squares(res, [a0, abs(a0) / 8, 0.02], bounds=([-500, 0, 1e-4], [500, 500, 1]))
        if best is None or sol.cost < best.cost: best = sol
    a, b, r = best.x
    n = len(t)
    y = sim((*base, a, b, r), (th[0], 0, 0), u, dt)
    PLOTS.append(('kick', fn, t, th, y[:, 0], u, np.gradient(ps, dt), y[:, 2]))
    rth = np.degrees(np.sqrt(np.mean((best.fun[:n] * sth)**2)))
    print(f'{fn}: a = {a:+.3f}  b = {b:.3f}  r = I_w/J = {r:.5f}   (U={meta.get("U")} V, '
          f'peak recoil {np.degrees(np.max(np.abs(th))):.1f} deg, peak wheel {np.max(np.abs(np.gradient(ps, dt))):.0f} rad/s, '
          f'fit rms {rth:.3f} deg)')
    return best.x

# ---------- upright model + LQR ----------
def lqr(lam2, c, a, b, r, q=(1/np.radians(2)**2, 1/1.0**2, 1/100**2), R=1/2.0**2, fs=1000):
    # x = [theta_up, theta', psi'], linearised at upright (sin -> -theta flips the sign of gravity)
    A = np.array([[0, 1, 0],
                  [lam2, -c, b],
                  [-lam2, c, -b / r - b]])
    B = np.array([[0], [-a], [a / r + a]])
    Q = np.diag(q); Rm = np.array([[R]])
    P = solve_continuous_are(A, B, Q, Rm)
    K = (np.linalg.solve(Rm, B.T @ P)).ravel()
    cl = np.linalg.eigvals(A - B @ K[None])
    return A, B, K, cl

def fit(args):
    swings = sorted(glob.glob('swing_*.csv')); kicks = sorted(glob.glob('kick_*.csv'))
    if not swings: sys.exit('no swing_*.csv here')
    print('--- swing tests (motor off) ---')
    S = [fit_swing(fn) for fn in swings]
    base = np.median([s[0] for s in S], axis=0); off = np.median([s[1] for s in S])
    lam2, c, fc = base
    print(f'=> lambda = {np.sqrt(lam2):.3f} rad/s : upright, a 1 deg lean grows e-fold every '
          f'{1000/np.sqrt(lam2):.0f} ms, doubles every {1000*np.log(2)/np.sqrt(lam2):.0f} ms')
    print(f'   hanging rest point is {np.degrees(off):+.2f} deg from your H zero '
          f'(balance point = hang_raw + 180 {np.degrees(off):+.2f})')
    if not kicks: print('(no kick_*.csv yet - run: sysid.py rec K)'); return
    print('--- kick tests (motor pulses) ---')
    Kp = np.median([fit_kick(fn, base) for fn in kicks], axis=0)
    a, b, r = Kp
    print(f'=> a = {a:+.3f} rad/s^2 per volt on the pendulum, wheel {a/r:+.1f} rad/s^2 per volt, '
          f'back-EMF time constant {1000*r/b/(1+r):.1f} ms, I_w/J = {r:.5f}')
    mgl_J = lam2
    print(f'   to hold a 1 deg lean needs u = {lam2*np.radians(1)/abs(a):.2f} V (briefly) - '
          f'max lean the motor can catch at 6 V ~ {np.degrees(6*abs(a)/lam2):.1f} deg (instantaneous)')
    A, B, K, cl = lqr(lam2, c, a, b, r)
    print('--- LQR, u = -(K0*theta_rad + K1*theta_dot + K2*wheel_rad_s), theta from upright ---')
    print(f'K = [{K[0]:.3f}, {K[1]:.4f}, {K[2]:.5f}]')
    print('closed-loop poles:', np.round(cl, 2))
    Ad = expm(A * 0.001)
    print(f'per degree of lean: {K[0]*np.radians(1):.2f} V   per 10 deg/s: {K[1]*np.radians(10):.2f} V   '
          f'per 100 rad/s wheel: {K[2]*100:.2f} V')
    open('model.txt', 'w').write(f'lam2 {lam2}\nc {c}\nfc {fc}\na {a}\nb {b}\nr {r}\noff_deg {np.degrees(off)}\n'
                                 f'K {K[0]} {K[1]} {K[2]}\n')
    print('saved model.txt')

PLOTS = []
def show_plots():
    import matplotlib.pyplot as plt
    if not PLOTS: return
    fig, axs = plt.subplots(len(PLOTS), 1, figsize=(11, 3.2 * len(PLOTS)), squeeze=False)
    for ax, (kind, fn, t, th, thm, u, wd, wm) in zip(axs[:, 0], PLOTS):
        ax.plot(t, np.degrees(th), '.', ms=2, color='0.4', label='measured tilt')
        if thm is not None: ax.plot(t, np.degrees(thm), color='C3', lw=1.2, label='model')
        ax.set_ylabel('tilt from hanging (deg)'); ax.set_title(fn); ax.grid(alpha=0.3)
        if u is not None:
            ax2 = ax.twinx()
            ax2.plot(t, wd, color='C0', lw=0.8, alpha=0.6, label='wheel speed (measured)')
            if wm is not None: ax2.plot(t, wm, color='C0', lw=1.2, ls='--', label='wheel speed (model)')
            ax2.step(t, u * 10, color='C2', lw=0.8, where='post', label='Uq x10 (V)')
            ax2.set_ylabel('wheel rad/s,  Uq x10')
            ax2.legend(loc='upper right', fontsize=8)
        ax.legend(loc='upper left', fontsize=8)
    axs[-1, 0].set_xlabel('time (s)')
    fig.tight_layout(); fig.savefig('sysid_plots.png', dpi=120); print('saved sysid_plots.png')
    plt.show()

def plot_raw():
    for fn in sorted(glob.glob('swing_*.csv')) + sorted(glob.glob('kick_*.csv')):
        meta, t, th, ps, u = load(fn)
        dt = t[1] - t[0]
        PLOTS.append(('kick' if 'kick' in fn else 'swing', fn, t, th, None,
                      u if 'kick' in fn else None, np.gradient(ps, dt), None))
    show_plots()

if __name__ == '__main__':
    ap = argparse.ArgumentParser(); ap.add_argument('what'); ap.add_argument('cmd', nargs='?')
    ap.add_argument('--port', default='/dev/ttyACM0'); a = ap.parse_args()
    if a.what == 'rec': rec(a.cmd, a.port)
    elif a.what == 'plot': plot_raw()
    else:
        fit(a); show_plots()
