# usage: python3 fit_closed_loop.py <P-dump.csv> [samples_ahead]   e.g. python3 fit_closed_loop.py run126_last2s.csv 25  (25 x 2 ms = 50 ms)
# Closed-loop check of the plant model on the real 126 s run (last 2 s, 2 ms log):
# drive the model with the LOGGED motor voltage from the LOGGED state, predict H ms ahead, compare.
import sys, numpy as np
from scipy.optimize import least_squares

LAM2, R_I, C_V, RATIO = 67.3548, 0.01173, 0.12, 8.82
d = np.genfromtxt(sys.argv[1] if len(sys.argv) > 1 else 'run126_last2s.csv', delimiter=',', names=True)
t = d['t_ms']/1000; th = np.radians(d['theta_deg']); w = d['wheel_rad_s']; u = d['u_V']; dt = 0.002
# starting tilt rate: a centred difference is too noisy, so each window's starting rate is fitted too
# (one Gauss-Newton step on the tilt error: the prediction is linear in it over 30 ms)
thd = np.gradient(th, dt)
def predict_best(p, i0, n):
    P0 = predict(p, i0, n); eps = 0.01; P1 = predict(p, i0, n, thd[i0] + eps)
    J = (P1[:, 0] - P0[:, 0])/eps; e = th[i0+1:i0+1+n] - P0[:, 0]
    return predict(p, i0, n, thd[i0] + J@e/(J@J))
def predict(p, i0, n, rate=None):
    tau, ratio, dist = p
    b = R_I/(tau*(1+R_I)); x = np.array([th[i0], thd[i0] if rate is None else rate, w[i0]]); out = []
    for k in range(n):
        for _ in range(20):
            h = dt/20; tq = b*(ratio*u[i0+k] - x[2])
            a = LAM2*np.sin(x[0]) - C_V*x[1] - tq + dist
            x = x + h*np.array([x[1], a, tq/R_I - a])
        out.append(x.copy())
    return np.array(out)
H = int(sys.argv[2]) if len(sys.argv) > 2 else 15          # samples ahead (15 = 30 ms)
end = np.argmax(np.abs(np.degrees(th)) > 5) or len(t)      # use the part before the fall
starts = range(5, end - H - 1, 5)
def resid(p):
    r = []
    for i in starts:
        P = predict_best(p, i, H)
        r += list(np.degrees(P[:, 0] - th[i+1:i+1+H]) / 0.05)  # tilt error, scaled by 0.05 deg
        r += list((P[:, 2] - w[i+1:i+1+H]) / 1.0)             # wheel error, scaled by 1 rad/s
    return np.array(r)
nom = np.array([0.0074, RATIO, 0.0])
fit = least_squares(resid, nom, bounds=([0.0005, 4, -5], [0.1, 15, 5]))
for name, p in [('identified model (hanging tests)', nom), ('fitted on balancing data', fit.x)]:
    rr = resid(p); n = len(rr)//2
    print(f'{name:34s} tau {p[0]*1000:5.1f} ms  ratio {p[1]:5.2f} rad/s/V  dist {p[2]:+6.2f} rad/s^2   '
          f'rms {np.sqrt(np.mean((rr.reshape(-1, 2*H)[:, :H]*0.05)**2)):.3f} deg, {np.sqrt(np.mean(rr.reshape(-1, 2*H)[:, H:]**2)):.2f} rad/s')
# plot a few predictions
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
fig, ax = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
ax[0].plot(t[:end], np.degrees(th[:end]), color='0.6', lw=1, label='measured'); ax[1].plot(t[:end], w[:end], color='0.6', lw=1)
for i in range(20, end - H - 1, 40):
    for p, c, lab in [(nom, '#d9534f', 'model from hanging tests'), (fit.x, '#2a7ab9', 'model fitted on this run')]:
        P = predict_best(p, i, H); tt = t[i+1:i+1+H]
        ax[0].plot(tt, np.degrees(P[:, 0]), color=c, lw=1.5, label=lab if i == 20 else None); ax[1].plot(tt, P[:, 2], color=c, lw=1.5)
ax[0].set_ylabel('tilt (deg)'); ax[1].set_ylabel('wheel (rad/s)'); ax[1].set_xlabel('s (last 2 s of the 126 s run)')
ax[0].legend(loc='lower left', fontsize=8); ax[0].set_title(f'{H*2} ms-ahead predictions from logged state and logged motor voltage')
for a in ax: a.grid(alpha=.3)
plt.tight_layout(); plt.savefig('closed_loop_fit.png', dpi=110)
