# simulate.py - time-domain check of the exact algorithm the sketch runs: nonlinear plant (sin, voltage limit),
# 1 ms loop, tilt measured 1 ms late, noise + 2-deg spikes while the motor is driven, innovation gate.
import numpy as np, sys
from design import Ts, LAM2, R_I, C_V, RATIO, K
from design2 import obs_gain
r = R_I; L = obs_gain(10)
KTRIM = 2.0e-4      # balance-point trim: zero moves by KTRIM rad per (rad/s of wheel speed x second)
def run(tau=0.0074, th0_deg=1.5, g=1.0, T=6.0, lam2=LAM2, ratio=RATIO, rr=R_I, seed=1, offset_deg=0.0, push_at=None,
        trim=False, fc=0.25, delay=1, spike_p=0.05, comp=0, umax=6.0, kick=0.0, kick_tau=0.005, Lg=None, slew=None, gate=1.0, ktrim=2.0e-4, kspring=0.0):
    rng = np.random.default_rng(seed)
    b = rr/(tau*(1+rr)); a = ratio*b
    x = np.array([np.radians(th0_deg), 0.0, 0.0])          # true theta, theta_dot, psi_dot
    th_h, p_h = np.radians(th0_deg), 0.0                    # observer starts at the first reading
    hist = [x[0]]*(delay+1); log = []; u = 0.0; rej = 0; z = 0.0; ulp = 0.0
    LL = L if Lg is None else Lg
    for k in range(int(T/Ts)):
        # measurement (1 sample old) with noise and spikes when the motor works hard
        ulp += (u - ulp)*Ts/kick_tau                         # sensor kick: a step in u reads as a decaying tilt error
        th_m = hist[0] + rng.normal(0, np.radians(0.03)) + np.radians(offset_deg) - z - np.radians(kick*(u - ulp))
        if abs(u) > 1.0 and rng.random() < spike_p: th_m += np.radians(rng.choice([-2.5, 2.5]))
        w_m = x[2] + rng.normal(0, 0.6)
        # --- controller, as in the sketch ---
        thd_h = (p_h - r*w_m)/(1+r)
        th_p = th_h + Ts*thd_h; p_p = p_h + Ts*(LAM2*th_h - C_V*thd_h)
        e = th_m - th_p
        if abs(e) > np.radians(gate) and rej < 20: rej += 1; e = 0.0
        else: rej = 0
        th_h = th_p + LL[0]*e; p_h = p_p + LL[1]*e
        thd_h = (p_h - r*w_m)/(1+r)
        th_c = th_h + comp*Ts*thd_h                            # look ahead over the known sensor delay
        un = -g*(K[0]*th_c + K[1]*thd_h + K[2]*w_m); un = float(np.clip(un, -umax, umax))
        u = un if slew is None else u + float(np.clip(un - u, -slew, slew))
        if trim and (trim == 'always' or abs(u) < umax - 0.5): z -= ktrim * w_m * Ts
        # --- true plant, 10 substeps ---
        hist = hist[1:] + [x[0]]
        for _ in range(10):
            th, thd, wd = x; h = Ts/10
            tq = b*(ratio*u - wd)                              # motor term, per J
            thdd = lam2*np.sin(th) - C_V*thd - tq - kspring*th      # kspring: cable spring, rad/s^2 per rad
            if fc:                                             # dry friction in the pivot (stick inside the band)
                if abs(thd) < 1e-3 and abs(thdd) < fc: thdd = 0.0; thd = 0.0; x[1] = 0.0
                else: thdd -= fc*np.sign(thd if abs(thd) >= 1e-3 else thdd)
            wdd = tq/rr - thdd
            x = x + h*np.array([thd, thdd, wdd])
        if push_at and abs(k*Ts - push_at) < 1e-9: x[1] += 0.3   # a 0.3 rad/s tap
        log.append((k*Ts, np.degrees(x[0]), x[2], u, np.degrees(z)))
        if abs(x[0]) > np.radians(25): break
    return np.array(log)
if __name__ == '__main__':
    for name, kw in [('nominal, 1.5 deg start', {}), ('3 deg start', dict(th0_deg=3)), ('motor tau 0.4 ms', dict(tau=0.0004)), ('motor tau 30 ms', dict(tau=0.03)),
                     ('lambda +8%', dict(lam2=LAM2*1.17)), ('ratio -25%', dict(ratio=RATIO*0.75)), ('gain 0.7', dict(g=0.7)), ('gain 1.6', dict(g=1.6)),
                     ('zero off by 0.5 deg', dict(offset_deg=0.5)), ('tap at 3 s', dict(push_at=3.0))]:
        lg = run(**kw); ok = lg[-1, 0] >= 5.99
        print(f'{name:24s} {"BALANCES" if ok else "FALLS at %.2f s" % lg[-1,0]:16s} max lean {np.max(np.abs(lg[:,1])):5.2f} deg  '
              f'end lean {lg[-1,1]:+.2f} deg  max wheel {np.max(np.abs(lg[:,2])):5.1f} rad/s  end wheel {lg[-1,2]:+6.1f}  u rms {np.sqrt(np.mean(lg[200:,3]**2)):.2f} V')
