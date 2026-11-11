# swingup_full.py - swing-up + catch for the reaction-wheel pendulum, written the way the firmware will run it:
# 1 ms loop, momentum observer on the (delayed, noisy) tilt reading, slew-limited voltage, then the v3.6 LQR.
#
#   python3 swingup_full.py                 one run, nominal plant, realistic sensor -> swingup_full.png
#   python3 swingup_full.py ideal           same, perfect sensor (to separate logic problems from sensor problems)
#   python3 swingup_full.py sweep           robustness: friction, motor, lambda, cable spring, sensor, seeds
#   python3 swingup_full.py fc=0.35 seed=4  any plant/controller setting by name (see PLANT and CTRL below)
#
# The swing-up, in three parts:
#  1. PUMP. The wheel is reversed each time the bar passes through the bottom, where the bar is fastest. By
#     conservation of angular momentum a wheel change dw gives the bar d(theta') = -r/(1+r) dw. Before each pass
#     the controller predicts the bar's speed at the bottom from its energy, works out the speed it needs to just
#     reach the top (plus the friction it will lose on the way), and sets the wheel to +h on the way down so that
#     reversing it to -h at the bottom gives exactly that. Early on h is capped (full pumping); on the last pass h
#     is small, so the wheel arrives at the top nearly stopped. If there is too much energy, h comes out negative
#     and the same reversal takes energy out.
#  2. APPROACH. In the top quarter the controller steers the bar onto the path that coasts exactly to upright
#     (theta' = -2 lambda sin(phi/2)), using the wheel the same way.
#  3. CATCH. The LQR takes over once the wheel can absorb what is left: |w + (1+r)/r * lambda * z| < 70% of the
#     wheel's top speed, z = phi + phi'/lambda (the falling mode). If the LQR loses it (> 35 deg), back to PUMP.
#     Past 200 deg from hanging in either direction (over the top) the motor stops.
import numpy as np, sys
LAM2, R_I, C_V, RATIO = 67.3548, 0.01173, 0.12, 8.82          # as identified; design values the controller uses
LAM = np.sqrt(LAM2); Ts = 0.001
K = np.array([-182.1729, -21.1444, -0.2273]); L_OBS = (0.101382, 5.480116)
PLANT = dict(tau=0.0072, ratio=8.6, lam2=LAM2, c=0.12, fc=0.25, kspring=0.0, kick=0.6, delay=1, noise=0.03,
             spike_p=0.02, seed=1, T=40.0)
CTRL = dict(umax=6.0, slew=0.3, wcap=45.0, g=0.85, catch_deg=30.0, budget=0.7, lose_deg=35.0, loss=1.0)

def wrap(a): return (a + np.pi) % (2*np.pi) - np.pi

class SwingUp:
    """The controller, step by step as the sketch will do it. All state from the observer + wheel encoder."""
    def __init__(s, th0, **c):
        s.c = c; s.r = R_I; s.wlim = RATIO*c['umax']
        s.thH = th0; s.pH = 0.0; s.u = 0.0; s.rej = 0
        s.mode = 'pump'; s.h = 0.0; s.lastSide = 0; s.unwrap = 0.0; s.prevThH = th0; s.tries = 0
        s.wt = 0.0
    def observer(s, thM, w):
        r = s.r
        thd = (s.pH - r*w)/(1+r)
        thP = s.thH + Ts*thd; pP = s.pH + Ts*(LAM2*np.sin(thP) - C_V*thd)
        e = wrap(thM - thP)
        gate = 1.0 if s.mode == 'lqr' else 3.0                 # swinging fast: allow a bigger innovation
        if abs(e) > np.radians(gate) and s.rej < 20: s.rej += 1; e = 0.0
        else: s.rej = 0
        s.thH = wrap(thP + L_OBS[0]*e); s.pH = pP + L_OBS[1]*e
        s.thdH = (s.pH - r*w)/(1+r)
        s.unwrap += wrap(s.thH - s.prevThH); s.prevThH = s.thH   # total angle travelled, for the over-the-top stop
    def step(s, thM, w):
        c = s.c; r = s.r; s.observer(thM, w)
        phi, thd = s.thH, s.thdH                                # phi: from upright (+-pi = hanging)
        E = 0.5*thd**2 + LAM2*(1 + np.cos(phi)); E0 = 2*LAM2
        z = phi + thd/LAM
        if s.mode == 'lqr':
            if abs(phi) > np.radians(c['lose_deg']): s.mode = 'pump'
        elif abs(phi) < np.radians(c['catch_deg']) and abs(w + (1+r)/r*LAM*z) < c['budget']*s.wlim:
            s.mode = 'lqr'; s.tries += 1
        if s.mode == 'lqr':
            un = -c['g']*(K[0]*phi + K[1]*thd + K[2]*w)
        else:
            # friction lost between here and the top: viscous c * integral(theta' dtheta) + dry fc * angle
            # (from the bottom on the separatrix: c*4*lam + fc*pi, about 4.7 in these units; scaled by angle left)
            frac = (1 + np.cos(phi))/2                          # 1 at the bottom, 0 at the top
            Lup = c['loss']*(C_V*4*LAM + 0.25*np.pi)*np.sqrt(frac)
            side = 1 if np.sin(phi) >= 0 else -1                # which side of the bottom the bar is on
            down = np.sign(thd) == np.sign(np.sin(phi)) or abs(thd) < 0.3    # heading towards the bottom (phi -> +-pi)
            if np.cos(phi) > np.cos(np.radians(70)):
                # APPROACH (within 70 deg of upright): aim the bar along the path that coasts to the top
                tgt = -np.sign(phi)*2*LAM*abs(np.sin(phi/2))   # coasting exactly to upright: E = E0
                if np.sign(thd) == -np.sign(phi):               # moving up towards upright
                    s.wt = w + (1+r)/r*(thd - tgt)              # wheel speed that would put theta' on target now
                    s.wt = float(np.clip(s.wt, -s.wlim, s.wlim))
                # (moving away from upright here = it fell back; keep the wheel where it is and let it swing)
            else:
                if side != s.lastSide and s.lastSide != 0:
                    s.wt = -s.wt                                # passed the bottom: reverse the wheel
                elif down:
                    # on the way down: predict the bottom speed, size the coming reversal (+h now -> -h at bottom)
                    Edown = E - (c['loss']*(C_V*4*LAM + 0.25*np.pi))*(1 - np.sqrt(frac))*0.5
                    vb = np.sqrt(max(2*Edown, 0)); vt = np.sqrt(2*(E0 + c['loss']*(C_V*4*LAM + 0.25*np.pi)))
                    h = (1+r)/(2*r)*(vt - vb)                   # >0 adds energy, <0 removes it
                    h = float(np.clip(h, -c['wcap'], c['wcap']))
                    # at the bottom the bar moves with sign(thd) (= sign(sin phi) on the way down); reversing the
                    # wheel from +sign*h to -sign*h there changes theta' by +sign*(vt - vb)
                    s.wt = float(np.sign(np.sin(phi)))*h
                s.lastSide = side
            un = s.wt/RATIO + 0.5*(s.wt - w)                    # wheel speed loop
        un = float(np.clip(un, -c['umax'], c['umax']))
        s.u += float(np.clip(un - s.u, -c['slew'], c['slew']))
        return s.u

def run(ideal=False, verbose=False, **kw):
    P = {**PLANT, **{k: v for k, v in kw.items() if k in PLANT}}; C = {**CTRL, **{k: v for k, v in kw.items() if k in CTRL}}
    rng = np.random.default_rng(int(P['seed']))
    b = R_I/(P['tau']*(1+R_I))
    x = np.array([np.pi - 0.03, 0.0, 0.0])                      # true phi (from upright), phi', wheel; hanging
    ctl = SwingUp(x[0], **C)
    hist = [x[0]]*(int(P['delay'])+1); ulp = 0.0; log = []; t_on = None; res = 'timeout'
    for k in range(int(P['T']/Ts)):
        t = k*Ts; u = ctl.u
        if ideal: thM, wM = x[0], x[2]
        else:
            ulp += (u - ulp)*Ts/0.005
            thM = wrap(hist[0] + rng.normal(0, np.radians(P['noise'])) - np.radians(P['kick']*(u - ulp)))
            if abs(u) > 1.0 and rng.random() < P['spike_p']: thM = wrap(thM + np.radians(rng.choice([-2.5, 2.5])))
            wM = x[2] + rng.normal(0, 0.6)
        u = ctl.step(thM, wM)
        hist = hist[1:] + [x[0]]
        for _ in range(10):
            th, thd, wd = x; h = Ts/10
            tq = b*(P['ratio']*u - wd)
            thdd = P['lam2']*np.sin(th) - P['c']*thd - tq - P['kspring']*th   # kspring: cables pulling towards upright-ish
            if P['fc']:
                if abs(thd) < 1e-3 and abs(thdd) < P['fc']: thdd = 0.0; thd = 0.0; x[1] = 0.0
                else: thdd -= P['fc']*np.sign(thd if abs(thd) >= 1e-3 else thdd)
            x = x + h*np.array([thd, thdd, tq/R_I - thdd])
        x[0] = wrap(x[0])
        if k % 5 == 0: log.append((t, np.degrees(wrap(x[0] + np.pi)), x[1], x[2], u, ctl.mode == 'lqr'))
        if ctl.mode == 'lqr' and t_on is None: t_on = t
        if ctl.mode != 'lqr': t_on = None
        if t_on is not None and t - t_on > 3.0: res = 'balanced'; break
        if abs(ctl.unwrap) > np.radians(200): res = 'over the top'; break   # total travel from hanging
    t_bal = (t_on if res == 'balanced' else None)
    return np.array(log), res, t_bal, ctl.tries

def plot(L, title, fn):
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    f, ax = plt.subplots(3, 1, figsize=(10, 7.5), sharex=True)
    ax[0].plot(L[:, 0], np.degrees(np.unwrap(np.radians(L[:, 1]))), lw=1.2); [ax[0].axhline(v, ls='--', c='0.5', lw=.8) for v in (180, -180)]
    ax[0].set_ylabel('angle from hanging (deg)'); ax[0].set_yticks([-180, -90, 0, 90, 180])
    ax[1].plot(L[:, 0], L[:, 3], lw=1); ax[1].set_ylabel('wheel speed (rad/s)')
    ax[2].plot(L[:, 0], L[:, 4], lw=.8); ax[2].set_ylabel('motor (V)'); ax[2].set_xlabel('time (s)')
    for a_ in ax:
        a_.grid(alpha=.3)
        a_.fill_between(L[:, 0], 0, 1, where=L[:, 5] > 0, color='tab:green', alpha=.12, transform=a_.get_xaxis_transform(), lw=0)
    ax[0].set_title(title, fontsize=9); plt.tight_layout(); plt.savefig(fn, dpi=110)

if __name__ == '__main__':
    args = sys.argv[1:]
    if args and args[0] == 'sweep':
        cases = [('nominal', {}), ('ideal sensor', dict(ideal=True)), ('fc 0.15', dict(fc=0.15)), ('fc 0.40', dict(fc=0.40)),
                 ('c 0.06', dict(c=0.06)), ('c 0.20', dict(c=0.20)), ('tau 4 ms', dict(tau=0.004)), ('tau 12 ms', dict(tau=0.012)),
                 ('ratio 8.0', dict(ratio=8.0)), ('ratio 9.2', dict(ratio=9.2)), ('lambda -3%', dict(lam2=LAM2*0.94)),
                 ('lambda +3%', dict(lam2=LAM2*1.06)), ('2 ms delay', dict(delay=2)), ('kick 1.2', dict(kick=1.2)),
                 ('spikes x3', dict(spike_p=0.06))]
        tot = ok = 0
        for name, kw in cases:
            out = []
            for seed in (1, 2, 3):
                L, res, tb, n = run(seed=seed, **kw); tot += 1; ok += res == 'balanced'
                out.append(f'{("%4.1f s/%d" % (tb, n)) if res == "balanced" else res:>13}')
            print(f'{name:14s} ' + ' '.join(out))
        print(f'balanced {ok}/{tot}  (time to balance / number of catch attempts)')
        sys.exit()
    kw = {}; ideal = False
    for a in args:
        if a == 'ideal': ideal = True
        else: k, v = a.split('='); kw[k] = float(v)
    L, res, tb, n = run(ideal=ideal, **kw)
    print(res, ('LQR holding from %.2f s' % tb) if tb else '', f'({n} catch attempts)')
    plot(L, f'Swing-up, 1 ms loop + observer {"(ideal sensor)" if ideal else "(1 ms delay, noise, spikes, voltage-step dips)"}: {res}', 'swingup_full.png')
