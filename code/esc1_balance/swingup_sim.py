# swingup_sim.py - feasibility sim: swing the reaction-wheel pendulum up from hanging, then hand off to the v3.6 LQR.
#
#   python3 swingup_sim.py                      one run with the nominal settings -> plot swingup_sim.png
#   python3 swingup_sim.py Ef=1.12 fc=0.35      change any setting (see below)
#   python3 swingup_sim.py sweep                robustness table over friction and energy target
#
# settings: umax (V limit, 6 = 12 V supply)  wcap (wheel speed used for pumping, rad/s)  kE (how fast the kicks
#   shrink as energy nears the top)  Ef (energy target, x what it takes to reach the top)  catch (deg from upright
#   where the LQR may take over)  zc (deg: how close to the coasting path it must be)  fc, c (bar friction)  T (s)
# theta from HANGING (0 = down, pi = up); psi' = wheel speed relative to the bar; same model as design.py.
import numpy as np, sys
LAM2, C, FC, R = 67.3548, 0.12, 0.25, 0.01173
TAU, RATIO = 0.0072, 8.6
b = R/(TAU*(1+R)); a = RATIO*b
K = np.array([-182.1729, -21.1444, -0.2273]); G = 0.85
def run(umax=6.0, wmax_pump=45.0, kE=0.5, catch_deg=20, T=20, dt=1e-4, th0=0.02, Ef=1.03, zc=6.0, wc=45.0):
    x = np.array([th0, 0.0, 0.0]); mode = 'pump'; log = []; t_catch = None; tries = 0
    E0 = 2*LAM2
    for k in range(int(T/dt)):
        th, thd, w = x
        phi = (th % (2*np.pi)) - np.pi                       # angle from upright
        E = 0.5*thd**2 + LAM2*(1 - np.cos(th))
        z = phi + thd/np.sqrt(LAM2)                          # unstable-mode coordinate: must be small to catch
        if mode == 'pump' and abs(phi) < np.radians(catch_deg) and abs(z) < np.radians(zc) and abs(x[2]) < wc:
            mode = 'lqr'; t_catch = k*dt; tries += 1
        if mode == 'pump':
            # reverse the wheel at every pass through the bottom, where the bar is fastest: by momentum
            # conservation each reversal adds 2W r/(1+r) to the bar's rate, in the direction it is moving.
            # W shrinks as the energy approaches what it takes to reach the top, so it arrives slow, wheel slow.
            W = float(np.clip(kE*(Ef*E0 - E), 0, wmax_pump))   # aim a little above: friction
            wt = -W*np.sign(np.sin(th))                      # target wheel speed: flips sign at the bottom
            u = wt/RATIO + 0.5*(wt - w)                      # wheel speed loop
        else:
            u = -G*(K[0]*phi + K[1]*thd + K[2]*w)
        u = float(np.clip(u, -umax, umax))
        tq = b*(RATIO*u - w)
        thdd = -LAM2*np.sin(th) - C*thd - FC*np.tanh(thd/0.05) - tq
        wdd = tq/R - thdd
        x = x + dt*np.array([thd, thdd, wdd])
        if k % 20 == 0: log.append((k*dt, np.degrees(th), thd, x[2], u, E/E0, mode == 'lqr'))
        if mode == 'lqr' and abs(phi) > np.radians(40): mode = 'pump'      # missed: fall back and pump again
        if abs(th) > np.radians(200) and k*dt > 0.5: return np.array(log), t_catch, False   # over the top
        if mode == 'lqr' and k*dt - t_catch > 3: return np.array(log), t_catch, True
    return np.array(log), t_catch, False
if __name__ == '__main__':
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    P = dict(umax=6.0, wcap=53.0, kE=3.0, Ef=1.08, catch=25.0, zc=6.0, fc=0.25, c=0.12, T=40.0)
    args = sys.argv[1:]
    if args and args[0] == 'sweep':
        for fc in (0.15, 0.25, 0.35):
            for c in (0.08, 0.12, 0.16):
                FC, C = fc, c; globals().update(FC=fc, C=c); res = []
                for Ef in (1.05, 1.08, 1.12):
                    L, tc, ok = run(6.0, 53, 3.0, Ef=Ef, T=40, catch_deg=25)
                    res.append(f'Ef {Ef:.2f}: ' + (f'balanced at {tc:4.1f} s' if ok else 'failed           '))
                print(f'fc {fc:.2f}  c {c:.2f} | ' + '  '.join(res))
        sys.exit()
    for a in args: k, v = a.split('='); P[k] = float(v)
    globals().update(FC=P['fc'], C=P['c'])
    L, tc, ok = run(P['umax'], P['wcap'], P['kE'], catch_deg=P['catch'], T=P['T'], Ef=P['Ef'], zc=P['zc'])
    print(('balanced: LQR took over at %.2f s' % tc) if ok else 'failed (went over the top, or never caught in time)')
    f, ax = plt.subplots(3, 1, figsize=(10, 7.5), sharex=True)
    ax[0].plot(L[:, 0], L[:, 1], lw=1.2); [ax[0].axhline(v, ls='--', c='0.5', lw=.8) for v in (180, -180)]
    ax[0].set_ylabel('angle from hanging (deg)'); ax[0].set_yticks([-180, -90, 0, 90, 180])
    ax[1].plot(L[:, 0], L[:, 3], lw=1); ax[1].set_ylabel('wheel speed (rad/s)')
    ax[2].plot(L[:, 0], L[:, 4], lw=.8); ax[2].set_ylabel('motor (V)'); ax[2].set_xlabel('time (s)')
    for a_ in ax:
        a_.grid(alpha=.3)
        a_.fill_between(L[:, 0], 0, 1, where=L[:, 6] > 0, color='tab:green', alpha=.12, transform=a_.get_xaxis_transform(), lw=0)
    ax[0].set_title('Simulated swing-up  ' + '  '.join(f'{k}={v:g}' for k, v in P.items()) + '   (green = LQR holding)', fontsize=9)
    plt.tight_layout(); plt.savefig('swingup_sim.png', dpi=110); print('saved swingup_sim.png')
