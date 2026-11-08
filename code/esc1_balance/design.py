# design.py - discrete LQR + steady-state Kalman observer for the reaction-wheel pendulum, 1 kHz
import numpy as np
from scipy.linalg import solve_discrete_are, expm
Ts = 0.001
LAM2 = 8.207**2; C_V = 0.12; R_I = 0.01173; RATIO = 268.32/30.43   # measured: lambda^2, viscous, I_w/J, wheel rad/s per V
def plant(tau=0.0074, lam2=LAM2, r=R_I, ratio=RATIO, c=C_V):
    b = r/(tau*(1+r)); a = ratio*b
    A = np.array([[0,1,0],[lam2,-c,b],[-lam2,c,-b/r-b]]); B = np.array([[0],[-a],[a/r+a]])
    M = np.zeros((4,4)); M[:3,:3]=A; M[:3,3:]=B; E = expm(M*Ts)
    return E[:3,:3], E[:3,3:]
Ad, Bd = plant()
Q = np.diag([1/np.radians(2)**2, 1/1.0**2, 1/100**2]); R = np.array([[1/2.0**2]])
P = solve_discrete_are(Ad, Bd, Q, R); K = np.linalg.solve(R + Bd.T@P@Bd, Bd.T@P@Ad).ravel()
# Kalman: measure theta and wheel speed
C = np.array([[1.0,0,0],[0,0,1.0]])
SIG_TH = np.radians(0.03); SIG_W = 0.6          # sensor noise: tilt 0.03 deg, wheel speed (1 ms difference) 0.6 rad/s
def kalman(q_th=200.0, q_w=2000.0):             # process noise: rad/s^2 on the pendulum and on the wheel
    W = np.diag([0, (q_th*Ts)**2, (q_w*Ts)**2]); V = np.diag([SIG_TH**2, SIG_W**2])
    Pk = solve_discrete_are(Ad.T, C.T, W, V)
    L = Pk@C.T@np.linalg.inv(C@Pk@C.T + V)     # filtered-form gain: x = x_pred + L (y - C x_pred)
    return L, W, V
def closed_loop(L, g=1.0, **pp):
    Ap, Bp = plant(**pp)
    # augmented state [x (true), xh (estimate)]; xh_pred = Ad xh + Bd u; xh = xh_pred + L (C x_next - C xh_pred)
    # u[k] = -g K xh[k];  x[k+1] = Ap x + Bp u
    n=3; I=np.eye(n)
    # xh[k+1] = (I - L C)(Ad xh + Bd u) + L C (Ap x + Bp u)
    U = -g*K[None]
    Fxx = Ap; Fxh = Bp@U
    Fhx = L@C@Ap; Fhh = (I-L@C)@(Ad + Bd@U) + L@C@Bp@U
    F = np.block([[Fxx, Fxh],[Fhx, Fhh]])
    return max(abs(np.linalg.eigvals(F))), F
def u_noise(L, g=1.0):
    # steady-state std of u from sensor noise only (true state at rest)
    n=3; I=np.eye(n); U=-g*K[None]
    _, F = closed_loop(L, g)
    G = np.zeros((6,2)); G[3:,:] = L    # noise enters the estimate through L
    S = np.zeros((6,6))
    V = np.diag([SIG_TH**2, SIG_W**2])
    from scipy.linalg import solve_discrete_lyapunov; S = solve_discrete_lyapunov(F, G@V@G.T)
    H = np.hstack([np.zeros((1,3)), U])
    return float(np.sqrt((H@S@H.T)[0, 0]))
if __name__ == '__main__':
    print('K =', np.round(K, 4))
    for qth in [50, 100, 200, 400, 800]:
        L,_,_ = kalman(qth)
        print(f'q_th {qth:4d}: u noise {u_noise(L):.3f} V   nominal |z|max {closed_loop(L)[0]:.5f}')
    L,_,_ = kalman(200)
    print('L =', np.round(L, 6).tolist())
    print('robustness, max |z| (must be < 1):')
    for name, pp in [('tau 0.4 ms', dict(tau=0.0004)), ('tau 50 ms', dict(tau=0.05)), ('lambda -10%', dict(lam2=LAM2*0.81)),
                     ('lambda +10%', dict(lam2=LAM2*1.21)), ('r -30%', dict(r=R_I*0.7)), ('r +30%', dict(r=R_I*1.3)),
                     ('ratio -20%', dict(ratio=RATIO*0.8)), ('ratio +20%', dict(ratio=RATIO*1.2))]:
        print(f'  {name:12s}', ' '.join(f'g={g}: {closed_loop(L, g, **pp)[0]:.5f}' for g in (0.6, 1.0, 2.0)))
