# design2.py - LQR + momentum observer (does not depend on the motor's unknown a, b)
#   p = (1+r)*theta_dot + r*psi_dot  (total angular momentum about the pivot / J)
#   p_dot = lam2*sin(theta) - c*theta_dot   -> the motor never appears; the wheel speed is measured
import numpy as np
from scipy.linalg import solve_discrete_are, solve_discrete_lyapunov
from design import plant, K, Ts, LAM2, R_I, C_V, SIG_TH, SIG_W
r = R_I
A2 = np.array([[1, Ts/(1+r)], [Ts*LAM2, 1 - Ts*C_V/(1+r)]])      # states [theta, p]
G2 = np.array([[-Ts*r/(1+r)], [Ts*C_V*r/(1+r)]])                   # input: measured psi_dot
C2 = np.array([[1.0, 0]])
def obs_gain(q_p):
    W = np.diag([0, (q_p*Ts)**2]); V = np.array([[SIG_TH**2]])
    Pk = solve_discrete_are(A2.T, C2.T, W, V)
    return (Pk@C2.T@np.linalg.inv(C2@Pk@C2.T + V)).ravel()
def loop(l, g=1.0, **pp):
    Ap, Bp = plant(**pp)
    # state z = [x(3) true, th_h, p_h]; measurements at k: th_m = x0 (+n1), w_m = x2 (+n2)
    # control: u = -g*(K0*th_h + K1*thd_h + K2*w_m), thd_h = (p_h - r*w_m)/(1+r)
    # next: x' = Ap x + Bp u ; observer predict from (th_h,p_h,w_m), correct with th_m' = x0'
    Ku = -g*np.array([K[0], K[1]/(1+r), K[2] - K[1]*r/(1+r)])       # u = Ku . [th_h, p_h, w_m]
    # express u as row over z (w_m = x2): u = Ku0*th_h + Ku1*p_h + Ku2*x2
    U = np.zeros((1,5)); U[0,3] = Ku[0]; U[0,4] = Ku[1]; U[0,2] = Ku[2]
    Px = np.hstack([np.eye(3), np.zeros((3,2))])
    Xn = Ap@Px + Bp@U                                                # x' rows over z
    pred = A2@np.vstack([np.eye(5)[3], np.eye(5)[4]]) + G2@np.eye(5)[[2]]   # [th_h, p_h]^- rows over z
    corr = pred + np.outer(l, Xn[0] - pred[0])
    F = np.vstack([Xn, corr])
    # noise inputs: n_th on the correction, n_w on the control and on the prediction input
    Nth = np.zeros((5,1)); Nth[3:,0] = l
    Nw = np.zeros((5,1)); Nw[:3,0] = (Bp*Ku[2]).ravel(); Nw[3:,0] = (G2 - np.outer(l, (Bp*Ku[2])[0])).ravel() if False else (G2.ravel())
    return F, U, Nth, Nw, Ku
def stats(l, g=1.0, **pp):
    F, U, Nth, Nw, Ku = loop(l, g, **pp)
    rho = max(abs(np.linalg.eigvals(F)))
    if rho >= 1: return rho, float('nan')
    S = solve_discrete_lyapunov(F, Nth@Nth.T*SIG_TH**2 + Nw@Nw.T*SIG_W**2)
    Uu = U.copy(); unoise = np.sqrt((Uu@S@Uu.T)[0,0] + (Ku[2]*SIG_W)**2)
    return rho, unoise
if __name__ == '__main__':
    for q in [1, 3, 10, 30, 100]:
        l = obs_gain(q); rho, un = stats(l)
        print(f'q_p {q:4}: L = [{l[0]:.5f}, {l[1]:.4f}]   |z|max {rho:.5f}   u noise {un:.3f} V')
    l = obs_gain(10)
    print('\nrobustness with q_p = 10, max |z| (< 1 stable):')
    for name, pp in [('nominal', {}), ('tau 0.4 ms', dict(tau=0.0004)), ('tau 50 ms', dict(tau=0.05)), ('lambda -10%', dict(lam2=LAM2*0.81)),
                     ('lambda +10%', dict(lam2=LAM2*1.21)), ('r -30%', dict(r=R_I*0.7)), ('r +30%', dict(r=R_I*1.3)),
                     ('ratio -30%', dict(ratio=268.32/30.43*0.7)), ('ratio +30%', dict(ratio=268.32/30.43*1.3))]:
        print(f'  {name:12s}', '  '.join(f'g={g}: {stats(l, g, **pp)[0]:.5f}' for g in (0.6, 1.0, 2.0)))
