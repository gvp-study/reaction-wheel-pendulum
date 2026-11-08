#!/usr/bin/env python3
"""plot_sim.py - plot one simulated balance run (lean, wheel speed, motor voltage).

  python3 plot_sim.py                 # nominal, 1.5 deg start
  python3 plot_sim.py offset=0.5      # balance point 0.5 deg wrong
  python3 plot_sim.py start=3         # 3 deg start
  python3 plot_sim.py tap=3           # a tap at 3 s
  python3 plot_sim.py gain=1.6 tau=0.02 start=2      (options can be combined)
Needs design.py, design2.py and simulate.py in the same folder.
"""
import sys
import matplotlib.pyplot as plt
from simulate import run

names = {'offset': 'offset_deg', 'start': 'th0_deg', 'tap': 'push_at', 'gain': 'g', 'tau': 'tau', 'time': 'T'}
kw = {}
for a in sys.argv[1:]:
    k, v = a.split('=')
    kw[names.get(k, k)] = float(v)
lg = run(**kw)
fig, ax = plt.subplots(3, 1, sharex=True, figsize=(9, 7))
for a, col, lab in zip(ax, (1, 2, 3), ('lean (deg)', 'wheel (rad/s)', 'motor volts')):
    a.plot(lg[:, 0], lg[:, col], lw=1.2); a.set_ylabel(lab); a.grid(alpha=.3)
ax[0].set_title('simulated balance  ' + (' '.join(sys.argv[1:]) or 'nominal'))
ax[-1].set_xlabel('time (s)')
fig.tight_layout(); fig.savefig('sim_run.png', dpi=120); print('saved sim_run.png')
plt.show()
