"""Write data/qp_fixture.json: 200 random controller states and what your REAL qp_balance.py does with them
(foot forces and stance torques). check.py 12 compares your Module 12 against it.
Needs the real project's dependencies (mujoco, scipy); you don't need to run it, the file is committed.
Run from anywhere:  python make_qp_fixture.py"""
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.append(str(HERE.parent / "python"))
from qeskf.qp_balance import QPBalanceController  # noqa: E402
from qeskf.sim import load_model  # noqa: E402

ctl = QPBalanceController(load_model(torque=True))
rng = np.random.default_rng(12)
q0 = np.tile([0.0, 0.9, -1.8], 4)
cases = []
for i in range(200):
    t = 0.0 if i < 20 else 2.0 + rng.uniform(0, 1)       # standing, then walking (two or four feet)
    big = i % 5 == 0                                     # every 5th asks for more than friction allows
    cmd = rng.uniform(-1, 1, 3) * (5.0 if big else 0.5)
    fb = (rng.uniform(-0.5, 0.5, 3), np.r_[rng.uniform(-0.2, 0.2, 2), rng.uniform(-3, 3)], rng.uniform(-1, 1, 3),
          ctl.h_des + rng.uniform(-0.03, 0.03))
    q = q0 + rng.normal(0, 0.1, 12)
    ctl.i_err = rng.uniform(-0.5, 0.5, 3)
    ctl._t_last = None                                   # so plan() uses i_err as given
    i_err = ctl.i_err.copy()
    ctl.plan(t, cmd, fb, q)
    cases.append({"stance": ctl.stance.tolist(), "q": q.tolist(), "v_body": fb[0].tolist(), "rpy": fb[1].tolist(),
                  "w_body": fb[2].tolist(), "height": float(fb[3]), "cmd": cmd.tolist(), "i_err": i_err.tolist(),
                  "f": ctl.f.tolist(), "tau": ctl.tau_ff.tolist()})
(HERE / "data").mkdir(exist_ok=True)
(HERE / "data" / "qp_fixture.json").write_text(json.dumps({"source": "python/qeskf/qp_balance.py plan()",
                                                           "cases": cases}))
print(f"wrote {len(cases)} cases to data/qp_fixture.json")
