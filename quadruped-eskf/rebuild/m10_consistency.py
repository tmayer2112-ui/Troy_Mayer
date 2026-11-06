"""Module 10: is the filter honest about its uncertainty (NEES), and what can it never know (observability)?

  1. Do WORKSHEET.md, Module 10 on paper first.
  2. Fill in the three TODOs below (about 9 lines).
  3. Run:  python check.py 10   then:  python run.py report

Same checks as the main project: pipeline.py `nees` and observability.py (the SVD of the stacked
observability matrix, which found exactly 4 unobservable directions).
"""
import numpy as np


def nees(err, P):
    """Normalized estimation error squared: err^T P^-1 err. If the filter is honest, its average is
    the number of components (3 for velocity)."""
    # TODO (1 line), worksheet 10.1: err^T P^-1 err   (np.linalg.solve(P, err) computes P^-1 err)
    raise NotImplementedError


def observability_matrix(Fs, Hs):
    """Stack O = [H_0; H_1 F_0; H_2 F_1 F_0; ...]. Each block says how an error at the window's start
    shows up in the measurement k steps later."""
    # TODO (about 7 lines), worksheet 10.3:
    #   Phi starts as the identity. For each k: append H_k @ Phi, then Phi = F_k @ Phi.
    #   Return np.vstack of the list.
    raise NotImplementedError


def count_unobservable(O, rel_tol=1e-10):
    """How many directions the measurements can't see: singular values below rel_tol times the largest."""
    # TODO (2 lines): s = np.linalg.svd(O, compute_uv=False); count the s below rel_tol * s[0]
    raise NotImplementedError


def linearize_window(d, E, k0, length):
    """(Provided.) F_k and H_k along the noise-free truth for `length` samples starting at k0, using
    YOUR error_transition and measurement. Only the body states and the feet planted for the whole
    window are kept, so a foot in the air doesn't count as "unobservable"."""
    from so3 import rot_to_quat
    planted = np.flatnonzero(d["contact"][k0:k0 + length].all(0))
    keep = list(range(E.NB)) + [E.foot_index(i) + j for i in planted for j in range(3)]
    Fs, Hs = [], []
    for k in range(k0, k0 + length):
        f = E.Eskf(E.Params(), d["p"][k], d["v"][k], d["q"][k], d["ba"][k], d["bg"][k], np.eye(E.NB))
        for i in planted:
            f.d[i] = d["feet_world"][k, i]
            f.active[i] = True
        Fs.append(f.error_transition(d["acc"][k], d["gyro"][k], d["dt"])[np.ix_(keep, keep)])
        Hs.append(np.vstack([f.measurement(i, d["legs_true"][k, i])[1][:, keep] for i in planted]))
    names = ["dp_x", "dp_y", "dp_z", "dv_x", "dv_y", "dv_z", "dth_x", "dth_y", "dth_z", "dba_x", "dba_y",
             "dba_z", "dbg_x", "dbg_y", "dbg_z"] + [f"foot{i}_{a}" for i in planted for a in "xyz"]
    return Fs, Hs, names
