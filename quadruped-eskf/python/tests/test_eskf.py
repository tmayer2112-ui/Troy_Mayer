"""Filter math tests. Same checks as the C++ GoogleTest suite, on the Python reference."""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from qeskf import kinematics as K  # noqa: E402
from qeskf.eskf import IBA, IBG, IP, ITH, IV, NB, Eskf, EskfParams  # noqa: E402
from qeskf.so3 import exp_so3, log_so3, quat_exp, quat_mul, quat_to_rot, rot_to_quat  # noqa: E402

RNG = np.random.default_rng(42)
Q_STAND = np.array([0.05, 0.8, -1.6] * 4)


def random_filter(n_feet=2, seed=0):
    rng = np.random.default_rng(seed)
    q = quat_exp(rng.normal(0, 0.3, 3))
    f = Eskf(EskfParams(), p=rng.normal(0, 1, 3), v=rng.normal(0, 0.5, 3), q=q,
             ba=rng.normal(0, 0.05, 3), bg=rng.normal(0, 0.01, 3), P0=np.eye(NB) * 1e-4)
    qenc = Q_STAND + rng.normal(0, 0.1, 12)
    for leg in range(n_feet):
        f.augment(leg, qenc)
    return f, qenc, rng


def boxplus(f, dx):
    """Return a copy of the filter state perturbed by the error vector dx (same convention as inject)."""
    g = Eskf(f.prm, f.x.p.copy(), f.x.v.copy(), f.x.q.copy(), f.x.ba.copy(), f.x.bg.copy(), f.P.copy())
    g.x.feet = {k: v.copy() for k, v in f.x.feet.items()}
    g.x.order = list(f.x.order)
    g.x.p = g.x.p + dx[IP:IP + 3]
    g.x.v = g.x.v + dx[IV:IV + 3]
    g.x.q = quat_mul(g.x.q, quat_exp(dx[ITH:ITH + 3]))
    g.x.ba = g.x.ba + dx[IBA:IBA + 3]
    g.x.bg = g.x.bg + dx[IBG:IBG + 3]
    for j, leg in enumerate(g.x.order):
        g.x.feet[leg] = g.x.feet[leg] + dx[NB + 3 * j:NB + 3 * j + 3]
    return g


def boxminus(a, b):
    """Error vector taking state b to state a: a = b [+] dx."""
    dx = np.zeros(a.dim)
    dx[IP:IP + 3] = a.x.p - b.x.p
    dx[IV:IV + 3] = a.x.v - b.x.v
    dx[ITH:ITH + 3] = log_so3(quat_to_rot(b.x.q).T @ quat_to_rot(a.x.q))
    dx[IBA:IBA + 3] = a.x.ba - b.x.ba
    dx[IBG:IBG + 3] = a.x.bg - b.x.bg
    for j, leg in enumerate(a.x.order):
        dx[NB + 3 * j:NB + 3 * j + 3] = a.x.feet[leg] - b.x.feet[leg]
    return dx


@pytest.mark.parametrize("seed", range(5))
def test_transition_jacobian_matches_finite_differences(seed):
    f, _, rng = random_filter(2, seed)
    acc = rng.normal(0, 3, 3) + [0, 0, 9.8]
    gyr = rng.normal(0, 2, 3)
    dt = 1e-3
    F = f.error_transition(acc, gyr, dt)
    ref = boxplus(f, np.zeros(f.dim))
    ref.propagate_nominal(acc, gyr, dt)
    h = 1e-6
    Ffd = np.zeros_like(F)
    for i in range(f.dim):
        e = np.zeros(f.dim)
        e[i] = h
        gp = boxplus(f, e)
        gm = boxplus(f, -e)
        gp.propagate_nominal(acc, gyr, dt)
        gm.propagate_nominal(acc, gyr, dt)
        Ffd[:, i] = (boxminus(gp, ref) - boxminus(gm, ref)) / (2 * h)
    np.testing.assert_allclose(F, Ffd, atol=1e-8)


@pytest.mark.parametrize("seed", range(5))
def test_measurement_jacobian_matches_finite_differences(seed):
    f, qenc, _ = random_filter(3, seed)
    legs = list(f.x.order)
    _, H, _ = f.measurement(legs, qenc)
    h = 1e-6
    Hfd = np.zeros_like(H)
    for i in range(f.dim):
        e = np.zeros(f.dim)
        e[i] = h
        rp, _, _ = boxplus(f, e).measurement(legs, qenc)
        rm, _, _ = boxplus(f, -e).measurement(legs, qenc)
        Hfd[:, i] = -(rp - rm) / (2 * h)   # r = z - h(x)  =>  dh/dx = -dr/dx
    np.testing.assert_allclose(H, Hfd, atol=1e-8)


@pytest.mark.parametrize("seed", range(5))
def test_augmentation_jacobian_matches_finite_differences(seed):
    f, qenc, _ = random_filter(1, seed)
    leg = 3

    def new_foot(g):
        return g.x.p + quat_to_rot(g.x.q) @ K.foot_position(leg, qenc[9:12])

    R = quat_to_rot(f.x.q)
    fk = K.foot_position(leg, qenc[9:12])
    J = np.zeros((3, f.dim))
    J[:, IP:IP + 3] = np.eye(3)
    from qeskf.so3 import skew
    J[:, ITH:ITH + 3] = -R @ skew(fk)
    h = 1e-6
    Jfd = np.zeros_like(J)
    for i in range(f.dim):
        e = np.zeros(f.dim)
        e[i] = h
        Jfd[:, i] = (new_foot(boxplus(f, e)) - new_foot(boxplus(f, -e))) / (2 * h)
    np.testing.assert_allclose(J, Jfd, atol=1e-8)
    # and the covariance the filter builds uses exactly that J
    P_before = f.P.copy()
    f.augment(leg, qenc)
    n = P_before.shape[0]
    np.testing.assert_allclose(f.P[n:, :n], J @ P_before, atol=1e-14)


def test_covariance_stays_symmetric_positive_definite():
    f, qenc, rng = random_filter(0, 7)
    contact = np.zeros(4, bool)
    for k in range(20000):
        if k % 97 == 0:
            contact = rng.random(4) > 0.4
        q = Q_STAND + 0.05 * np.sin(0.01 * k + np.arange(12))
        f.step(rng.normal([0, 0, 9.8], 0.5), rng.normal(0, 0.3, 3), q, contact, 1e-3)
        if k % 500 == 0:
            assert np.allclose(f.P, f.P.T, atol=0)
            assert np.linalg.eigvalsh(f.P).min() > 0


def test_quaternion_stays_unit_norm():
    f, qenc, rng = random_filter(2, 3)
    for k in range(100000):
        f.propagate_nominal(rng.normal([0, 0, 9.8], 1.0), rng.normal(0, 5.0, 3), 1e-3)
    assert abs(np.linalg.norm(f.x.q) - 1.0) < 1e-12


def test_marginalize_is_exact_gaussian_marginal():
    f, qenc, _ = random_filter(3, 5)
    P = f.P.copy()
    order = list(f.x.order)
    f.marginalize(order[1])
    keep = np.r_[0:NB + 3, NB + 6:NB + 9]
    np.testing.assert_array_equal(f.P, P[np.ix_(keep, keep)])
    assert f.x.order == [order[0], order[2]]


def test_so3_roundtrip():
    for _ in range(100):
        phi = RNG.normal(0, 1.0, 3)
        phi *= RNG.uniform(0, 3.1) / np.linalg.norm(phi)   # log is only unique for |phi| < pi
        np.testing.assert_allclose(log_so3(exp_so3(phi)), phi, atol=1e-9)
        R = exp_so3(phi)
        np.testing.assert_allclose(quat_to_rot(rot_to_quat(R)), R, atol=1e-12)
        np.testing.assert_allclose(quat_to_rot(quat_exp(phi)), R, atol=1e-12)
