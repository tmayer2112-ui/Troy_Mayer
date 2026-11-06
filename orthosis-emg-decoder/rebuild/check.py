"""Auto-checker for the rebuild.

    python check.py 1                check Module 1 (also 2 ... 7)
    python check.py all              check everything you've written so far
    python check.py 4 --solutions    run the checks against the reference solutions instead of your code

Each check either PASSes, FAILs with the numbers it expected (the worst entry, for big arrays), or
says the TODO isn't written yet. Expected values come from the worksheet's hand calculations, from the
real project's results files (named in the check), or from small independent implementations in this
file, never from solutions/.
"""
import contextlib
import importlib
import io
import math
import sys
import traceback
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
USE_SOLUTIONS = "--solutions" in sys.argv
if USE_SOLUTIONS:
    sys.path.insert(0, str(HERE / "solutions"))     # ahead of this folder, so every module comes from there
if str(HERE) not in sys.path:
    sys.path.append(str(HERE))                       # appended, never inserted: it must not shadow solutions/

RESULTS = []


class NotWritten(Exception):
    pass


def load(name):
    return importlib.import_module(name)


CHECKS = []


def check(label):
    """Register one check. It records PASS / FAIL / TODO when run."""
    def wrap(fn):
        CHECKS.append((label, fn))
        return fn
    return wrap


def run_checks():
    for label, fn in CHECKS:
        try:
            with contextlib.redirect_stdout(io.StringIO()):
                fn()
            RESULTS.append("PASS")
            print(f"  PASS  {label}")
        except NotImplementedError:
            RESULTS.append("TODO")
            print(f"  TODO  {label}  (not written yet)")
        except NotWritten as e:
            RESULTS.append("TODO")
            print(f"  TODO  {label}  ({e})")
        except AssertionError as e:
            RESULTS.append("FAIL")
            print(f"  FAIL  {label}\n        {e}")
        except Exception:
            RESULTS.append("FAIL")
            print(f"  FAIL  {label}  (crashed)")
            print("        " + traceback.format_exc().strip().replace("\n", "\n        "))
    CHECKS.clear()


def close(got, want, tol, what):
    got, want = np.asarray(got, float if not np.iscomplexobj(got) else complex), np.asarray(want)
    assert got.shape == want.shape, f"{what}: shape {got.shape}, expected {want.shape}"
    diff = np.abs(got - want)
    err = float(diff.max()) if got.size else 0.0
    if err <= tol:
        return
    if got.size <= 12:
        raise AssertionError(f"{what}: got {np.round(got, 6).tolist()}, expected {np.round(want, 6).tolist()}"
                             f" (max difference {err:.2e}, allowed {tol:.0e})")
    idx = np.unravel_index(np.argmax(diff), diff.shape)
    raise AssertionError(f"{what}: worst entry {list(map(int, idx))}: got {got[idx]:.6g}, expected {want[idx]:.6g}"
                         f" (difference {err:.2e}, allowed {tol:.0e})")


def needs(module, fn_name, probe, n):
    """Raise NotWritten if an earlier module's function this check depends on isn't written."""
    try:
        probe(getattr(load(module), fn_name))
    except NotImplementedError:
        raise NotWritten(f"finish {fn_name} in Module {n} first: this check uses it")


# ------------------------------------------------------------------ independent references
def ref_windows(x, win, step):
    starts = list(range(0, len(x) - win + 1, step))
    return np.array([x[s:s + win] for s in starts]), np.array(starts)


def ref_hudgins(w, thr):
    d = w[:, 1:] - w[:, :-1]
    sgn = np.sign(w)
    zc = np.sum((sgn[:, 1:] * sgn[:, :-1] == -1) & (np.abs(d) >= thr), axis=1)
    peak = np.sign(d[:, 1:]) * np.sign(d[:, :-1]) == -1
    ssc = np.sum(peak & (np.maximum(np.abs(d[:, 1:]), np.abs(d[:, :-1])) >= thr), axis=1)
    return np.concatenate([np.log(np.mean(np.abs(w), axis=1) + 1e-12),
                           np.log(np.sum(np.abs(d), axis=1) + 1e-12), zc, ssc], axis=1)


_DATA = {}


def dataset(flips=True):
    if flips not in _DATA:
        import simdata
        _DATA[flips] = simdata.make_dataset(flips=flips)
    return _DATA[flips]


def ref_table(recs):
    """Hudgins table built with the references above, so Module 4 can be checked without Module 1."""
    keys = sorted(recs)
    thr = 0.05 * np.sqrt(np.mean(np.concatenate([recs[k] for k in keys if k[0] <= 5]) ** 2, axis=0))
    X, y, day, rec, start, t0, t1, dur = [], [], [], [], [], [], [], []
    for i, k in enumerate(keys):
        w, s = ref_windows(recs[k], 400, 100)
        X.append(ref_hudgins(w, thr))
        n = len(s)
        y.append(np.full(n, k[1])); day.append(np.full(n, k[0])); rec.append(np.full(n, i)); start.append(s)
        t0.append(s / 2048); t1.append((s + 400) / 2048); dur.append(np.full(n, len(recs[k]) / 2048))
    from types import SimpleNamespace
    return SimpleNamespace(X=np.concatenate(X), y=np.concatenate(y), day=np.concatenate(day),
                           rec=np.concatenate(rec), start=np.concatenate(start), t0=np.concatenate(t0),
                           t1=np.concatenate(t1), dur=np.concatenate(dur))


def ref_lda_predict(Xtr, ytr, Xte):
    classes = np.unique(ytr)
    mu, sd = Xtr.mean(0), Xtr.std(0)
    sd[sd == 0] = 1.0
    Z, Zt = (Xtr - mu) / sd, (Xte - mu) / sd
    M = np.array([Z[ytr == k].mean(0) for k in classes])
    R = np.concatenate([Z[ytr == k] - M[i] for i, k in enumerate(classes)])
    P = np.linalg.inv(R.T @ R / (len(Z) - len(classes)))
    pri = np.array([np.mean(ytr == k) for k in classes])
    scores = Zt @ P @ M.T - 0.5 * np.einsum("kp,pq,kq->k", M, P, M) + np.log(pri)
    return classes[np.argmax(scores, axis=1)]


# ---------------------------------------------------------------------------------- Module 1
def module1():
    m = load("m1_features")
    x6 = np.array([[2.0], [-1.0], [-3.0], [1.0], [0.5], [-0.5]])       # Worksheet 1.2
    w6 = x6[None]

    @check("windows: 10 samples, win 4, step 2 -> starts [0, 2, 4, 6], window 1 = [2, 3, 4, 5]")
    def _():
        w, starts = m.windows(np.arange(10.0)[:, None], 4, 2)
        close(starts, [0, 2, 4, 6], 0, "starts")
        close(w.shape, (4, 4, 1), 0, "shape of w")
        close(w[1, :, 0], [2, 3, 4, 5], 0, "window 1")

    @check("worksheet 1.2 window [2, -1, -3, 1, 0.5, -0.5], dead band 1.5: MAV 1.333, WL 10.5, ZC 2, SSC 2")
    def _():
        close(m.mav(w6), [[8 / 6]], 1e-12, "MAV")
        close(m.wl(w6), [[10.5]], 1e-12, "WL")
        close(m.zc(w6, np.array([1.5])), [[2]], 0, "ZC (the 0.5 -> -0.5 crossing is inside the dead band)")
        close(m.ssc(w6, np.array([1.5])), [[2]], 0, "SSC")

    @check("all 16 features match an independent version on 2 synthetic recordings (4 channels)")
    def _():
        recs = dataset()
        for key in ((3, 4), (17, 9)):
            w, _ = m.windows(recs[key])
            thr = np.array([5.0, 3.0, 4.0, 6.0])
            close(m.hudgins(w, thr), ref_hudgins(*[ref_windows(recs[key], 400, 100)[0], thr]), 1e-9, f"features of {key}")

    @check("a reversed lead (x -> -x) changes no feature at all (worksheet 1.4)")
    def _():
        w, _ = m.windows(dataset()[(6, 2)])
        thr = np.full(4, 5.0)
        close(m.hudgins(-w, thr), m.hudgins(w, thr), 0.0, "features of -x minus features of x")

    @check("doubling the gain (and the dead band) shifts logMAV and logWL by ln 2 = 0.693; ZC, SSC unchanged")
    def _():
        w, _ = m.windows(dataset()[(6, 2)])
        thr = np.full(4, 5.0)
        diff = m.hudgins(2 * w, 2 * thr) - m.hudgins(w, thr)
        close(diff[:, :8], np.full(diff[:, :8].shape, math.log(2)), 1e-9, "logMAV, logWL shift")
        close(diff[:, 8:], np.zeros(diff[:, 8:].shape), 0, "ZC, SSC change")


# ---------------------------------------------------------------------------------- Module 2
def module2():
    m = load("m2_filter")
    K = 1 + 10e3 / 18e3

    @check("sallen_key_params: HPF 15k/470n -> f0 22.58 Hz, Q 0.6923; LPF 1k/330n -> 482.3 Hz  (board.json)")
    def _():
        close(m.sallen_key_params(15e3, 470e-9, K), (22.5752, 0.692308), 1e-4, "(f0, Q) of the HPF")
        close(m.sallen_key_params(1e3, 330e-9, K)[0], 482.2877, 1e-3, "f0 of the LPF")

    @check("analog_response at f0 is j K Q = 1.0769j for the high-pass, -j K Q for the low-pass")
    def _():
        Q = 1 / (3 - K)
        close(m.analog_response(np.array([22.5752]), 22.5752, Q, K, "high"), [1j * K * Q], 1e-9, "H_HP(j w0)")
        close(m.analog_response(np.array([482.29]), 482.29, Q, K, "low"), [-1j * K * Q], 1e-9, "H_LP(j w0)")

    @check("hpf_corner_ratio(Q = 0.6923) = 1.0218, so one HPF stage alone is -3 dB at 23.07 Hz")
    def _():
        close(m.hpf_corner_ratio(1 / (3 - K)), 1.021833, 1e-6, "Omega")

    @check("the two analog stages together peak at x 2.405 with corners 23.0 and 473.9 Hz  (board.json)")
    def _():
        f = np.logspace(0, np.log10(1024 * 0.999), 4000)
        h = m.cascade_analog(f)
        close(np.abs(h).max(), 2.405, 5e-4, "passband gain")
        close(m.corners(f, h), (23.0, 473.9), 0.06, "-3 dB corners, Hz")

    @check("bilinear_biquad matches H(s) with s = 2 fs (1 - z^-1)/(1 + z^-1) and the pre-warped w0")
    def _():
        fs, Q = 2048.0, 1 / (3 - K)
        f = np.linspace(5, 1000, 60)
        zi = np.exp(-2j * np.pi * f / fs)
        s = 2 * fs * (1 - zi) / (1 + zi)
        for f0, kind in ((22.5752, "high"), (482.2877, "low")):
            w0 = 2 * fs * np.tan(np.pi * f0 / fs)
            want = (K * s ** 2 if kind == "high" else K * w0 ** 2) / (s ** 2 + w0 / Q * s + w0 ** 2)
            b, a = m.bilinear_biquad(f0, Q, K, kind, fs)
            close(a[0], 1.0, 1e-12, f"{kind}-pass a[0]")
            close(m.digital_response(b, a, f, fs), want, 1e-9, f"{kind}-pass digital response")

    @check("at 2048 Hz: corners 23.0 and 476.4 Hz, and within 0.384 dB of the circuit below 400 Hz  (board.json)")
    def _():
        f = np.logspace(0, np.log10(1024 * 0.999), 4000)
        ha, hd = m.cascade_analog(f), m.cascade_digital(f, 2048)
        close(m.corners(f, hd), (23.0, 476.4), 0.06, "digital corners, Hz")
        err = np.max(np.abs(20 * np.log10(np.abs(hd[f < 400]) / np.abs(ha[f < 400]))))
        close(err, 0.384, 5e-4, "max |digital - analog| below 400 Hz, dB")


# ---------------------------------------------------------------------------------- Module 3
def module3():
    m = load("m3_frontend")

    @check("diff_amp_gains: buffered (Z = 0) gives G = 47, G_cm = 0; 22.5k / 17.5k gives 2.45 and -0.180")
    def _():
        close(m.diff_amp_gains(0.0, 0.0), (47.0, 0.0), 1e-12, "(G_diff, G_cm) at Z = 0")
        close(m.diff_amp_gains(22.5e3, 17.5e3), (2.450450, -0.180180), 1e-6, "(G_diff, G_cm) at 20 kOhm +/- 25 %")

    @check("v1 CMRR vs electrode impedance: 62.0, 32.3, 22.6, 17.8 dB  (board.json, v1_front_end_vs_electrode_impedance)")
    def _():
        got = [m.db(m.v1_cm_to_diff(*m.electrodes(z))) for z in (0.0, 5e3, 20e3, 50e3)]
        close(got, [62.0, 32.3, 22.6, 17.8], 0.05, "CMRR, dB")
        close(m.db(abs(m.diff_amp_gains(22.5e3, 17.5e3)[1]) / m.diff_amp_gains(22.5e3, 17.5e3)[0]), 22.7, 0.05,
              "imbalance term alone at 20 kOhm, dB (README: 22.7)")

    @check("input to clear the 0.4 V diode at max pot: 23.8 uV buffered, 455.8 uV at 20 kOhm  (board.json)")
    def _():
        got = [1e6 * m.input_to_clear_diode(m.diff_amp_gains(*m.electrodes(z))[0]) for z in (0.0, 20e3)]
        close(got, [23.8, 455.8], 0.05, "microvolts")

    @check("v2 with a 5 kOhm imbalance: 116.1 dB effective CMRR, so 1 V of hum leaks in as 1.57 uV  (board.json)")
    def _():
        r = m.v2_cm_to_diff(5e3)
        close(m.db(r), 116.1, 0.05, "dB")
        close(1e6 * r, 1.5655, 1e-3, "microvolts per volt")

    @check("peak_detector: worksheet 3.3, [0.5, 0.3, 0.9, 0.1] with 0.4 V drop, decay 0.5 -> [0.1, 0.05, 0.5, 0.25]")
    def _():
        close(m.peak_detector(np.array([[0.5], [0.3], [0.9], [0.1]]), 0.4, 0.5)[:, 0], [0.1, 0.05, 0.5, 0.25],
              1e-12, "e")

    @check("peak_detector handles every column at once and never goes negative")
    def _():
        v = np.random.default_rng(0).normal(0, 0.3, (3000, 3))
        e, want, out = 0.0, [], m.peak_detector(v, 0.4, 0.99)
        for c in range(3):
            e, col = 0.0, []
            for x in v[:, c]:
                e = max(e * 0.99, x - 0.4)
                col.append(e)
            want.append(col)
        close(out, np.array(want).T, 1e-12, "detector output")


# ---------------------------------------------------------------------------------- Module 4
def module4():
    m = load("m4_lda")

    @check("1-D, two classes {0, 2} and {4, 6}: the boundary is at 3 (worksheet 4.2)")
    def _():
        model = m.lda_fit(np.array([[0.0], [2.0], [4.0], [6.0]]), np.array([0, 0, 1, 1]))
        close(m.lda_predict(model, np.array([[2.9], [3.1], [-10.0], [10.0]])), [0, 1, 0, 1], 0, "predictions")

    @check("balanced accuracy: 9 of class 0 right, the 1 of class 1 wrong -> 0.5 (plain accuracy says 0.9)")
    def _():
        close(m.balanced_accuracy(np.array([0] * 9 + [1]), np.zeros(10, int)), 0.5, 1e-12, "balanced accuracy")
        close(m.balanced_accuracy(np.array([0, 0, 1, 1, 2]), np.array([0, 1, 1, 1, 0])), (0.5 + 1 + 0) / 3, 1e-12,
              "3 classes")

    @check("shares_a_sample: starts 0-900 step 100, windows 0-3 test -> [False, True, True, True] (worksheet 4.4)")
    def _():
        starts = np.arange(0, 1000, 100)
        close(m.shares_a_sample(starts, starts < 400), [False, True, True, True], 0, "shares a sample")

    @check("LDA on the synthetic decoder data predicts like an independent LDA (> 99.9 % agreement)")
    def _():
        T = ref_table(dataset())
        tr = np.isin(T.day, m.TRAIN_DAYS)
        got = m.lda_predict(m.lda_fit(T.X[tr], T.y[tr]), T.X[~tr])
        agree = np.mean(got == ref_lda_predict(T.X[tr], T.y[tr], T.X[~tr]))
        assert agree > 0.999, f"only {100 * agree:.2f} % of test windows get the same class as the reference"

    @check("cross_day: one balanced accuracy per test day, matching the independent LDA")
    def _():
        T = ref_table(dataset())
        got = m.cross_day(T)
        tr = np.isin(T.day, m.TRAIN_DAYS)
        want = []
        for d in m.TEST_DAYS:
            te = T.day == d
            p = ref_lda_predict(T.X[tr], T.y[tr], T.X[te])
            want.append(np.mean([np.mean(p[T.y[te] == k] == k) for k in range(11)]))
        close(got, want, 2e-3, "per-day balanced accuracy")

    @check("finding: the leaky shuffled split scores above cross-day, and > 99 % of its test windows share samples")
    def _():
        T = ref_table(dataset())
        acc, shared = m.random_split(T)
        cd = float(np.mean(m.cross_day(T)))
        assert acc > cd, f"shuffled {acc:.3f} is not above cross-day {cd:.3f}"
        assert shared > 0.99, f"only {100 * shared:.1f} % of test windows share a sample with training"


# ---------------------------------------------------------------------------------- Module 5
def module5():
    m = load("m5_polarity")

    @check("skewness([0, 0, 3]) = 0.7071 (worksheet 5.1); odd under x -> -x, unchanged by x -> 3x")
    def _():
        close(m.skewness(np.array([[0.0], [0.0], [3.0]])), [1 / math.sqrt(2)], 1e-12, "skewness")
        x = np.random.default_rng(0).gamma(2.0, 1.0, (500, 3))
        close(m.skewness(-x), -m.skewness(x), 1e-12, "skewness(-x) + skewness(x)")
        close(m.skewness(3 * x), m.skewness(x), 1e-12, "skewness(3x) - skewness(x)")

    @check("rms([3, -4]) = 3.5355; even under x -> -x")
    def _():
        close(m.rms(np.array([[3.0], [-4.0]])), [math.sqrt(12.5)], 1e-12, "rms")
        x = np.random.default_rng(1).normal(size=(50, 2))
        close(m.rms(-x), m.rms(x), 0, "rms(-x) - rms(x)")

    @check("beta: worksheet 5.2 profile gives 1.0196; a flipped profile gives -1, per channel")
    def _():
        s, ref = np.array([[0.4], [-0.2], [0.6]]), np.array([[0.5], [-0.1], [0.5]])
        close(m.beta(s, ref), [0.52 / 0.51], 1e-12, "beta")
        r = np.random.default_rng(2).normal(size=(11, 4))
        close(m.beta(r * np.array([1, -1, 0.5, -2]), r), [1, -1, 0.5, -2], 1e-12, "beta per channel")

    @check('call([0.9, -1.1, 0.2, -0.5]) = "+-?-"')
    def _():
        assert m.call(np.array([0.9, -1.1, 0.2, -0.5])) == "+-?-", f'got "{m.call(np.array([0.9, -1.1, 0.2, -0.5]))}"'

    @check("the audit recovers the generator's lead patterns: days 1-5 ----, 6-15 ++++, 16-20 --++")
    def _():
        aud = m.polarity_audit(dataset())
        wrong = [(d, c) for d, (c, _) in aud.items()
                 if c != ("----" if d <= 5 else "++++" if d <= 15 else "--++")]
        assert not wrong, f"wrong calls (day, pattern): {wrong[:5]}"

    @check("flip_augment: each window gets one of the 16 lead patterns, each lead reversed about half the time")
    def _():
        F = np.random.default_rng(3).normal(size=(4000, 2, 16))
        out = m.flip_augment(F, np.random.default_rng(4))
        assert out.shape == F.shape, f"shape {out.shape}, expected {F.shape}"
        pats = np.array([[np.array_equal(out[i], m.with_leads_flipped(F[i:i + 1], [p])[0]) for p in range(16)]
                         for i in range(4000)])
        assert pats.any(axis=1).all(), "some rows are not a lead pattern of their window"
        p = np.argmax(pats, axis=1)
        frac = [np.mean((p >> ch) & 1) for ch in range(4)]
        close(frac, [0.5] * 4, 0.03, "fraction of windows with each lead reversed")

    @check("finding: LDA on Hudgins features gives identical predictions with and without the lead flips")
    def _():
        needs("m4_lda", "lda_fit", lambda f: f(np.zeros((4, 1)) + np.arange(4)[:, None], np.array([0, 0, 1, 1])), 4)
        needs("m1_features", "mav", lambda f: f(np.ones((1, 2, 1))), 1)
        L, F1 = load("m4_lda"), load("m1_features")
        T, Tn = F1.hudgins_table(dataset()), F1.hudgins_table(dataset(False))
        tr = np.isin(T.day, L.TRAIN_DAYS)
        p = L.lda_predict(L.lda_fit(T.X[tr], T.y[tr]), T.X[~tr])
        pn = L.lda_predict(L.lda_fit(Tn.X[tr], Tn.y[tr]), Tn.X[~tr])
        close(p, pn, 0, "predictions with flips minus without")


# ---------------------------------------------------------------------------------- Module 6
def module6():
    m = load("m6_speed_loop")

    @check("imc_gains(K 187, tau 20 ms, lambda 50 ms) = (0.00214, 0.1070)  (firmware/results/metrics.json)")
    def _():
        close(m.imc_gains(187.0, 0.020, 0.050), (0.0021390, 0.1069519), 1e-7, "(Kp, Ki)")

    @check("feedforward: 15 deg/s -> 0.1302; 0.3 deg/s -> 0.0016 (no friction term); 10 deg/s at 150 deg/s^2 -> 0.1195")
    def _():
        close([m.feedforward(15.0, 0.0), m.feedforward(0.3, 0.0), m.feedforward(10.0, 150.0), m.feedforward(-15.0, 0.0)],
              [15 / 187 + 0.05, 0.3 / 187, 13 / 187 + 0.05, -(15 / 187 + 0.05)], 1e-12, "duty")

    @check("one PI step by hand (worksheet 6.3): ref 10, meas 8 -> duty 0.10775, integral 0.000214")
    def _():
        c = m.VelocityPI()
        c.prev_ref = 10.0
        out = c.step(10.0, 8.0, 0.001)
        close(out, 10 / 187 + 0.05 + 2 * 0.02 / (187 * 0.05), 1e-12, "duty")
        close(c.integ, 2 * 0.001 / (187 * 0.05), 1e-12, "integral term")

    @check("step() feeds the setpoint's acceleration forward: ref 9.9 -> 10 in 1 ms is 100 deg/s^2 -> duty 0.11417")
    def _():
        c = m.VelocityPI()
        c.prev_ref = 9.9
        close(c.step(10.0, 10.0, 0.001), (10 + 0.02 * 100) / 187 + 0.05, 1e-9, "duty (e = 0, so all feed-forward)")
        close(c.prev_ref, 10.0, 0, "prev_ref after the step")

    @check("anti-windup part 1: saturated and pushing further -> the integral doesn't move; pulling back -> it does")
    def _():
        c = m.VelocityPI()
        c.prev_ref, c.integ = 20.0, 0.5
        assert c.step(20.0, 0.0, 0.001) == 0.6, "the output should be clamped at 0.6"
        close(c.integ, 0.5, 1e-15, "integral while saturated and e > 0")
        c.step(20.0, 30.0, 0.001)
        assert c.integ < 0.5, f"with e < 0 the integral should drop, it is {c.integ}"
        c = m.VelocityPI(antiwindup=False)
        c.prev_ref, c.integ = 20.0, 0.5
        c.step(20.0, 0.0, 0.001)
        assert c.integ > 0.5, "with antiwindup=False the integral should keep charging"

    @check("anti-windup part 2: while ramping, a lagging joint doesn't charge the integral; a leading one discharges it")
    def _():
        c = m.VelocityPI()
        c.prev_ref = 10.0
        c.step(10.0, 8.0, 0.001, ramping=True)
        close(c.integ, 0.0, 0, "integral after a lagging, ramping tick")
        c.prev_ref = 10.0
        c.step(10.0, 12.0, 0.001, ramping=True)
        close(c.integ, -2 * 0.001 / (187 * 0.05), 1e-15, "integral after a leading, ramping tick")

    @check("the IMC promise: with an exact model, PI alone reaches 63 % of a step at t = lambda = 50 ms")
    def _():
        close(m.pi_only_step()[1], 0.050, 0.0015, "time to 63 %, s")

    @check("toy arm: lifting at 15 deg/s, feed-forward alone stalls (> 50 % error), the PI holds it under 2 %")
    def _():
        e_ff, e_cl = m.lift(False)[1], m.lift(True)[1]
        assert e_ff > 7.5, f"feed-forward only: error {e_ff:.2f} deg/s, expected a stall"
        assert abs(e_cl) < 0.3, f"closed loop: error {e_cl:.3f} deg/s ({100 * abs(e_cl) / 15:.1f} %)"

    @check("toy arm: at zero command the PI holds position (drift < 0.05 deg over 5 s)")
    def _():
        d = m.hold(True)[1]
        assert abs(d) < 0.05, f"drift {d:.3f} deg"


# ---------------------------------------------------------------------------------- Module 7
def module7():
    m = load("m7_bayes")

    @check("bayes_step, worksheet 7.2: levels [1, 2, 4], p [0.2, 0.6, 0.2], alpha 0.1, beta 0, sample 2")
    def _():
        q, est = m.bayes_step(np.array([0.2, 0.6, 0.2]), 2.0, np.array([1.0, 2.0, 4.0]), 0.1, 0.0)
        pred = np.array([0.24, 0.52, 0.24])
        like = np.exp(-math.sqrt(2) * 2.0 / np.array([1.0, 2.0, 4.0])) / np.array([1.0, 2.0, 4.0])
        want = pred * like / np.sum(pred * like)
        close(q, want, 1e-12, "posterior")
        close(est, math.exp(np.sum(want * np.log([1.0, 2.0, 4.0]))), 1e-12, "estimate")

    @check("bayes_step: the jump term and normalisation (alpha 0, beta 0.3, sample 0)")
    def _():
        lv = np.array([1.0, 2.0, 4.0, 8.0])
        q, _ = m.bayes_step(np.array([1.0, 0, 0, 0]), 0.0, lv, 0.0, 0.3)
        want = (0.7 * np.array([1.0, 0, 0, 0]) + 0.3 / 4) / lv
        close(q, want / want.sum(), 1e-12, "posterior")
        close(q.sum(), 1.0, 1e-12, "sum of the posterior")

    @check("two_pole at 4 Hz: one pole's coefficient and the first outputs on |x| = 1")
    def _():
        a = 1 - math.exp(-2 * math.pi * 4 / 1000)
        out = m.two_pole(np.array([1.0, -1.0, 1.0]), 4.0, 1000)
        e1 = [a, a + a * (1 - a), 0.0]
        e1[2] = e1[1] + a * (1 - e1[1])
        e2 = [a * e1[0]]
        e2.append(e2[0] + a * (e1[1] - e2[0]))
        e2.append(e2[1] + a * (e1[2] - e2[1]))
        close(out, e2, 1e-15, "two-pole output")

    @check("the firmware's Envelope test: 4 -> 200 RMS step, Bayes covers half within 10 ms, two-pole in 55-75 ms")
    def _():
        t_b, rest, cv = m.step_test(m.bayes_envelope)
        assert t_b <= 10, f"Bayes took {t_b} ms (firmware test: <= 10 ms)"
        assert rest < 10 and cv < 0.10, f"Bayes rest {rest:.1f} (< 10) and steady CV {cv:.3f} (< 0.10)"
        t_2 = m.step_test(lambda x: m.two_pole(x, 4.0))[0]
        assert 55 <= t_2 <= 75, f"two-pole took {t_2} ms (design: 67 ms)"

    @check("activate: on after confirm_n + 1 samples above onset; off only below release (worksheet 7.4)")
    def _():
        close(m.activate(np.array([0, 5, 5, 5, 0.0]), 4.0, 3.0, 2), [0, 0, 0, 1, 0], 0, "confirm 2")
        close(m.activate(np.array([5, 5, 5, 5, 3.5, 3.5, 2.9]), 4.0, 3.0, 0), [1, 1, 1, 1, 1, 1, 0], 0, "hysteresis")
        close(m.activate(np.array([5, 5, 0, 5, 5, 5, 5.0]), 4.0, 3.0, 2), [0, 0, 0, 0, 0, 1, 1], 0, "a dip resets the count")

    @check("finding: a 3 ms pop starts nothing with the 20 ms confirm, but a sustained contraction does")
    def _():
        import simdata
        x = np.random.default_rng(5).normal(0, 4.0, 400)
        x[100:103] += simdata.pop(400.0)
        env = m.bayes_envelope(x)
        onset, release = m.thresholds(env[50:100], 400.0)
        assert not m.activate(env, onset, release, 20).any(), "the pop activated the channel"
        y = np.concatenate([x[:100], np.random.default_rng(6).normal(0, 150.0, 200)])
        on = np.flatnonzero(m.activate(m.bayes_envelope(y), onset, release, 20))
        assert len(on) and 120 <= on[0] <= 135, f"the contraction should switch on 20-35 ms after it starts, got {on[:1] - 100}"


MODULES = {"1": module1, "2": module2, "3": module3, "4": module4, "5": module5, "6": module6, "7": module7}


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    which = args[0] if args else "all"
    keys = list(MODULES) if which == "all" else [which]
    if any(k not in MODULES for k in keys):
        print(__doc__)
        sys.exit(1)
    if USE_SOLUTIONS:
        print("(checking the REFERENCE SOLUTIONS, not your code)")
    for k in keys:
        print(f"\nModule {k}")
        try:
            MODULES[k]()
        except Exception:
            print("  FAIL  the module file doesn't load")
            print("        " + traceback.format_exc().strip().replace("\n", "\n        "))
            RESULTS.append("FAIL")
            continue
        run_checks()
    n = {s: RESULTS.count(s) for s in ("PASS", "FAIL", "TODO")}
    print(f"\n{n['PASS']} passed, {n['FAIL']} failed, {n['TODO']} not written yet")
    sys.exit(1 if n["FAIL"] else 0)


if __name__ == "__main__":
    main()
