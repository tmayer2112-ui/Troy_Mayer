"""Run the experiments with your code (provided).

    python run.py features     Module 1: what the features see
    python run.py filter       Module 2: the band-pass, analog and digital
    python run.py boards       Module 3: v1's loading and dead zone, v2's common-mode budget
    python run.py lda          Module 4: the protocols, and LDA through each board (~20 s)
    python run.py polarity     Module 5: the audit, and LDA vs a raw-waveform model (~40 s)
    python run.py loop         Module 6: the speed loop on the toy arm
    python run.py latency      Module 7: Bayesian vs two-pole envelope, the onset confirm (~10 s)
    python run.py report       everything, then synthetic vs real side by side (~1.5 min)
Options:
    --solutions    run the reference solutions instead of your code
    --plot         also save report.png (needs matplotlib; skipped if it isn't installed)

Every number this prints from the rebuild is SYNTHETIC. The "real" column comes from the project's own
results files, read at run time, and names the file and key.
"""
import json
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
if "--solutions" in sys.argv:
    sys.path.insert(0, str(HERE / "solutions"))     # ahead of this folder, so every module comes from there
if str(HERE) not in sys.path:
    sys.path.append(str(HERE))                       # appended, never inserted: it must not shadow solutions/

import simdata  # noqa: E402

DECODER = HERE.parent
FIRMWARE = HERE.parent.parent / "firmware"
PLOTS = {}


class Unwritten(Exception):
    pass


def mod(name):
    import importlib
    return importlib.import_module(name)


def real(path, key, scale=1.0, fmt="{:.1f}"):
    """A number from the real project's results, by file and '/'-separated key."""
    try:
        v = json.loads(Path(path).read_text())
        for k in key.split("/"):
            v = v[int(k)] if isinstance(v, list) else v[k]
        return fmt.format(v * scale)
    except (OSError, KeyError, ValueError, IndexError):
        return "(file not found)"


def pct(v):
    return f"{100 * float(np.mean(v)):.1f} %"


_DATA = {}


def data(flips=True):
    if flips not in _DATA:
        _DATA[flips] = simdata.make_dataset(flips=flips)
    return _DATA[flips]


# ---------------------------------------------------------------------------------- Module 1
def cmd_features():
    F = mod("m1_features")
    x = data()[(6, 2)]
    w, starts = F.windows(x)
    thr = F.dead_band([data()[(d, c)] for d in range(1, 6) for c in range(11)])
    X = F.hudgins(w, thr)
    d = x - x.mean(0)
    print(f"One synthetic hold (day 6, class 2, {len(x)} samples): {len(starts)} windows of {F.WIN}, every {F.STEP}")
    print(f"  RMS per channel        {np.round(np.sqrt(np.mean(x ** 2, 0)), 1)} uV")
    print(f"  kurtosis (Gaussian 3)  {np.round(np.mean(d ** 4, 0) / np.mean(d ** 2, 0) ** 2, 2)}   heavier-tailed")
    print(f"  dead band (5 % of RMS) {np.round(thr, 2)} uV")
    print("  first window, 16 features:")
    for name, row in zip(("logMAV", "logWL", "ZC", "SSC"), X[0].reshape(4, 4)):
        print(f"    {name:7s} {np.round(row, 2)}")
    flipped = F.hudgins(-w, thr)
    print(f"  max change if every lead is reversed: {np.abs(flipped - X).max():.1e}  (all four are even functions)")
    g = F.hudgins(1.5 * w, 1.5 * thr) - X
    print(f"  gain x 1.5: logMAV, logWL move by {g[:, :8].mean():.4f} (ln 1.5 = {np.log(1.5):.4f}); "
          f"ZC, SSC by {np.abs(g[:, 8:]).max():.0f}")
    print(f"  ZC in the first window without the dead band: {F.zc(w[:1], 0.0)[0]}, with it: {F.zc(w[:1], thr)[0]}")


# ---------------------------------------------------------------------------------- Module 2
def cmd_filter():
    M = mod("m2_filter")
    (f0h, Q, _), (f0l, _, _) = M.stages()
    f = np.logspace(0, np.log10(1024 * 0.999), 4000)
    ha, hd = M.cascade_analog(f), M.cascade_digital(f, 2048)
    om = M.hpf_corner_ratio(Q)
    b = DECODER / "results" / "board.json"
    print(f"Sallen-Key, K = {M.K_SK:.4f}:   Q = {Q:.4f}   HPF f0 = {f0h:.2f} Hz   LPF f0 = {f0l:.1f} Hz   "
          f"|H(j w0)| = K Q = {M.K_SK * Q:.4f}")
    print(f"One stage alone: -3 dB at Omega = {om:.4f}: HPF {f0h * om:.2f} Hz, LPF {f0l / om:.2f} Hz")
    lo, hi = M.corners(f, ha)
    print(f"Cascade, analog:     peak x {np.abs(ha).max():.3f}, corners {lo:.1f} and {hi:.1f} Hz   "
          f"(real: x {real(b, 'cascade_passband_gain', fmt='{:.3f}')}, {real(b, 'cascade_minus3db_hz_analog/0')} "
          f"and {real(b, 'cascade_minus3db_hz_analog/1')} Hz, board.json)")
    lo, hi = M.corners(f, hd)
    err = np.max(np.abs(20 * np.log10(np.abs(hd[f < 400]) / np.abs(ha[f < 400]))))
    print(f"Cascade, 2048 Hz:    corners {lo:.1f} and {hi:.1f} Hz, max error below 400 Hz {err:.3f} dB   "
          f"(real: {real(b, 'cascade_minus3db_hz_digital_fs2048/1')} Hz, "
          f"{real(b, 'max_digital_vs_analog_error_db_below_400hz', fmt='{:.3f}')} dB)")
    for f0, name in ((f0h, "HPF"), (f0l, "LPF")):
        print(f"Pre-warp, {name}: w0 = {2 * np.pi * f0:.0f} rad/s -> {2 * 2048 * np.tan(np.pi * f0 / 2048):.0f} rad/s")
    for fx in (60, 500, 1000):
        print(f"  gain at {fx:4d} Hz: analog {abs(M.cascade_analog(np.array([fx]))[0]):.3f}")
    PLOTS["filter"] = (f, ha, hd)


# ---------------------------------------------------------------------------------- Module 3
def cmd_boards():
    M = mod("m3_frontend")
    print("v1 difference amp, electrodes 25 % mismatched (real: board.json, v1_front_end_vs_electrode_impedance)")
    print("   Z        gain     CMRR    input to clear the diode (max pot)")
    for z in (0.0, 5e3, 20e3, 50e3):
        g, _ = M.diff_amp_gains(*M.electrodes(z))
        print(f"  {z / 1e3:4.0f}k   {g:6.2f}   {M.db(M.v1_cm_to_diff(*M.electrodes(z))):5.1f} dB   "
              f"{1e6 * M.input_to_clear_diode(g):7.1f} uV")
    r = M.v2_cm_to_diff(5e3)
    print(f"v2: INA333 100 dB + 5k imbalance vs {M.Z_CM / 1e6:.0f} MOhm ({M.db(5e3 / M.Z_CM):.1f} dB) -> "
          f"{M.db(r / 10 ** (-20 / 20)):.1f} dB, with the right-leg drive {M.db(r):.1f} dB: 1 V of hum -> {1e6 * r:.2f} uV")
    recs = {k: v for k, v in data().items() if k[0] <= 10}
    keys = sorted(recs)
    x = np.stack([recs[k] for k in keys], axis=-1) * 1e-6
    f = M.v1_filtered(x, 2048)
    pot = M.calibrate_pot(np.moveaxis(f, -1, 0).reshape(-1, 4), 47e3, 1.6)
    y = np.clip(f * (1 + (47e3 + pot[:, None]) / 1e3), -3.25, 1.8)
    print(f"Synthetic EMG through v1 (days 1-10): pots {np.round(pot / 1e3)} kOhm, "
          f"{100 * np.mean(y > 0.4):.2f} % of samples clear the 0.4 V diode (real: 0.22 %, decoder README)")
    out = M.v1_output(f[:, :, :1], pot[:, None], 2048, np.random.default_rng(0))
    print(f"  the v1 output is never negative (min {out.min():.4f} V), so zero crossings are always 0")


# ---------------------------------------------------------------------------------- Module 4
def cmd_lda():
    F, L, M3 = mod("m1_features"), mod("m4_lda"), mod("m3_frontend")
    lda = DECODER / "results" / "lda.json"
    T = F.hudgins_table(data())
    acc, shared = L.random_split(T)
    cd = L.cross_day(T)
    wd = L.within_day(T)
    parts = L.cross_day_parts(T)
    print(f"Synthetic, no board, 11 classes, chance 9.1 %  |  real v2 (lda.json)")
    print(f"  shuffled windows (leaky)  {pct(acc):>7s}  ({100 * shared:.2f} % of test windows share samples with "
          f"training)  |  {real(lda, 'main/v2@0.0/random_split/bal_acc', 100)} %")
    print(f"  cross-day                 {L.mean_sd(cd):>16s}  |  {real(lda, 'main/v2@0.0/cross_day/per_day/mean', 100)} %")
    print("  on identical windows, the last 40 % of each test hold:")
    print(f"    within-day (same day)   {L.mean_sd(wd):>16s}  |  {real(lda, 'main/v2@0.0/within_day/test_days/mean', 100)} %")
    print(f"    cross-day               {L.mean_sd(parts['late 40 %']):>16s}  |  "
          f"{real(lda, 'main/v2@0.0/cross_day/late_40pct/mean', 100)} %")
    print(f"  cross-day, first 60 %     {L.mean_sd(parts['early 60 %']):>16s}  |  "
          f"{real(lda, 'main/v2@0.0/cross_day/early_60pct/mean', 100)} %")
    print("Through each board (cross-day, Ledoit-Wolf shrinkage as the real decoder uses):")
    chains, pots = M3.run_chains(data())
    series = {}
    for name, recs in chains.items():
        Tc = F.hudgins_table(recs)
        tr = np.isin(Tc.day, L.TRAIN_DAYS)
        s = L.cross_day(Tc, shrinkage=L.ledoit_wolf(Tc.X[tr], Tc.y[tr]))
        series[name] = s
        key = {"no board": "reference", "v1 envelope": "v1_envelope", "v2": "v2"}[name]
        print(f"  {name:12s} {L.mean_sd(s):>16s}, best day {100 * max(s):.1f} %  |  "
              f"{real(lda, f'main/{key}@0.0/cross_day/per_day/mean', 100)} %")
    PLOTS["lda"] = series


# ---------------------------------------------------------------------------------- Module 5
def cmd_polarity():
    F, L, P = mod("m1_features"), mod("m4_lda"), mod("m5_polarity")
    aud = P.polarity_audit(data())
    segs = []
    for d, (c, _) in aud.items():
        if segs and segs[-1][2] == c:
            segs[-1][1] = d
        else:
            segs.append([d, d, c])
    print("Polarity audit (synthetic; reference days 6-10): " +
          ", ".join(f"days {a}-{b} {c}" for a, b, c in segs) +
          f";  median |beta| {np.median([np.abs(b) for _, b in aud.values()]):.2f}")
    T = F.hudgins_table(data())
    tr = np.isin(T.day, L.TRAIN_DAYS)

    def score(pred):
        return [L.balanced_accuracy(T.y[T.day == d], pred(T.day == d)) for d in simdata.TEST_DAYS]

    rows = {}
    m = L.lda_fit(T.X[tr], T.y[tr])
    rows["LDA, Hudgins features"] = score(lambda s: L.lda_predict(m, T.X[s]))
    t0 = time.time()
    Fr = P.raw_features(data())
    m = L.lda_fit(P.flat(Fr[tr]), T.y[tr])
    rows["raw-waveform model"] = score(lambda s: L.lda_predict(m, P.flat(Fr[s])))
    aug = []
    for seed in range(3):
        m = L.lda_fit(P.flat(P.flip_augment(Fr[tr], np.random.default_rng(seed))), T.y[tr])
        aug.append(score(lambda s: L.lda_predict(m, P.flat(Fr[s]))))
    rows["raw + polarity augmentation (3 seeds)"] = list(np.mean(aug, axis=0))
    seen, unseen = slice(0, 5), slice(5, 10)
    print("Days 11-15 have a lead pattern seen in training, days 16-20 don't (synthetic):")
    for name, r in rows.items():
        print(f"  {name:40s} seen {pct(r[seen])}   unseen {pct(r[unseen])}")
    print("Real (decoder README, E3): LDA 88.6 / 87.2, CNN 91.5 / 65.5, CNN + polarity augmentation 89.0 / 86.0")
    print("How the augmented model's cost depends on its number of base kernels (synthetic):")
    for nb in (4, 8, 16):
        Fb = Fr if nb == P.N_BASE else P.raw_features(data(), n_base=nb)
        m = L.lda_fit(P.flat(P.flip_augment(Fb[tr], np.random.default_rng(0))), T.y[tr])
        r = score(lambda s: L.lda_predict(m, P.flat(Fb[s])))
        print(f"  {nb:2d} kernels x 16 sign variants: seen {pct(r[seen])}   unseen {pct(r[unseen])}")
    print(f"  ({time.time() - t0:.0f} s for the raw-model part)")
    PLOTS["polarity"] = rows


# ---------------------------------------------------------------------------------- Module 6
def cmd_loop():
    M = mod("m6_speed_loop")
    met = FIRMWARE / "results" / "metrics.json"
    kp, ki = M.imc_gains(M.K_MODEL, M.TAU_MODEL, M.LAMBDA)
    print(f"IMC gains, K {M.K_MODEL:.0f}, tau {1e3 * M.TAU_MODEL:.0f} ms, lambda {1e3 * M.LAMBDA:.0f} ms: "
          f"Kp {kp:.5f} duty per deg/s, Ki {ki:.4f} duty per deg   "
          f"(real: {real(met, 'kp_duty_per_dps', fmt='{:.5f}')}, {real(met, 'ki_duty_per_deg', fmt='{:.4f}')}, metrics.json)")
    kp, ki = M.imc_gains(M.K_MODEL, M.TAU_MODEL, 0.030)
    print(f"  fast tier, lambda 30 ms, same model: Kp {kp:.5f}, Ki {ki:.4f}")
    (lf, ef), (lc, ec) = M.lift(False), M.lift(True)
    print(f"Toy arm, lifting at 15 deg/s (synthetic):  feed-forward only {ef:.2f} deg/s error ({100 * ef / 15:.0f} %), "
          f"closed loop {ec:.3f} deg/s ({100 * ec / 15:.2f} %)")
    print(f"  real (firmware sim): {real(met, 'lift_15dps_steady_error_ff_only_dps', fmt='{:.2f}')} and "
          f"{real(met, 'lift_15dps_steady_error_closed_dps', fmt='{:.3f}')} deg/s (metrics.json); "
          f"the test prints 98.3 % and 0.8 %")
    (hf, df), (hc, dc) = M.hold(False), M.hold(True)
    print(f"Holding at 40 deg, zero command, 5 s:  feed-forward only drifts {df:.1f} deg, closed loop {abs(dc):.4f} deg "
          f"(real: {real(met, 'hold_drift_5s_deg', fmt='{:.3f}')} deg)")
    _, t63 = M.pi_only_step()
    print(f"PI alone on an exact model: 63 % of the step at {1e3 * t63:.0f} ms (lambda = {1e3 * M.LAMBDA:.0f} ms)")
    for aw in (True, False):
        _, peak, integ = M.windup(aw)
        print(f"Joint held for 0.5 s at a 20 deg/s command, anti-windup {'on ' if aw else 'off'}: "
              f"integral {integ:.2f} duty at release, peak {peak:.0f} deg/s after it")
    PLOTS["loop"] = (lf, lc)


# ---------------------------------------------------------------------------------- Module 7
def cmd_latency():
    M = mod("m7_bayes")
    lat = FIRMWARE / "results" / "latency_study.json"
    print("Step test, Gaussian noise 4 -> 200 RMS (as in the firmware's Envelope test):")
    steps = {}
    for name, env in M.ENVELOPES.items():
        t, rest, cv = M.step_test(env)
        steps[name] = env
        print(f"  {name:24s} half the step after {t:3d} ms, steady CV {cv:.3f}")
    print("20 synthetic contractions (medians, ms; jitter in deg/s) and false starts with 1 pop per second:")
    rows = (("two-pole 4 Hz (before)", 0, "lp2 4 Hz"),
            ("Bayesian (shipped)", 0, "bayes laplace a=0.0001 b=1e-12 confirm 0"),
            ("Bayesian (shipped)", 20, "bayes laplace a=0.0001 b=1e-12 MEAN confirm 20"))
    for name, conf, key in rows:
        rs = [M.latency_trial(M.ENVELOPES[name], k, conf) for k in range(20)]
        med = {k: float(np.nanmedian([r[k] for r in rs])) for k in rs[0]}
        p90 = float(np.nanpercentile([r["t_off"] for r in rs], 90))
        fs = M.false_starts(M.ENVELOPES[name], conf)
        print(f"  {name:24s} confirm {conf:2d} ms: t_on {med['t_on']:5.1f}  t_50 {med['t_50']:5.1f}  "
              f"stop lag p90 {p90:5.1f}  jitter {med['jitter']:.2f}  false starts {fs:.0f}/min")
        print(f"    real ({key}): t_on {real(lat, key + '/t_on_ms')}  t_50 {real(lat, key + '/t_50_ms')}  "
              f"stop lag p90 {real(lat, key + '/t_off_p90_ms')}  jitter {real(lat, key + '/jitter_dps', fmt='{:.2f}')}  "
              f"false starts {real(lat, key + '/false_starts_per_min', fmt='{:.0f}')}/min")
    print("(the real 'confirm 0' row is the MAP-output variant: the shipped posterior-mean version was only run with "
          "15 and 20 ms confirms)")
    PLOTS["latency"] = steps


# ---------------------------------------------------------------------------------- report
STEPS = [("features", cmd_features, 1), ("filter", cmd_filter, 2), ("boards", cmd_boards, 3),
         ("lda", cmd_lda, 4), ("polarity", cmd_polarity, 5), ("loop", cmd_loop, 6), ("latency", cmd_latency, 7)]


def run_step(name, fn, n):
    print(f"\n=== {name} (Module {n}) " + "=" * (60 - len(name)))
    t0 = time.time()
    try:
        fn()
    except NotImplementedError:
        print(f"  needs code you haven't written yet: run `python check.py {n}` (and the modules before it)")
        return
    print(f"  ({time.time() - t0:.0f} s)")


def save_plot():
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib isn't installed: no plot (py -m pip install matplotlib)")
        return
    fig, ax = plt.subplots(2, 3, figsize=(13, 7))
    if "filter" in PLOTS:
        f, ha, hd = PLOTS["filter"]
        ax[0, 0].semilogx(f, 20 * np.log10(np.abs(ha)), lw=2, label="circuit")
        ax[0, 0].semilogx(f, 20 * np.log10(np.abs(hd)), "--", label="2048 Hz biquads")
        ax[0, 0].set(title="Module 2: the band-pass", xlabel="Hz", ylabel="dB", ylim=(-30, 10))
        ax[0, 0].legend()
    if "lda" in PLOTS:
        for name, s in PLOTS["lda"].items():
            ax[0, 1].plot(simdata.TEST_DAYS, 100 * np.array(s), "o-", label=name)
        ax[0, 1].set(title="Module 4: cross-day LDA per test day (synthetic)", xlabel="day", ylabel="%", ylim=(0, 100))
        ax[0, 1].legend()
    if "polarity" in PLOTS:
        for name, s in PLOTS["polarity"].items():
            ax[0, 2].plot(simdata.TEST_DAYS, 100 * np.array(s), "o-", label=name)
        ax[0, 2].axvline(15.5, color="0.5", ls="--")
        ax[0, 2].set(title="Module 5: unseen lead pattern from day 16", xlabel="day", ylabel="%", ylim=(0, 100))
        ax[0, 2].legend(fontsize=7)
    if "loop" in PLOTS:
        lf, lc = PLOTS["loop"]
        ax[1, 0].plot(lf["t"], lf["omega"], label="feed-forward only")
        ax[1, 0].plot(lc["t"], lc["omega"], label="feed-forward + PI")
        ax[1, 0].plot(lc["t"], lc["ref"], "k:", label="setpoint")
        ax[1, 0].set(title="Module 6: lifting at 15 deg/s (toy arm)", xlabel="s", ylabel="deg/s")
        ax[1, 0].legend()
    if "latency" in PLOTS:
        x, start, end = simdata.contraction_1khz(0)
        t = (np.arange(len(x)) - start)
        for name, env in PLOTS["latency"].items():
            ax[1, 1].plot(t, env(x), label=name)
        ax[1, 1].set(title="Module 7: envelope at onset (synthetic)", xlabel="ms after onset", ylabel="counts",
                     xlim=(-100, 300))
        ax[1, 1].legend()
    ax[1, 2].axis("off")
    fig.tight_layout()
    fig.savefig(HERE / "report.png", dpi=120)
    print(f"saved {HERE / 'report.png'}")


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    which = args[0] if args else "report"
    names = [s[0] for s in STEPS]
    if which not in names + ["report"]:
        print(__doc__)
        sys.exit(1)
    if "--solutions" in sys.argv:
        print("(running the REFERENCE SOLUTIONS, not your code)")
    print("All rebuild numbers below are SYNTHETIC. Real numbers name their results file.")
    for name, fn, n in STEPS:
        if which in (name, "report"):
            run_step(name, fn, n)
    if which == "report":
        print_numbers()
    if "--plot" in sys.argv:
        save_plot()


def print_numbers():
    """The real numbers behind the résumé entry, each with the file it comes from."""
    lda, cnn = DECODER / "results" / "lda.json", DECODER / "results" / "cnn.json"
    met = FIRMWARE / "results" / "metrics.json"
    print("\n=== your numbers (REAL, from the project's results files) " + "=" * 20)
    print(f"  cross-day LDA, v2 / no board / v1:  {real(lda, 'main/v2@0.0/cross_day/per_day/mean', 100)} / "
          f"{real(lda, 'main/reference@0.0/cross_day/per_day/mean', 100)} / "
          f"{real(lda, 'main/v1_envelope@0.0/cross_day/per_day/mean', 100)} %   (lda.json, .../cross_day/per_day/mean)")
    print(f"  CNN + polarity augmentation:        {real(cnn, 'v2+flip/test_mean_over_seeds', 100)} %   "
          f"(cnn.json, v2+flip; predates the 0.16 % INA333 gain fix)")
    print(f"  filter corners:                     {real(DECODER / 'results' / 'board.json', 'cascade_minus3db_hz_analog/0')}"
          f"-{real(DECODER / 'results' / 'board.json', 'cascade_minus3db_hz_analog/1')} Hz   (board.json)")
    print(f"  PI gains:                           Kp {real(met, 'kp_duty_per_dps', fmt='{:.5f}')}, "
          f"Ki {real(met, 'ki_duty_per_deg', fmt='{:.4f}')}   (firmware/results/metrics.json)")
    print("  latency 103 -> 43 ms, 0.8 % lift error, 26 tests: firmware/README.md and `make test` in firmware/")


if __name__ == "__main__":
    main()
