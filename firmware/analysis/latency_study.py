"""How fast can the EMG front end turn a contraction into a speed command?

Every trial is one real forearm contraction (exported by export_emg.py) spliced
between stretches of rest, through the same activation mapping the firmware
uses. For each envelope design this measures:

  t_on    contraction start -> first non-zero speed command
  t_50    contraction start -> command reaches half its steady value
  t_off   contraction end   -> command back to zero
  jitter  standard deviation of the command during the steady contraction (deg/s)
  false   commanded motion during 30 s of rest (s per minute)

Rest is 3 % of a real contraction (resting tone, EMG-shaped spectrum) plus the
board's electronic noise, about 2.3 ADC counts RMS (INA333 plus ADC at v2 gain).

    python analysis/latency_study.py        # writes results/latency_study.json
"""
import json
from pathlib import Path

import numpy as np
from numba import njit

HERE = Path(__file__).resolve().parent
FS = 1000
MAX_DPS = 25.0
REST_S, TAIL_S = 2.0, 1.5
MVC_OVER_STEADY = 2.0          # trials sit at ~50 % MVC: mid-range, where jitter shows


# --------------------------------------------------------------------------- envelopes
@njit(cache=True)
def env_lp2(x, fc):
    """Firmware's current design: DC track, rectify, two equal one-pole low passes."""
    a_dc = 1 - np.exp(-2 * np.pi * 0.5 / FS)
    a = 1 - np.exp(-2 * np.pi * fc / FS)
    dc, e1, e2 = x[0], 0.0, 0.0
    out = np.empty(len(x))
    for n in range(len(x)):
        dc += a_dc * (x[n] - dc)
        e1 += a * (abs(x[n] - dc) - e1)
        e2 += a * (e1 - e2)
        out[n] = e2
    return out


@njit(cache=True)
def env_bayes(x, alpha, beta, laplace=False, nbins=64, lo=1.0, hi=2000.0, mean_out=False):
    """Sanger (2007) Bayesian envelope: a posterior over EMG amplitude, updated
    every sample. Amplitude drifts slowly (diffusion alpha, per sample, in log
    space) or jumps (probability beta). A contraction onset is a jump, so the
    estimate moves in a few samples; in a steady contraction the diffusion prior
    averages over many samples. Output is the MAP amplitude (RMS, ADC counts)."""
    a_dc = 1 - np.exp(-2 * np.pi * 0.5 / FS)
    sig = np.exp(np.linspace(np.log(lo), np.log(hi), nbins))
    inv2s2 = 0.5 / sig ** 2
    p = np.full(nbins, 1.0 / nbins)
    q = np.empty(nbins)
    dc = x[0]
    out = np.empty(len(x))
    for n in range(len(x)):
        dc += a_dc * (x[n] - dc)
        e = x[n] - dc
        # predict: diffusion to the neighbouring bins, plus a small jump anywhere
        for i in range(nbins):
            left = p[i - 1] if i > 0 else p[i]
            right = p[i + 1] if i < nbins - 1 else p[i]
            q[i] = (1 - 2 * alpha) * p[i] + alpha * (left + right)
        s = 0.0
        for i in range(nbins):
            q[i] = (1 - beta) * q[i] + beta / nbins
            if laplace:   # heavier tails: one big sample moves the estimate less
                q[i] *= np.exp(-abs(e) * 1.41421356 / sig[i]) / sig[i]
            else:
                q[i] *= np.exp(-e * e * inv2s2[i]) / sig[i]
            s += q[i]
        best, bi, lm = -1.0, 0, 0.0
        for i in range(nbins):
            p[i] = q[i] / s
            lm += p[i] * np.log(sig[i])
            if p[i] > best:
                best, bi = p[i], i
        out[n] = np.exp(lm) if mean_out else sig[bi]
    return out


# --------------------------------------------------------------------------- mapping
def to_command(env, rest_env, mvc, confirm_ms=0):
    """The firmware's activation mapping (orthosis_core.h EmgChannel + intentToSpeed).
    confirm_ms: the envelope must stay above onset this long before it counts."""
    rest_mean, rest_sd = rest_env.mean(), rest_env.std()
    onset = max(rest_mean + 6 * rest_sd, rest_mean + 0.08 * (mvc - rest_mean))
    release = rest_mean + 0.7 * (onset - rest_mean)
    span = max(0.6 * mvc - onset, 1.0)
    act = np.zeros(len(env))
    on, above = False, 0
    for n, e in enumerate(env):
        above = above + 1 if e > onset else 0
        on = e > release if on else above > confirm_ms
        act[n] = min(max((e - onset) / span, 0.0), 1.0) if on else 0.0
    mag = np.clip((act - 0.08) / 0.92, 0, None)
    return mag * MAX_DPS


def make_rest(rng, contraction, seconds, spikes_per_s=0.0):
    n = int(seconds * FS)
    base = contraction - contraction.mean()
    reps = np.resize(base, n)
    x = 0.03 * reps + rng.normal(0, 2.3, n)
    # motion artefacts / electrode pops: 3 ms biphasic spikes, 100-400 counts
    for k in np.flatnonzero(rng.random(n) < spikes_per_s / FS):
        a = rng.uniform(100, 400) * rng.choice([-1, 1])
        x[k:k + 3] += a * np.array([1.0, -1.0, 0.5])[: len(x[k:k + 3])]
    return x


ONSET_RAMP_MS = 50     # voluntary contractions ramp up; a step would flatter every filter


def events(cmd):
    on = cmd > 0
    return int(np.sum(on[1:] & ~on[:-1]) + on[0])


def evaluate(envelope, contractions, rng, confirm_ms=0):
    stats = {k: [] for k in ("t_on", "t_50", "t_off", "jitter")}
    for c in contractions:
        c = c.astype(np.float64)
        body = c - c.mean()
        body[:ONSET_RAMP_MS] *= np.linspace(0, 1, ONSET_RAMP_MS)
        rest = make_rest(rng, c, REST_S + TAIL_S)
        n_rest = int(REST_S * FS)
        x = 2180 + np.concatenate([rest[:n_rest], body, rest[n_rest:]])
        env = envelope(x)
        cal_rest = env[500:n_rest]
        steady = env[n_rest + 1000:n_rest + len(body)]
        cmd = to_command(env, cal_rest, MVC_OVER_STEADY * steady.mean(), confirm_ms)
        start, end = n_rest, n_rest + len(body)
        on = np.flatnonzero(cmd[start:end] > 0)
        if not len(on):
            continue
        s_cmd = cmd[start + 1000:end]
        half = np.flatnonzero(cmd[start:end] >= 0.5 * s_cmd.mean())
        after = np.flatnonzero(cmd[end:] > 0)          # the command's last moment after the contraction ended
        off = [after[-1] + 1] if len(after) else [0]
        stats["t_on"].append(on[0])
        stats["t_50"].append(half[0] if len(half) else np.nan)
        stats["t_off"].append(off[0] if len(off) else np.nan)
        stats["jitter"].append(s_cmd.std())
    # false starts: 60 s of rest with 1 artefact spike per second, calibrated on
    # a clean rest stretch
    c = contractions[0].astype(np.float64)
    fake_mvc = MVC_OVER_STEADY * np.median([envelope(2180.0 + k - k.mean())[1000:].mean()
                                            for k in contractions[:20]])
    cal = envelope(2180 + make_rest(rng, c, 3.0))[500:]
    dirty = envelope(2180 + np.concatenate([make_rest(rng, c, 3.0), make_rest(rng, c, 60.0, spikes_per_s=1.0)]))
    cmd = to_command(dirty[3000:], cal, fake_mvc, confirm_ms)
    return {"t_on_ms": float(np.nanmedian(stats["t_on"])),
            "t_50_ms": float(np.nanmedian(stats["t_50"])),
            "t_off_ms": float(np.nanmedian(stats["t_off"])),
            "t_50_p90_ms": float(np.nanpercentile(stats["t_50"], 90)),
            "t_off_p90_ms": float(np.nanpercentile(stats["t_off"], 90)),
            "jitter_dps": float(np.median(stats["jitter"])),
            "false_starts_per_min": float(events(cmd)),
            "n": len(stats["t_on"])}


def main():
    z = np.load(HERE / "data" / "contractions.npz")
    keys = sorted(z.files)
    rng = np.random.default_rng(0)
    trials = [z[k] for k in rng.choice(keys, size=200, replace=False)]
    designs = {}
    for fc in (3, 4, 6, 8, 12):
        designs[f"lp2 {fc} Hz"] = (lambda x, fc=fc: env_lp2(x, float(fc)), 0)
    for lap in (False, True):
        for alpha in (1e-4, 1e-3):
            for beta in (1e-12, 1e-8):
                for confirm in (0, 10, 20):
                    name = f"bayes {'laplace' if lap else 'gauss'} a={alpha:g} b={beta:g} confirm {confirm}"
                    designs[name] = (lambda x, a=alpha, b=beta, l=lap: env_bayes(x, a, b, l), confirm)
    # output the posterior (log-)mean instead of the most likely bin: continuous, not stepped
    for alpha in (1e-4, 1e-3):
        for confirm in (15, 20):
            designs[f"bayes laplace a={alpha:g} b=1e-12 MEAN confirm {confirm}"] = (
                lambda x, a=alpha: env_bayes(x, a, 1e-12, True, mean_out=True), confirm)
    res = {}
    for name, (f, confirm) in designs.items():
        r = evaluate(f, trials, np.random.default_rng(1), confirm)
        res[name] = r
        print(f"{name:42s} t_on {r['t_on_ms']:5.0f}  t_50 {r['t_50_ms']:5.0f} (p90 {r['t_50_p90_ms']:4.0f})  "
              f"t_off {r['t_off_ms']:5.0f} (p90 {r['t_off_p90_ms']:4.0f})  jitter {r['jitter_dps']:5.2f} dps  "
              f"false starts {r['false_starts_per_min']:.0f}/min", flush=True)
    out = HERE.parent / "results"
    out.mkdir(exist_ok=True)
    (out / "latency_study.json").write_text(json.dumps(res, indent=1))


if __name__ == "__main__":
    main()
