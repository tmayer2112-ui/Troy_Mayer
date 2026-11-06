"""Module 7: the Bayesian envelope, latency and the onset confirm. REFERENCE SOLUTION.

Read one function at a time (the stuck rule), close it, then type it yourself.
Signals here are EMG in ADC counts at the firmware's 1 kHz, already centred on 0.
"""
import math

import numpy as np

import simdata

FS = 1000
ALPHA, BETA = 1e-4, 1e-12      # the firmware's bayes_alpha and bayes_beta (orthosis_core.h, Config)


def bayes_step(p, e, levels, alpha=ALPHA, beta=BETA):
    """One sample of the Bayesian amplitude estimator (Sanger 2007; orthosis_core.h, BayesEnvelope).

    p: probability of each amplitude level (sums to 1). e: this EMG sample. levels: the RMS levels.
      1. predict, drift: each level keeps 1 - 2 alpha of its probability and takes alpha from each
         neighbour. At the two ends the missing neighbour counts as the level itself.
      2. predict, jump:  q = (1 - beta) q + beta / n_levels
      3. update: multiply by the Laplace likelihood of e at each level s:  exp(-sqrt(2) |e| / s) / s
      4. normalise so q sums to 1 (if everything underflowed to 0, fall back to uniform)
      5. estimate = exp( sum of q * log(levels) ): the posterior mean of the log-amplitude
    Returns (q, estimate).   (Worksheet 7.1-7.2)
    """
    # TODO (about 9 lines). np.concatenate([p[:1], p[:-1]]) is "the left neighbour of every level".
    # --- solution ---
    left = np.concatenate([p[:1], p[:-1]])
    right = np.concatenate([p[1:], p[-1:]])
    q = (1 - 2 * alpha) * p + alpha * (left + right)
    q = (1 - beta) * q + beta / len(p)
    q = q * np.exp(-math.sqrt(2) * abs(e) / levels) / levels
    total = q.sum()
    q = q / total if total > 0 else np.full(len(p), 1.0 / len(p))
    return q, float(np.exp(np.sum(q * np.log(levels))))
    # --- end ---


def two_pole(x, fc=4.0, fs=FS):
    """The 'before' envelope: rectify, then two equal one-pole low-passes at fc (orthosis_core.h,
    OnePole; analysis/latency_study.py, env_lp2). Each pole: y += a (input - y), a = 1 - e^(-2 pi fc/fs).
    Returns the second pole's output for every sample, starting from 0.   (Worksheet 7.3)"""
    # TODO (6 lines): a loop over the samples
    # --- solution ---
    a = 1 - math.exp(-2 * math.pi * fc / fs)
    e1 = e2 = 0.0
    out = np.empty(len(x))
    for n, v in enumerate(x):
        e1 += a * (abs(v) - e1)
        e2 += a * (e1 - e2)
        out[n] = e2
    return out
    # --- end ---


def activate(env, onset, release, confirm_n):
    """Onset detection with hysteresis and a confirm window (orthosis_core.h, EmgChannel).

    Count consecutive samples with env > onset (reset to 0 on any sample that isn't).
    Off -> on when that count exceeds confirm_n. On -> off as soon as env < release.
    Returns a boolean array: is the channel active at each sample?   (Worksheet 7.4)
    """
    # TODO (8 lines)
    # --- solution ---
    active = np.zeros(len(env), bool)
    on, above = False, 0
    for n, v in enumerate(env):
        above = above + 1 if v > onset else 0
        on = (v >= release) if on else (above > confirm_n)
        active[n] = on
    return active
    # --- end ---


# ------------------------------------------------------------------ provided from here down
def levels(lo=1.0, hi=2000.0, n=64):
    """The 64 log-spaced RMS levels the firmware uses: 1 to 2000 ADC counts, each 12.8 % above the last."""
    return np.exp(np.linspace(math.log(lo), math.log(hi), n))


def bayes_envelope(x, alpha=ALPHA, beta=BETA):
    lv = levels()
    p = np.full(len(lv), 1.0 / len(lv))
    out = np.empty(len(x))
    for n, v in enumerate(x):
        p, out[n] = bayes_step(p, v, lv, alpha, beta)
    return out


ENVELOPES = {
    "two-pole 4 Hz (before)": lambda x: two_pole(x, 4.0),
    "Bayesian (shipped)": bayes_envelope,
}


def thresholds(rest_env, mvc):
    """Onset and release from a rest calibration, as the firmware sets them (EmgChannel.setThresholds):
    onset = max(rest mean + 6 rest SD, rest mean + 8 % of (MVC - rest mean)); release at 70 %."""
    m, sd = float(np.mean(rest_env)), float(np.std(rest_env))
    onset = max(m + 6 * sd, m + 0.08 * (mvc - m))
    return onset, m + 0.7 * (onset - m)


def to_speed(env, active, onset, mvc, max_dps=25.0, deadzone=0.08):
    """Activation (0 at onset, 1 at 60 % of MVC) -> speed command, one muscle only (intentToSpeed)."""
    act = np.where(active, np.clip((env - onset) / max(0.6 * mvc - onset, 1.0), 0.0, 1.0), 0.0)
    return np.clip((act - deadzone) / (1 - deadzone), 0.0, None) * max_dps


def latency_trial(envelope, trial, confirm_ms):
    """One synthetic contraction through an envelope and the firmware's mapping, measured the way
    analysis/latency_study.py measures it (ms): t_on (first non-zero command after the muscle starts),
    t_50 (command at half its steady value), t_off (command back to 0 after the muscle stops),
    and jitter (SD of the command during the steady part, deg/s)."""
    x, start, end = simdata.contraction_1khz(trial)
    env = envelope(x)
    rest, steady = env[500:start], env[start + 1000:end]
    mvc = 2.0 * steady.mean()                       # the trials sit at about 50 % of MVC
    onset, release = thresholds(rest, mvc)
    cmd = to_speed(env, activate(env, onset, release, confirm_ms), onset, mvc)
    on = np.flatnonzero(cmd[start:end] > 0)
    s_cmd = cmd[start + 1000:end]
    half = np.flatnonzero(cmd[start:end] >= 0.5 * s_cmd.mean())
    after = np.flatnonzero(cmd[end:] > 0)
    return {"t_on": float(on[0]) if len(on) else np.nan, "t_50": float(half[0]) if len(half) else np.nan,
            "t_off": float(after[-1] + 1) if len(after) else 0.0, "jitter": float(s_cmd.std())}


def false_starts(envelope, confirm_ms, seconds=60.0):
    """Starts of commanded motion per minute during rest with one electrode pop per second.
    Thresholds come from a clean rest stretch and a contraction's MVC, as in latency_study.py."""
    x, start, end = simdata.contraction_1khz(0)
    env = envelope(x)
    onset, release = thresholds(env[500:start], 2.0 * env[start + 1000:end].mean())
    e = envelope(simdata.rest_with_pops(seconds))
    on = to_speed(e, activate(e, onset, release, confirm_ms), onset, 2.0 * env[start + 1000:end].mean()) > 0
    return float(np.sum(on[1:] & ~on[:-1]) + on[0]) * 60.0 / seconds


def step_test(envelope, fs=FS):
    """The firmware's Envelope test: Gaussian noise steps from 4 to 200 RMS at t = 1 s. Returns the
    ms until the envelope covers half the change, the mean rest level, and the steady level's CV."""
    r = np.random.default_rng(1)
    x = np.concatenate([r.normal(0, 4.0, fs), r.normal(0, 200.0, fs)])
    env = envelope(x)
    rest, steady = env[500:fs].mean(), env[fs + 300:]
    half = rest + 0.5 * (steady.mean() - rest)
    return int(np.argmax(env[fs:] >= half)), float(rest), float(steady.std() / steady.mean())
