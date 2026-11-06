"""Module 5: electrode polarity.

  1. Do WORKSHEET.md, Module 5, on paper first.
  2. Fill in the TODOs below (about 7 lines in total).
  3. Run:  python check.py 5      then:  python run.py polarity
"""
import numpy as np

import m1_features
import simdata


def skewness(x):
    """Skewness of each column of x (n_samples, n_channels):  E[(x - mu)^3] / sigma^3.
    Odd: skewness(-x) = -skewness(x). Scale-free: skewness(g x) = skewness(x) for g > 0."""
    # TODO (2 lines)
    raise NotImplementedError


def rms(x):
    """Root mean square of each column. Even: rms(-x) = rms(x). It scales: rms(g x) = g rms(x)."""
    # TODO (1 line)
    raise NotImplementedError


def beta(profile, ref):
    """Regress one day's per-class skewness profile on the reference profile, with no intercept:
        beta = (s . s_ref) / (s_ref . s_ref)
    profile and ref have shape (n_classes, n_channels); return one beta per channel.
    beta near +1: same polarity as the reference; near -1: reversed.   (Worksheet 5.2)"""
    # TODO (1 line): sum over the class axis (axis 0)
    raise NotImplementedError


def call(b, min_beta=0.4):
    """Turn one day's betas (one per channel) into a pattern string like "+-+-".
    "+" for beta > 0, "-" for beta < 0, and "?" (no call) when |beta| < min_beta."""
    # TODO (1 line)
    raise NotImplementedError


def flip_augment(F, rng):
    """Polarity augmentation for the raw-waveform model: give every training window its own random
    lead pattern, each of the 4 leads reversed with probability 1/2, i.e. one of the 16 patterns
    0-15 with equal probability. with_leads_flipped(F, patterns) (provided, below) does the
    re-shuffling; you draw the patterns.   (Worksheet 5.4)"""
    # TODO (2 lines): rng.integers(0, 16, n) draws n patterns
    raise NotImplementedError


# ------------------------------------------------------------------ provided from here down
def skew_profiles(recs, days):
    """(day, class, channel) skewness for every recording."""
    return np.array([[skewness(recs[(d, c)]) for c in range(simdata.N_CLS)] for d in days])


def polarity_audit(recs, days=simdata.ALL_DAYS, ref_days=range(6, 11), min_beta=0.4):
    """scripts/audit_data.py in miniature: each day's profile regressed on the reference days' mean."""
    days = list(days)
    sk = skew_profiles(recs, days)
    ref = sk[[i for i, d in enumerate(days) if d in ref_days]].mean(axis=0)
    betas = np.array([beta(s, ref) for s in sk])
    return {d: (call(b, min_beta), b) for d, b in zip(days, betas)}


# The raw-waveform model. The real project uses a small CNN (emgdec/models.py, EMGNet) on the raw
# windows. This stand-in keeps the two properties that matter here and nothing else:
#   * its kernels span all 4 channels at once, like the CNN's first layer (Conv1d 4 -> 32), so a
#     detector responds to the signs of the channels TOGETHER;
#   * a thresholded ReLU after them, so the features are NOT even functions of the signal.
# Background: fixed random kernels with a learned linear classifier on top is the idea behind ROCKET
# ("random convolutional kernels"). Nothing here is trained except the LDA.
#
# Every base kernel is used in all 16 channel-sign variants (variant v negates channel ch when bit ch
# of v is set). Reversing leads then only re-shuffles columns: reversing the lead set p turns the
# feature of variant v into the feature of variant v XOR p. That makes augmentation cheap and exact.
N_BASE, KERNEL_LEN, N_PATTERNS = 16, 9, 16


def kernels(seed=0, n_base=N_BASE):
    k = simdata.rng_for(seed, "kernels").normal(0.0, 1.0, (n_base, simdata.N_CH, KERNEL_LEN))
    return k / np.linalg.norm(k, axis=(1, 2), keepdims=True)


def pattern_signs(v):
    """Lead pattern v (0-15) as 4 signs: bit ch set = channel ch reversed. 0 = '++++', 3 = '--++'."""
    return np.array([-1.0 if (v >> ch) & 1 else 1.0 for ch in range(simdata.N_CH)])


def raw_features(recs, cal_days=(1, 2, 3, 4, 5), seed=0, n_base=N_BASE):
    """Features of every window, in the same row order as m1_features.feature_table.

    For base kernel b and sign variant v: u = sum over channels of sign_v[ch] * (kernel_b[ch] * x[:, ch]),
    then the window mean of ReLU(u - t_b), logged. t_b is the kernel's threshold, the RMS of u on the
    calibration days averaged over the 16 variants. Returns F with shape (n_windows, n_base, 16).
    """
    ks = kernels(seed, n_base)
    signs = np.array([pattern_signs(v) for v in range(N_PATTERNS)])          # (16, ch)
    conv = {key: np.stack([[np.convolve(x[:, ch], k[ch], "valid") for ch in range(x.shape[1])] for k in ks])
            for key, x in recs.items()}                                       # (base, ch, n - 8)
    cal = np.concatenate([c for (d, _), c in conv.items() if d in cal_days], axis=2)
    thr = np.sqrt(np.sum(cal.var(axis=2), axis=1))                            # (base,): same for every variant
    n_u = m1_features.WIN - KERNEL_LEN + 1                                    # kernel positions in a window
    out = []
    for key, x in sorted(recs.items()):
        _, starts = m1_features.windows(x)
        u = np.einsum("vc,bcn->vbn", signs, conv[key])                       # (16, base, n - 8)
        c = np.cumsum(np.maximum(u - thr[None, :, None], 0.0), axis=2)
        c = np.concatenate([np.zeros(c.shape[:2] + (1,)), c], axis=2)
        mean = (c[:, :, starts + n_u] - c[:, :, starts]) / n_u               # (16, base, n_windows)
        out.append(np.log(mean + 1e-3).transpose(2, 1, 0))
    return np.concatenate(out)


def with_leads_flipped(F, patterns):
    """The features each window would have had with lead pattern patterns[i] reversed on top of
    whatever it was recorded with: column v moves to column v XOR pattern."""
    idx = np.arange(N_PATTERNS)[None, :] ^ np.asarray(patterns)[:, None]       # (n, 16)
    return np.take_along_axis(F, idx[:, None, :], axis=2)


def flat(F):
    """(n, base, 16) -> (n, base * 16), for the LDA."""
    return F.reshape(len(F), -1)
