"""Module 2: the filter band. REFERENCE SOLUTION.

Read one function at a time (the stuck rule), close it, then type it yourself.
"""
import numpy as np

# Schematic values (orthosis-emg-decoder/emgdec/board.py lines 26-30). Both boards share them.
K_SK = 1 + 10e3 / 18e3          # each stage's gain, set by the 10k / 18k feedback divider
HPF_RC = (15e3, 470e-9)         # high-pass: R = 15 kOhm, C = 470 nF
LPF_RC = (1e3, 330e-9)          # low-pass:  R = 1 kOhm,  C = 330 nF


def sallen_key_params(R, C, K):
    """Equal-component Sallen-Key stage (R1 = R2 = R, C1 = C2 = C) with gain K.
    Returns (f0 in Hz, Q)."""
    # TODO (2 lines): w0 = 1/(RC), so f0 = w0 / (2 pi); Q = 1 / (3 - K)   (Worksheet 2.1)
    # --- solution ---
    f0 = 1.0 / (2 * np.pi * R * C)
    return f0, 1.0 / (3.0 - K)
    # --- end ---


def analog_response(f, f0, Q, K, kind):
    """H(j 2 pi f) of one stage, as a complex array. kind is "high" or "low".

        H_HP(s) = K s^2 / (s^2 + (w0/Q) s + w0^2)        H_LP(s) = K w0^2 / (same denominator)
    """
    # TODO (4 lines): s = j 2 pi f, w0 = 2 pi f0
    # --- solution ---
    s = 2j * np.pi * np.asarray(f, float)
    w0 = 2 * np.pi * f0
    num = K * s ** 2 if kind == "high" else K * w0 ** 2
    return num / (s ** 2 + (w0 / Q) * s + w0 ** 2)
    # --- end ---


def hpf_corner_ratio(Q):
    """One high-pass stage alone is -3 dB at f = Omega * f0. Return Omega.

    Setting |H|^2 = K^2 / 2 gives  Omega^4 + (2 - 1/Q^2) Omega^2 - 1 = 0   (Worksheet 2.3).
    That is a quadratic in u = Omega^2: take the positive root, then the square root.
    """
    # TODO (2 lines)
    # --- solution ---
    b = 2.0 - 1.0 / Q ** 2
    return np.sqrt((-b + np.sqrt(b * b + 4.0)) / 2.0)
    # --- end ---


def bilinear_biquad(f0, Q, K, kind, fs):
    """Discrete version of one stage: the bilinear transform, pre-warped so f0 lands exactly.

    Returns (b, a), each of length 3 with a[0] = 1, for the difference equation
        y[n] = b0 x[n] + b1 x[n-1] + b2 x[n-2] - a1 y[n-1] - a2 y[n-2]

    Steps (Worksheet 2.4): pre-warp w0 = 2 fs tan(pi f0 / fs); substitute s = c (1 - z^-1)/(1 + z^-1)
    with c = 2 fs; multiply top and bottom by (1 + z^-1)^2; divide everything by a0.
    """
    # TODO (about 8 lines). With A = c^2, B = (w0/Q) c, C = w0^2 the denominator becomes
    #   (A + B + C) + (2C - 2A) z^-1 + (A - B + C) z^-2
    # and the numerator is K A (1 - 2 z^-1 + z^-2) for the high-pass, K C (1 + 2 z^-1 + z^-2) for the low-pass.
    # --- solution ---
    w0 = 2 * fs * np.tan(np.pi * f0 / fs)
    c = 2 * fs
    A, B, C = c * c, (w0 / Q) * c, w0 * w0
    a0 = A + B + C
    a = np.array([a0, 2 * C - 2 * A, A - B + C]) / a0
    if kind == "high":
        b = K * A * np.array([1.0, -2.0, 1.0]) / a0
    else:
        b = K * C * np.array([1.0, 2.0, 1.0]) / a0
    return b, a
    # --- end ---


def adc(v, rng, full_scale=3.1, bits=12, enob=10.0):
    """The ESP32-S3 ADC as the project models it (emgdec/board.py, adc). Returns volts.
      1. add Gaussian noise with SD = (full_scale / 2^enob) / sqrt(12): the noise that leaves only
         `enob` effective bits (a uniform error one LSB wide has SD LSB / sqrt(12))
      2. clip to 0 .. full_scale (the pin can't read outside it)
      3. round to the nearest of the 2^bits codes, one LSB = full_scale / 2^bits apart
    (Worksheet 2.5)"""
    # TODO (3 lines). np.shape(v) gives the noise the right shape.
    # --- solution ---
    v = v + rng.normal(0.0, full_scale / 2 ** enob / np.sqrt(12), np.shape(v))
    lsb = full_scale / 2 ** bits
    return np.round(np.clip(v, 0.0, full_scale) / lsb) * lsb
    # --- end ---


def alias_frequency(f, fs):
    """Where a tone at f Hz shows up after sampling at fs: sampling can't tell f from f - k fs, so
    the tone folds to the nearest multiple of fs and lands between 0 and fs/2.   (Worksheet 2.6)"""
    # TODO (1 line): |f - fs * round(f / fs)|
    # --- solution ---
    return abs(f - fs * round(f / fs))
    # --- end ---


# ------------------------------------------------------------------ provided from here down
def stages():
    """The two stages, high-pass first: [(f0, Q, kind), ...] from the schematic values."""
    return [(*sallen_key_params(*HPF_RC, K_SK), "high"), (*sallen_key_params(*LPF_RC, K_SK), "low")]


def cascade_analog(f):
    h = np.ones(len(f), complex)
    for f0, Q, kind in stages():
        h = h * analog_response(f, f0, Q, K_SK, kind)
    return h


def digital_response(b, a, f, fs):
    """H(e^{j w}) of a biquad at frequencies f (Hz)."""
    zi = np.exp(-2j * np.pi * np.asarray(f, float) / fs)        # z^-1
    return (b[0] + b[1] * zi + b[2] * zi ** 2) / (a[0] + a[1] * zi + a[2] * zi ** 2)


def cascade_digital(f, fs):
    h = np.ones(len(f), complex)
    for f0, Q, kind in stages():
        h = h * digital_response(*bilinear_biquad(f0, Q, K_SK, kind, fs), f, fs)
    return h


def corners(f, h):
    """The -3 dB corners measured from the response's own peak, as the real project does
    (scripts/verify_board.py, minus_3db): the lowest and highest f within 3 dB of the peak."""
    mag = np.abs(h) / np.abs(h).max()
    inside = f[mag >= 10 ** (-3 / 20)]
    return float(inside.min()), float(inside.max())


def biquad_filter(b, a, x):
    """Run the difference equation along axis 0 of x (every column at once). Causal, like lfilter."""
    x = np.asarray(x, float)
    y = np.empty_like(x)
    x1 = x2 = y1 = y2 = np.zeros(x.shape[1:])
    for n in range(len(x)):
        y[n] = b[0] * x[n] + b[1] * x1 + b[2] * x2 - a[1] * y1 - a[2] * y2
        x2, x1, y2, y1 = x1, x[n], y1, y[n]
    return y


def bandpass(x, fs, clip=None):
    """The board's HPF then LPF, each stage optionally clipped at the op-amp swing (lo, hi)."""
    for f0, Q, kind in stages():
        x = biquad_filter(*bilinear_biquad(f0, Q, K_SK, kind, fs), x)
        if clip is not None:
            x = np.clip(x, *clip)
    return x
