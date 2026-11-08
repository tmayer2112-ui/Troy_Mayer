"""Synthetic surface EMG (provided, you don't write this). Pure numpy, no download.

It stands in for LibEMG MultiDay, the 2 GB real dataset behind the decoder results. It keeps what
the decoder modules (1-5) depend on and drops the rest. Nothing it produces is a project result.

Kept:
  * 4 channels, 11 classes, one sustained hold per class per day, sampled at 2048 Hz
  * 20 days: days 1-10 train, days 11-20 test (the real split is days <= 60 / > 60)
  * EMG built the way it arises: motor-unit action potentials (MUAPs) fired at random times.
    The sum is band-limited (most power 50-200 Hz), heavier-tailed than Gaussian, and skewed,
    because one electrode pair sees an asymmetric MUAP (a tall peak, a shallow trough).
  * 6 muscles; each class drives each muscle between 10 % and 100 %, and every channel picks up
    every muscle (crosstalk), more from near ones. So classes overlap the way real gestures do:
    they share muscles, and differ in amplitude pattern across channels and in spectrum.
  * the electrodes go back on a little differently each day (each channel's pickup of each
    muscle x e^N(0, 0.2)), and each channel's gain drifts (x e^N(0, 0.1))
  * effort varies a little between holds (x e^N(0, 0.1) per hold), and fatigues within each hold:
    amplitude and mean frequency fall, sized so the last 40 % of a hold has about the RMS
    (x 0.92) and mean frequency (x 0.94) the real data shows against the first 60 %
  * lead polarity, the real pattern in miniature: days 1-5 all four channels reversed, days 6-15
    all normal, days 16-20 channels 1-2 reversed. Training (1-10) sees only "----" and "++++";
    days 16-20 have a pattern training never saw, like days 69+ in MultiDay.
  * every random draw is seeded from (seed, day, class, purpose), as in the real project
Dropped:
  * real anatomy: amplitudes and MUAP shapes are random, not muscles. In particular class 0
    ("no motion") is NOT the loudest class here; in MultiDay it is, on every day.
  * the board and the ADC (Modules 2-3 add them on top of this). 60 Hz hum is optional and off
    by default: `body_hum`, used by Module 3's board chains.
  * electrode shift onto a different muscle, transitions between gestures, motion artefacts
  * duplicate days, and anything above 1024 Hz

Module 7 uses `contraction_1khz` and `pop`: the same MUAP model at the firmware's 1 kHz rate, in
ADC counts, with a 50 ms voluntary ramp like firmware/analysis/latency_study.py.

Units: decoder data in microvolts (MultiDay's unit), Module 7 data in ADC counts.
"""
import zlib

import numpy as np

FS = 2048
N_CH = 4
N_CLS = 11
CLASSES = ["no motion", "wrist extension", "wrist flexion", "wrist adduction", "wrist abduction",
           "supination", "pronation", "hand open", "hand closed", "key grip", "index point"]
TRAIN_DAYS = list(range(1, 11))
TEST_DAYS = list(range(11, 21))
ALL_DAYS = TRAIN_DAYS + TEST_DAYS
HOLD_S = 3.0
SEEN_TEST_DAYS = list(range(11, 16))       # polarity pattern seen in training
UNSEEN_TEST_DAYS = list(range(16, 21))     # pattern never seen in training


def polarity(day):
    """Sign of each channel on `day`: +1 normal, -1 lead reversed."""
    if day <= 5:
        return np.array([-1, -1, -1, -1])
    if day <= 15:
        return np.array([1, 1, 1, 1])
    return np.array([-1, -1, 1, 1])


def rng_for(seed, *key):
    """A generator seeded from (seed, day, class, purpose): the same draw every run."""
    return np.random.default_rng([seed] + [zlib.crc32(str(k).encode()) for k in key])


def muap(width_s, fs):
    """One motor-unit potential: a tall positive peak and a shallow, wider, later trough.
    Its samples sum to zero (no DC), and it is lopsided in amplitude, which makes the EMG skewed."""
    t = np.arange(-6 * width_s, 14 * width_s, 1 / fs)
    peak = np.exp(-0.5 * (t / width_s) ** 2)
    trough = np.exp(-0.5 * ((t - 1.5 * width_s) / (2 * width_s)) ** 2)
    h = peak - trough * peak.sum() / trough.sum()
    return h / np.sqrt(np.sum(h ** 2))


def shot_noise(n, rate_hz, width_s, fs, rng):
    """MUAPs fired at Poisson times with log-normal sizes. Scaled to unit RMS on average
    (Campbell's theorem), so the amplitude pattern sets the RMS, not the random draw."""
    h = muap(width_s, fs)
    size = rng.lognormal(0.0, 0.5, n)
    fire = rng.random(n) < rate_hz / fs
    x = np.convolve(fire * size, h)[:n]
    mean_sq = rate_hz / fs * np.exp(2 * 0.5 ** 2) * np.sum(h ** 2)   # E[size^2] = e^(2 sigma^2)
    return x / np.sqrt(mean_sq)


N_MUSCLES = 6
SCALE_UV = 100.0
# Fatigue over each hold, sized from the real data (README, "Why the last 40 % is harder": the last
# 40 % of a hold has median RMS x 0.92 and mean frequency x 0.94 against the first 60 %).
FATIGUE_RMS_END = 0.85         # amplitude at the end of the hold, linear from 1
FATIGUE_WIDTH_END = 1.13       # MUAP width at the end of the hold, cross-faded from 1


def anatomy(seed=0):
    """The fixed 'arm': how hard each class drives each muscle, how much of each muscle each
    channel picks up, and the MUAP width each channel sees from each muscle."""
    r = rng_for(seed, "anatomy")
    drive = r.uniform(0.1, 1.0, (N_CLS, N_MUSCLES))           # class -> muscle activation, 0-1
    pickup = np.exp(r.normal(0.0, 0.7, (N_CH, N_MUSCLES)))     # channel <- muscle, near or far
    width = r.uniform(0.8e-3, 2.0e-3, (N_MUSCLES, N_CH))       # wider = farther = lower frequency
    return drive, pickup, width


def recording(day, cls, seed=0, flips=True, fs=FS, hold_s=HOLD_S):
    """One sustained hold: (n_samples, 4) float64, microvolts.

    Each muscle fires its own MUAP train (faster and bigger the harder the class drives it). Every
    channel sees every muscle, through that day's pickup: the electrodes go back on a little
    differently each day (x e^N(0, 0.2) per channel and muscle), and the channel gain drifts.
    """
    drive, pickup, width = anatomy(seed)
    n = int(round(hold_s * fs))
    placed = pickup * np.exp(rng_for(seed, day, "placement").normal(0.0, 0.2, pickup.shape))
    gain = np.exp(rng_for(seed, day, "gain").normal(0.0, 0.1, N_CH))
    effort = np.exp(rng_for(seed, day, cls, "effort").normal(0.0, 0.1))
    t = np.linspace(0.0, 1.0, n)[:, None]          # 0 -> 1 over the hold
    fade = 1.0 - (1.0 - FATIGUE_RMS_END) * t
    r = rng_for(seed, day, cls, "emg")
    x = np.zeros((n, N_CH))
    for m in range(N_MUSCLES):
        a = drive[cls, m]
        fire = (r.random(n) < (100.0 + 500.0 * a) / fs) * r.lognormal(0.0, 0.5, n)
        mean_sq = (100.0 + 500.0 * a) / fs * np.exp(2 * 0.5 ** 2)      # Campbell, as in shot_noise
        for ch in range(N_CH):
            # fatigue slows conduction, so MUAPs widen: cross-fade from the fresh shape to a wider one
            fresh = np.convolve(fire, muap(width[m, ch], fs))[:n]
            tired = np.convolve(fire, muap(width[m, ch] * FATIGUE_WIDTH_END, fs))[:n]
            x[:, ch] += placed[ch, m] * a * ((1 - t[:, 0]) * fresh + t[:, 0] * tired) / np.sqrt(mean_sq)
    x *= SCALE_UV * effort * fade * gain
    x += r.normal(0.0, 2.0, x.shape)            # amplifier floor, 2 uV RMS
    if flips:
        x *= polarity(day)
    return x


def make_dataset(seed=0, flips=True, days=ALL_DAYS):
    """{(day, class): (n, 4) array}. About 1 s to build."""
    return {(d, c): recording(d, c, seed, flips) for d in days for c in range(N_CLS)}


# ----------------------------------------------------------------- Module 3: optional 60 Hz hum
# None of this is used unless you ask for hum, and it draws from its own random streams, so the EMG
# above is byte-for-byte the same with or without it.
def body_hum(day, key, n, fs=FS, seed=0):
    """60 Hz mains on the body, unit amplitude. The level is drawn per DAY (x 0.5-1.5) and the phase per
    recording, as in the real model: drawn per recording, the level would identify the recording and
    so the class (decoder DEBUG_LOG #1)."""
    level = rng_for(seed, day, "hum-level").uniform(0.5, 1.5)
    phase = rng_for(seed, *key, "hum-phase").uniform(0.0, 2 * np.pi)
    return level * np.sin(2 * np.pi * 60.0 * np.arange(n) / fs + phase)


def electrode_imbalance(day, seed=0):
    """Each channel's (Z+ - Z-) / Z on `day`, redrawn every day, uniform on +/-50 % (decoder README,
    "Mains hum"). It decides how much of the hum each channel turns into signal, and with what sign."""
    return rng_for(seed, day, "imbalance").uniform(-0.5, 0.5, N_CH)


def board_signs(seed=0):
    """The sign of each channel's own CMRR error, fixed by its parts: drawn once per board."""
    return rng_for(seed, "board-signs").choice([-1.0, 1.0], N_CH)


# ----------------------------------------------------------------- Module 7: 1 kHz, ADC counts
FS_FW = 1000


def contraction_1khz(trial, rest_s=2.0, hold_s=2.5, tail_s=1.5, active_rms=200.0, rest_rms=4.0,
                     ramp_ms=50, seed=0):
    """Rest, a voluntary contraction that ramps up over `ramp_ms`, then rest again, at 1 kHz.

    Returns (x, start, end): x in ADC counts about 0 (the firmware's DC tracker has removed the
    offset), start/end = sample indices of the contraction. Rest is 4 counts RMS of noise,
    active about 200 counts RMS: the numbers of the firmware's Envelope test.
    """
    r = rng_for(seed, trial, "contraction")
    width, rate = r.uniform(0.8e-3, 2.0e-3), r.uniform(150.0, 600.0)
    n_rest, n_hold, n_tail = (int(s * FS_FW) for s in (rest_s, hold_s, tail_s))
    n = n_rest + n_hold + n_tail
    env = np.zeros(n)
    env[n_rest:n_rest + n_hold] = active_rms
    env[n_rest:n_rest + ramp_ms] *= np.linspace(0.0, 1.0, ramp_ms)
    x = env * shot_noise(n, rate, width, FS_FW, r) + r.normal(0.0, rest_rms, n)
    return x, n_rest, n_rest + n_hold


def pop(amplitude):
    """A 3 ms biphasic electrode pop, the artefact model in firmware/analysis/latency_study.py."""
    return amplitude * np.array([1.0, -1.0, 0.5])


def rest_with_pops(seconds=60.0, per_s=1.0, rest_rms=4.0, seed=0):
    """Rest noise with `per_s` electrode pops per second (100-400 counts each), at 1 kHz."""
    r = rng_for(seed, "pops")
    n = int(seconds * FS_FW)
    x = r.normal(0.0, rest_rms, n)
    for k in np.flatnonzero(r.random(n) < per_s / FS_FW):
        p = pop(r.uniform(100, 400) * r.choice([-1, 1]))[: n - k]
        x[k:k + len(p)] += p
    return x
