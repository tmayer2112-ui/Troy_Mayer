"""Module 3: why v1 failed and v2 passes. REFERENCE SOLUTION.

Read one function at a time (the stuck rule), close it, then type it yourself.
"""
import numpy as np

import m2_filter

Z_CM = 1 / (2 * np.pi * 60 * 3e-12)     # INA333 common-mode input: 3 pF at 60 Hz = 884 MOhm


def diff_amp_gains(z_pos, z_neg, r1=1e3, r2=47e3):
    """v1's LM324 difference amp with the electrode impedances in series with its 1k inputs.

        a = R2/(R1 + Z+ + R2) * (1 + R2/(R1 + Z-))    gain from the + electrode
        b = R2/(R1 + Z-)                              gain from the - electrode
    Returns (G_diff, G_cm) = ((a + b)/2, a - b).     (Worksheet 3.1)
    """
    # TODO (3 lines)
    # --- solution ---
    a = r2 / (r1 + z_pos + r2) * (1 + r2 / (r1 + z_neg))
    b = r2 / (r1 + z_neg)
    return (a + b) / 2, a - b
    # --- end ---


def v1_cm_to_diff(z_pos, z_neg, resistor_cmrr_db=62.0):
    """How much common-mode voltage reaches v1's differential path, referred to the input (1/CMRR):
    the electrode imbalance term |G_cm| / G_diff plus the 1 % resistor-matching limit (62 dB)."""
    # TODO (2 lines). 62 dB as a ratio is 10^(-62/20).
    # --- solution ---
    gd, gc = diff_amp_gains(z_pos, z_neg)
    return abs(gc) / gd + 10 ** (-resistor_cmrr_db / 20)
    # --- end ---


def v2_cm_to_diff(dz, z_cm=Z_CM, ina_cmrr_db=100.0, rld_db=20.0):
    """The same for v2: the INA333's own CMRR plus the electrode imbalance dz against its
    common-mode input impedance z_cm, all divided by the right-leg drive's 20 dB.  (Worksheet 3.4)"""
    # TODO (1 line)
    # --- solution ---
    return (10 ** (-ina_cmrr_db / 20) + abs(dz) / z_cm) * 10 ** (-rld_db / 20)
    # --- end ---


def input_to_clear_diode(g_front, pot_ohms=100e3, vf=0.4):
    """Smallest EMG amplitude (volts, at the electrodes) that reaches v1's SS14 diode drop vf.
    Total gain = front-end gain x K^2 (both Sallen-Key stages) x (1 + (47k + pot)/1k)."""
    # TODO (1 line). Use m2_filter.K_SK for K.
    # --- solution ---
    return vf / (g_front * m2_filter.K_SK ** 2 * (1 + (47e3 + pot_ohms) / 1e3))
    # --- end ---


def peak_detector(v, vf, decay):
    """v1's SS14 diode into C9 || R15. Each sample the capacitor bleeds (e *= decay), and the diode
    charges it instantly whenever the input minus the diode drop is higher:
        e[n] = max(e[n-1] * decay, v[n] - vf),   starting from e = 0.
    v has shape (n_samples, ...) and every column is independent.   (Worksheet 3.3)
    """
    # TODO (5 lines): a loop over n; np.maximum works on whole rows at once
    # --- solution ---
    out = np.empty_like(v, dtype=float)
    e = np.zeros(v.shape[1:])
    for n in range(len(v)):
        e = np.maximum(e * decay, v[n] - vf)
        out[n] = e
    return out
    # --- end ---


def hum_at_rectifier(v_cm, cmrr_db, total_gain):
    """Peak 60 Hz voltage that reaches v1's diode: the body's common-mode v_cm, let into the differential
    path by the front end's CMRR, then multiplied by the whole chain's gain.   (Worksheet 3.5)"""
    # TODO (1 line)
    # --- solution ---
    return v_cm * 10 ** (-cmrr_db / 20) * total_gain
    # --- end ---


# ------------------------------------------------------------------ provided from here down
def db(ratio):
    """A gain ratio below 1 as positive dB of rejection: 1e-3 -> 60 dB."""
    return -20 * np.log10(ratio)


def electrodes(z, mismatch=0.25):
    """(Z+, Z-) for a nominal impedance z and a fractional mismatch: 20k +/- 25 % = 22.5k, 17.5k."""
    return z * (1 + mismatch / 2), z * (1 - mismatch / 2)


VREF = 1.65
V2_INA_GAIN = 1 + 100e3 / (5.49e3 + 5.49e3)     # R_G = R21 + R22


def v1_filtered(x_volts, fs, z=20e3):
    """v1 up to its tunable amp: difference amp (loaded by the electrodes), then the band-pass.
    The LM324 on +/-3.3 V can't swing above about +1.8 V (emgdec/board.py, BoardV1)."""
    g, _ = diff_amp_gains(*electrodes(z))
    clip = (-3.3 + 0.05, 3.3 - 1.5)
    return m2_filter.bandpass(np.clip(g * x_volts, *clip), fs, clip)


def v2_filtered(x_volts, fs):
    """v2 up to its gain stage: INA333 (G = 10.1, 100 GOhm inputs), then the band-pass, +/-1.6 V."""
    clip = (-1.6, 1.6)
    return m2_filter.bandpass(np.clip(V2_INA_GAIN * x_volts, *clip), fs, clip)


def calibrate_pot(filtered, fixed_ohms, target_v, max_pot=100e3):
    """Bench trim, as calibrate_pots in emgdec/board.py: the smallest pot that puts the 99.9th
    percentile of |output| at target_v, clamped to the 0-100k the pot can reach."""
    p = np.percentile(np.abs(filtered), 99.9, axis=0)
    return np.clip((target_v / np.maximum(p, 1e-12) - 1) * 1e3 - fixed_ohms, 0, max_pot)


def v1_output(filtered, pot, fs, rng, envelope=True):
    """v1 after its tunable amp: SS14 + C9 || R15 peak detector (tau = 180 ms), then the ADC."""
    y = np.clip(filtered * (1 + (47e3 + pot) / 1e3), -3.3 + 0.05, 3.3 - 1.5)
    if envelope:
        y = peak_detector(y, 0.4, np.exp(-1.0 / (fs * 10e-6 * 18e3)))
    else:
        y = np.maximum(y - 0.4, 0.0)
    return m2_filter.adc(y, rng)


def v2_output(filtered, pot, rng):
    """v2 after its gain stage: centred on VREF into the ADC, VREF removed after (as the firmware does)."""
    y = np.clip(filtered * (1 + (10e3 + pot) / 1e3), -1.6, 1.6)
    return m2_filter.adc(VREF + y, rng) - VREF


def v1_cm_coupling(mismatch, sign, z=20e3, resistor_cmrr_db=62.0):
    """Signed common-mode -> differential gain of each v1 channel, referred to the input: the electrode
    imbalance term plus that channel's resistor-matching error (emgdec/board.py, BoardV1.cm_coupling)."""
    m = np.asarray(mismatch, float)
    gd, gc = diff_amp_gains(z * (1 + m / 2), z * (1 - m / 2))
    return gc / gd + np.asarray(sign, float) * 10 ** (-resistor_cmrr_db / 20)


def v2_cm_coupling(mismatch, sign, z=20e3, ina_cmrr_db=100.0, rld_db=20.0):
    """The same for v2 (emgdec/board.py, BoardV2.cm_coupling)."""
    dz = z * np.asarray(mismatch, float)
    return (np.asarray(sign, float) * 10 ** (-ina_cmrr_db / 20) + dz / Z_CM) * 10 ** (-rld_db / 20)


def run_chains(recs, fs=2048, cal_days=(1, 2, 3, 4, 5), seed=0, hum_v=0.0):
    """Every recording through each board. Returns {chain: {(day, class): output}} and the pots.

    recs are in microvolts. Pots are trimmed on the calibration days only, then frozen:
    v1 to 1.6 V, v2 to 1.2 V, as in the real project. hum_v: peak 60 Hz common-mode on the body, in
    volts (0 = none). Each channel turns it into signal through its own signed coupling, per day.
    pots also carries "v1 share above the diode": the fraction of v1 samples that clear the 0.4 V drop.
    """
    import simdata
    keys = sorted(recs)
    x = np.stack([recs[k] for k in keys], axis=-1) * 1e-6           # (n, ch, recordings), volts
    out, pots = {"no board": {k: recs[k] for k in keys}}, {}
    cal = np.array([k[0] in cal_days for k in keys])
    rng = np.random.default_rng(seed)
    if hum_v:
        hum = np.stack([simdata.body_hum(k[0], k, len(x), fs, seed) for k in keys], axis=-1)   # (n, recordings)
        mismatch = np.stack([simdata.electrode_imbalance(k[0], seed) for k in keys], axis=-1)  # (ch, recordings)
    for name, filt, fixed, target in (("v1 envelope", v1_filtered, 47e3, 1.6), ("v2", v2_filtered, 10e3, 1.2)):
        xb = x
        if hum_v:
            couple = v1_cm_coupling if name.startswith("v1") else v2_cm_coupling
            xb = x + hum_v * hum[:, None, :] * couple(mismatch, simdata.board_signs(seed)[:, None])[None]
        f = filt(xb, fs)
        cal_f = np.moveaxis(f[..., cal], -1, 0).reshape(-1, f.shape[1])
        pot = calibrate_pot(cal_f, fixed, target)
        pots[name] = pot
        if name.startswith("v1"):
            pre = np.clip(f * (1 + (47e3 + pot[:, None]) / 1e3), -3.3 + 0.05, 3.3 - 1.5)
            pots["v1 share above the diode"] = float(np.mean(pre > 0.4))
        y = v1_output(f, pot[:, None], fs, rng) if name.startswith("v1") else v2_output(f, pot[:, None], rng)
        out[name] = {k: y[..., i] for i, k in enumerate(keys)}
    return out, pots
