"""Emulate the v1 and v2 orthosis EMG boards on recorded EMG.

Both boards share the equal-component Sallen-Key pair: a 15k/470n high-pass and a
1k/330n low-pass, each with gain K = 1 + 10k/18k. They differ everywhere else.

v1 (as fabricated, openeremg.kicad_sch / emgcomp.kicad_sch):
    electrodes -> 1k/47k LM324 difference amp (unbuffered) -> HPF -> LPF
    -> 1 + (47k + pot)/1k -> SS14 -> [C9 10u || R15 18k, optional] -> ADC
    on +/-3.3 V (ICL7660).
v2 (redesign):
    electrodes -> 10k -> INA333, G = 1 + 100k/11k -> HPF -> LPF
    -> 1 + (10k + pot)/1k -> ADC, on a single 3.3 V rail referenced to
    VREF = 1.65 V, with a right-leg drive.

Signals go in as differential electrode voltage in volts, shape (n_samples, n_channels).
Common-mode hum is modelled separately (see `hum`) because the two boards reject it
very differently.
"""
from dataclasses import dataclass

import numpy as np
from numba import njit
from scipy.signal import bilinear, lfilter

# Shared Sallen-Key filters (values read off the v1 schematic, unchanged in v2).
K_SK = 1 + 10e3 / 18e3                      # 1.556 per stage
Q_SK = 1 / (3 - K_SK)                       # 0.692, equal-R equal-C Sallen-Key
HPF_F0 = 1 / (2 * np.pi * 15e3 * 470e-9)    # 22.6 Hz
LPF_F0 = 1 / (2 * np.pi * 1e3 * 330e-9)     # 482 Hz

# ESP32-S3 SAR ADC at 12 dB attenuation: 12-bit codes over roughly 0-3.1 V.
ADC_BITS = 12
ADC_FULL_SCALE = 3.1
ADC_ENOB = 10.0

VREF = 1.65


def sallen_key_analog(f0, kind, q=Q_SK, gain=K_SK):
    """Continuous-time (b, a) of an equal-component Sallen-Key stage."""
    w0 = 2 * np.pi * f0
    a = [1.0, w0 / q, w0 ** 2]
    b = [0.0, 0.0, gain * w0 ** 2] if kind == "low" else [gain, 0.0, 0.0]
    return b, a


def sallen_key(f0, kind, fs, q=Q_SK, gain=K_SK):
    """Discrete (b, a) via the bilinear transform, prewarped so f0 lands exactly."""
    w0 = 2 * fs * np.tan(np.pi * f0 / fs)
    a = [1.0, w0 / q, w0 ** 2]
    b = [0.0, 0.0, gain * w0 ** 2] if kind == "low" else [gain, 0.0, 0.0]
    return bilinear(b, a, fs)


def adc(v, rng, bits=ADC_BITS, full_scale=ADC_FULL_SCALE, enob=ADC_ENOB):
    """Sample-and-quantise. ADC noise is set so the converter delivers `enob` bits."""
    lsb_eff = full_scale / 2 ** enob
    v = v + rng.normal(0.0, lsb_eff / np.sqrt(12), v.shape)
    lsb = full_scale / 2 ** bits
    return np.round(np.clip(v, 0.0, full_scale) / lsb) * lsb


@njit(cache=True)
def _peak_detector(v, vf, decay):
    """SS14 into C9 || R15: charges through the diode, bleeds through R15."""
    out = np.empty_like(v)
    for c in range(v.shape[1]):
        e = 0.0
        for n in range(v.shape[0]):
            e *= decay
            x = v[n, c] - vf
            if x > e:
                e = x
            out[n, c] = e
    return out


# --------------------------------------------------------------------------- front ends

def diff_amp_gains(z_pos, z_neg, r1=1e3, r2=47e3):
    """Differential and common-mode gain of the v1 unbuffered difference amp.

    The electrode impedances sit in series with the 1k input resistors, so they
    both cut the gain and unbalance the bridge that sets CMRR.
    """
    a = r2 / (r1 + z_pos + r2) * (1 + r2 / (r1 + z_neg))   # gain from E+
    b = r2 / (r1 + z_neg)                                  # gain from E-
    return (a + b) / 2, a - b


@dataclass
class Electrodes:
    """Skin-electrode impedance seen by each input (resistive approximation)."""
    z_nominal: float = 20e3     # gel Ag/AgCl on prepared skin, EMG band
    mismatch: float = 0.25      # |Z+ - Z-| / Z_nominal

    @property
    def z_pos(self):
        return self.z_nominal * (1 + self.mismatch / 2)

    @property
    def z_neg(self):
        return self.z_nominal * (1 - self.mismatch / 2)


@dataclass
class BoardV1:
    """LM324 board as fabricated."""
    electrodes: Electrodes = None
    envelope: bool = True           # C9/R15 populated; False = "unpopulated for raw output"
    pot_ohms: np.ndarray = None     # per-channel 100k rheostat setting, 0..100k
    cmrr_resistors_db: float = 62.0  # 1% resistors, (1 + G)/(4 tol) worst case
    v_high: float = 3.3 - 1.5       # LM324 output cannot reach V+ (VOH ~ V+ - 1.5 V)
    v_low: float = -3.3 + 0.05
    diode_vf: float = 0.4           # SS14 forward drop
    tau_env: float = 10e-6 * 18e3   # C9 * R15 = 180 ms

    def __post_init__(self):
        self.electrodes = self.electrodes or Electrodes()

    @property
    def front_gain(self):
        gd, _ = diff_amp_gains(self.electrodes.z_pos, self.electrodes.z_neg)
        return gd

    @property
    def cm_to_diff(self):
        """Common-mode voltage -> output, referred to the input (1 / CMRR)."""
        gd, gc = diff_amp_gains(self.electrodes.z_pos, self.electrodes.z_neg)
        return abs(gc) / gd + 10 ** (-self.cmrr_resistors_db / 20)

    def out_gain(self, n_ch):
        pot = np.zeros(n_ch) if self.pot_ohms is None else np.asarray(self.pot_ohms, float)
        return 1 + (47e3 + np.clip(pot, 0, 100e3)) / 1e3

    def _clip(self, v):
        return np.clip(v, self.v_low, self.v_high)

    def filtered(self, x, fs):
        """Difference amp, HPF and LPF: the signal entering the tunable amp."""
        y = self._clip(self.front_gain * x)
        y = self._clip(lfilter(*sallen_key(HPF_F0, "high", fs), y, axis=0))
        return self._clip(lfilter(*sallen_key(LPF_F0, "low", fs), y, axis=0))

    def pre_rectifier(self, x, fs):
        """Tunable amp output, in volts about 0 V."""
        return self._clip(self.out_gain(x.shape[1]) * self.filtered(x, fs))

    def __call__(self, x, fs, rng):
        y = self.pre_rectifier(x, fs)
        if self.envelope:
            decay = np.exp(-1.0 / (fs * self.tau_env))
            y = _peak_detector(np.ascontiguousarray(y), self.diode_vf, decay)
        else:
            # Best case for the unpopulated option: assumes the ADC pin discharges
            # the node every sample. With nothing on it, the node actually floats.
            y = np.maximum(y - self.diode_vf, 0.0)
        return adc(y, rng)


@dataclass
class BoardV2:
    """INA333 single-supply board."""
    electrodes: Electrodes = None
    pot_ohms: np.ndarray = None
    ina_gain: float = 1 + 100e3 / 11e3
    cmrr_ina_db: float = 100.0      # INA333 datasheet minimum, G >= 10
    z_cm: float = 1 / (2 * np.pi * 60 * 3e-12)   # 100G || 3 pF at 60 Hz
    rld_db: float = 20.0            # common-mode reduction from the right-leg drive
    swing: float = 1.60             # rail-to-rail about VREF, less output headroom

    def __post_init__(self):
        self.electrodes = self.electrodes or Electrodes()

    @property
    def front_gain(self):
        return self.ina_gain

    @property
    def cm_to_diff(self):
        e = self.electrodes
        mismatch = abs(e.z_pos - e.z_neg) / self.z_cm
        return (10 ** (-self.cmrr_ina_db / 20) + mismatch) * 10 ** (-self.rld_db / 20)

    def out_gain(self, n_ch):
        pot = np.zeros(n_ch) if self.pot_ohms is None else np.asarray(self.pot_ohms, float)
        return 1 + (10e3 + np.clip(pot, 0, 100e3)) / 1e3

    def _clip(self, v):
        return np.clip(v, -self.swing, self.swing)

    def filtered(self, x, fs):
        """INA333, HPF and LPF: the signal entering the gain stage, about VREF."""
        y = self._clip(self.ina_gain * x)
        y = self._clip(lfilter(*sallen_key(HPF_F0, "high", fs), y, axis=0))
        return self._clip(lfilter(*sallen_key(LPF_F0, "low", fs), y, axis=0))

    def pre_adc(self, x, fs):
        return self._clip(self.out_gain(x.shape[1]) * self.filtered(x, fs))

    def __call__(self, x, fs, rng):
        return adc(VREF + self.pre_adc(x, fs), rng) - VREF   # firmware removes mid-scale


def calibrate_pots(board, recordings, fs, target_v, max_pot=100e3):
    """Set each channel's pot like you would on the bench.

    Picks the smallest gain that puts the 99.9th percentile of |output| at
    `target_v`, clamped to what the pot can reach.
    """
    p = np.percentile(np.concatenate([np.abs(board.filtered(x, fs)) for x in recordings]),
                      99.9, axis=0)
    fixed = 47e3 if isinstance(board, BoardV1) else 10e3
    want = (target_v / np.maximum(p, 1e-12) - 1) * 1e3 - fixed
    board.pot_ohms = np.clip(want, 0, max_pot)
    return board.pot_ohms


def hum(n, fs, v_cm, cm_to_diff, rng, f=60.0):
    """Mains pickup that reaches the differential input, per channel, random phase."""
    t = np.arange(n) / fs
    phase = rng.uniform(0, 2 * np.pi, 1)
    return (v_cm * cm_to_diff * np.sin(2 * np.pi * f * t + phase))[:, None]


def reference_chain(x, fs):
    """No board: a clean 20-450 Hz digital band-pass, the usual EMG preprocessing."""
    from scipy.signal import butter, sosfilt
    sos = butter(4, [20, 450], btype="band", fs=fs, output="sos")
    return sosfilt(sos, x, axis=0)
