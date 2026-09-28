import numpy as np
import pytest
from scipy.signal import freqz

from emgdec import board as B
from emgdec.features import hudgins, windows

FS = 2048


def gain_at(f0, kind, f):
    return abs(freqz(*B.sallen_key(f0, kind, FS), worN=[f], fs=FS)[1][0])


def test_sallen_key_values_match_schematic():
    assert B.K_SK == pytest.approx(1.5556, abs=1e-4)
    assert B.Q_SK == pytest.approx(0.6923, abs=1e-4)
    assert B.HPF_F0 == pytest.approx(22.58, abs=0.01)
    assert B.LPF_F0 == pytest.approx(482.3, abs=0.1)


def test_filters_pass_band_and_reject_edges():
    assert gain_at(B.HPF_F0, "high", 200) == pytest.approx(B.K_SK, rel=0.02)
    assert gain_at(B.LPF_F0, "low", 50) == pytest.approx(B.K_SK, rel=0.02)
    # Equal-component stage at f0: |H| = K * Q
    assert gain_at(B.HPF_F0, "high", B.HPF_F0) == pytest.approx(B.K_SK * B.Q_SK, rel=1e-3)
    assert gain_at(B.LPF_F0, "low", B.LPF_F0) == pytest.approx(B.K_SK * B.Q_SK, rel=1e-3)
    assert gain_at(B.HPF_F0, "high", 2) < 0.02


def test_v1_diff_amp_is_ideal_with_zero_source_impedance():
    gd, gc = B.diff_amp_gains(0.0, 0.0)
    assert gd == pytest.approx(47.0)
    assert gc == pytest.approx(0.0, abs=1e-12)


def test_v1_electrode_impedance_cuts_gain_and_cmrr():
    gd, gc = B.diff_amp_gains(22.5e3, 17.5e3)
    assert gd < 3
    assert abs(gc) / gd > 0.05          # worse than 26 dB


def test_v1_dead_zone_hides_small_signals():
    # 200 uV peak is a moderate contraction; with 20k electrodes and the pot at
    # minimum it never clears the SS14's 0.4 V, so the ADC sees only its own noise.
    t = np.arange(FS) / FS
    small = (200e-6 * np.sin(2 * np.pi * 100 * t))[:, None].repeat(4, 1)
    b = B.BoardV1(envelope=False)
    assert np.all(b.pre_rectifier(small, FS) < b.diode_vf)
    buffered = B.BoardV1(electrodes=B.Electrodes(0.0, 0.0), envelope=False)
    assert np.any(buffered.pre_rectifier(small, FS) > buffered.diode_vf)


def test_v2_is_centred_and_linear_for_small_signals():
    rng = np.random.default_rng(0)
    t = np.arange(4 * FS) / FS
    x = (50e-6 * np.sin(2 * np.pi * 100 * t))[:, None].repeat(4, 1)
    b = B.BoardV2(pot_ohms=np.zeros(4))
    y = b(x, FS, rng)[FS:]
    expected = 50e-6 * b.ina_gain * B.K_SK ** 2 * 11 * np.sqrt(0.5)
    assert np.sqrt(np.mean(y ** 2)) == pytest.approx(expected, rel=0.05)
    assert abs(y.mean()) < 5e-3


def test_hudgins_counts_zero_crossings():
    t = np.arange(800) / FS
    x = np.sin(2 * np.pi * 100 * t)[:, None].repeat(4, 1)
    w, _ = windows(x, 400, 400)
    f = hudgins(w, np.full(4, 0.01))
    # 100 Hz over 400 samples at 2048 Hz = 19.5 cycles -> ~39 crossings
    assert 37 <= f[0, 8] <= 40
