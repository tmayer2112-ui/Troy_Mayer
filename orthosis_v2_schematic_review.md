# v2 EMG board: schematic review

Checked against [`orthosis_v2_schematic.pdf`](orthosis_v2_schematic.pdf) (KiCad 9.0.7, exported 2026-09-29):
page 1 is the main sheet (`tmayeremgboard.kicad_sch`), pages 2–5 are channels 1–4 (`emgcomp.kicad_sch`, one
hierarchical sheet used four times, so all four channels are wired identically). The `.kicad_sch` files
aren't in this repo, so the changes below are for you to make in KiCad. Designators are channel 1's
(page 2); channels 2–4 use the same parts with other numbers (U7/U8, U9/U10, U11/U12).

Datasheet values marked *check* are quoted from memory; confirm them against the current datasheet.

## Fix before fabrication

### 1. The DC-coupled gain chain can rail the channel output

After C19 AC-couples the INA333, U6A (high-pass), U6B (low-pass) and U6C (gain) are DC-coupled, and R31
(1k) returns U6C's gain leg straight to VREF. Each op-amp's input offset is multiplied by every stage after it:

```
Eout,DC − VREF = K²·G·Vos(U6A) + K·G·Vos(U6B) + G·Vos(U6C)        K = 1.556,  G = 1 + (R32 + RV1)/R31 = 11 … 111
```

The MCP6004's maximum offset is ±4.5 mV (*check*). With all three offsets in the same direction:

| RV1 setting | G | Worst-case DC at Eout | vs. the ±1.6 V swing |
|---|---|---|---|
| 100k (where calibration puts channels 3–4 in the decoder emulation) | 111 | 4.5 mV × 552 = **2.49 V** | rails |
| ~47k (channel 1 in emulation) | 58 | 1.30 V | most of the headroom gone |
| 0 | 11 | 0.25 V | fine |

A railed channel also trips the firmware's lead-off fault, which treats a channel sitting within 40 counts of
a rail as a detached electrode (`firmware/emg_orthosis/orthosis_core.h:307-313`). On channels 1–2 that
would be a false fault.

**Fix:** put a capacitor in series with R31, so U6C's DC gain is 1 and its AC gain is unchanged. Worst-case
DC at Eout drops to 4.5 mV × (K² + K + 1) = 22 mV. The capacitor adds a high-pass pole at 1/(2π·R31·C), which
has to sit well below the 23 Hz band edge:

| Series C | Pole | Extra loss at 23 Hz |
|---|---|---|
| 4.7 µF | 34 Hz | about −5 dB: moves the band edge |
| 10 µF | 15.9 Hz | −1.7 dB: moves the lower −3 dB corner up |
| 47 µF | 3.4 Hz | −0.1 dB |
| 100 µF | 1.6 Hz | −0.02 dB |

Use 47–100 µF. It only ever sees millivolts, so a small ceramic is fine and polarity doesn't matter. If you
make this change, add the pole to `orthosis-emg-decoder/emgdec/board.py` (`BoardV2.filtered`) so the
emulation keeps matching the board.

### 2. Open electrode inputs have no defined DC level

E+ and E− reach the INA333 through R19/R20 (10k) with no bias resistor. With the electrodes on the skin, the
right-leg drive (U3B → R17 → REF_ELEC) gives them a DC path through the body. With a lead off, the input
floats and its level isn't defined. The firmware's lead-off check assumes a detached electrode drives the
INA333 to a rail (`orthosis_core.h:307-313`); on this schematic nothing guarantees that.

**Fix:** pull the inputs to *opposite rails* with high-value resistors on the electrode side of R19/R20,
e.g. 10 MΩ from E+ to +3V3 and 10 MΩ from E− to GND. Any open lead then rails the INA333, so the lead-off
fault fires reliably. With electrodes on, each resistor sources about (3.3 − 1.65) V / 10 MΩ = 165 nA: 3.3 mV
across a 20 kΩ electrode, which C19 blocks.

Don't pull both inputs to VREF. A floating lead would then read as a quiet, relaxed muscle. That is safe,
since nothing moves, but it disables the lead-off fault the firmware relies on.

## Worth improving

### 3. The right-leg drive senses channel 1 only

On page 1, only `tmayeremg1`'s CMD output connects to the CM net (R10 → U3B pin 6). The CMD pins of
`tmayeremg2`–`4` are marked no-connect. If channel 1's electrodes come off, the drive loses its input.

**Fix:** connect all four CMD outputs to U3B pin 6 through four 40.2k resistors, replacing R10 (10k).
True common-mode then sees 200k / (40.2k / 4) = 19.9, the same as today's 200k / 10k = 20, and losing one
pair shifts the average instead of disabling the drive.

### 4. VREF has 47 Ω of output impedance, shared by every channel

U3A closes its feedback at its own output (pin 1). VREF is on the far side of R18 (47 Ω), with C15 (100 nF)
to GND, and that RC only rolls off above 34 kHz. Across the EMG band VREF therefore looks like ~47 Ω, and
every channel returns current into it: R31, R25, R29, C22, the INA333 REF pin, and C24, which also carries the
ADC's sampling kick.

*How big:* every stage in a channel is referenced to VREF, so a disturbance on VREF reaches each channel's
output with gain ≈ 1. Each gain stage outputs VREF + G·(signal), so the disturbance rides along without being
multiplied by G. The two biggest sources, at a full-scale 1.2 V peak output:
- **C24:** at 100 Hz it carries 33 nF × 2π × 100 × 1.2 = 25 µA, or 1.2 mV on VREF; at 500 Hz, about 6 mV.
- **R31:** the signal at U6C's input over 1 kΩ. That is 1.2 V / 11 / 1 kΩ = 109 µA (5 mV) at the lowest
  gain, and about 1 mV at the gains calibration picks.

That is roughly −46 to −60 dB of crosstalk into the other channels: small, but free to remove.

**Fix:** take U3A's feedback from the VREF side of R18, and add 10–100 nF from U3A's output to its inverting
input. That is the usual in-loop isolation for a capacitive load; R18 stays to keep U3A stable. The INA333
also wants a low-impedance REF pin, because series resistance there costs CMRR (*check* how much 47 Ω costs
in its datasheet).

## Checked and not bugs

An external review flagged the items below. The PDF shows each is wired correctly; don't change them.

| Claim | What the schematic shows |
|---|---|
| USB D+/D− left unrouted: J2 uses labels `C+`/`C−`, the ESP32 uses `D+`/`D−` | J2 A6/B6 carry `D+` and A7/B7 carry `D−`; U2 pin 13 (USB_D−) is `D−` and pin 14 (USB_D+) is `D+` (page 1). No `C+`/`C−` label exists in the PDF. |
| C22 tied to U6B's inverting input, breaking the low-pass | C22 runs from U6B pin 5 (+) to the node where R29's *bottom* meets VREF. R29's top is on pin 6 with R30 (page 2). This is the correct Sallen-Key low-pass. |
| R7 (10k) in series with MOT_5V, browning out the ISO7763 | J3 pin 1 goes straight to MOT_5V (C10 to MOT_GND), and U1 Vcc2 (pin 16) goes straight to MOT_5V with C7/C8. R7 and R11 are 10k pull-ups from MOT_5V to ENC_A_ISO and ENC_B_ISO, on the J3 side of R8/R9. |
| VREF disturbances are amplified ~110× by U6C | Gain ≈ 1 (item 4). The crosstalk path is real but small. |
| Fix the DC gain with 4.7–10 µF in series with R31 | Right idea, too small a capacitor (item 1). |

## Found in the firmware during this review

`firmware/emg_orthosis/emg_orthosis.ino` named the driver pull-downs R67–R70, which are channel-4 parts
(page 5). On this schematic they are R2 and R3 (PWM lines to GND), R6 (MOT_EN to GND) and R5 (EN_ISO to
MOT_GND). The comment is corrected.
