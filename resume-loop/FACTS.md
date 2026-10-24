# Fact bank: the only things a resume line may say

**The rule:** every line on every version must trace to a fact below. A line that adds a skill,
number, result, scope or status that isn't here is false, and the version scores zero.
Rewording and dropping are fine. Adding is not.

Sources, from most to least authoritative:
- **U**: the uploaded resume (Word, created 2026-10-22). The current truth. Where it conflicts with another source, U wins.
- **C**: `quadruped-eskf/claims.json` and `orthosis-emg-decoder/claims.json`. `make claims` checks each number against the result file that produced it.
- **F**: `firmware/README.md` and `firmware/results/*.json`, pinned by the firmware's GoogleTest suite in CI.
- **S**: the portfolio site, `index.html` (published 2026-09-29/30). Troy's own longer descriptions.
- **R**: the repo's older resume PDF (2026-09-30). Details U dropped for space but didn't contradict.

Confirmed by Troy in this session: **GPA is 3.32** (S and R say 3.4, which is out of date).

## Header
- H1 Troy Mayer · Mountainside, NJ 07092 · 908-873-9490 · tmayer2112@gmail.com · US Citizen · linkedin.com/in/troyarkin [U]
- H2 Portfolio site: tmayer2112-ui.github.io/Troy_Mayer. Pages serves only `docs/` (the site and figures), not the code. Code: github.com/tmayer2112-ui/Troy_Mayer is public and holds the code, tests and write-ups for the orthosis firmware, EMG decoder and quadruped EKF. Only the firmware tests run in CI (`.github/workflows/firmware.yml`). [repo]
- H3 Open to Summer 2027 internships in robot software, controls and data. [S]

## Education
- E1 Purdue University, School of Mechanical Engineering and School of Science, West Lafayette, IN [U]
- E2 B.S. Mechanical Engineering; B.S. Applied Statistics; Minor in Economics; May 2028 [U]
- E3 GPA 3.32/4 [U, confirmed]
- E4 Clubs: Club Swimming, House Band [U]
- (No coursework, honors or scholarships are on record. Don't add any.)

## Boston Dynamics: Sensors and Perception Co-Op – Atlas · Waltham, MA · July 2026 – Present [U]
- BD1 Maps impacts onto the robot's mesh using encoders and forward kinematics. [U]
- BD2 Estimates impact energy from Khatib's operational-space effective mass and an OLS velocity fit. [U]
- BD3 Building a physics-based MuJoCo impact simulation. [U]
- BD4 Forward kinematics computes where each limb is and how fast it moves at impact. It propagates 6-DOF floating-base poses through the joint chains with quaternions, roll-pitch-yaw matrices and Rodrigues' rotation formula. Velocities come from exact rigid-body velocity composition (base velocity plus ω × r) rather than numerical differentiation, and are fused with high-rate IMU data to measure energy change across impacts. [R, S]
- BD5 Designed a Δv (delta-v) impact-severity metric that stays accurate when sensors miss the sharpest part of an impact (sensor bandwidth limits that hurt peak-based metrics). [R, S]
- BD6 Characterized the Butterworth anti-alias filtering against synthetic pulses of known amplitude. Quantified the metric's noise floor with a null-control experiment on quiet data. Landed every change only after bit-identical regression checks across the full event table. [R, S]
- BD7 Impact forensics pipeline and Flask/Plotly dashboard on a Linux VM, covering ~28k impact events from ~900 robot test logs. [U]
- BD8 ETL from a time-series telemetry store into SQLite. 3D impact-location heatmap, normalized by per-facet solid angle so it reads as density. Per-event waveform inspection. Used by engineers during testing. [U, S]
- BD9 Models hardware thermal behavior in Python (Pandas, SciPy): Savitzky-Golay filtering [U, R; the site dropped it on 2026-10-09, U re-added it on 10-22], heating-regime detection, multi-start least-squares fits of exponential heating curves, predicting steady-state temperatures across hundreds of units without full-length tests, gated on residual RMSE. [U, S, R]
- BD10 Designs CATIA (3DEXPERIENCE) fixtures, mounts and shear features for standalone impact testing, plus protective hardware so sensitive parts survive repeated impacts. Uses pin-shear failure analysis to optimize the geometry. [U, S]
- (Keep it proprietary-safe: method only, no BD data. Tense: the role is ongoing.)

## Caterpillar (via Purdue Data Mine): Machine Learning Researcher · West Lafayette, IN · Jan 2026 – May 2026 [U]
- CAT1 Built LSTM and XGBoost models forecasting next-day primer, topcoat and solvent consumption from five years of paint-line data, with Monte Carlo confidence intervals on each forecast. [U]
- CAT2 4.7% reduction in material usage. [U]
- CAT3 Built Python pipelines to ingest, clean and consolidate five years of noisy legacy (month-by-month) data into one daily series, contributing to a 3.3% reduction in manufacturing cycle time. [U, S]
- CAT4 Surfaced results through a Python dashboard and Power BI. Isolated process inefficiencies between the manual and robotic paint lines. Snowflake. [S]

## Chortos Lab, Soft Adaptive Systems and Structures: Undergraduate Researcher · West Lafayette, IN · Aug 2025 – May 2026 [U]
- CH1 Built data acquisition systems in LabVIEW and MATLAB, processing and normalizing datasets of 50,000+ points to quantify sub-millimeter material behavior. [U]
- CH2 Wrote a Python computer-vision tracker resolving cantilever displacement to 0.01 mm (on a 29.45 mm cantilever), for the geometry the Keyence laser couldn't follow. [U, S]
- CH3 This guided gel formulations that increased force output by over 50%. Troy formulated and synthesized the ionic dielectric gel variants. [U, S]
- CH4 Designed 15 custom Fusion 360 test fixtures. Keyence laser displacement head into an NI USB-6001 and LabVIEW. DIW 3D printing. [S]

## Emerson, Discrete Automation (ASCO): Mechanical and Manufacturing Engineer Intern · Florham Park, NJ · Jun 2025 – Aug 2025 [S only, not on U]
- EM1 Electrical and mechanical validation of XM-5 and Red Hat II solenoid valves: PPS vs. epoxy insulation and the effect of pre-heating. HiPot to UL 429 Sec. 37, torque (150 in-lb), bending (200 in-lb) and impact (6.5 J) tests. [S]
- EM2 Designed and manufactured an adjustable spring canister, piston and connection system for a heavy industrial rotary valve in Creo. Machined on a CNC lathe and band saw. Ran the stress calculations, built the BOM, released GD&T drawings. [S]
- EM3 Assembled and tested 159 hydro-pump and solenoid actuators. Updated 298 pneumatic valve drawings. Designed custom Instron tooling. [S]
- EM4 Analyzed actuator deadband from process-capability reports. Modified 400+ ASCO 890 signaling boxes. [S]

## Purdue ASME Biomechanics, Active Orthosis: Electronics Team Lead · West Lafayette, IN · April 2024 – May 2026 [U]
- OR1 Led a 5-engineer electronics team. [U, S]
- OR2 Designed and implemented a 1 kHz PI velocity controller in C++ (C++17) on an ESP32-S3, run by a 1 kHz FreeRTOS task. Tuned from a first-order motor model (IMC tuning) with plant-inversion feed-forward and anti-windup. [U, S, F]
- OR3 In simulation, lifting the forearm at 15°/s: speed error 0.8% closed loop vs. 98% with feed-forward alone. [F, S]
- OR4 Protected by 10 runtime fault monitors (runaway, stall, electrode lead-off, tracking error, over-travel and others) that latch the motor off, a hardware watchdog and a kill switch. [U, S, F]
- OR5 The controller core has no hardware dependencies and compiles into a simulator of the motor, transmission and forearm. 26 GoogleTest unit, closed-loop and real-EMG latency tests, plus an ESP32-S3 build, run in CI (GitHub Actions). [F, S, R]
- OR6 Cut EMG-to-motion latency from 103 ms to 43 ms with a Bayesian EMG amplitude estimator and a 20 ms onset confirm, measured in closed-loop simulation driven by real recorded EMG. Stop lag went from 89 ms to 32 ms. The 43 ms configuration combines three changes: the Bayesian estimator (Sanger 2007), the 20 ms onset confirm, and a faster loop tuning tier. Don't credit the estimator alone. Measured on 24 real contractions (p90 82 ms). [U, F, S, R]
- OR7 The 20 ms onset confirm takes false starts from motion artefacts from 62/min to zero. [F, S]
- OR8 Redesigned the 4-channel EMG board in KiCad: INA333 front end, right-leg drive, 23–474 Hz band-pass, ISO7763 isolation between body-side electronics and the motor driver, hardware kill switch. The v2 schematic is complete. Layout and fabrication come next. v1 was fabricated. [U, S, C]
- OR9 Raised cross-day (day-to-day) 11-class gesture decoding from 31% (v1) to 87% (v2) in emulation: 115 days of real 4-channel forearm EMG run through a sample-level emulation of each board. 87.3% vs. an 87.9% ceiling with no board. [U, C, S]
- OR10 Traced a PyTorch CNN's accuracy loss on later days to electrode-polarity changes, using per-channel skewness. Polarity augmentation brought it to 86.3%. [C, S]
- OR11 Firmware design: a 1 kHz FreeRTOS task pinned to core 1 (`xTaskDelayUntil`), with serial output in a separate task so printing can't delay the loop. The encoder ISR is IRAM-safe (direct GPIO register reads, table-free Gray-code decode). Speed comes from timing encoder edges, under 0.02% error vs. 12–111% for per-tick counting at arm speeds. [F: firmware/README.md lines 27, 36, 37]
- (Truth guard: every firmware and decoder number is from simulation or emulation, not from the board. Keep "simulation" or "emulation" on each one. firmware/README.md says: "designed and verified in simulation. It has not run on the board yet: the v2 PCB isn't fabricated." So write "for an ESP32-S3", not "on", and don't call the controller "deployed" or "embedded real-time" as if it ran on hardware.)

## Asteroid Hopping Robot: Independent Research · Purdue University & Universidad Carlos III de Madrid · October 2026 – Present [U]
- AH1 Building a three-legged hopping robot that changes leg shape between hops, and a pendulum rig to test landings. [U]
- AH2 Designing the leg motor control that pushes off for each hop, optimizing it to hit a target launch speed. Knee impedance control for landing. Reaction-wheel control to reorient mid-air. [U]
- AH3 Simulating it in MuJoCo to size each hop, comparing risk-limited rules that trade distance against the chance of escaping the asteroid or tipping over. [U]
- (All in progress. Use present tense, no results.)

## Legged Robot State Estimation and Control: Personal Project · August 2026 – October 2026 [U]
- LR1 Contact-aided error-state EKF in C++/Eigen (with a Python reference) for a simulated Unitree Go1 in MuJoCo. Fuses a BMI088-class IMU at 1 kHz with leg kinematics. Adds each planted foot to the state at touchdown and marginalizes it at liftoff. [U, C, S]
- LR2 0.013 m/s velocity RMSE over a 60-second walk (mean of 10 sensor-noise seeds) vs. 1.38 m/s for IMU-only dead reckoning. [C]
- LR3 1 kHz, one update per IMU sample. The C++ step takes about 20 µs of the 1 ms budget (median; README: p50 18 µs, p99 60 µs, SCHED_FIFO + mlockall on a non-real-time host; max ~4 ms, 0.135% missed deadlines). Say "median". Don't say "real-time" without that context. [C, quadruped-eskf/README.md, results/timing.json]
- LR11 Code quality: C++17/Eigen. 0 warnings under -Wall -Wextra -Wpedantic. 26 C++ (GoogleTest) tests and 29 Python tests, run locally with `make build test`. The covariance predict exploits the structure of F, O(15²·n) instead of O(n³), and a parity test holds it equal to the dense Python version. [quadruped-eskf/README.md lines 53, 121]
- LR10 Which code runs where: the closed loop (controllers fed the estimate) runs the **Python** reference filter and **Python** controllers inside the 1 kHz MuJoCo loop. The QP is "Python at 500 Hz in simulation ... with no timing guarantee". The C++ filter runs in offline replay and the timing benchmark, and matches the Python filter to 1e-13. Never word a line so the C++ filter reads as being in the control loop. [quadruped-eskf/README.md, python/qeskf/closed_loop.py]
- LR4 Jacobians verified against finite differences in GoogleTest. The C++ filter matches the Python reference to 1e-13 on a recorded walk (say "to 1e-13", not "within"; the max attitude difference is 1.2e-12). These tests run locally (`make test`), not in CI. [C, S]
- LR5 Observability: roll, pitch and velocity stay bounded while position and yaw drift (exactly four unobservable directions in every window tested). Derived analytically, confirmed numerically. [C, S]
- LR6 Failure experiments: about 2 cm/s of foot slip doubles velocity error. With force-only contact detection, 10 ms of contact latency costs 6.6° of yaw over the walk; gating contact on the gait schedule removes the failure. A tutorial-style Q = 0.01·I loses attitude. [C, S]
- LR7 Walking and QP balance controllers run on the estimate. Fed the estimate, the walking controller completes 5 × 60 s walks with 0 falls, with velocity tracking within 0.004 m/s of the ground-truth-fed run. [U, C]
- LR8 Found that tilt is only weakly observable on a straight walk (about 4° lean fed the estimate). A 4 s startup turn makes it observable and removes the lean. [C, S]
- LR9 QP force-balance controller (MIT Cheetah 3 style: single rigid body, friction-limited foot forces solved as non-negative least squares at 500 Hz). Velocity tracking 0.091 m/s vs. 0.169 m/s for the scripted trot (fed ground truth). Fed the estimate after the startup turn: 0 falls in 5 × 60 s walks. [C, S]
- (Truth guard: simulated robot. Say "simulated".)

## Skills and languages
- SK1 Programming: C++ (C++17), Python, MATLAB, SQL. [U, S]
- SK2 Robotics and controls: rigid-body kinematics, PI + feed-forward control, QP force control, error-state EKF, sensor fusion, FreeRTOS, ESP32, Eigen, MuJoCo. [S]
- SK3 Software and tooling: Linux, Git, GitHub Actions CI, CMake, GoogleTest, Flask, Plotly, SQLite. [S, U]
- SK4 Data and ML: PyTorch, scikit-learn, Pandas, NumPy, SciPy, LSTM, XGBoost, Monte Carlo, least-squares fitting, Snowflake, Power BI. [S, U]
- SK5 Signals and instrumentation: filter design, computer vision, surface EMG, KiCad, LabVIEW, DAQ. [S, U]
- SK6 Mechanical and CAD: CATIA (3DEXPERIENCE), NX, Fusion 360, Creo, FEA, GD&T, CNC / G-code, DIW 3D printing. [S, U]
- SK7 Spoken languages: English (native), Turkish (fluent), Spanish (fluent, New Jersey Seal of Biliteracy). [U]
- SK8 Hobbies: lifting, hiking, guitar, saxophone. [U]

## Not in the master. Never claim these.
ROS / ROS 2, Simulink, Rust, CUDA, Java, AWS / cloud, MoveIt, PX4 / ArduPilot, SLAM, motion planning / trajectory
optimization as a project (there is none), RL training, hardware validation of the v2 board, firmware numbers measured
on hardware, publications, coursework, honors, team sizes or user counts other than those listed above.
