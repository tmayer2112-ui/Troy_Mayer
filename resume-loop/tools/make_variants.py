"""Write the five angle variants (v1-v5). Every line traces to FACTS.md."""
import json
from pathlib import Path

CONTACT = ["Mountainside, NJ 07092", "908-873-9490", "tmayer2112@gmail.com", "US Citizen"]
LINKS = ["linkedin.com/in/troyarkin", "Portfolio and code: tmayer2112-ui.github.io/Troy_Mayer"]
EDU = {"org": "Purdue University, School of Mechanical Engineering and School of Science", "loc": "West Lafayette, IN",
       "role": "B.S. Mechanical Engineering; B.S. Applied Statistics; Minor in Economics", "dates": "May 2028",
       "lines": ["GPA 3.32/4 | Clubs: Club Swimming, House Band"]}
BD = dict(org="Boston Dynamics", loc="Waltham, MA", role="Sensors and Perception Co-Op, Atlas", dates="Jul 2026 – Present")
CAT = dict(org="Caterpillar (via Purdue Data Mine)", loc="West Lafayette, IN", role="Machine Learning Researcher", dates="Jan 2026 – May 2026")
CH = dict(org="Chortos Lab, Soft Adaptive Systems and Structures", loc="West Lafayette, IN", role="Undergraduate Researcher", dates="Aug 2025 – May 2026")
EM = dict(org="Emerson, Discrete Automation (ASCO)", loc="Florham Park, NJ", role="Mechanical and Manufacturing Engineer Intern", dates="Jun 2025 – Aug 2025")
LR = dict(inline=True, org="Legged Robot State Estimation and Control", role="Personal Project", dates="Aug 2026 – Oct 2026")
OR = dict(inline=True, org="Active Orthosis, Purdue ASME Biomechanics", role="Electronics Team Lead", dates="Apr 2024 – May 2026")
AH = dict(inline=True, org="Asteroid Hopping Robot", role="Independent Research, Purdue University & UC3M (Madrid)", dates="Oct 2026 – Present")
STYLE = {"margin_tb": 0.4, "before_section": 4, "after_entry": 1.5}

def e(base, bullets, **kw):
    return {**base, **kw, "bullets": bullets}

V = {}

# ---------- v1 KEYWORDS: the listings' own vocabulary, wherever it is true ----------
V["v1_keywords"] = {"sections": [
  {"title": "Education", "entries": [EDU]},
  {"title": "Technical Skills", "rows": [
    ["Controls and estimation", "control algorithms (PI, feed-forward, anti-windup, impedance, QP force control), model-based controller tuning, state estimation, Kalman filtering (error-state EKF), sensor fusion, observability analysis"],
    ["Robotics and simulation", "rigid-body kinematics and dynamics, floating-base forward kinematics, MuJoCo physics simulation, Eigen, closed-loop simulation and validation"],
    ["Software", "C++17, Python, MATLAB, SQL, Linux, Git, CMake, GoogleTest unit testing, GitHub Actions CI, real-time embedded (FreeRTOS, ESP32-S3)"],
    ["Data, ML and hardware", "PyTorch, XGBoost, LSTM, Pandas, SciPy, Flask, Plotly, SQLite, Snowflake | KiCad, LabVIEW, DAQ, CATIA, NX, Creo, FEA"]]},
  {"title": "Experience", "entries": [
    e(BD, [
      "Developing rigid-body kinematics and dynamics for Atlas impacts: floating-base forward kinematics (quaternions, Rodrigues' rotations, velocity composition) from joint encoders maps impacts onto the robot mesh, and Khatib's operational-space effective mass estimates impact energy.",
      "Designed a delta-v impact-severity metric robust to sensor bandwidth limits, validating its filtering against synthetic pulses of known size and measuring its noise floor; building a physics-based MuJoCo impact simulation.",
      "Built an impact data-analysis pipeline and Flask/Plotly dashboard on Linux covering ~28k impact events from ~900 robot test logs (telemetry ETL into SQLite, 3D impact heatmap, per-event waveforms), used by engineers during testing.",
      "Modeling hardware thermal behavior in Python (SciPy), fitting heating curves by multi-start least squares to predict steady-state temperatures across hundreds of units without full-length tests."]),
    e(CAT, [
      "Built LSTM and XGBoost machine learning models forecasting next-day paint consumption from five years of data, with Monte Carlo confidence intervals, achieving a 4.7% reduction in material usage; Python data pipelines contributed to a 3.3% cycle-time reduction."]),
    e(CH, [
      "Wrote a Python computer-vision tracker resolving displacement to 0.01 mm and built LabVIEW/MATLAB data acquisition for 50,000+ point datasets, guiding gel formulations that increased force output by over 50%."])]},
  {"title": "Robotics Projects", "entries": [
    e(LR, [
      "State estimation: built a contact-aided error-state Kalman filter in C++/Eigen fusing a 1 kHz IMU with leg kinematics on a simulated Unitree Go1 in MuJoCo, reaching 0.013 m/s velocity RMSE vs. 1.38 m/s IMU-only, in ~20 µs per real-time update.",
      "Verification: Jacobians checked against finite differences in GoogleTest unit tests, C++ matched to a Python reference within 1e-13, and observability analysis confirming position and yaw drift while roll, pitch, and velocity stay bounded.",
      "Control: ran walking and 500 Hz QP force-balance controllers on the estimate with 0 falls in 5 × 60 s walks (QP after a 4 s startup turn); the QP controller cut ground-truth velocity-tracking error from 0.169 to 0.091 m/s."]),
    e(OR, [
      "Embedded real-time control: designed and implemented a 1 kHz PI velocity controller in C++ on an ESP32-S3 (FreeRTOS), model-based tuned with feed-forward and anti-windup (0.8% speed error in simulation), with 10 fault monitors, a hardware watchdog, and a kill switch.",
      "Signal processing: cut EMG-to-motion latency from 103 ms to 43 ms with a Bayesian EMG estimator, validated in closed-loop simulation on real recorded EMG with 26 GoogleTest tests in CI.",
      "Led a 5-engineer team; redesigned the 4-channel EMG board in KiCad (INA333, right-leg drive, ISO7763 isolation), raising cross-day gesture decoding from 31% to 87% in emulation."]),
    e(AH, [
      "Motion control and optimization: designing optimized push-off control to hit a target launch speed, knee impedance control for landing, and reaction-wheel attitude control, with MuJoCo simulation comparing risk-limited hop rules."])]}
]}

# ---------- v2 IMPACT: every bullet opens with its result ----------
V["v2_impact"] = {"sections": [
  {"title": "Education", "entries": [EDU]},
  {"title": "Professional Experience", "entries": [
    e(BD, [
      "Gave Atlas test engineers a view of ~28k impact events from ~900 robot test logs by building an impact forensics pipeline and Flask/Plotly dashboard on a Linux VM, with telemetry ETL into SQLite, a 3D impact-location heatmap, and per-event waveforms.",
      "Predicting steady-state temperatures across hundreds of units without full-length tests by fitting heating curves with multi-start least squares in Python (Pandas, SciPy).",
      "Computing limb velocity and energy at impact with floating-base forward kinematics from joint encoders and Khatib's operational-space effective mass, mapping impacts onto the robot's mesh, and building a MuJoCo impact simulation."]),
    e(CAT, [
      "Cut paint-line material usage 4.7% with LSTM and XGBoost models forecasting next-day primer, topcoat, and solvent consumption from five years of data, each forecast carrying a Monte Carlo confidence interval.",
      "Contributed to a 3.3% reduction in manufacturing cycle time by building Python pipelines that consolidated five years of noisy legacy data into one daily series."]),
    e(CH, [
      "Increased gel force output by over 50% across formulation rounds, measured with a Python computer-vision tracker I wrote that resolves displacement to 0.01 mm, plus LabVIEW/MATLAB acquisition of 50,000+ point datasets."])]},
  {"title": "Projects", "entries": [
    e(LR, [
      "Reached 0.013 m/s velocity RMSE over a 60 s walk, 100× lower than IMU-only dead reckoning (1.38 m/s), with a contact-aided error-state EKF in C++/Eigen fusing a 1 kHz IMU and leg kinematics on a simulated Unitree Go1, at ~20 µs per update.",
      "0 falls in 5 × 60 s walks with the walking controller running on the estimate, tracking within 0.004 m/s of the ground-truth-fed run; a 500 Hz QP force-balance controller then cut ground-truth tracking error 46% (0.169 to 0.091 m/s).",
      "Removed a 4° lean by tracing it to weak tilt observability on straight walks and adding a 4 s startup turn; Jacobians verified against finite differences in GoogleTest."]),
    e(OR, [
      "Cut EMG-to-motion latency 58% (103 ms to 43 ms) with a Bayesian EMG estimator and a 20 ms onset confirm that took false starts to zero, in closed-loop simulation on real recorded EMG, pinned by 26 GoogleTest tests in CI.",
      "Held speed error to 0.8% (vs. 98% feed-forward alone) lifting the forearm in simulation with a 1 kHz PI velocity controller in C++ on an ESP32-S3, protected by 10 fault monitors, a hardware watchdog, and a kill switch.",
      "Raised cross-day gesture decoding from 31% to 87% in emulation over 115 days of real EMG by leading a 5-engineer team and redesigning the 4-channel EMG board in KiCad (INA333, right-leg drive, ISO7763 isolation)."]),
    e(AH, [
      "Designing push-off control optimized to hit a target launch speed, knee impedance control for landing, and reaction-wheel reorientation for a three-legged hopping robot, simulated in MuJoCo to trade hop distance against escape and tip-over risk."])]},
  {"title": "Skills", "rows": [
    ["Programming", "C++17, Python, MATLAB, SQL | Languages: English, Turkish (fluent), Spanish (fluent)"],
    ["Robotics and controls", "Kalman filtering (error-state EKF), sensor fusion, rigid-body kinematics, PI and feed-forward control, QP force control, MuJoCo, Eigen, FreeRTOS, ESP32"],
    ["Software and data", "Linux, Git, GitHub Actions CI, CMake, GoogleTest, Flask, Plotly, SQLite, Pandas, SciPy, PyTorch, XGBoost, Snowflake"],
    ["Hardware and CAD", "KiCad, LabVIEW, DAQ, CATIA, NX, Fusion 360, Creo, FEA"]]}
]}

# ---------- v3 STRUCTURE: relevance tiers instead of employer-vs-project ----------
V["v3_structure"] = {"sections": [
  {"title": "Education", "entries": [{"org": "Purdue University", "loc": "West Lafayette, IN",
     "role": "B.S. Mechanical Engineering and B.S. Applied Statistics, Minor in Economics | GPA 3.32/4", "dates": "May 2028"}]},
  {"title": "Skills", "rows": [
    ["Languages and tools", "C++17, Python, MATLAB, SQL, Linux, Git, CMake, GoogleTest, GitHub Actions CI, Eigen, MuJoCo, FreeRTOS"],
    ["Methods", "error-state EKF and sensor fusion, rigid-body kinematics, PI and feed-forward control, QP force control, least-squares fitting, Monte Carlo, LSTM and XGBoost"],
    ["Hardware", "ESP32-S3, KiCad, LabVIEW, DAQ, CATIA, NX, Fusion 360, Creo, FEA"]]},
  {"title": "Robotics Experience", "entries": [
    e(BD, [
      "Computing limb position and velocity at impact with floating-base forward kinematics (quaternions, Rodrigues' rotations, rigid-body velocity composition) from joint encoders, and mapping impacts onto the robot's mesh.",
      "Estimating impact energy from Khatib's operational-space effective mass and an OLS velocity fit, and building a physics-based MuJoCo impact simulation.",
      "Built an impact forensics pipeline and Flask/Plotly dashboard on a Linux VM covering ~28k impact events from ~900 robot test logs, with ETL from a time-series telemetry store into SQLite, used by engineers during testing.",
      "Modeling hardware thermal behavior in Python, fitting heating curves by multi-start least squares to predict steady-state temperatures across hundreds of units without full-length tests."]),
    e(LR, [
      "Built a contact-aided error-state EKF in C++/Eigen fusing a 1 kHz IMU with leg kinematics on a simulated Unitree Go1: 0.013 m/s velocity RMSE over a 60 s walk vs. 1.38 m/s IMU-only, at ~20 µs per update, Jacobians verified in GoogleTest.",
      "Ran walking and 500 Hz QP force-balance controllers on the estimate with 0 falls in 5 × 60 s walks, after tracing a 4° lean to weak tilt observability and removing it with a 4 s startup turn."]),
    e(OR, [
      "Designed and implemented a 1 kHz PI velocity controller in C++ on an ESP32-S3, tuned from a first-order motor model with feed-forward and anti-windup, protected by 10 fault monitors, a hardware watchdog, and a kill switch.",
      "Cut EMG-to-motion latency from 103 ms to 43 ms with a Bayesian EMG estimator, validated in closed-loop simulation on real recorded EMG with 26 GoogleTest tests in CI.",
      "Led a 5-engineer team and redesigned the 4-channel EMG board in KiCad (INA333 front end, right-leg drive, ISO7763 isolation), raising cross-day gesture decoding from 31% to 87% in emulation."]),
    e(AH, [
      "Designing push-off control optimized to hit a target launch speed, knee impedance control for landing, and reaction-wheel control to reorient mid-air, with MuJoCo simulation comparing risk-limited hop rules and a pendulum rig to test landings."])]},
  {"title": "Additional Experience", "entries": [
    e(CAT, [
      "Built LSTM and XGBoost models forecasting next-day paint consumption from five years of data with Monte Carlo confidence intervals (4.7% less material used), plus Python pipelines contributing to a 3.3% cut in cycle time."]),
    e(CH, [
      "Wrote a Python computer-vision tracker resolving displacement to 0.01 mm and LabVIEW/MATLAB data acquisition for 50,000+ point datasets, guiding gel formulations that increased force output by over 50%."]),
    e(EM, [
      "Designed and CNC-machined an adjustable spring canister for an industrial rotary valve in Creo with stress calculations and GD&T drawings; ran UL 429 HiPot, torque, bending, and impact validation on solenoid valves."])]}
]}

# ---------- v4 IDENTITY: one clear story, stated up top ----------
V["v4_identity"] = {"summary": "Robotics software and controls: robot kinematics, state estimation, and real-time control in C++, built with tests and simulation. Mechanical engineering plus applied statistics; currently on Atlas at Boston Dynamics.",
 "sections": [
  {"title": "Education", "entries": [EDU]},
  {"title": "Experience", "entries": [
    e(BD, [
      "Kinematics: computing limb position and velocity at impact with floating-base forward kinematics (quaternions, Rodrigues' rotations, rigid-body velocity composition) from joint encoders, and mapping impacts onto the robot's mesh.",
      "Dynamics: estimating impact energy from Khatib's operational-space effective mass and an OLS velocity fit, and building a physics-based MuJoCo impact simulation.",
      "Tooling: built an impact forensics pipeline and Flask/Plotly dashboard on a Linux VM covering ~28k impact events from ~900 robot test logs (ETL into SQLite, 3D impact heatmap, per-event waveforms), used by engineers during testing.",
      "Statistics: modeling hardware thermal behavior in Python, fitting heating curves by multi-start least squares to predict steady-state temperatures across hundreds of units without full-length tests."]),
    e(CAT, [
      "Built Python pipelines consolidating five years of legacy data and LSTM/XGBoost models forecasting next-day paint consumption with Monte Carlo confidence intervals, achieving a 4.7% reduction in material usage and contributing to a 3.3% cut in cycle time."]),
    e(CH, [
      "Wrote a Python computer-vision tracker resolving displacement to 0.01 mm and built LabVIEW/MATLAB data acquisition for 50,000+ point datasets, guiding gel formulations that increased force output by over 50%."])]},
  {"title": "Robotics Projects", "entries": [
    e(LR, [
      "Built a contact-aided error-state EKF in C++/Eigen fusing a 1 kHz IMU with leg kinematics on a simulated Unitree Go1: 0.013 m/s velocity RMSE over a 60 s walk vs. 1.38 m/s IMU-only, at ~20 µs per update.",
      "Proved it before trusting it: Jacobians checked against finite differences in GoogleTest, C++ matched to a Python reference within 1e-13, and an observability analysis showing why position and yaw drift while roll, pitch, and velocity stay bounded.",
      "Closed the loop: walking and 500 Hz QP force-balance controllers run on the estimate with 0 falls in 5 × 60 s walks, after tracing a 4° lean to weak tilt observability and fixing it with a 4 s startup turn."]),
    e(OR, [
      "Designed and implemented a 1 kHz PI velocity controller in C++ on an ESP32-S3, tuned from a first-order motor model with feed-forward and anti-windup, protected by 10 fault monitors, a hardware watchdog, and a kill switch.",
      "Cut EMG-to-motion latency from 103 ms to 43 ms with a Bayesian EMG estimator, validated in closed-loop simulation on real recorded EMG with 26 GoogleTest tests in CI.",
      "Led a 5-engineer team and redesigned the 4-channel EMG board in KiCad (INA333, right-leg drive, ISO7763 isolation), raising cross-day gesture decoding from 31% to 87% in emulation."]),
    e(AH, [
      "Designing push-off control optimized to hit a target launch speed, knee impedance control for landing, and reaction-wheel control to reorient mid-air for a three-legged hopping robot, simulated in MuJoCo."])]},
  {"title": "Skills", "rows": [
    ["Programming", "C++17, Python, MATLAB, SQL | Languages: English, Turkish (fluent), Spanish (fluent)"],
    ["Robotics and controls", "Kalman filtering (error-state EKF), sensor fusion, rigid-body kinematics, PI and feed-forward control, QP force control, MuJoCo, Eigen, FreeRTOS"],
    ["Software, data, hardware", "Linux, Git, GitHub Actions CI, CMake, GoogleTest, Flask, Plotly, SQLite, Pandas, SciPy, PyTorch | KiCad, LabVIEW, CATIA, NX, Creo"]]}
]}

# ---------- v5 GAPS: answer what a screener would hold against it ----------
V["v5_gaps"] = {"sections": [
  {"title": "Education", "entries": [{**EDU, "lines": ["GPA 3.32/4 | Available for Summer 2027 internships | Clubs: Club Swimming, House Band"]}]},
  {"title": "Professional Experience", "entries": [
    e(BD, [
      "Computing limb position and velocity at impact on the real robot with floating-base forward kinematics from joint encoders, mapping impacts onto its mesh, and estimating impact energy from Khatib's operational-space effective mass.",
      "Built an impact forensics pipeline and Flask/Plotly dashboard on a Linux VM covering ~28k impact events from ~900 robot test logs, with ETL from a time-series telemetry store into SQLite, used by engineers during testing.",
      "Modeling hardware thermal behavior in Python, fitting heating curves by multi-start least squares to predict steady-state temperatures across hundreds of units without full-length tests.",
      "Designing CATIA fixtures, mounts, and shear features for impact testing, using pin-shear failure analysis to optimize geometry."]),
    e(CAT, [
      "Built LSTM and XGBoost models forecasting next-day paint consumption from five years of data with Monte Carlo confidence intervals, achieving a 4.7% reduction in material usage; Python pipelines contributed to a 3.3% cut in cycle time."]),
    e(CH, [
      "Wrote a Python computer-vision tracker resolving displacement to 0.01 mm and built LabVIEW/MATLAB data acquisition for 50,000+ point datasets, guiding gel formulations that increased force output by over 50%."]),
    e(EM, [
      "Designed and CNC-machined an adjustable spring canister for an industrial rotary valve in Creo with stress calculations and GD&T drawings; assembled and tested 159 actuators and ran UL 429 HiPot validation on solenoid valves."])]},
  {"title": "Projects", "entries": [
    e(LR, [
      "Built a contact-aided error-state EKF in C++/Eigen fusing a 1 kHz IMU with leg kinematics on a simulated Unitree Go1: 0.013 m/s velocity RMSE over a 60 s walk vs. 1.38 m/s IMU-only, ~20 µs per update, Jacobians verified in GoogleTest.",
      "Ran walking and 500 Hz QP force-balance controllers on the estimate with 0 falls in 5 × 60 s walks (QP after a 4 s startup turn); stress-tested foot slip and contact latency, and gated contact detection on the gait schedule to remove 6.6° of yaw drift."]),
    e(OR, [
      "Designed and implemented a 1 kHz PI velocity controller in C++ on an ESP32-S3, tuned from a first-order motor model with feed-forward and anti-windup, with 10 fault monitors, a hardware watchdog, and a kill switch.",
      "Cut EMG-to-motion latency from 103 ms to 43 ms in closed-loop simulation on real recorded EMG, with 26 GoogleTest tests and an ESP32-S3 build running in CI.",
      "Led a 5-engineer team and redesigned the 4-channel EMG board in KiCad after the v1 board was fabricated (INA333, right-leg drive, ISO7763 isolation), raising cross-day gesture decoding from 31% to 87% in emulation."]),
    e(AH, [
      "Designing push-off, knee impedance, and reaction-wheel control for a three-legged hopping robot, simulated in MuJoCo, plus a pendulum rig to test landings."])]},
  {"title": "Skills", "rows": [
    ["Programming", "C++17, Python, MATLAB, SQL, Linux, Git, CMake, GoogleTest, GitHub Actions CI | Spanish and Turkish (fluent)"],
    ["Robotics and controls", "Kalman filtering (error-state EKF), sensor fusion, rigid-body kinematics, PI and feed-forward control, QP force control, MuJoCo, Eigen, FreeRTOS"],
    ["Hardware and data", "KiCad, LabVIEW, DAQ, CATIA, NX, Creo, FEA, CNC, GD&T | Pandas, SciPy, PyTorch, Flask, Plotly, SQLite"]]}
]}

out = Path(__file__).resolve().parent.parent / "variants"
for stem, body in V.items():
    spec = {"file_stem": stem, "name": "Troy Mayer", "contact": CONTACT, "links": LINKS, "style": dict(STYLE), **body}
    (out / f"{stem}.json").write_text(json.dumps(spec, indent=1, ensure_ascii=False))
    print("wrote", stem)
