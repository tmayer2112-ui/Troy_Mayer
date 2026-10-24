# What happened to the 131 flags

The seven reviewers raised 131 flags on the winning version (v1_keywords): 1 reject, 60 major, 70 minor.
Verdicts: six said borderline and the skeptic said reject.
The flags overlap heavily. Below they're grouped by the change that answers them.
Each flag is either **fixed**, **sent to Troy** (fixing it truthfully needs a fact only he has), or **declined** with a reason.
The fixed page is `variants/v6_final.json` and `build/Troy_Mayer_Resume.{docx,pdf}`.

## Fixed

| Theme | Raised by | What changed |
|---|---|---|
| **The firmware has never run on hardware** (the one reject) | R3, R4, R5 | "Embedded real-time control … on an ESP32-S3" became "for an ESP32-S3". The co-simulation with a motor-to-forearm plant model is now stated as the method. "Real-time embedded" is gone from skills. The firmware README says it hasn't run on the board. |
| "Real-time" on a host-side median | R4, R5, R7 | "~20 µs per real-time update" became "~20 µs median step" (README: p50 18 µs, p99 60 µs, on a non-real-time host). |
| C++ filter implied in the control loop | R4 (verified in `closed_loop.py`) | "Closed the loop in Python simulation". The C++/Python parity is stated separately. Recorded as LR10 in FACTS. |
| 0.8% with no condition or baseline | R3, R5, audit | "0.8% speed error lifting at 15°/s vs. 98% feed-forward-only". |
| 103→43 ms credited to one component | R5 | It now names all three changes: the Bayesian amplitude estimator, the 20 ms onset confirm, and faster loop tuning. |
| v2 board reads as built | R5, R6 | "redesigned the … board **schematic**". The decoding result is "emulated", with the 11 classes and 115 days stated. |
| 26 tests attached to the wrong claim | R4, R5 | The firmware's 26 GoogleTest tests and ESP32-S3 build now sit on the firmware bullet. The EKF has its own 26 GoogleTest cases plus parity and -Wpedantic. |
| The startup turn read like a hack | R3, R5 | It's now a finding: "Traced a ~4° lean to weak tilt observability … fixed it with a 4 s startup turn". |
| No failure analysis | R3 | Added the foot-slip result (~2 cm/s doubles error) and the 6.6° yaw failure from 10 ms contact latency, removed by gait-schedule gating. |
| QP compared against a weak baseline | R3, R5 | Dropped the 0.169→0.091 m/s comparison. Kept the estimate-fed result (0 falls; within 0.004 m/s of ground-truth-fed) and named the QP as MIT Cheetah 3-style (NNLS, 500 Hz). |
| BD bullet 1: garden path, overreach, name-drop | R2, R3, R5, R7 | "Developing … dynamics for Atlas" became "Mapping Atlas impacts … with floating-base forward kinematics". Dropped "Khatib's". Restored the OLS velocity fit and exact velocity composition. |
| BD bullet 2: fragment and tense switch | R5, R7 | The MuJoCo clause moved out. "Validated" became "characterized". |
| Software-engineering signals missing | R4 | Added the bit-identical regression checks (BD6), the pinned FreeRTOS task, the IRAM-safe ISR and the latching fault monitors (OR11), and 0 warnings under -Wall -Wextra -Wpedantic (LR11). All verified in the repo READMEs. |
| "Built" vs. "Building" (tense) | R5, audit | The BD dashboard is now "Building", matching the current resume. The asteroid robot is "Building / designing". |
| The code link reached no code | R4, R5, R2, audit | The header now carries `github.com/tmayer2112-ui/Troy_Mayer` and the Pages site. The project heading links straight to `quadruped-eskf/`. |
| No clickable links | R1, R2, R7 | Real hyperlinks: email (mailto), LinkedIn, GitHub, the portfolio, and the project code link. |
| Robotics evidence started 65% down the page | R2, R3, R4 | New order: Education → Skills → **Robotics Experience and Projects** (BD, legged robot, orthosis, asteroid) → Additional Experience. |
| Skills: an 8-line keyword wall | R2, R5, R1 | Four rows with languages first. "C++ (C++17)" so the bare token parses. No nested parentheses and no pipes inside lists. Dropped impedance (in progress only), "closed-loop simulation and validation", NX, LSTM and Snowflake. Added "classical feedback control" and "IMC tuning" (both true, from OR2). |
| Role target never stated | R2, R6 | Added a line under the header: "Open to Summer 2027 internships in robot software and controls" (from H3 on the site). |
| Education parsing and "Expected" | R1, R6, R7 | "Purdue University, West Lafayette, IN", "Expected May 2028", "GPA: 3.32/4.00". The two degrees are joined with "and". |
| Emerson internship missing | R6 | Added under Additional Experience. It's real hands-on work: CNC, GD&T, testing 159 actuators, UL 429 HiPot. |
| Pipes in headings; ×, ~ and µ symbols | R1, R7 | No pipes in project headings. "5 × 60 s" became "five 60 s walks". "~28k" became "about 28k". µs kept once, in "~20 µs median step". |
| Run-in labels not parallel | R7, R2 | All labels dropped. Every bullet opens with a verb. |
| Widows, the "closed-/loop" split, unit wrapping | R7, R2 | No bullet ends on a near-empty line. Non-breaking spaces join numbers to units. Typographic apostrophes. |
| Dates column jumps between lines | R7 | Dates sit on the first line of every entry. |
| Caterpillar bullet | R5, R7 | "Trained" (not a second "Built"). The 3.3% stays credited to the pipelines, not the models. |
| Chortos credit | R5, audit | "Wrote a Python computer-vision tracker … and used them to formulate ionic dielectric gels with over 50% more force output". Troy did both. |
| PDF metadata | R1, R7 | Author "Troy Mayer", Title "Troy Mayer Resume", file named `Troy_Mayer_Resume.pdf`. |

## Sent to Troy (these need facts only he has)

1. **Boston Dynamics co-op end date**, and whether a return rotation is committed. The page says "Jul 2026 – Present". (R6)
2. **Any closed-loop result on real hardware**: the orthosis loop on the v1 board or a bench motor, or the pendulum rig. Every controls number is simulated. (R3, R5, R6)
3. **The portfolio site still says GPA 3.4** and serves the 2026-09-30 resume. One click from this page, that contradicts it. Fix `index.html`, `docs/index.html` and `Troy_Mayer_Resume.pdf`. (R5, R6)
4. **Caterpillar**: team size and your share, and whether 4.7% / 3.3% were measured in production or projected. (R5)
5. **ROS / ROS 2, Simulink, inverse kinematics, data structures or OS coursework**: any of it, if it exists. These are the only keyword gaps left, and none can be claimed today. (R1, R3, R4, R6)
6. **Analytic leg IK** in `quadruped-eskf/python/qeskf/gait.py`: if you wrote it, "inverse kinematics" becomes truthful. (R3)
7. **Repo tidy-up before recruiters click**: a root README indexing the three projects, the loose images moved under `assets/`, the committed `__pycache__` removed, and CI added for `quadruped-eskf`. (R4)
8. **LinkedIn**: confirm `linkedin.com/in/troyarkin` shows GPA 3.32 and the same dates. (R6)
9. **Location**: whether to show Waltham, MA during the co-op, or "open to relocation". (R6)

## Declined

- **Dropping GPA on listings with no cutoff** (R6). It stays: hiding it invites the assumption that it's lower, and two listings ask for 3.0+.
- **Cutting the asteroid project** (R6). It stays at two lines. It's current, it has hardware (the pendulum rig), and it's worded as in progress.
- **Using MATLAB/Simulink keywords for Anduril** (R1). Simulink isn't true, so it isn't on the page.
- **Bolding many phrases** (R2). Only the three headline phrases are bold: one per robotics entry.
- **Expanding whitespace further** (R7). One page is the constraint. The extra space went to the reviewers' content asks instead.
