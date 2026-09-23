# Troy Mayer — Portfolio

Single-page portfolio site. No build step, no dependencies — `index.html` is the whole site.
Publish it with GitHub Pages (Settings → Pages → Deploy from branch → `main` / root).

## Structure

| Section | Anchor | What's in it |
|---|---|---|
| Hero | `#top` | Name, current role, metric strip |
| About | `#about` | Bio, fast facts, interests |
| Experience | `#experience` | Timeline: Boston Dynamics → Caterpillar → Chortos Lab → Emerson |
| Projects | `#work` | Filterable card grid |
| Write-ups | `#state-estimation` `#orthosis` `#soft-robotics` `#caterpillar` `#emerson` | Full project detail pages |
| Skills | `#skills` | Grouped skill matrix |
| Contact | `#contact` | Email, LinkedIn, résumé |

Features: light/dark theme toggle (remembers your choice), mobile nav, scroll-reveal
animation (disabled under `prefers-reduced-motion`), active-section nav highlighting,
project category filter, back-to-top.

All content is sourced from `Troy_Mayer_Resume.pdf`. **If you update the résumé,
update the timeline bullets and the hero metric strip to match.**

---

## Images still needed

Everything below has a styled placeholder or a weak stand-in on the live page.
Drop the file into the repo root with the filename listed and swap the `src`.

### 1. Legged Robot State Estimation — **highest priority**
This is the project that most closely matches where you're headed, and it is the only
one with zero visuals. Two shots would carry it:

- **`ekf_trajectory.png`** — a plot overlaying estimated base velocity against MuJoCo
  ground truth over the 60-second run, with the 0.08 m/s RMSE annotated. Matplotlib or
  MATLAB, light background, large fonts. *Replace the placeholder in `#state-estimation`.*
- **`ekf_drift.png`** — a second plot showing the observability result: roll/pitch/velocity
  error staying bounded while position and yaw walk off. This is the most interesting
  claim on the site and right now it's text only.
- Optional: **`ekf_sim.png`** — a MuJoCo viewport screenshot of the quadruped mid-gait,
  for the project card thumbnail.

### 2. Hero portrait — **replace `image_f58a52.png`**
The current shot is a formal headshot on a gray studio backdrop. Every strong engineering
portfolio uses a photo of the person *in context*. Shoot a replacement:
you at a bench with the orthosis arm, in the Chortos Lab, or at a workstation with CAD
on screen. Landscape or square, good light, 1200px+ on the short edge.

### 3. Orthosis — DONE
`orthosis_built.jpg` is in. It now leads the gallery and is the project card image.
A short video of the arm actuating from a muscle flex would still beat it.

### 4. Emerson — **`valve_machined.jpg`** (check with Emerson first)
A photo of a piston you actually machined, or you at the lathe, would prove the
"CAD to shop floor" claim that the CAD renders don't. Only use it if it clears the
confidentiality agreement — the disclaimer on that section covers CAD, not photos.

### 5. Chortos Lab — DONE
`gel_samples.jpg` (the DIW-printed sample batch) and `keyence_rig.jpg` (the laser
displacement bench) are both in. The samples shot is the project card image.

### 6. `og-card.png` (optional)
A 1200×630 preview card for link sharing. Right now the Open Graph image points at the
portrait, which crops badly in Slack and LinkedIn.

### Image specs
- Photos: JPG, 1600px wide, under ~400 KB.
- Plots and screenshots: PNG, 1400–2000px wide, readable axis labels.
- Everything already in the repo is used except `autonetwork_code.png`,
  `autonetwork_terminal.png` (see below), `about.jpeg` and `image_0518a3.png`
  (both removed — see the git history for why).
- **Before publishing any lab photo, read it at full size first.** `keyence_rig.jpg`
  had to be cropped because a login PIN was legible on tape stuck to the laptop.

---

## Removed: AI Networking Automator

The AutoNetworker project was removed from the site. Its images
(`autonetwork_code.png`, `autonetwork_terminal.png`) are still in the repo, and the
section is recoverable from git history if you want it back.

Why it was cut: the project scraped Google results, guessed corporate emails through
Hunter.io, and ranked strangers by fraternity and hometown to cold-email them. A
recruiter reading that sees an unsolicited-email tool and a ranking heuristic built on
affiliation — and it sits next to a Boston Dynamics co-op and a Kalman filter, which is
not a comparison it wins. The Python and API work is real, but it's the weakest item on
the page and the only one with a reputational downside.

If you want a fifth software project, the state estimator is already stronger, and
anything with a public GitHub repo behind it would be better still.
