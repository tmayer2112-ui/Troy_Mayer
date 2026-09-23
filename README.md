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

### 3. Orthosis — **`orthosis_assembled.jpg`**
There are three renders and an FEA study but no photo of the physical thing. One picture
of the assembled arm — ideally worn, with electrodes on — is worth more than all three
renders. Add it as the first item in the `#orthosis` gallery and use it as the card image.

### 4. Emerson — **`valve_machined.jpg`** (check with Emerson first)
A photo of a piston you actually machined, or you at the lathe, would prove the
"CAD to shop floor" claim that the CAD renders don't. Only use it if it clears the
confidentiality agreement — the disclaimer on that section covers CAD, not photos.

### 5. Chortos Lab — **`gel_fixture.jpg`**
A photo of one of the 15 printed test fixtures with a gel sample loaded. The current
gallery is all screenshots; one physical object grounds it.

### 6. `og-card.png` (optional)
A 1200×630 preview card for link sharing. Right now the Open Graph image points at the
portrait, which crops badly in Slack and LinkedIn.

### Image specs
- Photos: JPG, 1600px wide, under ~400 KB.
- Plots and screenshots: PNG, 1400–2000px wide, readable axis labels.
- Everything already in the repo is used except `autonetwork_code.png` and
  `autonetwork_terminal.png` — see below.

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
