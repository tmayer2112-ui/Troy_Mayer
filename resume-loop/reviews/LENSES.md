# The seven rejection reviewers

Each reviewer gets the winning version, the 10 listings and the fact bank. Each one's whole job
is to find reasons to reject the page through one lens. Every flag needs an exact quote, a severity,
and a fix. A fix may only use facts already in `FACTS.md`. If a fix would need something new,
the reviewer marks it `needs Troy` instead of inventing it.

Severity: **reject** (on its own, this gets the page tossed), **major** (costs interviews), **minor** (polish).

1. **ATS parser and keyword gatekeeper.** Reads only the text layer, the way Workday, Greenhouse and Lever
   parse it. Checks section headings, the contact block, dates and titles. Catches symbols that garble
   (×, µ, ~, Δ, |) and keyword coverage against the 10 listings' required and preferred terms.
2. **Six-second recruiter skim.** What lands in the top third. Whether the role fit is obvious without reading.
   Density, bullet length, walls of text, and whether the strongest signal is findable at a glance.
3. **Controls / state-estimation hiring manager** on a humanoid or legged-robot team. Technical precision and
   credibility. Whether the metrics mean something, how clear the sim-vs-hardware line is, and what's missing for this job.
4. **Robot-software interviewer** (SDE lens). Does it read like someone who ships real software?
   C++ depth, testing, CI, code links, design. Or does it read like an ME who scripts?
5. **Skeptic / overclaim detector.** Probes every bullet the way an interviewer would ("walk me through
   that"). Flags name-dropping, numbers without baselines, credit ambiguity (team vs. me), and anything not
   supported by `FACTS.md` and the sources.
6. **Red flags and gaps.** Dates, overlaps, items marked "Present", a project that just started, GPA vs. common
   cutoffs, graduation date vs. eligibility windows, the internship record, location and work authorization,
   and sim-only evidence.
7. **Copy editor and typographer.** Grammar, tense (present for ongoing work), parallel structure,
   punctuation, units, widows and orphans, alignment, whitespace, hierarchy, consistent dates, typos.
