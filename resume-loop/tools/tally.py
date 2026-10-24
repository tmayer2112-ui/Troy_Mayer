#!/usr/bin/env python3
"""Unblind and tally a screening round.

    python3 tally.py screen1

Interviews = recruiter advances + hiring-manager interviews (2 per listing). A version that
failed the truth audit scores zero, per the rule.
"""
import json, sys
from pathlib import Path
from statistics import mean

rnd = Path(sys.argv[1])
key = json.loads((rnd / "_key/key.json").read_text())
audit_path = rnd / "truth_audit.json"
audit = json.loads(audit_path.read_text()) if audit_path.exists() else {}
rows = {L: {"adv": 0, "int": 0, "s": [], "h": [], "rank": []} for L in key}
listings = sorted(rnd.glob("results/L*.json"))
for f in listings:
    d = json.loads(f.read_text())
    for L, r in d["results"].items():
        rows[L]["adv"] += bool(r["screen_advance"]); rows[L]["int"] += bool(r["hm_interview"])
        rows[L]["s"].append(r["screen_score"]); rows[L]["h"].append(r["hm_score"])
    for i, L in enumerate(d["rank"]):
        rows[L]["rank"].append(i + 1)
n = len(listings)
out = []
for L, r in rows.items():
    verdict = audit.get(L, {}).get("verdict", "n/a")
    total = r["adv"] + r["int"]
    out.append({"version": key[L], "letter": L, "truth": verdict,
                "interviews": 0 if verdict == "FAIL" else total, "raw_interviews": total,
                "recruiter_advances": r["adv"], "hm_interviews": r["int"],
                "screen_avg": round(mean(r["s"]), 2), "hm_avg": round(mean(r["h"]), 2),
                "avg_rank": round(mean(r["rank"]), 2)})
out.sort(key=lambda x: (-x["interviews"], x["avg_rank"]))
print(f"{n} listings x 2 readers = {2*n} possible interviews per version\n")
print(f"{'version':14} {'truth':5} {'score':>5} {'raw':>4} {'rec':>4} {'hm':>4} {'scr avg':>7} {'hm avg':>6} {'rank':>5}")
for o in out:
    print(f"{o['version']:14} {o['truth']:5} {o['interviews']:>5} {o['raw_interviews']:>4} {o['recruiter_advances']:>4} "
          f"{o['hm_interviews']:>4} {o['screen_avg']:>7} {o['hm_avg']:>6} {o['avg_rank']:>5}")
(rnd / "tally.json").write_text(json.dumps(out, indent=1))
