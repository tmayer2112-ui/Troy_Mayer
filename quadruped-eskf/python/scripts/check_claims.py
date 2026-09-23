"""Every number the resume / portfolio states about this project, checked against the files that produced it.

  PYTHONPATH=python python3 python/scripts/check_claims.py

For each entry in claims.json: read the value from the named results file, compare with the claimed
value, and print where the evidence lives (log + figure). Then scan the portfolio page and the resume
PDF for "<number> m/s" / "<number> kHz" / "<number>-second" near this project and flag anything that
doesn't match a claim. Exit code 1 on any mismatch, so a stale number can't slip through.
"""
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SITE = ROOT.parent / "index.html"
RESUME = ROOT.parent / "Troy_Mayer_Resume.pdf"


def get(d, path):
    for k in path.split("."):
        d = d[int(k)] if isinstance(d, list) else d[k]
    return d


def check_claims():
    claims = json.loads((ROOT / "claims.json").read_text())
    ok = True
    for c in claims["claims"]:
        src = ROOT / c["source"]
        if not src.exists():
            print(f"[MISSING] {c['id']}: {c['source']} not found - run `make reproduce`")
            ok = False
            continue
        data = json.loads(src.read_text())
        actual = get(data, c["path"])
        if "row" in c:   # make sure an index-based path still points at the row the claim is about
            row = get(data, c["row"])
            bad = {k: row.get(k) for k, v in c["row_fields"].items() if row.get(k) != v}
            if bad:
                print(f"[MISMATCH] {c['id']}: {c['row']} is no longer the row the claim describes: {bad}")
                ok = False
                continue
        if "equals" in c:
            good = actual == c["equals"]
        elif "max" in c:
            good = actual <= c["max"]
        elif "min" in c:
            good = actual >= c["min"]
        else:
            good = abs(actual - c["value"]) <= c.get("abs_tol", 0.0) + c.get("rel_tol", 0.0) * abs(c["value"])
        ok &= good
        shown = f"{actual:.6g}" if isinstance(actual, float) else str(actual)
        print(f"[{'OK' if good else 'MISMATCH'}] {c['id']}: claim \"{c['text']}\" <- {c['source']}:{c['path']} = {shown}")
        print(f"         evidence: {', '.join(c['evidence'])}")
    return ok, claims


def scan_documents(claims):
    """Look for numbers in the published documents that no claim backs."""
    allowed = {s.lower() for s in claims["published_strings"]}
    texts = {}
    if SITE.exists():
        html = SITE.read_text()
        m = re.search(r'id="state-estimation".*?</section>', html, re.S)
        block = (m.group(0) if m else "") + " ".join(re.findall(r"[^<]{0,200}(?:EKF|state estimat)[^<]{0,300}", html, re.I))
        texts["index.html"] = re.sub(r"<[^>]+>", " ", block).replace("&nbsp;", " ")
    if RESUME.exists():
        try:
            import pypdf
            t = " ".join(p.extract_text() for p in pypdf.PdfReader(str(RESUME)).pages)
            m = re.search(r"Legged Robot State Estimation(.*?)(SKILLS|$)", t, re.S)
            texts["Troy_Mayer_Resume.pdf"] = m.group(1) if m else ""
        except Exception as e:  # pypdf missing or unreadable PDF
            print(f"[SKIP] resume PDF not scanned ({e.__class__.__name__}); check it by hand")
    pat = re.compile(r"\d+(?:\.\d+)?\s*(?:m/s|kHz|-second|\s?s\b|µs|us\b|%)", re.I)
    ok = True
    for name, text in texts.items():
        found = sorted({re.sub(r"\s+", " ", x.strip()) for x in pat.findall(text)})
        for s in found:
            if s.lower() not in allowed:
                print(f"[STALE?] {name}: \"{s}\" is not backed by claims.json")
                ok = False
    if ok:
        print(f"[OK] no unbacked numbers in: {', '.join(texts) or 'nothing scanned'}")
    return ok


def main():
    ok1, claims = check_claims()
    ok2 = scan_documents(claims)
    sys.exit(0 if ok1 and ok2 else 1)


if __name__ == "__main__":
    main()
