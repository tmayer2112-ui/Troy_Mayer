"""Every number the portfolio page and résumé state about the decoder, checked against the files that produced it.

  PYTHONPATH=. python3 scripts/check_claims.py

For each entry in claims.json: read the value from the named results file, compare with the claimed
value, and print where the evidence lives. Then scan the decoder text on the portfolio page and in the
résumé for percentages, frequencies, voltages and day counts that no entry backs. Exit code 1 on any
mismatch, so a stale number can't slip through. Same scheme as quadruped-eskf/claims.json.
"""
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SITE = ROOT.parent / "index.html"
RESUME = ROOT.parent / "Troy_Mayer_Resume.pdf"


def get(d, path):
    for k in path.split("/"):
        d = d[int(k)] if isinstance(d, list) else d[k]
    return d


def check_claims():
    claims = json.loads((ROOT / "claims.json").read_text())
    ok = True
    for c in claims["claims"]:
        src = ROOT / c["source"]
        if not src.exists():
            print(f"[MISSING] {c['id']}: {c['source']} not found")
            ok = False
            continue
        actual = get(json.loads(src.read_text()), c["path"])
        if "reduce" in c:
            actual = {"max": max, "min": min}[c["reduce"]](actual)
        if "equals" in c:
            good = actual == c["equals"]
        elif "max" in c:
            good = actual <= c["max"]
        elif "min" in c:
            good = actual >= c["min"]
        else:
            good = abs(actual - c["value"]) <= c.get("abs_tol", 0.0)
        ok &= good
        shown = f"{actual:.6g}" if isinstance(actual, float) else str(actual)
        how = f"{c['reduce']}(…)" if "reduce" in c else ""
        print(f"[{'OK' if good else 'MISMATCH'}] {c['id']}: \"{c['text']}\" <- {c['source']}:{c['path']} {how}= {shown}")
        print(f"         evidence: {', '.join(c['evidence'])}")
    return ok, claims


def scan_documents(claims):
    """Look for numbers in the published decoder text that no claim backs."""
    allowed = {s.lower() for s in claims["published_strings"]}
    texts = {}
    if SITE.exists():
        html = SITE.read_text()
        blocks = [re.search(p, html, re.S) for p in (r"<h4>Decoder</h4>(.*?)</div>",
                                                     r"<h4>v2 EMG Board</h4>(.*?)</div>",
                                                     r"<figcaption>The machine-learning result(.*?)</figcaption>")]
        texts["index.html"] = re.sub(r"<[^>]+>", " ", " ".join(m.group(1) for m in blocks if m))
    if RESUME.exists():
        try:
            import pypdf
            t = " ".join(p.extract_text() for p in pypdf.PdfReader(str(RESUME)).pages)
            m = re.search(r"gesture decoding.*?emulation", re.sub(r"\s+", " ", t))
            texts["Troy_Mayer_Resume.pdf"] = m.group(0) if m else ""
        except Exception as e:  # pypdf missing or unreadable PDF
            print(f"[SKIP] résumé PDF not scanned ({e.__class__.__name__}); check it by hand")
    pat = re.compile(r"\d+(?:\.\d+)?(?:\s*[–-]\s*\d+(?:\.\d+)?)?\s*(?:%|Hz|V\b|days?\b|-class)")
    ok = True
    for name, text in texts.items():
        for s in sorted({re.sub(r"\s+", " ", x.strip()) for x in pat.findall(text)}):
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
