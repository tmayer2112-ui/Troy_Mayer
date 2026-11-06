"""Regenerate the module files you edit from solutions/ (maintainers only; you never need this).

    python make_skeletons.py            write m1_features.py ... m7_bayes.py
    python make_skeletons.py --check    exit 1 if any of them differs from what it would write

Every block between "# --- solution ---" and "# --- end ---" becomes `raise NotImplementedError`,
and the docstring's "REFERENCE SOLUTION" lines become the day's instructions. Everything else is
copied as is, so the provided code is identical in both versions.
"""
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent

MODULES = {
    "m1_features": (1, "about 10 lines", "features"),
    "m2_filter": (2, "about 18 lines", "filter"),
    "m3_frontend": (3, "about 13 lines", "boards"),
    "m4_lda": (4, "about 22 lines", "lda"),
    "m5_polarity": (5, "about 7 lines", "polarity"),
    "m6_speed_loop": (6, "about 16 lines", "loop"),
    "m7_bayes": (7, "about 23 lines", "latency"),
}

HEADER = re.compile(r"\. REFERENCE SOLUTION\.\n\nRead one function at a time \(the stuck rule\), close it, "
                    r"then type it yourself\.\n")
BLOCK = re.compile(r"^(?P<indent>[ \t]*)# --- solution ---\n.*?^[ \t]*# --- end ---\n", re.S | re.M)


def skeleton(name):
    n, lines, cmd = MODULES[name]
    src = (HERE / "solutions" / f"{name}.py").read_text()
    instructions = (f".\n\n  1. Do WORKSHEET.md, Module {n}, on paper first.\n"
                    f"  2. Fill in the TODOs below ({lines} in total).\n"
                    f"  3. Run:  python check.py {n}      then:  python run.py {cmd}\n")
    out, k = HEADER.subn(instructions, src, count=1)
    assert k == 1, f"{name}: solution header not found"
    out, k = BLOCK.subn(lambda m: f"{m['indent']}raise NotImplementedError\n", out)
    assert k > 0, f"{name}: no solution blocks"
    compile(out, name, "exec")
    return out


def main():
    stale = []
    for name in MODULES:
        path, text = HERE / f"{name}.py", skeleton(name)
        if "--check" in sys.argv:
            if not path.exists() or path.read_text() != text:
                stale.append(path.name)
        else:
            path.write_text(text)
            print(f"wrote {path.name}")
    if stale:
        print("out of date: " + ", ".join(stale))
        sys.exit(1)


if __name__ == "__main__":
    main()
