"""Regenerate the module files you edit from solutions/ (maintainers only; you never need this).

    python make_skeletons.py            write m1_features.py ... m11_mujoco.py and cpp/velocity_pi.hpp
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
    "m2_filter": (2, "about 22 lines", "filter"),
    "m3_frontend": (3, "about 14 lines", "boards"),
    "m4_lda": (4, "about 27 lines", "lda"),
    "m5_polarity": (5, "about 7 lines", "polarity"),
    "m6_speed_loop": (6, "about 18 lines", "loop"),
    "m7_bayes": (7, "about 23 lines", "latency"),
    "m8_safety": (8, "about 13 lines", "safety"),
    "m11_mujoco": (11, "about 3 lines", "mujoco"),
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


CPP_HEADER = " REFERENCE SOLUTION.\n"
CPP_INSTRUCTIONS = ("\n//   1. Do WORKSHEET.md, Module 9, on paper first.\n"
                    "//   2. Fill in the three TODOs (about 14 lines), translating your Python m6_speed_loop.py.\n"
                    "//   3. Build and run:  cmake --build build && ./build/rebuild_tests\n")
CPP_BLOCK = re.compile(r"^(?P<indent>[ \t]*)// --- solution ---\n.*?^[ \t]*// --- end ---\n", re.S | re.M)


def cpp_skeleton():
    """cpp/velocity_pi.hpp from cpp/solutions/velocity_pi.hpp: each solution block throws instead."""
    src = (HERE / "cpp" / "solutions" / "velocity_pi.hpp").read_text()
    first, rest = src.split("\n", 1)
    assert first.endswith(CPP_HEADER.strip()), "C++ solution header not found"
    out = first[: -len(CPP_HEADER.strip())].rstrip() + CPP_INSTRUCTIONS + rest
    out, k = CPP_BLOCK.subn(
        lambda m: f'{m["indent"]}throw std::logic_error("TODO (Module 9): not written yet");\n', out)
    assert k == 3, f"expected 3 solution blocks, found {k}"
    return out


def main():
    stale = []
    targets = [(HERE / f"{name}.py", skeleton(name)) for name in MODULES]
    targets.append((HERE / "cpp" / "velocity_pi.hpp", cpp_skeleton()))
    for path, text in targets:
        if "--check" in sys.argv:
            if not path.exists() or path.read_text() != text:
                stale.append(str(path.relative_to(HERE)))
        else:
            path.write_text(text)
            print(f"wrote {path.relative_to(HERE)}")
    if stale:
        print("out of date: " + ", ".join(stale))
        sys.exit(1)


if __name__ == "__main__":
    main()
