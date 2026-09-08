"""Compatibility harness: prove a code change does not alter any layout.

The design profiles under ``designs/profiles`` are the real test suite of this
repo — several of them have been fabricated, so their GDS output must not
drift when the builder is refactored.  This tool builds every profile and
records a canonical digest of the geometry, then compares two such snapshots.

    # before changing anything
    python tools/gdscheck.py snapshot -o /tmp/before.json

    # ... edit the builder ...

    python tools/gdscheck.py snapshot -o /tmp/after.json
    python tools/gdscheck.py compare /tmp/before.json /tmp/after.json

``check`` does both halves in one step against the committed reference:

    python tools/gdscheck.py check

Builds are redirected into a temporary directory via ``build.py --out``, so
running this never overwrites anything in ``out/``.

Why digest geometry instead of hashing the file?  ``gdstk`` stamps the current
wall-clock time into every GDS it writes, so two identical builds differ in
bytes if they straddle a second boundary.  The digest covers everything that
reaches the mask — polygon layer/datatype/vertices and label
layer/texttype/text/origin — quantised to the database grid, plus the emission
order and cell names, which together determine the written bytes.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
import tempfile
from pathlib import Path

import gdstk

REPO = Path(__file__).resolve().parent.parent
PROFILE_ROOT = REPO / "designs" / "profiles"
REFERENCE = Path(__file__).resolve().parent / "reference.json"

# 9 decimals on a micron value is sub-picometre — far below the 1 nm database
# grid, so float noise never registers as a difference.
_QUANT = 9


def _q(v: float) -> str:
    return f"{v:.{_QUANT}f}"


def _sha(records) -> str:
    return hashlib.sha256("\n".join(records).encode()).hexdigest()


def cell_digest(cell: gdstk.Cell) -> dict:
    polygons = [
        f"P|{p.layer}|{p.datatype}|" + ";".join(f"{_q(x)},{_q(y)}" for x, y in p.points)
        for p in cell.get_polygons(depth=None)
    ]
    labels = [
        f"L|{l.layer}|{l.texttype}|{l.text}|{_q(l.origin[0])},{_q(l.origin[1])}"
        for l in cell.get_labels(depth=None)
    ]
    return {
        "name": cell.name,
        "polygons": len(polygons),
        "labels": len(labels),
        # Order-independent — the chip itself.
        "sha256": _sha(sorted(polygons) + sorted(labels)),
        # Order-dependent — also pins the byte layout of the written file.
        "sha256_ordered": _sha(polygons + labels),
    }


def library_digest(path: Path) -> dict:
    lib = gdstk.read_gds(str(path))
    tops = sorted(lib.top_level(), key=lambda c: c.name)
    cells = [cell_digest(c) for c in tops]
    return {
        "unit": lib.unit,
        "precision": lib.precision,
        "cell_count": len(lib.cells),
        "cell_names": [c.name for c in lib.cells],
        "top_cells": cells,
        "sha256": _sha([f"{c['name']}:{c['sha256']}" for c in cells]),
        "sha256_ordered": _sha(
            [f"{c['name']}:{c['sha256_ordered']}" for c in cells]
        ),
    }


def profiles() -> list[Path]:
    return sorted(PROFILE_ROOT.rglob("*.yaml"))


def snapshot(verbose: bool = True) -> dict:
    """Build every profile into a scratch directory and digest the results."""
    results: dict[str, dict] = {}
    with tempfile.TemporaryDirectory(prefix="gdscheck-") as tmp:
        tmpdir = Path(tmp)
        for i, profile in enumerate(profiles(), 1):
            rel = profile.relative_to(REPO).as_posix()
            target = tmpdir / f"{i:03d}.gds"
            proc = subprocess.run(
                [sys.executable, "build.py", rel, "--out", str(target)],
                cwd=REPO, capture_output=True, text=True, timeout=1800,
            )
            entry: dict = {"returncode": proc.returncode}
            if proc.returncode != 0:
                # Keep only the final line: a traceback embeds absolute paths
                # that differ between checkouts and would create false diffs.
                tail = [l for l in proc.stderr.strip().splitlines() if l.strip()]
                entry["error"] = tail[-1] if tail else "<no stderr>"
            elif not target.exists():
                entry["error"] = "build reported success but wrote no file"
                entry["returncode"] = 1
            else:
                entry["bytes"] = target.stat().st_size
                entry["digest"] = library_digest(target)
            results[rel] = entry
            if verbose:
                mark = "ok  " if entry["returncode"] == 0 else "FAIL"
                print(f"  [{i:2d}/{len(profiles())}] {mark} {rel}")
    return results


def compare(base: dict, cand: dict) -> tuple[int, list[str], list[str]]:
    """Return (identical_count, breaks, notes)."""
    breaks: list[str] = []
    notes: list[str] = []

    for key in sorted(set(base) - set(cand)):
        breaks.append(f"profile missing from new snapshot: {key}")
    for key in sorted(set(cand) - set(base)):
        notes.append(f"new profile not in reference: {key}")

    identical = 0
    for key in sorted(set(base) & set(cand)):
        b, c = base[key], cand[key]
        b_ok, c_ok = b["returncode"] == 0, c["returncode"] == 0

        if b_ok and not c_ok:
            breaks.append(f"REGRESSION {key}: built before, now fails "
                          f"({c.get('error')})")
            continue
        if not b_ok and c_ok:
            notes.append(f"FIXED {key}: previously failed ({b.get('error')})")
            continue
        if not b_ok:
            if b.get("error") != c.get("error"):
                notes.append(f"still failing, new error {key}: "
                             f"{b.get('error')} -> {c.get('error')}")
            continue

        bd, cd = b["digest"], c["digest"]
        if bd["sha256"] != cd["sha256"]:
            breaks.append(f"GEOMETRY CHANGED {key}")
            for bc, cc in zip(bd["top_cells"], cd["top_cells"]):
                if bc["sha256"] != cc["sha256"]:
                    breaks.append(
                        f"    cell {bc['name']}: polygons "
                        f"{bc['polygons']}->{cc['polygons']}, "
                        f"labels {bc['labels']}->{cc['labels']}"
                    )
        elif bd["sha256_ordered"] != cd["sha256_ordered"]:
            breaks.append(f"EMISSION ORDER CHANGED {key} "
                          "(same geometry, different bytes)")
        elif bd["cell_names"] != cd["cell_names"]:
            breaks.append(f"CELL NAMES CHANGED {key}")
        elif b["bytes"] != c["bytes"]:
            breaks.append(f"SIZE CHANGED {key}: {b['bytes']} -> {c['bytes']}")
        else:
            identical += 1

    return identical, breaks, notes


def _report(identical: int, breaks: list[str], notes: list[str]) -> int:
    print(f"\n{identical} profiles identical")
    for n in notes:
        print(f"  note: {n}")
    if breaks:
        print(f"\n{len(breaks)} COMPATIBILITY BREAK(S):")
        for b in breaks:
            print(f"  {b}")
        return 1
    print("OK - no compatibility break.")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_snap = sub.add_parser("snapshot", help="build every profile and digest it")
    p_snap.add_argument("-o", "--out", required=True, help="snapshot JSON path")
    p_snap.add_argument("-q", "--quiet", action="store_true")

    p_cmp = sub.add_parser("compare", help="compare two snapshots")
    p_cmp.add_argument("baseline")
    p_cmp.add_argument("candidate")

    p_chk = sub.add_parser(
        "check", help="snapshot now and compare against tools/reference.json")
    p_chk.add_argument("-q", "--quiet", action="store_true")

    p_upd = sub.add_parser(
        "update-reference",
        help="overwrite tools/reference.json with the current output")

    args = parser.parse_args(argv)

    if args.cmd == "snapshot":
        data = snapshot(verbose=not args.quiet)
        Path(args.out).write_text(json.dumps(data, indent=2, sort_keys=True))
        ok = sum(1 for v in data.values() if v["returncode"] == 0)
        print(f"{ok}/{len(data)} profiles built -> {args.out}")
        return 0

    if args.cmd == "compare":
        base = json.loads(Path(args.baseline).read_text())
        cand = json.loads(Path(args.candidate).read_text())
        return _report(*compare(base, cand))

    if args.cmd == "check":
        if not REFERENCE.exists():
            print(f"error: no reference at {REFERENCE}. Create one with:\n"
                  f"  python tools/gdscheck.py update-reference",
                  file=sys.stderr)
            return 2
        base = json.loads(REFERENCE.read_text())
        cand = snapshot(verbose=not args.quiet)
        return _report(*compare(base, cand))

    if args.cmd == "update-reference":
        data = snapshot()
        REFERENCE.write_text(json.dumps(data, indent=2, sort_keys=True))
        ok = sum(1 for v in data.values() if v["returncode"] == 0)
        print(f"{ok}/{len(data)} profiles built -> {REFERENCE}")
        return 0

    return 2


if __name__ == "__main__":
    sys.exit(main())
