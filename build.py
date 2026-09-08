"""YAML-driven photonic layout builder.

Usage::

    python build.py designs/profiles/my_design.yaml
    python build.py my_design.yaml -o out/somewhere_else.gds
    python build.py --list-types

Reads a design profile, instantiates parametric cells, places and connects
them via ports, applies routing, and writes a GDS-II file.

This file is the command-line front end only.  The engine lives in
:mod:`src.builder` and can be driven directly from Python::

    from src.design import resolve_design, resolve_layers, resolve_top_name
    from src.builder import build_library
    import src.catalog  # registers the standard components

    cfg = resolve_design("designs/profiles/my_design.yaml")
    lib = build_library(cfg, resolve_layers(cfg), resolve_top_name(cfg))
"""

from __future__ import annotations

import argparse
import os
import sys

import src.catalog  # noqa: F401  — importing registers the standard catalog
from src import registry
from src.builder import build_library
from src.design import (
    DesignError,
    resolve_design,
    resolve_layers,
    resolve_output_path,
    resolve_top_name,
    warn_unknown_keys,
)

DEFAULT_PROFILE = "designs/profiles/playground.yaml"


def build(profile: str, out_path: str | None = None,
          dry_run: bool = False) -> str:
    """Build *profile* and write the GDS.  Returns the path written.

    With *dry_run* the design is assembled and validated but nothing is
    written — useful for checking a profile before a long sweep.
    """
    cfg = resolve_design(profile)
    warn_unknown_keys(cfg)

    layers = resolve_layers(cfg)
    target = out_path or resolve_output_path(cfg)
    lib = build_library(cfg, layers, resolve_top_name(cfg))

    if dry_run:
        return target

    parent = os.path.dirname(target)
    if parent:
        os.makedirs(parent, exist_ok=True)
    lib.write_gds(target)
    return target


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Build a GDS-II layout from a YAML design profile.",
    )
    parser.add_argument(
        "profile", nargs="?", default=DEFAULT_PROFILE,
        help=f"design profile to build (default: {DEFAULT_PROFILE})",
    )
    parser.add_argument(
        "-o", "--out", default=None,
        help="override the profile's chip.out path",
    )
    parser.add_argument(
        "-n", "--dry-run", action="store_true",
        help="assemble and validate the design without writing a file",
    )
    parser.add_argument(
        "--list-types", action="store_true",
        help="list the available instance 'type' keys and exit",
    )
    args = parser.parse_args(argv)

    if args.list_types:
        for type_key in registry.available():
            print(type_key)
        return 0

    try:
        written = build(args.profile, args.out, args.dry_run)
    except DesignError as exc:
        # A malformed profile is a user error, not a crash: report it plainly.
        print(f"error: {exc}", file=sys.stderr)
        return 1

    print(f"Would write {written}" if args.dry_run else f"Wrote {written}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
