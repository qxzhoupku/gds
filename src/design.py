"""Design-profile loading: YAML inheritance, layer resolution, validation.

A *design profile* is a YAML file describing one chip.  This module turns it
into a plain ``dict`` with inheritance resolved, and provides the small
accessors the builder needs so that key names and defaults live in exactly
one place.

Recognised top-level keys::

    extends     path to another profile to inherit from (see resolve_design)
    defaults    layers / width_layers / grid_um
    chip        name (top cell) and out (GDS path)
    instances   name -> {type, params}
    placement   ordered list of {place: ...} / {connect: ...} steps
    routes      ordered list of {straight|manhattan|euler: ...}
    macros      reusable named placement+route groups
    blocks      instantiations of macros at an offset

Keys outside this set are reported by :func:`warn_unknown_keys` rather than
ignored silently, so a typo surfaces instead of quietly dropping geometry.
"""

from __future__ import annotations

import os
import warnings
from typing import Any, Dict

import yaml


class DesignError(Exception):
    """Raised when a design profile is malformed or internally inconsistent."""


# Top-level keys the builder understands.
KNOWN_TOP_LEVEL = frozenset({
    "extends", "defaults", "chip", "instances",
    "placement", "routes", "macros", "blocks",
})

# ``chip`` sub-keys.  ``die`` is accepted and deliberately unused: chip
# dimensions stay in the profile as documentation, because BEAMER must receive
# device geometry only (a die outline would be written as a real exposure).
KNOWN_CHIP_KEYS = frozenset({"name", "out", "die"})

DEFAULT_LAYERS: Dict[str, Any] = {"WG": 1, "PORT": 99, "TEXT": 100}
DEFAULT_OUT = "out/chip_demo.gds"
DEFAULT_TOP_NAME = "TOP"

# Maximum ``extends`` chain length, purely a guard against a cycle.
_MAX_EXTENDS_DEPTH = 16


def load_yaml(path: str) -> dict:
    """Parse a YAML file, returning ``{}`` for an empty document."""
    try:
        with open(path, "r", encoding="utf-8") as f:
            return yaml.safe_load(f) or {}
    except FileNotFoundError as exc:
        raise DesignError(f"Profile not found: {path}") from exc
    except yaml.YAMLError as exc:
        raise DesignError(f"Invalid YAML in {path}: {exc}") from exc


def deep_update(base: dict, upd: dict) -> dict:
    """Recursively merge *upd* into *base* (mutates and returns *base*)."""
    for k, v in upd.items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            deep_update(base[k], v)
        else:
            base[k] = v
    return base


def resolve_design(profile_path: str) -> dict:
    """Load a profile and resolve its ``extends`` chain.

    ``extends`` is followed transitively — a profile may extend a profile that
    itself extends another — with the *child* always winning.  Paths are
    resolved relative to the current working directory first (which is how
    every existing profile is written, e.g. ``designs/base.yaml``) and then
    relative to the extending file, so profiles remain movable.
    """
    chain: list[dict] = []
    seen: list[str] = []
    path = profile_path

    for _ in range(_MAX_EXTENDS_DEPTH):
        real = os.path.realpath(path)
        if real in seen:
            cycle = " -> ".join(seen + [real])
            raise DesignError(f"Circular 'extends' chain: {cycle}")
        seen.append(real)

        cfg = load_yaml(path)
        chain.append(cfg)
        parent = cfg.get("extends")
        if not parent:
            break
        path = _resolve_extends_path(parent, path)
    else:
        raise DesignError(
            f"'extends' chain deeper than {_MAX_EXTENDS_DEPTH} starting at "
            f"{profile_path}."
        )

    # Merge oldest ancestor first so the original profile overrides everything.
    merged: dict = {}
    for cfg in reversed(chain):
        deep_update(merged, {k: v for k, v in cfg.items() if k != "extends"})
    return merged


def _resolve_extends_path(parent: str, child_path: str) -> str:
    """Resolve an ``extends`` target, preferring the historical CWD-relative
    interpretation and falling back to a path relative to the child profile."""
    if os.path.isfile(parent):
        return parent
    sibling = os.path.join(os.path.dirname(os.path.abspath(child_path)), parent)
    if os.path.isfile(sibling):
        return sibling
    # Neither exists — report the originally requested path.
    return parent


def warn_unknown_keys(cfg: dict) -> None:
    """Warn about top-level and ``chip`` keys the builder does not read.

    A key whose name starts with ``_`` is skipped: the sweep profiles park
    YAML anchors under ``_templates`` so they can be referenced with ``*``
    aliases further down, and that holder is deliberately not a builder
    section.  Anything else is more likely a typo than a convention, and a
    silently dropped section means silently missing geometry.
    """
    for key in sorted(set(cfg) - KNOWN_TOP_LEVEL):
        if str(key).startswith("_"):
            continue
        warnings.warn(
            f"Unknown top-level key {key!r} in design profile — ignored. "
            f"Known keys: {sorted(KNOWN_TOP_LEVEL)}. "
            f"Prefix a key with '_' to mark it as intentionally unused.",
            stacklevel=2,
        )
    chip = cfg.get("chip") or {}
    if isinstance(chip, dict):
        for key in sorted(set(chip) - KNOWN_CHIP_KEYS):
            warnings.warn(
                f"Unknown 'chip' key {key!r} — ignored.", stacklevel=2,
            )


def resolve_layers(cfg: dict) -> dict:
    """Build the ``layers`` dict passed to every PCell factory.

    ``width_layers`` lives under ``defaults`` in YAML but
    :func:`src.layer_map.resolve_wg_layer` looks for it inside ``layers``, so
    it is folded in here.  The result is a fresh dict; the profile is not
    mutated.
    """
    defaults = cfg.get("defaults") or {}
    layers = dict(defaults.get("layers") or DEFAULT_LAYERS)
    if "width_layers" in defaults:
        layers["width_layers"] = defaults["width_layers"]
    return layers


def resolve_output_path(cfg: dict) -> str:
    """Return the GDS output path declared by the profile."""
    return (cfg.get("chip") or {}).get("out") or DEFAULT_OUT


def resolve_top_name(cfg: dict) -> str:
    """Return the name for the top-level cell."""
    return str((cfg.get("chip") or {}).get("name") or DEFAULT_TOP_NAME)


def steps(cfg: dict, key: str) -> list:
    """Return list-valued section *key*, treating absent and null alike.

    A profile whose every entry is commented out parses as ``key: None``
    rather than an empty list, so ``cfg.get(key, [])`` is not enough.
    """
    return cfg.get(key) or []
