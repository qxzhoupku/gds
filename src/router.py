"""Route dispatch — turns a YAML ``routes`` entry into waveguide geometry.

A route entry is a single-key mapping naming the route kind::

    - straight:     { from: A.E, to: B.W }
    - manhattan:    { from: A.E, to: B.W, r: 50 }
    - euler:        { from: A.E, to: B.W, Rmin: 80 }
    - clothoid:  { from: A.E, to: B.W, Rmin: 80 }

``from``/``to`` are ``instance.port`` references resolved by the caller, which
keeps this module independent of how aliases are scoped (top-level routes and
macro-local routes resolve names differently but route identically).

Adding a route kind means adding one :data:`ROUTERS` entry — the handler plus
the keys it requires and the keys it accepts — and both the top-level and macro
route paths pick it up automatically.
"""

from __future__ import annotations

import warnings
from typing import Callable, Dict

import gdstk

from . import clothoid
from .design import DesignError
from .layer_map import resolve_wg_layer
from .place import (
    route_euler_bend,
    route_clothoid,
    route_manhattan,
    route_straight,
)
from .ports import Port


def route_layer(a: Port, b: Port, layers: dict) -> int:
    """GDS layer for a route joining *a* and *b*.

    The narrower of the two ports decides, because the route is drawn at the
    narrower width and it is the width that sets the e-beam dose recipe.
    """
    return resolve_wg_layer(min(a.width, b.width), layers)


def _number(spec: dict, kind: str, key: str, default=None) -> float:
    """Read a numeric route key, reporting a bad value as a profile error.

    A bare ``float()`` on profile data lets ``Rmin: 80um`` or ``p: [0.4]``
    escape as a ``ValueError``/``TypeError`` traceback, and a malformed profile
    is a user error.
    """
    value = spec.get(key, default)
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise DesignError(
            f"Route {kind!r} key {key!r} must be a number, got {value!r}."
        ) from None
    return number


def _straight(parent: gdstk.Cell, a: Port, b: Port, spec: dict, layer: int):
    route_straight(parent, a, b, layer=layer)


def _manhattan(parent: gdstk.Cell, a: Port, b: Port, spec: dict, layer: int):
    route_manhattan(parent, a, b, _number(spec, "manhattan", "r"), layer=layer)


def _euler(parent: gdstk.Cell, a: Port, b: Port, spec: dict, layer: int):
    route_euler_bend(parent, a, b, _number(spec, "euler", "Rmin"), layer=layer)


def _clothoid(parent: gdstk.Cell, a: Port, b: Port, spec: dict, layer: int):
    route_clothoid(
        parent, a, b, _number(spec, "clothoid", "Rmin"), layer=layer,
        p=_number(spec, "clothoid", "p", clothoid.DEFAULT_P),
        tolerance=_number(spec, "clothoid", "tolerance",
                          clothoid.DEFAULT_TOLERANCE),
        Rmax=None if spec.get("Rmax") is None
        else _number(spec, "clothoid", "Rmax"),
        ref=f"{spec['from']} -> {spec['to']}",
    )


# route kind -> (handler, required extra spec keys, optional extra spec keys)
ROUTERS: Dict[str, tuple[Callable, tuple[str, ...], tuple[str, ...]]] = {
    "straight": (_straight, (), ()),
    "manhattan": (_manhattan, ("r",), ()),
    "euler": (_euler, ("Rmin",), ()),
    "clothoid": (_clothoid, ("Rmin",), ("p", "tolerance", "Rmax")),
}


def apply_route(parent: gdstk.Cell, entry: dict, resolve, layers: dict) -> None:
    """Draw one route *entry* into *parent*.

    *resolve* maps an ``"instance.port"`` string to a placed :class:`Port`.
    """
    kinds = [k for k in entry if k in ROUTERS]
    if not kinds:
        raise DesignError(
            f"Route entry {entry!r} names no known route kind. "
            f"Available: {sorted(ROUTERS)}"
        )
    if len(kinds) > 1:
        raise DesignError(
            f"Route entry names several kinds {kinds!r}; use one per entry."
        )

    kind = kinds[0]
    spec = entry[kind] or {}
    handler, required, optional = ROUTERS[kind]

    missing = [k for k in ("from", "to", *required) if k not in spec]
    if missing:
        raise DesignError(
            f"Route {kind!r} is missing required key(s) {missing!r}: {spec!r}"
        )

    # A misspelled optional key would otherwise be dropped in silence and the
    # route drawn with the default — a different waveguide than was asked for.
    # Warned rather than raised, matching src.design.warn_unknown_keys.
    unknown = sorted(set(spec) - {"from", "to", *required, *optional})
    if unknown:
        warnings.warn(
            f"Route {kind!r} has unknown key(s) {unknown!r} — ignored. "
            f"Known keys: {sorted({'from', 'to', *required, *optional})}.",
            stacklevel=2,
        )

    a = resolve(spec["from"])
    b = resolve(spec["to"])
    handler(parent, a, b, spec, route_layer(a, b, layers))


def apply_routes(parent: gdstk.Cell, entries, resolve, layers: dict) -> None:
    """Draw every route in *entries*, in order."""
    for entry in entries:
        apply_route(parent, entry, resolve, layers)
