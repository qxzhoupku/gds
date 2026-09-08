"""Route dispatch — turns a YAML ``routes`` entry into waveguide geometry.

A route entry is a single-key mapping naming the route kind::

    - straight:  { from: A.E, to: B.W }
    - manhattan: { from: A.E, to: B.W, r: 50 }
    - euler:     { from: A.E, to: B.W, Rmin: 80 }

``from``/``to`` are ``instance.port`` references resolved by the caller, which
keeps this module independent of how aliases are scoped (top-level routes and
macro-local routes resolve names differently but route identically).

Adding a route kind means adding one :data:`ROUTERS` entry; both the top-level
and macro route paths pick it up automatically.
"""

from __future__ import annotations

from typing import Callable, Dict

import gdstk

from .design import DesignError
from .layer_map import resolve_wg_layer
from .place import route_euler_bend, route_manhattan, route_straight
from .ports import Port


def route_layer(a: Port, b: Port, layers: dict) -> int:
    """GDS layer for a route joining *a* and *b*.

    The narrower of the two ports decides, because the route is drawn at the
    narrower width and it is the width that sets the e-beam dose recipe.
    """
    return resolve_wg_layer(min(a.width, b.width), layers)


def _straight(parent: gdstk.Cell, a: Port, b: Port, spec: dict, layer: int):
    route_straight(parent, a, b, layer=layer)


def _manhattan(parent: gdstk.Cell, a: Port, b: Port, spec: dict, layer: int):
    route_manhattan(parent, a, b, float(spec["r"]), layer=layer)


def _euler(parent: gdstk.Cell, a: Port, b: Port, spec: dict, layer: int):
    route_euler_bend(parent, a, b, float(spec["Rmin"]), layer=layer)


# route kind -> (handler, required extra spec keys)
ROUTERS: Dict[str, tuple[Callable, tuple[str, ...]]] = {
    "straight": (_straight, ()),
    "manhattan": (_manhattan, ("r",)),
    "euler": (_euler, ("Rmin",)),
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
    handler, required = ROUTERS[kind]

    missing = [k for k in ("from", "to", *required) if k not in spec]
    if missing:
        raise DesignError(
            f"Route {kind!r} is missing required key(s) {missing!r}: {spec!r}"
        )

    a = resolve(spec["from"])
    b = resolve(spec["to"])
    handler(parent, a, b, spec, route_layer(a, b, layers))


def apply_routes(parent: gdstk.Cell, entries, resolve, layers: dict) -> None:
    """Draw every route in *entries*, in order."""
    for entry in entries:
        apply_route(parent, entry, resolve, layers)
