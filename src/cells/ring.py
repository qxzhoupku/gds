"""Point-coupled ring resonator above a horizontal bus."""

import math
import gdstk
from ..ports import Port


def PCellRingCoupler(params, layers):
    """Ring resonator with point coupling to a straight bus.

    params:
        R      — ring radius          (default 20.0 µm)
        gap    — edge-to-edge gap     (default 0.25 µm)
        w_ring — ring waveguide width (default 0.5 µm)
        w_bus  — bus waveguide width  (default 0.5 µm)
        L_bus  — bus length, or None  (default: 2·R)
        name   — cell name            (default "RING")
    """
    R      = float(params.get("R", 20.0))
    gap    = float(params.get("gap", 0.25))
    w_ring = float(params.get("w_ring", 0.5))
    w_bus  = float(params.get("w_bus", 0.5))
    L_bus  = params.get("L_bus", None)
    name   = str(params.get("name", "RING"))

    WG   = layers.get("WG", 1)
    TEXT = layers.get("TEXT", 100)

    if L_bus is None:
        L_bus = 2 * R
    L_bus = float(L_bus)

    cell = gdstk.Cell(name)

    # Bus waveguide at y = 0
    x0, x1 = -L_bus / 2, L_bus / 2
    bus = gdstk.RobustPath((x0, 0.0), w_bus, layer=WG)
    bus.segment((x1, 0.0), width=w_bus)
    cell.add(bus)

    # Ring centred above bus
    y_c = gap + 0.5 * w_bus + R
    ring = gdstk.ellipse((0.0, y_c), radius=R, inner_radius=R - w_ring, layer=WG)
    cell.add(ring)

    cell.add(gdstk.Label(f"RING R={R} gap={gap}", (0, -R - 10), layer=TEXT))

    ports = {
        "W": Port("W", x0, 0.0, math.pi, w_bus, WG),
        "E": Port("E", x1, 0.0, 0.0,     w_bus, WG),
    }
    return cell, ports
