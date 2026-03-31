"""Racetrack (stadium) resonator above a horizontal bus."""

import math
import gdstk
from ..ports import Port


def PCellRacetrack(params, layers):
    """Racetrack resonator with point coupling to a straight bus.

    Geometry (above bus at y = 0):
      - Two semicircular turns of radius R
      - Two straight sections of length L_straight
      - Bus length defaults to 2·R if not specified

    The coupling gap is edge-to-edge between the bus top and the
    racetrack bottom.

    params:
        R          — bend radius               (default 50.0 µm)
        L_straight — straight section length    (default 30.0 µm)
        gap        — edge-to-edge gap           (default 0.2 µm)
        w_ring     — racetrack waveguide width  (default 0.5 µm)
        w_bus      — bus waveguide width        (default 0.5 µm)
        L_bus      — bus length, or None        (default: 2·R)
        name       — cell name                  (default "RACETRACK")
    """
    R          = float(params.get("R", 50.0))
    L_straight = float(params.get("L_straight", 30.0))
    gap        = float(params.get("gap", 0.2))
    w_ring     = float(params.get("w_ring", 0.5))
    w_bus      = float(params.get("w_bus", 0.5))
    L_bus      = params.get("L_bus", None)
    name       = str(params.get("name", "RACETRACK"))

    WG   = layers.get("WG", 1)
    TEXT = layers.get("TEXT", 100)

    if L_bus is None:
        L_bus = 2 * R
    L_bus = float(L_bus)

    cell = gdstk.Cell(name)

    # ---- Bus (horizontal at y = 0) ----
    x0, x1 = -L_bus / 2.0, L_bus / 2.0
    bus = gdstk.RobustPath((x0, 0.0), w_bus, layer=WG)
    bus.segment((x1, 0.0), width=w_bus)
    cell.add(bus)

    # ---- Racetrack (above bus) ----
    # Centerline y of the lower semicircle
    y_c = gap + 0.5 * (w_bus + w_ring)

    x_left  = -L_straight / 2.0
    x_right =  L_straight / 2.0

    # Start at bottom-left of the loop, go up → top semicircle → down → bottom semicircle
    rp = gdstk.RobustPath((x_left, y_c + R), w_ring, layer=WG)
    rp.segment((x_left, y_c + R + L_straight), width=w_ring)  # left straight (up)
    rp.turn(R, -math.pi)                                       # top semicircle (CW)
    rp.segment((x_right, y_c + R), width=w_ring)               # right straight (down)
    rp.turn(R, -math.pi)                                       # bottom semicircle (CW)
    cell.add(rp)

    cell.add(gdstk.Label(
        f"RACETRACK R={R} Ls={L_straight} gap={gap}",
        (0, y_c - R - 10), layer=TEXT,
    ))

    ports = {
        "W": Port("W", x0, 0.0, math.pi, w_bus, WG),
        "E": Port("E", x1, 0.0, 0.0,     w_bus, WG),
    }
    return cell, ports
