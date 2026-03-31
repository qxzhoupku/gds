"""Racetrack (stadium) resonator above a horizontal bus."""

import math
import gdstk
from ..ports import Port
from ..layer_map import resolve_wg_layer


def PCellRacetrack(params, layers):
    """Racetrack resonator with point coupling to a straight bus.

    Geometry (above bus at y = 0):
      - Two semicircular turns of radius R
      - Two straight sections of length L_straight
      - Bus length defaults to 2·R if not specified

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

    layer_bus  = resolve_wg_layer(w_bus,  layers)
    layer_ring = resolve_wg_layer(w_ring, layers)
    TEXT = layers.get("TEXT", 100)

    if L_bus is None:
        L_bus = 2 * R
    L_bus = float(L_bus)

    cell = gdstk.Cell(name)

    # ---- Bus (horizontal at y = 0) ----
    x0, x1 = -L_bus / 2.0, L_bus / 2.0
    bus = gdstk.RobustPath((x0, 0.0), w_bus, layer=layer_bus)
    bus.segment((x1, 0.0), width=w_bus)
    cell.add(bus)

    # ---- Racetrack (above bus) ----
    y_c = gap + 0.5 * (w_bus + w_ring)
    x_left  = -L_straight / 2.0
    x_right =  L_straight / 2.0

    rp = gdstk.RobustPath((x_left, y_c + R), w_ring, layer=layer_ring)
    rp.segment((x_left, y_c + R + L_straight), width=w_ring)
    rp.turn(R, -math.pi)
    rp.segment((x_right, y_c + R), width=w_ring)
    rp.turn(R, -math.pi)
    cell.add(rp)

    cell.add(gdstk.Label(
        f"RACETRACK R={R} Ls={L_straight} gap={gap}",
        (0, y_c - R - 10), layer=TEXT,
    ))

    ports = {
        "W": Port("W", x0, 0.0, math.pi, w_bus, layer_bus),
        "E": Port("E", x1, 0.0, 0.0,     w_bus, layer_bus),
    }
    return cell, ports
