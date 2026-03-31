"""Waveguide crossing (WX) — 4-port multimode interference cross."""

import math
import gdstk
from ..ports import Port

# Extra straight extension beyond the taper region on each arm (µm).
# Provides clearance for routing and avoids abrupt path terminations.
_ARM_EXTENSION = 1.0


def PCellWx(params, layers):
    """Waveguide crossing with tapered multimode region.

    params:
        WM    — multimode region width   (default 1.6 µm)
        LM    — multimode region length  (default 8.0 µm)
        LT    — taper length             (default 10.0 µm)
        w_in  — single-mode input width  (default 0.5 µm)
        name  — cell name                (default "WX")
    """
    WM   = float(params.get("WM", 1.6))
    LM   = float(params.get("LM", 8.0))
    LT   = float(params.get("LT", 10.0))
    w_in = float(params.get("w_in", 0.5))
    name = str(params.get("name", "WX"))

    WG   = layers.get("WG", 1)
    TEXT = layers.get("TEXT", 100)

    cell = gdstk.Cell(name)

    half_arm = LM / 2 + LT + _ARM_EXTENSION

    # Horizontal arm
    left, right = -half_arm, half_arm
    rp_h = gdstk.RobustPath((left, 0.0), w_in, layer=WG)
    rp_h.segment((-LM / 2 - LT, 0.0), width=w_in)
    rp_h.segment((-LM / 2,      0.0), width=WM)
    rp_h.segment(( LM / 2,      0.0), width=WM)
    rp_h.segment(( LM / 2 + LT, 0.0), width=w_in)
    rp_h.segment((right,         0.0), width=w_in)
    cell.add(rp_h)

    # Vertical arm
    bottom, top = -half_arm, half_arm
    rp_v = gdstk.RobustPath((0.0, bottom), w_in, layer=WG)
    rp_v.segment((0.0, -LM / 2 - LT), width=w_in)
    rp_v.segment((0.0, -LM / 2),      width=WM)
    rp_v.segment((0.0,  LM / 2),      width=WM)
    rp_v.segment((0.0,  LM / 2 + LT), width=w_in)
    rp_v.segment((0.0,  top),         width=w_in)
    cell.add(rp_v)

    cell.add(gdstk.Label(
        f"WX WM={WM} LM={LM} LT={LT} w_in={w_in}", (0, -LM / 2 - 6), layer=TEXT
    ))

    ports = {
        "W": Port("W", left,  0.0,    math.pi,       w_in, WG),
        "E": Port("E", right, 0.0,    0.0,           w_in, WG),
        "S": Port("S", 0.0,   bottom, 3 * math.pi / 2, w_in, WG),
        "N": Port("N", 0.0,   top,    math.pi / 2,  w_in, WG),
    }
    return cell, ports
