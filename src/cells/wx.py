"""Waveguide crossing (WX) — 4-port multimode interference cross."""

import math
import gdstk
from ..ports import Port
from ..layer_map import resolve_wg_layer

# Extra straight extension beyond the taper region on each arm (µm).
# Provides clearance for routing and avoids abrupt path terminations.
_ARM_EXTENSION = 1.0


def _arm_segments(start, end, LM, LT, w_in, WM, layers, horizontal=True):
    """Build one arm as separate paths so each width lands on its own layer.

    Returns a list of gdstk.RobustPath objects.
    """
    layer_narrow = resolve_wg_layer(w_in, layers)
    layer_wide   = resolve_wg_layer(WM,   layers)

    half = LM / 2
    paths = []

    def _pt(a):
        return (a, 0.0) if horizontal else (0.0, a)

    # 1) Input extension — narrow
    rp = gdstk.RobustPath(_pt(start), w_in, layer=layer_narrow)
    rp.segment(_pt(-half - LT), width=w_in)
    paths.append(rp)

    # 2) Input taper — assigned to the wider layer
    rp = gdstk.RobustPath(_pt(-half - LT), w_in, layer=layer_wide)
    rp.segment(_pt(-half), width=WM)
    paths.append(rp)

    # 3) Multimode centre — wide
    rp = gdstk.RobustPath(_pt(-half), WM, layer=layer_wide)
    rp.segment(_pt(half), width=WM)
    paths.append(rp)

    # 4) Output taper — wider layer
    rp = gdstk.RobustPath(_pt(half), WM, layer=layer_wide)
    rp.segment(_pt(half + LT), width=w_in)
    paths.append(rp)

    # 5) Output extension — narrow
    rp = gdstk.RobustPath(_pt(half + LT), w_in, layer=layer_narrow)
    rp.segment(_pt(end), width=w_in)
    paths.append(rp)

    return paths


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

    TEXT = layers.get("TEXT", 100)

    cell = gdstk.Cell(name)
    half_arm = LM / 2 + LT + _ARM_EXTENSION

    # Horizontal arm — split by width
    for seg in _arm_segments(-half_arm, half_arm, LM, LT, w_in, WM, layers,
                             horizontal=True):
        cell.add(seg)

    # Vertical arm — split by width
    for seg in _arm_segments(-half_arm, half_arm, LM, LT, w_in, WM, layers,
                             horizontal=False):
        cell.add(seg)

    cell.add(gdstk.Label(
        f"WX WM={WM} LM={LM} LT={LT} w_in={w_in}", (0, -LM / 2 - 6), layer=TEXT,
    ))

    layer_port = resolve_wg_layer(w_in, layers)
    ports = {
        "W": Port("W", -half_arm, 0.0,       math.pi,         w_in, layer_port),
        "E": Port("E",  half_arm, 0.0,       0.0,             w_in, layer_port),
        "S": Port("S",  0.0,     -half_arm,  3 * math.pi / 2, w_in, layer_port),
        "N": Port("N",  0.0,      half_arm,  math.pi / 2,     w_in, layer_port),
    }
    return cell, ports
