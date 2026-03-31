"""Linear waveguide taper."""

import math
import gdstk
from ..ports import Port
from ..layer_map import resolve_wg_layer


def PCellTaper(params, layers):
    """Linear width taper from w0 to w1 over length L.

    The taper is placed on the layer corresponding to max(w0, w1),
    since the wider dimension determines the e-beam dose recipe.

    params:
        w0   — input width   (default 0.5 µm)
        w1   — output width  (default 3.0 µm)
        L    — taper length  (default 150.0 µm)
        name — cell name     (default "TAPER")
    """
    w0   = float(params.get("w0", 0.5))
    w1   = float(params.get("w1", 3.0))
    L    = float(params.get("L", 150.0))
    name = str(params.get("name", "TAPER"))

    WG   = resolve_wg_layer(max(w0, w1), layers)
    TEXT = layers.get("TEXT", 100)

    cell = gdstk.Cell(name)
    rp = gdstk.RobustPath((0.0, 0.0), w0, layer=WG)
    rp.segment((L, 0.0), width=w1)
    cell.add(rp)

    cell.add(gdstk.Label(f"TAPER {w0}->{w1} L={L}", (L / 2, -5), layer=TEXT))

    ports = {
        "W": Port("W", 0.0, 0.0, math.pi, w0, resolve_wg_layer(w0, layers)),
        "E": Port("E", L,   0.0, 0.0,     w1, resolve_wg_layer(w1, layers)),
    }
    return cell, ports
