"""Circular arc waveguide of arbitrary angle."""

import math
import gdstk
from ..ports import Port
from ..layer_map import resolve_wg_layer


def PCellAnyArc(params, layers):
    """Circular arc waveguide, counter-clockwise from (radius, 0).

    params:
        radius    — centerline radius  (default 10.0 µm)
        width     — waveguide width    (default 0.5 µm)
        angle_deg — arc sweep in degrees (default 90)
        name      — cell name          (default "ARC")
    """
    radius    = float(params.get("radius", 10.0))
    width     = float(params.get("width", 0.5))
    angle_deg = float(params.get("angle_deg", 90.0))
    name      = str(params.get("name", "ARC"))

    WG   = resolve_wg_layer(width, layers)
    TEXT = layers.get("TEXT", 100)

    cell = gdstk.Cell(name)
    theta = math.radians(angle_deg)

    # Arc centred at origin, starting at (radius, 0)
    path = gdstk.RobustPath((radius, 0), width=width, layer=WG)
    path.arc(radius, 0, theta)
    cell.add(path)

    # End position
    end_x = radius * math.cos(theta)
    end_y = radius * math.sin(theta)

    cell.add(gdstk.Label(
        f"ARC R={radius} w={width} θ={angle_deg}°",
        (0, 0), layer=TEXT,
    ))

    # Port tangent directions (perpendicular to radius, pointing outward)
    ports = {
        "W": Port("W", radius, 0.0,   -math.pi / 2,        width, WG),
        "E": Port("E", end_x,  end_y,  theta + math.pi / 2, width, WG),
    }
    return cell, ports
