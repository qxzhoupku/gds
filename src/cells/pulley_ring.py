# src/cells/pulley_ring.py
import math
import gdstk
from ..ports import Port


def PCellPulleyRing(params, layers):
    """
    Ring resonator + pulley coupler.

    Required params (all in microns):
      - ring_radius
      - ring_width
      - coupler_width
      - gap                (edge-to-edge between ring and coupler)
      - pulley_angle_deg   (wrap angle, degrees, (0, 360))

    Optional params:
      - bend_radius        (radius of the 2 connector bends)
      - bus_extension      (straight length before/after bends)
      - layer_ring         (layer key in layers dict; default "WG")
      - layer_bus          (layer key in layers dict; default "WG")
      - name               (cell name; default "PULLEY_RING")

    Notes:
      - Port angles are in DEGREES (to match your existing transform/placement code).
      - Coupler centerline radius is computed from edge gap:
          Rc = R + wr/2 + gap + wc/2
      - The bus ports are horizontal (parallel), but generally at different y (still parallel).
    """
    # ---- read params
    R = float(params["ring_radius"])
    wr = float(params["ring_width"])
    wc = float(params["coupler_width"])
    gap = float(params["gap"])
    phi_deg = float(params["pulley_angle_deg"])
    name = str(params.get("name", "PULLEY_RING"))

    if not (0.0 < phi_deg < 360.0):
        raise ValueError(f"pulley_angle_deg must be in (0, 360). Got {phi_deg}.")

    phi = math.radians(phi_deg)

    # Layer mapping
    layer_ring_name = params.get("layer_ring", "WG")
    layer_bus_name = params.get("layer_bus", "WG")
    layer_ring = layers.get(layer_ring_name, layers.get("WG", 1))
    layer_bus = layers.get(layer_bus_name, layers.get("WG", 1))

    # Coupler centerline radius from edge-to-edge gap
    Rc = R + 0.5 * wr + gap + 0.5 * wc

    # Bend radius + extension (derived defaults; you can override in YAML)
    bend_radius = float(params.get("bend_radius", max(5.0 * wc, 0.15 * Rc)))
    bus_ext = float(params.get("bus_extension", max(50.0, 10.0 * wc)))

    # ---- build cell
    cell = gdstk.Cell(name)

    # ---- 1) ring resonator
    ring = gdstk.RobustPath((R, 0.0), width=wr, layer=layer_ring)
    ring.arc(R, 0.0, 2.0 * math.pi)
    cell.add(ring)

    # ---- 2) pulley bus: straight -> bend -> coupling arc -> bend -> straight
    a1 = -0.5 * phi
    a2 = +0.5 * phi

    # Coupling arc endpoints (centered at origin)
    P1 = (Rc * math.cos(a1), Rc * math.sin(a1))
    P2 = (Rc * math.cos(a2), Rc * math.sin(a2))

    # Horizontal tangent corresponds to polar angle -90° on a circle
    ang_h = -0.5 * math.pi

    # Bend 1: choose center so arc ends at P1 with final_angle=a1
    C1 = (P1[0] - bend_radius * math.cos(a1), P1[1] - bend_radius * math.sin(a1))
    P0 = (C1[0], C1[1] - bend_radius)  # point where polar angle is -90° (horizontal tangent)

    # Bend 2: similarly, arc starts at P2 with initial_angle=a2 and ends at horizontal (angle=-90°)
    C2 = (P2[0] - bend_radius * math.cos(a2), P2[1] - bend_radius * math.sin(a2))
    P3 = (C2[0], C2[1] - bend_radius)

    # Port points (straight extensions)
    Pin = (P0[0] - bus_ext, P0[1])
    Pout = (P3[0] + bus_ext, P3[1])

    bus = gdstk.RobustPath(Pin, width=wc, layer=layer_bus)
    bus.segment(P0)                 # input straight
    bus.arc(bend_radius, ang_h, a1) # bend into coupling arc start (ends at P1)
    bus.arc(Rc, a1, a2)             # coupling arc around ring
    bus.arc(bend_radius, a2, ang_h) # bend back to horizontal (ends at P3)
    bus.segment(Pout)               # output straight
    cell.add(bus)

    # ---- ports (DEGREES)
    ports = {
        "IN":   Port("IN",  x=Pin[0],  y=Pin[1],  angle=180.0, width=wc, layer=layer_bus),
        "OUT":  Port("OUT", x=Pout[0], y=Pout[1], angle=0.0,   width=wc, layer=layer_bus),

        # Coupling endpoints (tangent directions)
        "CPL_A": Port("CPL_A", x=P1[0], y=P1[1], angle=math.degrees(a1 + math.pi/2), width=wc, layer=layer_bus),
        "CPL_B": Port("CPL_B", x=P2[0], y=P2[1], angle=math.degrees(a2 + math.pi/2), width=wc, layer=layer_bus),

        # Ring probe (tangent +y at (R,0))
        "RING_P": Port("RING_P", x=R, y=0.0, angle=90.0, width=wr, layer=layer_ring),
    }

    return cell, ports
