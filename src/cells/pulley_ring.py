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
    # bend_radius = float(params.get("bend_radius", max(5.0 * wc, 0.15 * Rc)))
    bus_ext = float(params.get("bus_extension", max(50.0, 10.0 * wc)))

    # ---- build cell
    cell = gdstk.Cell(name)

    # ---- 1) ring resonator
    ring = gdstk.RobustPath((R, 0.0), width=wr, layer=layer_ring)
    ring.arc(R, 0.0, 2.0 * math.pi)
    cell.add(ring)

    # ---- 2) pulley bus: straight -> bend -> coupling arc -> bend -> straight
    # Coupling arc endpoints (centered at origin)
    # P1 = (Rc * math.sin(-0.5 * phi), -Rc * math.cos(-0.5 * phi))
    # P2 = (Rc * math.sin(+0.5 * phi), -Rc * math.cos(+0.5 * phi))

    P0 = (-2 * Rc * math.sin(0.5 * phi), -Rc + 2 * Rc * (1 - math.cos(0.5 * phi)))  # start of first bend
    P3 = ( 2 * Rc * math.sin(0.5 * phi), -Rc + 2 * Rc * (1 - math.cos(0.5 * phi)))   # end of second bend

    # Port points (straight extensions)
    Pin = (P0[0] - bus_ext, P0[1])
    Pout = (P3[0] + bus_ext, P3[1])

    bus = gdstk.RobustPath(Pin, width=wc, layer=layer_bus)
    bus.segment(P0)                 # input straight
    bus.turn(Rc, -0.5*phi)
    bus.turn(Rc, phi)          # coupling arc
    bus.turn(Rc, -0.5*phi)
    bus.segment(Pout)               # output straight
    cell.add(bus)

    # ---- ports (DEGREES)
    ports = {
        "W": Port("W", Pin[0], Pin[1], math.pi, wc, layer_bus),
        "E": Port("E", Pout[0], Pout[1], 0.0, wc, layer_bus),
    }

    dx, dy = -P0[0] + bus_ext, -P0[1]  # vector from origin to first bend start

    ring.translate(dx, dy)
    bus.translate(dx, dy)

    for port in ports.values():
        port.x += dx
        port.y += dy

    return cell, ports


def PCellADDDROPPulleyRing(params, layers):
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
    wc_drop = float(params.get("coupler_drop_width", wc))  # separate drop width (default to same as add)
    gap = float(params["gap"])
    gap_drop = float(params.get("gap_drop", gap))  # separate drop gap (default to same as add)
    phi_deg = float(params["pulley_angle_deg"])
    phi_drop_deg = float(params.get("pulley_drop_angle_deg", phi_deg))  # separate drop angle (default to same as add)
    name = str(params.get("name", "PULLEY_RING"))

    if not (0.0 < phi_deg < 360.0):
        raise ValueError(f"pulley_angle_deg must be in (0, 360). Got {phi_deg}.")

    phi = math.radians(phi_deg)
    phi_drop = math.radians(phi_drop_deg)

    # Layer mapping
    layer_ring_name = params.get("layer_ring", "WG")
    layer_bus_name = params.get("layer_bus", "WG")
    layer_ring = layers.get(layer_ring_name, layers.get("WG", 1))
    layer_bus = layers.get(layer_bus_name, layers.get("WG", 1))

    # Coupler centerline radius from edge-to-edge gap
    Rc = R + 0.5 * wr + gap + 0.5 * wc
    Rc_drop = R + 0.5 * wr + gap_drop + 0.5 * wc_drop
    bus_ext = float(params.get("bus_extension", max(50.0, 10.0 * wc)))

    # ---- build cell
    cell = gdstk.Cell(name)

    # ---- 1) ring resonator
    ring = gdstk.RobustPath((R, 0.0), width=wr, layer=layer_ring)
    ring.arc(R, 0.0, 2.0 * math.pi)
    cell.add(ring)

    # ---- 2) pulley bus: straight -> bend -> coupling arc -> bend -> straight
    P0 = (-2 * Rc * math.sin(0.5 * phi), -Rc + 2 * Rc * (1 - math.cos(0.5 * phi)))  # start of first bend
    P3 = ( 2 * Rc * math.sin(0.5 * phi), -Rc + 2 * Rc * (1 - math.cos(0.5 * phi)))   # end of second bend
    Pin = (P0[0] - bus_ext, P0[1])
    Pout = (P3[0] + bus_ext, P3[1])
    bus = gdstk.RobustPath(Pin, width=wc, layer=layer_bus)
    bus.segment(P0)                 # input straight
    bus.turn(Rc, -0.5*phi)
    bus.turn(Rc, phi)          # coupling arc
    bus.turn(Rc, -0.5*phi)
    bus.segment(Pout)               # output straight
    cell.add(bus)

    P0_drop = (-2 * Rc_drop * math.sin(0.5 * phi_drop), Rc_drop - 2 * Rc_drop * (1 - math.cos(0.5 * phi_drop)))  # start of first bend
    P3_drop = ( 2 * Rc_drop * math.sin(0.5 * phi_drop), Rc_drop - 2 * Rc_drop * (1 - math.cos(0.5 * phi_drop)))   # end of second bend
    bus_drop_ext = R - 2 * Rc_drop * math.sin(0.5 * phi_drop)
    Pin_drop = (P0_drop[0] - bus_drop_ext, P0_drop[1])
    Pout_drop = (P3_drop[0] + bus_drop_ext, P3_drop[1])
    bus_drop = gdstk.RobustPath(Pin_drop, width=wc_drop, layer=layer_bus)
    bus_drop.segment(P0_drop)                 # input straight
    bus_drop.turn(Rc_drop, 0.5*phi_drop)
    bus_drop.turn(Rc_drop, -phi_drop)          # coupling arc
    bus_drop.turn(Rc_drop, 0.5*phi_drop)
    bus_drop.segment(Pout_drop)               # output straight
    cell.add(bus_drop)

    # ---- ports (DEGREES)
    ports = {
        "W": Port("W", Pin[0], Pin[1], math.pi, wc, layer_bus),
        "E": Port("E", Pout[0], Pout[1], 0.0, wc, layer_bus),
        "E_DROP": Port("E_DROP", Pout_drop[0], Pout_drop[1], 0.0, wc_drop, layer_bus),
    }

    dx, dy = -P0[0] + bus_ext, -P0[1]  # vector from origin to first bend start

    ring.translate(dx, dy)
    bus.translate(dx, dy)
    bus_drop.translate(dx, dy)

    for port in ports.values():
        port.x += dx
        port.y += dy

    return cell, ports