"""Pulley-coupled ring resonators (single-bus and add-drop variants).

A pulley coupler wraps the bus waveguide around part of the ring to increase
the effective coupling length compared to a simple point coupler.
"""

import math
import gdstk
from ..ports import Port
from ..layer_map import resolve_wg_layer


def PCellPulleyRing(params, layers):
    """Single-bus pulley-coupled ring resonator.

    Required params (µm / degrees):
        ring_radius      — ring centerline radius
        ring_width       — ring waveguide width
        coupler_width    — bus waveguide width
        gap              — edge-to-edge gap between ring and coupler
        pulley_angle_deg — wrap angle (0, 360)

    Optional params:
        bus_extension — straight run before/after bends (default max(50, 10·coupler_width))
        name          — cell name (default "PULLEY_RING")
    """
    R        = float(params["ring_radius"])
    wr       = float(params["ring_width"])
    wc       = float(params["coupler_width"])
    gap      = float(params["gap"])
    phi_deg  = float(params["pulley_angle_deg"])
    name     = str(params.get("name", "PULLEY_RING"))

    if not (0.0 < phi_deg < 360.0):
        raise ValueError(f"pulley_angle_deg must be in (0, 360), got {phi_deg}.")

    phi = math.radians(phi_deg)

    layer_ring = resolve_wg_layer(wr, layers)
    layer_bus  = resolve_wg_layer(wc, layers)
    TEXT = layers.get("TEXT", 100)

    # Coupler centerline radius (edge-gap convention)
    Rc = R + 0.5 * wr + gap + 0.5 * wc

    bus_ext = float(params.get("bus_extension", max(50.0, 10.0 * wc)))

    cell = gdstk.Cell(name)

    # ---- Ring resonator (full circle at origin) ----
    ring = gdstk.RobustPath((R, 0.0), width=wr, layer=layer_ring)
    ring.arc(R, 0.0, 2.0 * math.pi)
    cell.add(ring)

    # ---- Pulley bus: straight → bend → coupling arc → bend → straight ----
    P0 = (-2 * Rc * math.sin(0.5 * phi),
           -Rc + 2 * Rc * (1 - math.cos(0.5 * phi)))
    P3 = ( 2 * Rc * math.sin(0.5 * phi),
           -Rc + 2 * Rc * (1 - math.cos(0.5 * phi)))

    Pin  = (P0[0] - bus_ext, P0[1])
    Pout = (P3[0] + bus_ext, P3[1])

    bus = gdstk.RobustPath(Pin, width=wc, layer=layer_bus)
    bus.segment(P0)
    bus.turn(Rc, -0.5 * phi)
    bus.turn(Rc,  phi)
    bus.turn(Rc, -0.5 * phi)
    bus.segment(Pout)
    cell.add(bus)

    # ---- Normalise: translate so W port sits at origin ----
    dx = -P0[0] + bus_ext
    dy = -P0[1]
    ring.translate(dx, dy)
    bus.translate(dx, dy)

    cell.add(gdstk.Label(
        f"PULLEY R={R} gap={gap} θ={phi_deg}°",
        (dx, dy - R - 10), layer=TEXT,
    ))

    ports = {
        "W": Port("W", Pin[0]  + dx, Pin[1]  + dy, math.pi, wc, layer_bus),
        "E": Port("E", Pout[0] + dx, Pout[1] + dy, 0.0,     wc, layer_bus),
    }
    return cell, ports


# ---------------------------------------------------------------------------
# Add-drop variant
# ---------------------------------------------------------------------------

# Default vertical clearance between add and drop bus waveguides (µm).
_DROP_VERTICAL_TARGET = 250.0
# Extra horizontal run appended to each drop port (µm).
_DROP_PORT_EXTENSION = 20.0
# Vertical padding subtracted when computing drop routing length (µm).
_DROP_VERTICAL_PADDING = 50.0


def PCellADDDROPPulleyRing(params, layers):
    """Add-drop pulley-coupled ring resonator.

    Builds a ring with two pulley-coupled bus waveguides (add on the bottom,
    drop on the top).  The drop bus is routed with 90° bends so that its
    ports exit horizontally at the same y-level as the add-bus ports.

    Required params — same as PCellPulleyRing, plus:
        (all PCellPulleyRing required params)

    Optional params:
        coupler_drop_width    — drop bus width       (default: coupler_width)
        gap_drop              — drop edge gap         (default: gap)
        pulley_drop_angle_deg — drop wrap angle       (default: pulley_angle_deg)
        bend_drop_radius      — 90° bend radius for drop routing (default 100 µm)
        bus_extension         — straight run on add bus  (default max(50, 10·wc))
        drop_vertical_target  — target vertical spacing  (default 250 µm)
        drop_port_extension   — horizontal stub at drop ports (default 20 µm)
        drop_vertical_padding — padding in drop routing  (default 50 µm)
        name                  — cell name (default "PULLEY_RING")
    """
    # ---- Parameters ----
    R        = float(params["ring_radius"])
    wr       = float(params["ring_width"])
    wc       = float(params["coupler_width"])
    wc_drop  = float(params.get("coupler_drop_width", wc))
    gap      = float(params["gap"])
    gap_drop = float(params.get("gap_drop", gap))
    phi_deg      = float(params["pulley_angle_deg"])
    phi_drop_deg = float(params.get("pulley_drop_angle_deg", phi_deg))
    name = str(params.get("name", "PULLEY_RING"))

    if not (0.0 < phi_deg < 360.0):
        raise ValueError(f"pulley_angle_deg must be in (0, 360), got {phi_deg}.")

    phi      = math.radians(phi_deg)
    phi_drop = math.radians(phi_drop_deg)

    layer_ring = resolve_wg_layer(wr,      layers)
    layer_bus  = resolve_wg_layer(wc,      layers)
    layer_drop = resolve_wg_layer(wc_drop, layers)
    TEXT = layers.get("TEXT", 100)

    Rc      = R + 0.5 * wr + gap      + 0.5 * wc
    Rc_drop = R + 0.5 * wr + gap_drop + 0.5 * wc_drop

    bend_drop_r    = float(params.get("bend_drop_radius", 100))
    bus_ext        = float(params.get("bus_extension", max(50.0, 10.0 * wc)))
    vert_target    = float(params.get("drop_vertical_target",  _DROP_VERTICAL_TARGET))
    port_ext       = float(params.get("drop_port_extension",   _DROP_PORT_EXTENSION))
    vert_padding   = float(params.get("drop_vertical_padding", _DROP_VERTICAL_PADDING))

    cell = gdstk.Cell(name)

    # ---- 1) Ring resonator ----
    ring = gdstk.RobustPath((R, 0.0), width=wr, layer=layer_ring)
    ring.arc(R, 0.0, 2.0 * math.pi)
    cell.add(ring)

    # ---- 2) Add bus (bottom) ----
    P0 = (-2 * Rc * math.sin(0.5 * phi),
           -Rc + 2 * Rc * (1 - math.cos(0.5 * phi)))
    P3 = ( 2 * Rc * math.sin(0.5 * phi),
           -Rc + 2 * Rc * (1 - math.cos(0.5 * phi)))
    Pin  = (P0[0] - bus_ext, P0[1])
    Pout = (P3[0] + bus_ext, P3[1])

    bus = gdstk.RobustPath(Pin, width=wc, layer=layer_bus)
    bus.segment(P0)
    bus.turn(Rc, -0.5 * phi)
    bus.turn(Rc,  phi)
    bus.turn(Rc, -0.5 * phi)
    bus.segment(Pout)
    cell.add(bus)

    # ---- 3) Drop bus (top) + routing bends ----
    P0_drop = (-2 * Rc_drop * math.sin(0.5 * phi_drop),
                Rc_drop - 2 * Rc_drop * (1 - math.cos(0.5 * phi_drop)))
    P3_drop = ( 2 * Rc_drop * math.sin(0.5 * phi_drop),
                Rc_drop - 2 * Rc_drop * (1 - math.cos(0.5 * phi_drop)))
    bus_drop_ext = R - 2 * Rc_drop * math.sin(0.5 * phi_drop)
    Pin_drop  = (P0_drop[0] - bus_drop_ext, P0_drop[1])
    Pout_drop = (P3_drop[0] + bus_drop_ext, P3_drop[1])

    # East drop arm
    bus_drop = gdstk.RobustPath(Pin_drop, width=wc_drop, layer=layer_drop)
    bus_drop.segment(P0_drop)
    bus_drop.turn(Rc_drop,  0.5 * phi_drop)
    bus_drop.turn(Rc_drop, -phi_drop)
    bus_drop.turn(Rc_drop,  0.5 * phi_drop)
    bus_drop.segment(Pout_drop)

    # Route drop east port downward then rightward
    bus_drop.turn(bend_drop_r, -math.pi / 2)
    dis_drop = (Rc * (2 * math.cos(0.5 * phi) - 1)
                + Rc_drop * (2 * math.cos(0.5 * phi_drop) - 1)
                - 2 * bend_drop_r - vert_padding)
    spine = bus_drop.spine()[-1]
    bus_drop.segment((spine[0], spine[1] - dis_drop))
    bus_drop.turn(bend_drop_r, math.pi / 2)
    spine = bus_drop.spine()[-1]
    bus_drop.segment((spine[0] + port_ext, spine[1]))
    loc_end_E = bus_drop.spine()[-1]
    cell.add(bus_drop)

    # West drop arm — near-zero segment establishes heading for first turn
    bus_drop_W = gdstk.RobustPath(Pin_drop, width=wc_drop, layer=layer_drop)
    bus_drop_W.segment((Pin_drop[0] - 1e-10, Pin_drop[1]))
    bus_drop_W.turn(bend_drop_r, -math.pi / 2)
    dis_drop_W = (vert_target
                  - Rc * (2 * math.cos(0.5 * phi) - 1)
                  - Rc_drop * (2 * math.cos(0.5 * phi_drop) - 1))
    spine = bus_drop_W.spine()[-1]
    bus_drop_W.segment((spine[0], spine[1] + dis_drop_W))
    bus_drop_W.turn(bend_drop_r, math.pi / 2)
    spine = bus_drop_W.spine()[-1]
    bus_drop_W.segment((spine[0] - port_ext, spine[1]))
    loc_end_W = bus_drop_W.spine()[-1]
    cell.add(bus_drop_W)

    # ---- Normalise: translate so W (add) port sits at origin ----
    dx = -P0[0] + bus_ext
    dy = -P0[1]

    for geom in (ring, bus, bus_drop, bus_drop_W):
        geom.translate(dx, dy)

    cell.add(gdstk.Label(
        f"ADD-DROP PULLEY R={R} gap={gap}/{gap_drop} θ={phi_deg}/{phi_drop_deg}°",
        (dx, dy - R - 10), layer=TEXT,
    ))

    ports = {
        "W":      Port("W",      Pin[0]        + dx, Pin[1]        + dy, math.pi, wc,      layer_bus),
        "E":      Port("E",      Pout[0]       + dx, Pout[1]       + dy, 0.0,     wc,      layer_bus),
        "E_DROP": Port("E_DROP", loc_end_E[0]  + dx, loc_end_E[1]  + dy, 0.0,     wc_drop, layer_drop),
        "W_DROP": Port("W_DROP", loc_end_W[0]  + dx, loc_end_W[1]  + dy, math.pi, wc_drop, layer_drop),
    }
    return cell, ports
