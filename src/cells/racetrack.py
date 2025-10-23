# src/cells/racetrack.py
import math
import gdstk
from ..ports import Port

def PCellRacetrack(
    R=50.0,                 # bend radius (um) for the two semicircles
    L_straight=30.0,        # straight length between bends (um) measured between bend tangency points
    gap=0.2,                # bus-to-resonator edge gap (um)
    w_ring=0.5,             # ring waveguide width (um)
    w_bus=0.5,              # bus waveguide width (um)
    L_bus=None,             # bus length (um); default auto to span racetrack
    layers=None,
    name="RACETRACK",
    bend="circular"         # "circular" (default). Reserved for future: "euler"
):
    """True racetrack resonator + straight bus coupler.

    Geometry (centerline of ring path):
      - Start at (-L_straight/2, yc + R) → straight to (L_straight/2, yc + R)
      - 180° bend (radius R) to (L_straight/2, yc - R)
      - Straight to (-L_straight/2, yc - R)
      - 180° bend (radius R) back to start

    The path is drawn as a RobustPath with constant width = w_ring, yielding the ring body.
    """
    if layers is None:
        layers = {"WG": 1, "PORT": 99, "TEXT": 100}
    WG = layers["WG"]
    TEXT = layers.get("TEXT", 100)

    cell = gdstk.Cell(name)

    # ---------- Bus waveguide (horizontal) ----------
    # Auto bus length to comfortably span the racetrack extent
    if L_bus is None:
        L_bus = (2 * R + L_straight) + 40.0
    x0, x1 = -L_bus / 2.0, L_bus / 2.0
    bus = gdstk.RobustPath((x0, 0.0), w_bus, layer=WG)
    bus.segment((x1, 0.0), width=w_bus)
    cell.add(bus)

    # ---------- Racetrack ring (above the bus) ----------
    # Place ring centerline so the BOTTOM of the ring is 'gap' above the top edge of the bus:
    # bus top edge is at +w_bus/2, ring bottom edge is at yc - R - w_ring/2.
    # Enforce: (yc - R - w_ring/2) - (w_bus/2) = gap  => yc = R + (w_ring + w_bus)/2 + gap
    yc = R + 0.5 * (w_ring + w_bus) + gap

    # Start point on the TOP straight (heading +x)
    start = (-L_straight / 2.0, yc + R)

    rp = gdstk.RobustPath(start, w_ring, layer=WG)

    # 1) Top straight: to the right
    rp.segment((L_straight / 2.0, yc + R), width=w_ring)

    # 2) Right 180° bend (circular): from +x heading to -x heading, moving downward by 2R
    if bend == "circular":
        rp.turn(R, -math.pi)  # negative sign: bend clockwise going down
    else:
        # Placeholder: Euler bends could be implemented with parametric control points
        rp.turn(R, -math.pi)

    # 3) Bottom straight: to the left
    rp.segment((-L_straight / 2.0, yc - R), width=w_ring)

    # 4) Left 180° bend: from -x heading back to +x, moving upward by 2R to the start y
    if bend == "circular":
        rp.turn(R, -math.pi)
    else:
        rp.turn(R, -math.pi)

    # Close by returning to the start x,y (a short zero-length segment is fine)
    rp.segment(start, width=w_ring)

    cell.add(rp)

    # ---------- Annotation ----------
    cell.add(gdstk.Label(f"RACETRACK R={R} Ls={L_straight} gap={gap}", (0, yc - R - 10), layer=TEXT))

    # ---------- Ports on the bus ----------
    ports = {
        "W": Port("W", x0, 0.0, 180.0, w_bus, WG),
        "E": Port("E", x1, 0.0,   0.0, w_bus, WG),
    }
    return cell, ports
