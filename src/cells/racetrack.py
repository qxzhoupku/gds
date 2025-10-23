# src/cells/racetrack.py
import math
import gdstk
from ..ports import Port

def PCellRacetrack(
    R=50.0,
    L_straight=30.0,
    gap=0.2,
    w_ring=0.5,
    w_bus=0.5,
    L_bus=None,
    layers=None,
    name="RACETRACK",
):
    """
    True racetrack resonator above a horizontal bus at y=0.

    Geometry:
      - Bus: horizontal RobustPath at y=0, width = w_bus, length = L_bus.
      - Racetrack loop (width = w_ring):
          * Two vertical straight sections of length L_straight,
          * Two semicircular turns of radius R.
        The lower semicircle sits above the bus, providing point coupling.

    Coupling placement:
      Let y_bottom be the bottom-most point of the racetrack boundary near the bus.
      We enforce:
         (y_bottom) - (w_bus/2) = gap + (w_ring/2)
      For the centerline, bottom-most point is y_c - R, so:
         y_c = gap + 0.5*(w_bus + w_ring) + R

    NOTE:
      RobustPath.turn(angle, radius)  # angle FIRST, THEN radius.
      Using angle = -pi produces a clockwise (right) 180° bend.
    """
    if layers is None:
        layers = {"WG": 1, "PORT": 99, "TEXT": 100}
    WG = layers["WG"]
    TEXT = layers.get("TEXT", 100)

    if L_bus is None:
        # Make bus comfortably longer than racetrack width
        L_bus = 2 * R
    cell = gdstk.Cell(name)

    # ---------------- Bus (horizontal at y=0) ----------------
    x0, x1 = -L_bus / 2.0, L_bus / 2.0
    bus = gdstk.RobustPath((x0, 0.0), w_bus, layer=WG)
    bus.segment((x1, 0.0), width=w_bus)
    cell.add(bus)

    # ---------------- Racetrack (above bus) ----------------
    y_c = gap + 0.5 * (w_bus + w_ring)   # centerline y of lower semicircle

    x_left = -L_straight / 2.0
    x_right = L_straight / 2.0

    rp = gdstk.RobustPath((x_left, y_c + R), w_ring, layer=WG)

    # 1) Left vertical straight (upwards)
    rp.segment((x_left, y_c + R + L_straight), width=w_ring)

    # 2) Top semicircle: 180° clockwise (to the right)
    rp.turn(R, -math.pi)   # angle first, then radius

    # 3) Right vertical straight (downwards)
    rp.segment((x_right, y_c + R), width=w_ring)

    # 4) Bottom semicircle: 180° clockwise (to the right), closing the loop
    rp.turn(R, -math.pi)

    cell.add(rp)

    # Label
    cell.add(gdstk.Label(f"RACETRACK R={R} Ls={L_straight} gap={gap}", (0, y_c - R - 10), layer=TEXT))

    ports = {
        "W": Port("W", x0, 0.0, math.pi, w_bus, WG),
        "E": Port("E", x1, 0.0,   0.0, w_bus, WG),
    }
    return cell, ports
