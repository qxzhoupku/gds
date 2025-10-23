import gdstk
from ..ports import Port

def PCellRingCoupler(R=20.0, gap=0.25, w_ring=0.5, w_bus=0.5, L_bus=None, layers=None, name="RING"):
    if layers is None:
        layers = {"WG": 1, "PORT": 99, "TEXT": 100}
    WG = layers["WG"]
    TEXT = layers.get("TEXT", 100)
    if L_bus is None:
        L_bus = 2*R + 00.0

    cell = gdstk.Cell(name)
    x0, x1 = -L_bus/2, L_bus/2
    bus = gdstk.RobustPath((x0, 0.0), w_bus, layer=WG)
    bus.segment((x1, 0.0), width=w_bus)
    cell.add(bus)

    y_c = gap + 0.5*w_bus + R
    ring = gdstk.ellipse((0.0, y_c), radius=R, inner_radius=R - w_ring, layer=WG)
    cell.add(ring)

    cell.add(gdstk.Label(f"RING R={R} gap={gap}", (0, -R - 10), layer=TEXT))

    ports = {"W": Port("W", x0, 0.0, 180.0, w_bus, WG), "E": Port("E", x1, 0.0, 0.0, w_bus, WG)}
    return cell, ports
