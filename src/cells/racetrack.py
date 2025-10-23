import gdstk
from ..ports import Port

def PCellRacetrack(R=50.0, L_straight=30.0, gap=0.2, w_ring=0.5, w_bus=0.5, L_bus=None, layers=None, name="RACETRACK"):
    if layers is None:
        layers = {"WG": 1, "PORT": 99, "TEXT": 100}
    WG = layers["WG"]
    TEXT = layers.get("TEXT", 100)
    if L_bus is None:
        L_bus = 2*R + L_straight + 40.0

    cell = gdstk.Cell(name)
    x0, x1 = -L_bus/2, L_bus/2
    bus = gdstk.RobustPath((x0, 0.0), w_bus, layer=WG)
    bus.segment((x1, 0.0), width=w_bus)
    cell.add(bus)

    yc = gap + 0.5*w_bus + R
    # outer loop approximation using FlexPath closed polyline
    fp = gdstk.FlexPath([( -L_straight/2, yc+R), ( L_straight/2, yc+R),
                         ( L_straight/2, yc-R), ( -L_straight/2, yc-R),
                         ( -L_straight/2, yc+R)], w_ring, layer=WG, ends="round")
    cell.add(fp)

    cell.add(gdstk.Label(f"RACETRACK R={R} Ls={L_straight} gap={gap}", (0, yc - R - 10), layer=TEXT))

    ports = {"W": Port("W", x0, 0.0, 180.0, w_bus, WG), "E": Port("E", x1, 0.0, 0.0, w_bus, WG)}
    return cell, ports
