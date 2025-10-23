import gdstk
from ..ports import Port

def PCellTaper(w0=0.5, w1=3.0, L=150.0, layers=None, name="TAPER"):
    if layers is None:
        layers = {"WG": 1, "PORT": 99, "TEXT": 100}
    WG = layers["WG"]
    TEXT = layers.get("TEXT", 100)

    cell = gdstk.Cell(name)
    rp = gdstk.RobustPath((0.0, 0.0), w0, layer=WG)
    rp.segment((L, 0.0), width=w1)
    cell.add(rp)

    cell.add(gdstk.Label(f"TAPER {w0}->{w1} L={L}", (L/2, -5), layer=TEXT))

    ports = {"W": Port("W", 0.0, 0.0, 180.0, w0, WG), "E": Port("E", L, 0.0, 0.0, w1, WG)}
    return cell, ports
