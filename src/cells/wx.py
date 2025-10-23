import gdstk
from ..ports import Port
import math

def PCellWx(WM=1.6, LM=8.0, LT=10.0, w_in=0.5, layers=None, name="WX"):
    if layers is None:
        layers = {"WG": 1, "PORT": 99, "TEXT": 100}
    WG = layers["WG"]
    TEXT = layers.get("TEXT", 100)

    cell = gdstk.Cell(name)

    left = - (LM/2 + LT + 1.0)
    right =  (LM/2 + LT + 1.0)
    rp_h = gdstk.RobustPath((left, 0.0), w_in, layer=WG)
    rp_h.segment((-LM/2 - LT, 0.0), width=w_in)
    rp_h.segment((-LM/2,      0.0), width=WM)
    rp_h.segment(( LM/2,      0.0), width=WM)
    rp_h.segment(( LM/2 + LT, 0.0), width=w_in)
    rp_h.segment(( right,     0.0), width=w_in)
    cell.add(rp_h)

    bottom = - (LM/2 + LT + 1.0)
    top    =   (LM/2 + LT + 1.0)
    rp_v = gdstk.RobustPath((0.0, bottom), w_in, layer=WG)
    rp_v.segment((0.0, -LM/2 - LT), width=w_in)
    rp_v.segment((0.0, -LM/2),      width=WM)
    rp_v.segment((0.0,  LM/2),      width=WM)
    rp_v.segment((0.0,  LM/2 + LT), width=w_in)
    rp_v.segment((0.0,  top),       width=w_in)
    cell.add(rp_v)

    cell.add(gdstk.Label(f"WX WM={WM} LM={LM} LT={LT} w_in={w_in}", (0, -LM/2 - 6), layer=TEXT))

    ports = {
        "W": Port("W", left, 0.0, math.pi, w_in, WG),
        "E": Port("E", right, 0.0, 0.0,  w_in, WG),
        "S": Port("S", 0.0, bottom, 3*math.pi/2, w_in, WG),
        "N": Port("N", 0.0, top, math.pi/2, w_in, WG),
    }
    return cell, ports
