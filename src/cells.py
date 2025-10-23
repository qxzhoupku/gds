# src/cells.py
import gdstk
from math import pi
from .rules import L_WG, L_PORT, L_TEXT, L_DIE

def _add_port(cell, x, y, angle_deg, name, w=0.5, h=3.0):
    theta = angle_deg * pi / 180.0
    import math
    dx, dy = 0.5*h*math.cos(theta), 0.5*h*math.sin(theta)
    nx, ny = 0.5*w*(-math.sin(theta)), 0.5*w*(math.cos(theta))
    pts = [(x-dx-nx, y-dy-ny), (x+dx-nx, y+dy-ny), (x+dx+nx, y+dy+ny), (x-dx+nx, y-dy+ny)]
    cell.add(gdstk.Polygon(pts, layer=L_PORT))
    cell.add(gdstk.Label(name, (x, y), layer=L_TEXT))

def PCellCrossingUnits(WM=1.6, LM=8.0, LT=10.0, w_in=0.5,
                       arm_clear=5.0, die_size=150.0, add_die=True, name="WX_UNITS"):
    """Two identical units (horizontal & vertical, 90° apart):
       singlemode WG -> linear taper -> MMW (length=LM, width=WM) -> linear taper -> singlemode WG
       Center overlap is just the intersection of the two constant-width MMWs (no extra square polygon).
    """
    lib = gdstk.Library(unit=1e-6, precision=1e-9)
    cell = lib.new_cell(name)

    if add_die:
        cell.add(gdstk.rectangle((-die_size/2, -die_size/2),
                                 ( die_size/2,  die_size/2), layer=L_DIE))

    # Port extents
    left_x, right_x = -die_size/2 + arm_clear, die_size/2 - arm_clear
    bot_y,  top_y   = -die_size/2 + arm_clear, die_size/2 - arm_clear

    # Horizontal unit path (left -> right), centered so MMW spans [-LM/2, +LM/2] at y=0
    rp_h = gdstk.RobustPath((left_x, 0.0), w_in, layer=L_WG)
    rp_h.segment((-LM/2 - LT, 0.0), width=w_in)     # singlemode WG to taper start
    rp_h.segment((-LM/2,      0.0), width=WM)       # linear taper up to WM
    rp_h.segment(( LM/2,      0.0), width=WM)       # constant MMW at WM, length LM
    rp_h.segment(( LM/2 + LT, 0.0), width=w_in)     # linear taper down to w_in
    rp_h.segment(( right_x,   0.0), width=w_in)     # singlemode WG
    cell.add(rp_h)

    # Vertical unit path (bottom -> top), centered so MMW spans [-LM/2, +LM/2] at x=0
    rp_v = gdstk.RobustPath((0.0, bot_y), w_in, layer=L_WG)
    rp_v.segment((0.0, -LM/2 - LT), width=w_in)     # singlemode WG to taper start
    rp_v.segment((0.0, -LM/2),      width=WM)       # linear taper up to WM
    rp_v.segment((0.0,  LM/2),      width=WM)       # constant MMW at WM, length LM
    rp_v.segment((0.0,  LM/2 + LT), width=w_in)     # linear taper down to w_in
    rp_v.segment((0.0,  top_y),     width=w_in)     # singlemode WG
    cell.add(rp_v)

    # Ports
    _add_port(cell, left_x, 0.0,  180, "PORT_W", w=w_in)
    _add_port(cell, right_x, 0.0,   0, "PORT_E", w=w_in)
    _add_port(cell, 0.0, bot_y,   -90, "PORT_S", w=w_in)
    _add_port(cell, 0.0, top_y,    90, "PORT_N", w=w_in)

    # Label
    cell.add(gdstk.Label(f"WX UNITS WM={WM} LM={LM} LT={LT} w_in={w_in}",
                         (0, -LM/2 - 6.0), layer=L_TEXT))
    return lib, cell
