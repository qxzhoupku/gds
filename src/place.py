import math, gdstk
from .ports import Port

def _R(theta_deg: float):
    t = math.radians(theta_deg)
    return ((math.cos(t), -math.sin(t)), (math.sin(t), math.cos(t)))

def _apply_R(p, R):
    return (R[0][0]*p[0] + R[0][1]*p[1], R[1][0]*p[0] + R[1][1]*p[1])

def transform_ports(ports: dict, origin=(0.0,0.0), rotation=0.0):
    R = _R(rotation)
    out = {}
    for name, p in ports.items():
        pr = _apply_R((p.x, p.y), R)
        out[name] = Port(name, pr[0]+origin[0], pr[1]+origin[1], (p.angle+rotation)%360, p.width, p.layer)
    return out

def place_by_ports(parent: gdstk.Cell, child_cell: gdstk.Cell, child_port: Port, target_port: Port):
    rot = target_port.angle - (child_port.angle + math.pi)
    R = _R(rot)
    pf_rot = _apply_R((child_port.x, child_port.y), R)
    tx, ty = target_port.x - pf_rot[0], target_port.y - pf_rot[1]
    ref = gdstk.Reference(child_cell, origin=(tx, ty), rotation=rot)
    parent.add(ref)
    return ref

def route_straight(parent: gdstk.Cell, A: Port, B: Port, layer=1):
    w = min(A.width, B.width)
    rp = gdstk.RobustPath((A.x, A.y), w, layer=layer)
    rp.segment((B.x, B.y), width=w)
    parent.add(rp)

def route_manhattan(parent: gdstk.Cell, A: Port, B: Port, r: float, layer=1):
    """L-shaped route with a quarter-circle bend of radius r."""
    import math
    w = min(A.width, B.width)
    rp = gdstk.RobustPath((A.x, A.y), w, layer=layer)

    # Decide bend direction: up (90°) or down (-90°)
    going_up = B.y >= A.y
    sign = 1 if going_up else -1

    # First horizontal segment
    midx = B.x - r
    rp.segment((midx, A.y), width=w)

    # Add a 90° arc (initial angle = 0 rad = +x)
    initial_angle = 0.0
    final_angle = sign * math.pi / 2
    center = (midx, A.y - sign * r)
    rp.arc(r, initial_angle, final_angle, width=w)

    # Vertical segment to the target
    rp.segment((B.x, B.y), width=w)
    parent.add(rp)

