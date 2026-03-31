"""Placement and routing primitives for photonic layout.

Provides:
  - transform_ports / place_by_ports  — port-based cell placement
  - route_straight                    — point-to-point waveguide
  - route_manhattan                   — L-bend with quarter-circle arc
  - route_euler_bend                  — approximate Euler-style bend (circular / cycloid)
"""

import math
import warnings

import gdstk

from .ports import Port


# ---------------------------------------------------------------------------
# Low-level 2-D helpers
# ---------------------------------------------------------------------------

def _rot_matrix(theta):
    """2×2 rotation matrix for angle *theta* (radians)."""
    c, s = math.cos(theta), math.sin(theta)
    return ((c, -s), (s, c))


def _apply_rot(point, R):
    """Apply 2×2 rotation matrix *R* to a 2-D *point*."""
    return (R[0][0] * point[0] + R[0][1] * point[1],
            R[1][0] * point[0] + R[1][1] * point[1])


def _to_local(port, x, y):
    """World → port-local coords (port at origin, heading along +x)."""
    dx, dy = x - port.x, y - port.y
    c, s = math.cos(-port.angle), math.sin(-port.angle)
    return (c * dx - s * dy, s * dx + c * dy)


def _to_world(port, xl, yl):
    """Port-local → world coords (inverse of *_to_local*)."""
    c, s = math.cos(port.angle), math.sin(port.angle)
    return (port.x + c * xl - s * yl, port.y + s * xl + c * yl)


# ---------------------------------------------------------------------------
# Port transforms & placement
# ---------------------------------------------------------------------------

def transform_ports(ports, origin=(0.0, 0.0), rotation=0.0):
    """Rotate then translate every Port in *ports* dict.  Returns a new dict."""
    R = _rot_matrix(rotation)
    out = {}
    for name, p in ports.items():
        pr = _apply_rot((p.x, p.y), R)
        out[name] = Port(
            name,
            pr[0] + origin[0],
            pr[1] + origin[1],
            (p.angle + rotation) % (2 * math.pi),
            p.width,
            p.layer,
        )
    return out


def place_by_ports(parent, child_cell, child_port, target_port):
    """Place *child_cell* so that *child_port* meets *target_port* face-to-face.

    Returns the ``gdstk.Reference`` that was added to *parent*.
    """
    rot = target_port.angle - (child_port.angle + math.pi)
    R = _rot_matrix(rot)
    pf_rot = _apply_rot((child_port.x, child_port.y), R)
    tx = target_port.x - pf_rot[0]
    ty = target_port.y - pf_rot[1]
    ref = gdstk.Reference(child_cell, origin=(tx, ty), rotation=rot)
    parent.add(ref)
    return ref


# ---------------------------------------------------------------------------
# Routing
# ---------------------------------------------------------------------------

def route_straight(parent, A, B, layer=1):
    """Straight waveguide segment between ports *A* and *B*."""
    w = min(A.width, B.width)
    rp = gdstk.RobustPath((A.x, A.y), w, layer=layer)
    rp.segment((B.x, B.y), width=w)
    parent.add(rp)


def route_manhattan(parent, A, B, r, layer=1, samples=24):
    """L-shaped route A→B with a circular quarter-bend of radius *r*.

    The path in A's local frame is: straight along +x → 90° arc → straight
    along ±y.  *r* is auto-reduced when the geometry is too tight.
    """
    bx, by = _to_local(A, B.x, B.y)

    # Nearly collinear → fall back to straight
    if abs(by) < 1e-12 or abs(bx) < 1e-12:
        route_straight(parent, A, B, layer=layer)
        return

    # Fit radius to available space
    r_fit = max(1e-6, min(abs(bx), abs(by), r))
    sgn = 1.0 if by >= 0 else -1.0

    # Pre-bend straight length
    s = bx - r_fit
    if s < 0:
        r_fit = max(1e-6, r_fit + s)  # shrink radius to fit
        s = 0.0

    w = min(A.width, B.width)
    rp = gdstk.RobustPath((A.x, A.y), w, layer=layer)

    # 1) Pre-bend straight
    if s > 1e-12:
        xw, yw = _to_world(A, s, 0.0)
        rp.segment((xw, yw), width=w)

    # 2) Quarter-circle arc (sampled)
    cx, cy = s, sgn * r_fit
    for k in range(1, samples + 1):
        theta = -sgn * math.pi / 2.0 + sgn * (k / samples) * (math.pi / 2.0)
        xl = cx + r_fit * math.cos(theta)
        yl = cy + r_fit * math.sin(theta)
        xw, yw = _to_world(A, xl, yl)
        rp.segment((xw, yw), width=w)

    # 3) Post-bend straight
    xw, yw = _to_world(A, bx, by)
    rp.segment((xw, yw), width=w)

    parent.add(rp)


def route_euler_bend(parent, A, B, Rmin, layer, n=100):
    """Approximate Euler-style bend from port *A* to port *B*.

    Handles four geometry cases (in A's local frame):
      1. Collinear horizontal → straight segment
      2. Collinear vertical   → straight segment
      3. L-bend (room in both axes) → circular quarter-arc, scaled if needed
      4. S-bend (tight in one axis)  → cycloid-based smooth curve

    .. note::
       This is a circular/cycloid *approximation*, not a true clothoid.
       A proper Euler spiral implementation would provide smoother curvature
       transitions and lower optical loss at the junctions.
    """
    x1, y1, a1, w1 = A.x, A.y, A.angle, A.width
    x2, y2, a2, w2 = B.x, B.y, B.angle, B.width

    dx, dy = x2 - x1, y2 - y1
    cos1, sin1 = math.cos(a1), math.sin(a1)
    x_rel = dx * cos1 + dy * sin1
    y_rel = -dx * sin1 + dy * cos1

    # Characteristic footprint of a 90° bend at Rmin
    L = 1.65 * Rmin
    x_req, y_req = abs(x_rel), abs(y_rel)

    if x_req > L and y_req < 1e-3:
        # Case 1 — collinear horizontal
        pts = [(0, 0), (x_rel, 0)]

    elif y_req > L and x_req < 1e-3:
        # Case 2 — collinear vertical
        pts = [(0, 0), (0, y_rel)]

    elif x_req > L and y_req > L:
        # Case 3 — L-bend
        L_max = min(x_req, y_req)
        if L > L_max:
            scale = L_max / L
            Rmin = Rmin * scale
            warnings.warn(f"Euler bend Rmin scaled to {Rmin:.3f} to fit geometry.")
        sgn = 1.0 if y_rel >= 0 else -1.0
        t_vals = [i / (n - 1) for i in range(n)]
        pts = [
            (Rmin * math.sin(math.pi * 0.5 * t),
             sgn * Rmin * (1 - math.cos(math.pi * 0.5 * t)))
            for t in t_vals
        ]

    else:
        # Case 4 — S-bend (cycloid approximation)
        mid_y = y_rel / 2
        Rmin_s = max(1.0, min(abs(mid_y) / 1.1, Rmin))
        pts = []
        for i in range(n):
            phi = math.pi * i / (n - 1)
            x = Rmin_s * (phi - math.sin(phi))
            y = mid_y - Rmin_s * (math.cos(phi) - 1)
            pts.append((x, y))

    # Local → world
    world_pts = [
        (x1 + x * cos1 - y * sin1, y1 + x * sin1 + y * cos1)
        for x, y in pts
    ]

    width = w1 if abs(w1 - w2) < 1e-3 else min(w1, w2)
    path = gdstk.FlexPath(world_pts, width, layer=layer)
    parent.add(path)
    return path
