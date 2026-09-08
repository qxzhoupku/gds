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


def _euler_local_points(x_rel, y_rel, Rmin, n):
    """Sample a smooth A→B curve in A's local frame (A at origin, heading +x).

    Every branch starts exactly at ``(0, 0)`` and ends exactly at
    ``(x_rel, y_rel)``, so the emitted path is attached to both ports.

    *Rmin* is treated as a constraint to check rather than a shape parameter:
    between two fixed ports the curve is determined by the endpoints, so where
    the geometry forces a tighter bend than *Rmin* this warns instead of
    silently drawing a lossy corner.
    """
    straight_tol = 1e-9

    # Collinear along either local axis — a straight segment reaches B exactly.
    if abs(y_rel) < straight_tol or abs(x_rel) < straight_tol:
        return [(0.0, 0.0), (x_rel, y_rel)]

    sgn = 1.0 if y_rel >= 0 else -1.0

    # L-bend: room for a quarter turn ahead of B, and B is off to one side.
    # Path is straight along +x, a quarter arc, then straight along ±y.
    if x_rel > 0 and abs(x_rel) >= Rmin and abs(y_rel) >= Rmin:
        r = Rmin
        pts = [(0.0, 0.0), (x_rel - r, 0.0)]
        cx, cy = x_rel - r, sgn * r
        for k in range(1, n + 1):
            # From -sgn·π/2 (heading +x) to 0 (heading ±y).
            theta = -sgn * (math.pi / 2.0) * (1.0 - k / n)
            pts.append((cx + r * math.cos(theta), cy + r * math.sin(theta)))
        pts.append((x_rel, y_rel))
        return pts

    # S-bend: raised cosine. Zero slope at both ends, so it leaves A and
    # arrives at B tangentially, and it lands on B by construction.
    #   x(t) = x_rel·t,  y(t) = (y_rel/2)·(1 − cos πt),  t ∈ [0, 1]
    # Curvature peaks at the ends, where the radius is
    #   R = 2·x_rel² / (π²·|y_rel|)
    if abs(x_rel) > straight_tol:
        implied_r = 2.0 * x_rel * x_rel / (math.pi ** 2 * abs(y_rel))
        if implied_r < Rmin:
            warnings.warn(
                f"Euler S-bend between these ports implies a "
                f"{implied_r:.3f} um radius, tighter than the requested "
                f"Rmin={Rmin:.3f} um. Increase the along-axis separation "
                f"to {math.pi * math.sqrt(Rmin * abs(y_rel) / 2.0):.1f} um "
                f"to satisfy it.",
                stacklevel=3,
            )
    return [
        (x_rel * (i / n), 0.5 * y_rel * (1.0 - math.cos(math.pi * i / n)))
        for i in range(n + 1)
    ]


def route_euler_bend(parent, A, B, Rmin, layer, n=100):
    """Smooth bend from port *A* to port *B*, honouring a minimum radius.

    In A's local frame the shape is chosen from the relative position of B:
    a straight segment when the ports are collinear, a straight/quarter-arc/
    straight L when there is room for a full turn, and a raised-cosine S-bend
    otherwise.  All three terminate on B.

    .. note::
       These are circular and raised-cosine curves, not true clothoids.  A
       real Euler spiral would ramp curvature linearly along the arc and so
       cut the junction loss further; ``Rmin`` here bounds the *peak*
       curvature only.
    """
    x1, y1, a1, w1 = A.x, A.y, A.angle, A.width
    w2 = B.width

    dx, dy = B.x - x1, B.y - y1
    cos1, sin1 = math.cos(a1), math.sin(a1)
    x_rel = dx * cos1 + dy * sin1
    y_rel = -dx * sin1 + dy * cos1

    pts = _euler_local_points(x_rel, y_rel, float(Rmin), int(n))

    # Local → world
    world_pts = [
        (x1 + x * cos1 - y * sin1, y1 + x * sin1 + y * cos1)
        for x, y in pts
    ]

    width = w1 if abs(w1 - w2) < 1e-3 else min(w1, w2)
    path = gdstk.FlexPath(world_pts, width, layer=layer)
    parent.add(path)
    return path
