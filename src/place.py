import math, gdstk
from .ports import Port

def _R(theta_deg: float):
    t = theta_deg
    return ((math.cos(t), -math.sin(t)), (math.sin(t), math.cos(t)))

def _apply_R(p, R):
    return (R[0][0]*p[0] + R[0][1]*p[1], R[1][0]*p[0] + R[1][1]*p[1])

def transform_ports(ports: dict, origin=(0.0,0.0), rotation=0.0):
    R = _R(rotation)
    out = {}
    for name, p in ports.items():
        pr = _apply_R((p.x, p.y), R)
        out[name] = Port(name, pr[0]+origin[0], pr[1]+origin[1], (p.angle+rotation)%(2*math.pi), p.width, p.layer)
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

def route_manhattan(parent: gdstk.Cell, A: Port, B: Port, r: float, layer=1, samples=24):
    """
    Tangent-aligned L-route A->B with a circular 90° bend of radius r.
    Local path (in A frame): straight (+x) → quarter-circle (±90°) → straight (±y).
    If requested r won't fit, it is reduced to min(bx, |by|) - eps.
    """
    import math, gdstk

    # --- local transforms aligned to A ---
    def _to_local(A, x, y):
        dx, dy = x - A.x, y - A.y
        t = -math.radians(A.angle)
        c, s = math.cos(t), math.sin(t)
        return (c*dx - s*dy, s*dx + c*dy)

    def _to_world(A, xl, yl):
        t = math.radians(A.angle)
        c, s = math.cos(t), math.sin(t)
        return (A.x + c*xl - s*yl, A.y + s*xl + c*yl)

    # B in A-local coords
    bx, by = _to_local(A, B.x, B.y)

    # If nearly colinear, just draw a straight
    if abs(by) < 1e-12 or abs(bx) < 1e-12:
        w = min(A.width, B.width)
        rp = gdstk.RobustPath((A.x, A.y), w, layer=layer)
        rp.segment((B.x, B.y), width=w)
        parent.add(rp)
        return

    # We need at least bx >= r and |by| >= r for a single quarter-circle L-bend
    # Auto-shrink r if needed
    r_fit = max(1e-6, min(abs(bx), abs(by), r))

    # bend direction (up if by>0, down if by<0)
    sgn = 1.0 if by >= 0 else -1.0

    # Pre-bend straight (local along +x)
    s = bx - r_fit
    if s < 0:
        # If still negative due to numeric noise, clamp and slightly reduce radius
        r_fit = max(1e-6, r_fit + s)  # r_fit := r_fit + (bx - r_fit) = bx
        s = 0.0

    # Geometry in local frame:
    # 1) pre-straight to (s, 0)
    # 2) quarter circle centered at (s, sgn*r_fit) from angle -sgn*pi/2 to 0
    # 3) post-straight to (bx, by)

    # Build in world coords
    w = min(A.width, B.width)
    rp = gdstk.RobustPath((A.x, A.y), w, layer=layer)

    # 1) pre-straight
    xw, yw = _to_world(A, s, 0.0)
    if s > 1e-12:
        rp.segment((xw, yw), width=w)

    # 2) arc: sample the quarter circle to guarantee correct footprint
    cx, cy = s, sgn * r_fit
    for k in range(1, samples + 1):
        theta = (-sgn * math.pi / 2.0) + (sgn * (k / samples) * (math.pi / 2.0))
        xl = cx + r_fit * math.cos(theta)
        yl = cy + r_fit * math.sin(theta)
        xw, yw = _to_world(A, xl, yl)
        rp.segment((xw, yw), width=w)

    # 3) post-straight
    xw, yw = _to_world(A, bx, by)
    rp.segment((xw, yw), width=w)

    parent.add(rp)


# ---------- Local transforms (for port-aligned routing) ----------
def _rotmat(theta_rad: float):
    import math
    c, s = math.cos(theta_rad), math.sin(theta_rad)
    return ((c, -s), (s, c))

def _to_local(A, x, y):
    """Translate by -A then rotate by -A.angle so A is at (0,0) with heading +x."""
    import math
    dx, dy = x - A.x, y - A.y
    t = -math.radians(A.angle)
    c, s = math.cos(t), math.sin(t)
    return (c*dx - s*dy, s*dx + c*dy)

def _to_world(A, xl, yl):
    """Inverse of _to_local: rotate by +A.angle then translate by A."""
    import math
    t = math.radians(A.angle)
    c, s = math.cos(t), math.sin(t)
    return (A.x + c*xl - s*yl, A.y + s*xl + c*yl)

# ---------- Euler bend sampler (symmetric 90° clothoid) ----------
def _euler_bend_90(Rmin, n=200, turn_sign=+1):
    """
    Returns points (x,y) of a symmetric 90° Euler bend starting at (0,0) with
    heading +x and ending with heading ±y. 'turn_sign' = +1 (bend up/CCW),
    -1 (bend down/CW). Minimum radius occurs at mid-bend (Rmin).
    """
    import math
    theta = math.pi / 2.0   # total turn
    phi = theta * 0.5
    Lh = 2.0 * Rmin * phi               # half-length of the Euler bend
    a  = 1.0 / (Rmin * Rmin * phi)      # curvature slope
    sign = 1.0 if turn_sign >= 0 else -1.0

    pts = [(0.0, 0.0)]
    x = y = psi = 0.0
    # First half: k(s) = a*s
    m1 = max(2, n // 2)
    ds1 = Lh / m1
    for i in range(m1):
        s = (i + 0.5) * ds1
        k = sign * a * s
        psi += k * ds1
        x += math.cos(psi) * ds1
        y += math.sin(psi) * ds1
        pts.append((x, y))
    # Second half: k(s') = k_max - a*s'
    kmax = sign * (1.0 / Rmin)
    m2 = n - m1
    ds2 = Lh / max(2, m2)
    for i in range(max(2, m2)):
        s = (i + 0.5) * ds2
        k = kmax - sign * a * s
        psi += k * ds2
        x += math.cos(psi) * ds2
        y += math.sin(psi) * ds2
        pts.append((x, y))
    return pts  # start at (0,0), end near (X_end, ±Y_end)

# ---------- Public API: Euler L-bend aligned to A ----------
def route_euler_bend(parent: gdstk.Cell, A: Port, B: Port, Rmin: float, layer=1, n=200):
    """
    L-shaped route A->B with a 90° Euler bend that STARTS aligned to A's tangent.
    Path in A-local frame: straight (+x) → Euler(±90°) → straight (±y).
    If not enough room for requested Rmin, it auto-scales Rmin to fit.
    """
    import math, gdstk

    # Transform B into A-local coordinates
    bx, by = _to_local(A, B.x, B.y)

    # Bend direction: up if B is above A in local frame
    sign = +1 if by >= 0 else -1

    # Sample the Euler bend in local coords (starts at (0,0), heading +x)
    pts = _euler_bend_90(Rmin, n=max(40, n), turn_sign=sign)
    X_end = pts[-1][0]
    Y_end = pts[-1][1]     # signed already by 'sign'

    # Ensure there is enough local +x distance to fit the bend footprint
    if bx < X_end:
        scale = max(0.1, bx / max(X_end, 1e-9))
        Rmin *= scale
        pts = _euler_bend_90(Rmin, n=max(40, n), turn_sign=sign)
        X_end = pts[-1][0]
        Y_end = pts[-1][1]

    # Pre-bend straight length (along +x in local)
    s = bx - X_end

    # Build the path in WORLD coords
    w = min(A.width, B.width)
    rp = gdstk.RobustPath((A.x, A.y), w, layer=layer)

    # 1) Straight to bend start (local (s, 0))
    x1w, y1w = _to_world(A, s, 0.0)
    if abs(s) > 1e-12:
        rp.segment((x1w, y1w), width=w)

    # 2) Euler bend polyline (offset by (s,0) in local; map each point to world)
    for (px, py) in pts[1:]:
        xw, yw = _to_world(A, s + px, py)
        rp.segment((xw, yw), width=w)

    # 3) Final straight along ±y to exactly hit B
    x2w, y2w = _to_world(A, bx, by)
    rp.segment((x2w, y2w), width=w)

    parent.add(rp)
