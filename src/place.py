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

# --- Euler bend router -------------------------------------------------
def _euler_bend_points(theta, Rmin, n=200, turn_sign=+1):
    """
    Sample a symmetric Euler bend (clothoid) of total turn 'theta' (rad),
    with minimum radius Rmin at the mid-point (max curvature). Returns a list
    of (x, y) points starting at (0,0) with initial heading +x, ending with
    heading rotated by 'theta'. 'turn_sign' = +1 (CCW/up), -1 (CW/down).
    """
    import math

    # Split into two halves with linear curvature ramp up then down.
    # For a half-bend with angle phi = theta/2:
    #   k(s) = a*s,   s in [0, Lh],   with a = 1 / (Rmin^2 * phi)
    #   phi = ∫ k ds = a * Lh^2 / 2  and  k(Lh) = a*Lh = 1/Rmin
    # => Lh = 2 * Rmin * phi  (total Euler length = 2 * Lh)
    phi = abs(theta) * 0.5
    Lh = 2.0 * Rmin * phi
    a = 1.0 / (Rmin * Rmin * phi)  # curvature slope

    # integrate first half (ramp up): k(s) = a*s
    # integrate second half (ramp down): k(s) = k_max - a*s
    def integrate_half(x0, y0, psi0, L, k0, k_slope, steps):
        """Integrate one half with curvature k(s) = k0 + k_slope * s."""
        import math
        x, y, psi = x0, y0, psi0
        ds = L / steps
        pts = []
        for i in range(steps):
            s = (i + 0.5) * ds
            k = k0 + k_slope * s
            psi_i = psi0 + (k0 * s + 0.5 * k_slope * s * s)  # ∫k ds
            x_i = x0 + ( (math.sin(psi_i) - math.sin(psi0)) / (k_slope*s if k_slope != 0 else 1e-18) ) if False else None  # not used
            # simple forward Euler for position with small ds:
            psi_step = psi + k * ds
            x += math.cos(psi_step) * ds
            y += math.sin(psi_step) * ds
            psi = psi_step
            pts.append((x, y, psi))
        return pts

    # First half
    sign = 1.0 if turn_sign >= 0 else -1.0
    pts1 = []
    x = y = psi = 0.0
    ds = Lh / (n // 2)
    for i in range(n // 2):
        s = (i + 0.5) * ds
        k = sign * a * s
        psi += k * ds
        x += math.cos(psi) * ds
        y += math.sin(psi) * ds
        pts1.append((x, y, psi))

    # Second half (ramp down): k(s') = k_max - a*s'
    kmax = sign * (1.0 / Rmin)
    pts2 = []
    for i in range(n - n // 2):
        s = (i + 0.5) * ds
        k = kmax - sign * a * s
        psi += k * ds
        x += math.cos(psi) * ds
        y += math.sin(psi) * ds
        pts2.append((x, y, psi))

    # collect XY only
    xy = [(0.0, 0.0)] + [(px, py) for (px, py, _) in pts1 + pts2]
    return xy

def route_euler_bend(parent: gdstk.Cell, A: Port, B: Port, Rmin: float, layer=1, n=200):
    """
    L-shaped route A->B with a 90° Euler bend of minimum radius Rmin.
    Automatically chooses up/down bend and horizontal-first vs vertical-first
    based on available footprint.
    """
    import math, gdstk

    w = min(A.width, B.width)
    rp = gdstk.RobustPath((A.x, A.y), w, layer=layer)

    # Desired total turn magnitude (90°)
    theta = math.pi / 2.0
    turn_up = (B.y >= A.y)  # bend direction
    sign = +1 if turn_up else -1

    # Sample the Euler bend to know its XY footprint (in local coordinates)
    bend_pts = _euler_bend_points(theta=theta, Rmin=Rmin, n=n, turn_sign=sign)
    X_end = bend_pts[-1][0]  # local x-span of the bend
    Y_end = sign * abs(bend_pts[-1][1])  # local y-span (signed)

    dx = B.x - A.x
    dy = B.y - A.y

    # Decide whether to go horizontal-first (then bend) or vertical-first
    horizontal_first = dx >= X_end + 1e-3
    if not horizontal_first:
        # Try vertical-first by rotating the bend 90° the other way
        # We'll route vertically to (A.x, B.y - Y_end), then bend into +x.
        theta_v = math.pi / 2.0
        turn_right = (B.x >= A.x)  # when coming from below/above
        sign_v = +1 if (turn_right if dy >= 0 else not turn_right) else -1
        bend_pts_v = _euler_bend_points(theta=theta_v, Rmin=Rmin, n=n, turn_sign=sign_v)
        X_end_v = bend_pts_v[-1][0]
        Y_end_v = sign_v * abs(bend_pts_v[-1][1])
        vertical_first = abs(dy) >= abs(Y_end_v) + 1e-3
        if vertical_first:
            # 1) vertical to the bend start
            rp.segment((A.x, B.y - Y_end_v), width=w)
            # 2) add Euler points rotated +90° (swap axes) if needed
            #    But here bend_pts_v already computed for the vertical-first case.
            #    Stitch the bend polyline from the current point:
            x0, y0 = A.x, B.y - Y_end_v
            for (px, py) in bend_pts_v[1:]:
                rp.segment((x0 + px, y0 + py), width=w)
            # 3) horizontal to B
            rp.segment((B.x, B.y), width=w)
            parent.add(rp)
            return
        else:
            # Not enough room either way: reduce Rmin automatically to fit
            scale = max(0.1, dx / max(X_end, 1e-9), abs(dy) / max(abs(Y_end), 1e-9))
            Rmin *= scale
            bend_pts = _euler_bend_points(theta=theta, Rmin=Rmin, n=n, turn_sign=sign)
            X_end = bend_pts[-1][0]
            Y_end = sign * abs(bend_pts[-1][1])
            horizontal_first = True  # force

    # Horizontal-first case:
    # 1) straight along +x to the bend start
    bend_start_x = B.x - X_end
    rp.segment((bend_start_x, A.y), width=w)

    # 2) stitch the Euler bend polyline
    x0, y0 = bend_start_x, A.y
    for (px, py) in bend_pts[1:]:
        rp.segment((x0 + px, y0 + py), width=w)

    # 3) final vertical to target B
    rp.segment((B.x, B.y), width=w)

    parent.add(rp)
