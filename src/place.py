import math
import gdstk
import warnings
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
        t = -A.angle
        c, s = math.cos(t), math.sin(t)
        return (c*dx - s*dy, s*dx + c*dy)

    def _to_world(A, xl, yl):
        t = A.angle
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
    t = -A.angle
    c, s = math.cos(t), math.sin(t)
    return (c*dx - s*dy, s*dx + c*dy)

def _to_world(A, xl, yl):
    """Inverse of _to_local: rotate by +A.angle then translate by A."""
    import math
    t = A.angle
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



def route_euler_bend(parent, A, B, Rmin, layer, n=100):
    """
    Route an Euler (clothoid) bend between two ports A and B with tangent continuity.
    
    This function adds a smooth Euler bend path to the given GDS cell `parent`, 
    connecting Port A to Port B. The Euler bend transitions from A’s orientation 
    to B’s orientation with curvature that increases linearly (0 -> 1/R -> 0). 
    It ensures the path starts tangent to Port A and ends tangent to Port B.
    
    If the specified minimum bend radius Rmin is too large to fit the two ports, 
    the radius is automatically reduced (warning issued) so that the bend can 
    reach Port B. The path is sampled with `n` points along the Euler curve.
    
    Parameters:
        parent (gdstk.Cell):     Cell to which the waveguide path will be added.
        A, B (Port-like):        Port objects each having attributes x, y (coordinates), 
                                 angle (orientation in degrees), and width.
        Rmin (float):            Minimum bend radius (in the same units as coordinates).
        layer (int, tuple):      GDS layer number (or (layer, datatype) tuple) for the path.
        n (int):                 Number of points to sample along the Euler path.
    
    Returns:
        gdstk.FlexPath: The created FlexPath object representing the routed bend.
    """
    # Extract port parameters
    x1, y1, ang1, w1 = A.x, A.y, A.angle, A.width
    x2, y2, ang2, w2 = B.x, B.y, B.angle, B.width
    # Ensure angles are in radians for calculation
    theta1 = ang1
    theta2 = ang2
    # Coordinates of B relative to A in A's local coordinate system
    # Translate so that A is at origin
    dx = x2 - x1
    dy = y2 - y1
    # Rotate by -theta1 so that A's orientation aligns with the +X axis
    cosA = math.cos(theta1)
    sinA = math.sin(theta1)
    # Apply rotation: (dx, dy) -> (dx', dy')
    x_rel =  dx * cosA + dy * sinA
    y_rel = -dx * sinA + dy * cosA
    # Compute orientation difference (final angle relative to A's angle)
    dtheta = (theta2 - theta1)
    # Normalize dtheta to range [-pi, pi] for consistency
    dtheta = math.atan2(math.sin(dtheta), math.cos(dtheta))
    
    # Handle nearly colinear cases (angle difference ~0 or ~180) separately
    if abs(dtheta) < 1e-6:
        # Ports are parallel (tangent directions almost equal)
        if abs(y_rel) < 1e-9:
            # Ports are essentially on the same line - route straight
            path_points = [complex(0, 0), complex(x_rel, y_rel)]
            # Use width from port A; warn if B's width differs (no taper implemented here)
            if abs(w1 - w2) > 1e-9:
                warnings.warn("Ports have different widths; using A.width for route.")
            flexpath = gdstk.FlexPath(path_points, w1, layer=layer)
            parent.add(flexpath)
            return flexpath
        else:
            # Parallel but offset (S-bend needed). We create a gentle S-shaped Euler curve.
            # We achieve this by connecting A and B with two mirrored Euler bends of half the offset.
            # Compute half of the lateral offset and use a small angle for a smooth transition.
            mid_y = y_rel / 2.0
            # Use a small deflection angle (e.g., 25 degrees) for the S-bend segments
            bend_angle = math.radians(25.0) * math.copysign(1, mid_y)
            # Create intermediate port at halfway with slight angle, then connect in two Euler bends.
            # First half: from A to mid (deflect up), second half: from mid to B (deflect down).
            # Define an intermediate "Port" as a dict (to avoid needing a Port class definition).
            mid_x = x_rel / 2.0
            # Intermediate port orientation: same as A (tangent horizontal) for simplicity
            # (A small non-zero angle could be used if needed for smoother curvature continuity)
            mid_port = type('Port', (), {})()  # create a simple dummy object
            mid_port.x = x1 + cosA * mid_x - sinA * (-mid_y)  # transform back to global for mid
            mid_port.y = y1 + sinA * mid_x + cosA * (-mid_y)
            mid_port.angle = ang1  # maintain same angle as A (horizontal)
            mid_port.width = w1
            # Recursively route two Euler bends (this will call this function for sub-sections)
            path1 = route_euler_bend(parent, A, mid_port, Rmin, layer, n=max(n//2, 50))
            path2 = route_euler_bend(parent, mid_port, B, Rmin, layer, n=max(n//2, 50))
            # We assume route_euler_bend adds the sub-paths to parent. Return a combined FlexPath is not trivial,
            # so we return None as the overall path. (The parent cell now contains the two path segments.)
            return None
    
    # Determine turning direction: use sign of y_rel to choose left vs right turn
    turn_sign = 1 if y_rel >= 0 else -1
    # If dtheta is negative (B oriented clockwise relative to A) but y_rel is positive, or vice versa,
    # it indicates a possibly contradictory geometry; use dtheta sign if so.
    if turn_sign == 0: 
        turn_sign = 1 if dtheta >= 0 else -1
    
    # We will solve for optimal curvature (K = 1/R) and length distribution to hit B.
    # Initial guess for curvature: K_target = 1/Rmin (minimum curvature)
    K_target = 1.0 / float(Rmin) if Rmin > 0 else 1e-6  # avoid division by zero
    phi = abs(dtheta)  # total turn angle magnitude (radians)
    
    # Define a function to compute endpoint for a given K and L1 fraction.
    def compute_endpoint(K, L1_ratio):
        """
        Computes the end coordinates (x_end, y_end) after an Euler curve with curvature K,
        given a fraction L1_ratio = L1 / (L1+L2) (0<ratio<1). The total length L = L1+L2 is set by K and phi.
        Returns (x_end, y_end).
        """
        # Calculate total arc length required for curvature K and angle phi: L_total = 2*phi / K
        L_total = 2 * phi / K
        L1 = L1_ratio * L_total
        L2 = L_total - L1
        # Integrate the Euler curve in two segments: 
        # segment1 (length L1, curvature from 0 to K), segment2 (length L2, curvature from K down to 0).
        steps = max(int(n * (L_total / (Rmin * phi if Rmin*phi != 0 else 1))), 200)  # adaptive steps based on length
        # Ensure at least a minimum number of steps for integration accuracy
        steps1 = int(steps * (L1 / L_total)) if L_total > 0 else 1
        steps2 = steps - steps1 if steps > steps1 else 1
        x_end = 0.0
        y_end = 0.0
        theta = 0.0
        # First segment integration (0 <= s <= L1)
        for i in range(steps1):
            # param along segment
            s = (i + 0.5) * (L1 / steps1)
            k = (K / L1) * s if L1 != 0 else K  # linear ramp from 0 to K
            theta += k * (L1 / steps1)
            x_end += math.cos(theta) * (L1 / steps1)
            y_end += math.sin(theta) * (L1 / steps1)
        # Second segment integration (L1 < s <= L1+L2)
        for j in range(steps2):
            t = (j + 0.5) * (L2 / steps2)
            k = K * (1 - t / L2) if L2 != 0 else 0.0  # linear ramp down from K to 0
            theta += k * (L2 / steps2)
            x_end += math.cos(theta) * (L2 / steps2)
            y_end += math.sin(theta) * (L2 / steps2)
        return x_end, y_end
    
    # Outer iteration: adjust curvature K to meet X alignment
    # We will binary-search on K such that X_end matches x_rel (within tolerance), 
    # while adjusting L1 internally to match y_rel.
    # Determine search bounds for K
    K_low = 0.0
    K_high = None
    # Tolerance for matching position
    tol = 1e-6
    # We'll iterate up to ~30 times (binary search convergence or small tolerance reached)
    for iter_count in range(30):
        # Given current K_target, find L1_ratio such that y_end ~ y_rel using bisection on L1_ratio.
        # Compute y at extremes to bracket:
        L1_lo = 1e-6  # near 0
        L1_hi = 1.0 - 1e-6  # near 1 (can't be exactly 1 or 0)
        # Compute Y at these extremes
        _, y_lo = compute_endpoint(K_target, L1_lo)
        _, y_hi = compute_endpoint(K_target, L1_hi)
        # Adjust if needed: If target y_rel outside [y_hi, y_lo], then current K might not reach target Y.
        if y_rel > max(y_lo, y_hi) + 1e-9 or y_rel < min(y_lo, y_hi) - 1e-9:
            # If y_rel is greater than max achievable (y_lo when L1 -> 0 yields max y), 
            # or y_rel is less than min achievable (y_hi when L2 -> 0 yields min y),
            # adjust curvature K.
            if y_rel > max(y_lo, y_hi):
                # Need more vertical reach -> increase curvature (smaller R)
                K_low = max(K_low, K_target)
                K_target = K_target * 2 if K_high is None else (K_low + K_high) / 2
            elif y_rel < min(y_lo, y_hi):
                # Too much vertical overshoot -> decrease curvature (larger R)
                K_high = K_target if K_high is None else min(K_high, K_target)
                K_target = K_target / 2 if K_low == 0.0 and K_high is None else (K_low + K_high) / 2
            continue  # go to next iteration with adjusted K_target
        
        # Bisection on L1_ratio for y matching
        target_ratio = None
        y_diff = None
        L1_lo_ratio = L1_lo
        L1_hi_ratio = L1_hi
        for _ in range(30):
            mid_ratio = 0.5 * (L1_lo_ratio + L1_hi_ratio)
            _, y_mid = compute_endpoint(K_target, mid_ratio)
            if abs(y_mid - y_rel) < tol:
                target_ratio = mid_ratio
                y_diff = y_mid - y_rel
                break
            # Decide which half contains the solution
            if (y_mid - y_rel) * (y_lo - y_rel) > 0:
                # mid and low have same sign difference, move low up
                L1_lo_ratio = mid_ratio
                y_lo = y_mid
            else:
                # mid and hi have same sign, move high down
                L1_hi_ratio = mid_ratio
                y_hi = y_mid
            target_ratio = mid_ratio
            y_diff = y_mid - y_rel
        # Now we have an L1 ratio that yields y ~ y_rel (within tolerance or max iterations).
        x_end, y_end = compute_endpoint(K_target, target_ratio)
        # Check how close the x_end is to target x_rel
        x_diff = x_end - x_rel
        if abs(x_diff) < tol and (y_diff is None or abs(y_diff) < tol):
            # Converged: found K and L1 that matches both X and Y within tolerance
            break
        # Adjust K bounds based on X difference to converge X
        if x_diff > 0:
            # Path ended too far in X (overshot) -> need to bend more (increase K)
            K_low = max(K_low, K_target)
            if K_high is None:
                # If no upper bound yet, increase K
                K_target *= 2
            else:
                K_target = 0.5 * (K_low + K_high)
        else:
            # Path ended short in X -> bend less (decrease K)
            K_high = K_target if K_high is None else min(K_high, K_target)
            if K_low == 0.0 and K_high is None:
                K_target /= 2
            else:
                K_target = 0.5 * (K_low + K_high)
    else:
        warnings.warn("Euler bend solver did not fully converge; result may be approximate.")
    
    # Final optimized parameters
    K_final = K_target
    # Solve one more time for final L1_ratio using K_final
    # (We reuse target_ratio from above iteration if available)
    if 'target_ratio' not in locals() or target_ratio is None:
        # If not found in loop (which is unlikely), compute it now
        # Use same bisection logic for y on final K
        L1_lo_ratio = 1e-6
        L1_hi_ratio = 1.0 - 1e-6
        _, y_lo = compute_endpoint(K_final, L1_lo_ratio)
        _, y_hi = compute_endpoint(K_final, L1_hi_ratio)
        for _ in range(30):
            mid_ratio = 0.5 * (L1_lo_ratio + L1_hi_ratio)
            _, y_mid = compute_endpoint(K_final, mid_ratio)
            if abs(y_mid - y_rel) < tol:
                target_ratio = mid_ratio
                break
            if (y_mid - y_rel) * (y_lo - y_rel) > 0:
                L1_lo_ratio = mid_ratio
                y_lo = y_mid
            else:
                L1_hi_ratio = mid_ratio
                y_hi = y_mid
            target_ratio = mid_ratio
    L1_ratio_final = target_ratio
    
    # Determine if Rmin was scaled down (if K_final > 1/Rmin)
    if K_final > 1.0 / float(Rmin) + 1e-9:
        new_Rmin = 1.0 / K_final
        warnings.warn(f"route_euler_bend: Reduced Rmin from {Rmin} to {new_Rmin:.3f} to fit bend.")
    
    # Compute the final set of points along the Euler curve with the found parameters
    L_total = 2 * phi / K_final
    L1_final = L1_ratio_final * L_total
    L2_final = L_total - L1_final
    # Sample points along the path (n points)
    path_points = []
    total_steps = n - 1  # number of intervals between n points
    # We will accumulate arc length and sample at equal arc length intervals
    # Compute a high-resolution list of points, then down-sample to n points.
    msteps = max(5 * n, 1000)  # use a finer resolution for smoothing
    xs, ys = [0.0], [0.0]
    theta = 0.0
    # First segment (0 to L1_final)
    if L1_final > 1e-9:
        for i in range(1, int(msteps * (L1_final / (L_total)) + 1)):
            s = i * (L1_final / (msteps * (L1_final / L_total)))
            k = (K_final / L1_final) * s
            theta += k * (L1_final / (msteps * (L1_final / L_total)))
            x_new = xs[-1] + math.cos(theta) * (L1_final / (msteps * (L1_final / L_total)))
            y_new = ys[-1] + math.sin(theta) * (L1_final / (msteps * (L1_final / L_total)))
            xs.append(x_new)
            ys.append(y_new)
    # Second segment (L1_final to L_total)
    if L2_final > 1e-9:
        for j in range(1, int(msteps * (L2_final / L_total) + 1)):
            t = j * (L2_final / (msteps * (L2_final / L_total)))
            k = K_final * (1 - t / L2_final)
            theta += k * (L2_final / (msteps * (L2_final / L_total)))
            x_new = xs[-1] + math.cos(theta) * (L2_final / (msteps * (L2_final / L_total)))
            y_new = ys[-1] + math.sin(theta) * (L2_final / (msteps * (L2_final / L_total)))
            xs.append(x_new)
            ys.append(y_new)
    # Down-sample to n points (approximately uniform along length)
    total_length = 0.0
    lengths = [0.0]
    # Calculate cumulative lengths for the fine path
    for i in range(1, len(xs)):
        dx_i = xs[i] - xs[i-1]
        dy_i = ys[i] - ys[i-1]
        seg_len = math.hypot(dx_i, dy_i)
        total_length += seg_len
        lengths.append(total_length)
    # Now pick n points at equal spacing along total_length
    target_spacing = total_length / (n - 1 if n > 1 else 1)
    idx = 0
    for j in range(n):
        L_target = j * target_spacing
        # Find index such that lengths[idx] <= L_target < lengths[idx+1]
        while idx < len(lengths)-1 and lengths[idx] < L_target:
            idx += 1
        if idx >= len(lengths):
            idx = len(lengths) - 1
        # Linear interpolate between point idx-1 and idx for better accuracy
        if idx == 0 or lengths[idx] == lengths[idx-1]:
            x_interp = xs[idx]
            y_interp = ys[idx]
        else:
            # fractional position between idx-1 and idx
            t_frac = (L_target - lengths[idx-1]) / (lengths[idx] - lengths[idx-1])
            x_interp = xs[idx-1] + t_frac * (xs[idx] - xs[idx-1])
            y_interp = ys[idx-1] + t_frac * (ys[idx] - ys[idx-1])
        # Transform back to global coordinates and add to list
        x_global = x1 + cosA * x_interp - sinA * (turn_sign * y_interp)
        y_global = y1 + sinA * x_interp + cosA * (turn_sign * y_interp)
        path_points.append(complex(x_global, y_global))
    
    # Create the FlexPath for the Euler bend
    width = w1  # assume port widths are equal; otherwise, using A.width
    if abs(w1 - w2) > 1e-9:
        warnings.warn("Ports have different widths; using A.width for route.")
        width = w1
    # If layer is given as just an int, default datatype=0
    if isinstance(layer, tuple):
        layer_num, datatype_num = layer
    else:
        layer_num, datatype_num = layer, 0
    flexpath = gdstk.FlexPath(path_points, width, layer=layer_num, datatype=datatype_num)
    parent.add(flexpath)
    return flexpath

del route_euler_bend

def route_euler_bend(parent, A, B, Rmin, layer, n=100):
    """
    Fast Euler bend route from Port A to Port B using symmetric L or S-shaped bend.
    - Starts aligned to A.angle
    - Ends approximately aligned to B.angle (only if geometry allows)
    - Automatically scales down Rmin if necessary, with warning
    - Adds a gdstk.FlexPath to `parent` and returns it
    """
    x1, y1, a1, w1 = A.x, A.y, A.angle, A.width
    x2, y2, a2, w2 = B.x, B.y, B.angle, B.width
    theta1 = a1

    dx, dy = x2 - x1, y2 - y1
    cos1, sin1 = math.cos(theta1), math.sin(theta1)
    x_rel =  dx * cos1 + dy * sin1
    y_rel = -dx * sin1 + dy * cos1

    L = 1.65 * Rmin
    x_req, y_req = abs(x_rel), abs(y_rel)

    if x_req > L and y_req < 1e-3:
        pts = [(0, 0), (x_rel, 0)]
    elif y_req > L and x_req < 1e-3:
        pts = [(0, 0), (0, y_rel)]
    elif x_req > L and y_req > L:
        L_max = min(x_req, y_req)
        if L > L_max:
            scale = L_max / L
            Rmin_scaled = Rmin * scale
            warnings.warn(f"Euler bend Rmin scaled from {Rmin} to {Rmin_scaled:.3f} to fit geometry.")
            Rmin = Rmin_scaled
            L = 1.65 * Rmin
        t_vals = [i / (n - 1) for i in range(n)]
        pts = [(Rmin * math.sin(math.pi * 0.5 * t),
                Rmin * (1 - math.cos(math.pi * 0.5 * t))) for t in t_vals]
    else:
        mid_y = y_rel / 2
        Rmin_s = max(1.0, min(abs(mid_y) / 1.1, Rmin))
        t_vals = [i / (n - 1) for i in range(n)]
        pts = []
        for t in t_vals:
            phi = math.pi * t
            x = Rmin_s * (phi - math.sin(phi))
            y = mid_y - Rmin_s * (math.cos(phi) - 1)
            pts.append((x, y))

    world_pts = [(x1 + x * cos1 - y * sin1,
                  y1 + x * sin1 + y * cos1) for (x, y) in pts]

    width = w1 if abs(w1 - w2) < 1e-3 else min(w1, w2)
    path = gdstk.FlexPath(world_pts, width, layer=layer)
    parent.add(path)
    return path
