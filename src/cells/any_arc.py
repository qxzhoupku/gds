
import gdstk
import math
from ..ports import Port

def PCellAnyArc(radius=10.0, width=0.5, angle_deg=90, layer=1, name="QUARTER_ARC"):
    """
    Returns (cell, ports) of a circle arc waveguide, turning counter-clockwise.
    - radius: centerline radius
    - width: waveguide width
    - angle_deg: arc angle in degrees (e.g., 90 for quarter turn, 180 for U-bend)
    - layer: GDS layer (int or (layer, datatype))
    """
    cell = gdstk.Cell(name)
    theta0 = 0.0
    theta1 = math.radians(angle_deg)
    npts = max(2**10, int(0.5 * abs(angle_deg)))  # sample density

    # Define centerline points
    pts = []
    for i in range(npts + 1):
        t = theta0 + (theta1 - theta0) * i / npts
        pts.append((radius * math.cos(t), radius * math.sin(t)))

    # Create path
    path = gdstk.FlexPath(pts, width, layer=layer)
    cell.add(path)

    # Ports: input at start, output at end
    p0 = Port("W", x=pts[0][0], y=pts[0][1], angle=theta0-math.pi/2, width=width, layer=layer)
    p1 = Port("E", x=pts[-1][0], y=pts[-1][1], angle=theta1+math.pi/2, width=width, layer=layer)
    return cell, {"W": p0, "E": p1}
