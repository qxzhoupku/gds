"""Circular ring resonator with a periodic, linearly varying width."""

import math

import gdstk

from ..layer_map import resolve_wg_layer
from ..ports import Port


def _positive_float(params, key, default=None):
    """Read a strictly positive floating-point PCell parameter."""
    if default is None:
        value = float(params[key])
    else:
        value = float(params.get(key, default))
    if value <= 0:
        raise ValueError(f"{key} must be positive, got {value}.")
    return value


def _positive_integer(params, key, default=None):
    """Read a strictly positive integer PCell parameter without truncation."""
    raw = params[key] if default is None else params.get(key, default)
    value = float(raw)
    if not value.is_integer() or value < 1:
        raise ValueError(f"{key} must be a positive integer, got {raw}.")
    return int(value)


def PCellWidthVaryingRing(params, layers):
    """Point-coupled circular resonator with triangular width modulation.

    The ring centerline is a circle.  Starting at the bottom coupling point,
    each modulation period consists of two equal-length transitions::

        w_1 -> w_2 -> w_1

    Width is linear in arc length within each transition, so ``period_num=N``
    repeats this triangular modulation exactly N times around the ring.

    Required parameters (all lengths in micrometres):
        ring_radius  -- ring centerline radius
        w_1          -- ring width at the bus coupling point
        w_2          -- alternate ring width
        period_num   -- positive integer number of modulation periods

    Optional parameters:
        gap          -- ring-to-bus edge gap at the coupling point (default 0.5)
        w_bus        -- straight bus width (default 1.0)
        L_bus        -- straight bus length (default 2 * ring_radius)
        tolerance    -- RobustPath polygonization tolerance (default 0.001)
        name         -- cell name (default "WIDTH_VARYING_RING")

    Ports ``W`` and ``E`` are the west and east ends of the straight bus.
    """
    radius = _positive_float(params, "ring_radius")
    w_1 = _positive_float(params, "w_1", 1.8)
    w_2 = _positive_float(params, "w_2", 0.8)
    period_num = _positive_integer(params, "period_num", 1)
    gap = _positive_float(params, "gap", 0.5)
    w_bus = _positive_float(params, "w_bus", 1.0)
    bus_length = _positive_float(params, "L_bus", 2.0 * radius)
    tolerance = _positive_float(params, "tolerance", 0.001)
    name = str(params.get("name", "WIDTH_VARYING_RING"))

    if radius <= 0.5 * max(w_1, w_2):
        raise ValueError(
            "ring_radius must exceed half the maximum ring width; "
            f"got radius={radius}, max_width={max(w_1, w_2)}."
        )

    ring_layer = resolve_wg_layer(max(w_1, w_2), layers)
    bus_layer = resolve_wg_layer(w_bus, layers)
    text_layer = layers.get("TEXT", 100)

    cell = gdstk.Cell(name)

    # The local bus is horizontal at y=0.  The ring is shifted upward so the
    # requested edge gap is exact where its local width is w_1.
    x_west = -0.5 * bus_length
    x_east = 0.5 * bus_length
    bus = gdstk.RobustPath((x_west, 0.0), w_bus, layer=bus_layer)
    bus.segment((x_east, 0.0), width=w_bus)
    cell.add(bus)

    center_y = radius + gap + 0.5 * (w_bus + w_1)
    start_angle = -0.5 * math.pi
    start = (
        radius * math.cos(start_angle),
        center_y + radius * math.sin(start_angle),
    )

    ring = gdstk.RobustPath(
        start,
        w_1,
        tolerance=tolerance,
        layer=ring_layer,
    )

    # Split the circle at all w_1/w_2 turning points.  RobustPath's linear
    # width interpolation on a circular arc is linear in angle and therefore
    # also linear in arc length.
    transition_angle = math.pi / period_num
    for transition in range(2 * period_num):
        angle_0 = start_angle + transition * transition_angle
        angle_1 = angle_0 + transition_angle
        target_width = w_2 if transition % 2 == 0 else w_1
        ring.arc(
            radius,
            angle_0,
            angle_1,
            width=(target_width, "linear"),
        )
    cell.add(ring)

    cell.add(
        gdstk.Label(
            f"WIDTH-RING R={radius:g} w={w_1:g}/{w_2:g} N={period_num} gap={gap:g}",
            (0.0, center_y + radius + 10.0),
            anchor="s",
            layer=text_layer,
        )
    )

    ports = {
        "W": Port("W", x_west, 0.0, math.pi, w_bus, bus_layer),
        "E": Port("E", x_east, 0.0, 0.0, w_bus, bus_layer),
    }
    return cell, ports


def PCellConstantWidthRing(params, layers):
    """Point-coupled circular resonator with a constant waveguide width.

    This reference component uses the same centerline-radius convention,
    straight bus, edge-gap definition, and facet-facing ports as
    :func:`PCellWidthVaryingRing`.

    Required parameters (all lengths in micrometres):
        ring_radius  -- ring centerline radius
        width        -- constant ring waveguide width

    Optional parameters:
        gap          -- ring-to-bus edge gap (default 0.5)
        w_bus        -- straight bus width (default 1.0)
        L_bus        -- straight bus length (default 2 * ring_radius)
        tolerance    -- RobustPath polygonization tolerance (default 0.001)
        name         -- cell name (default "CONSTANT_WIDTH_RING")

    Ports ``W`` and ``E`` are the west and east ends of the straight bus.
    """
    radius = _positive_float(params, "ring_radius")
    width = _positive_float(params, "width")
    gap = _positive_float(params, "gap", 0.5)
    w_bus = _positive_float(params, "w_bus", 1.0)
    bus_length = _positive_float(params, "L_bus", 2.0 * radius)
    tolerance = _positive_float(params, "tolerance", 0.001)
    name = str(params.get("name", "CONSTANT_WIDTH_RING"))

    if radius <= 0.5 * width:
        raise ValueError(
            "ring_radius must exceed half the ring width; "
            f"got radius={radius}, width={width}."
        )

    ring_layer = resolve_wg_layer(width, layers)
    bus_layer = resolve_wg_layer(w_bus, layers)
    text_layer = layers.get("TEXT", 100)

    cell = gdstk.Cell(name)

    x_west = -0.5 * bus_length
    x_east = 0.5 * bus_length
    bus = gdstk.RobustPath((x_west, 0.0), w_bus, layer=bus_layer)
    bus.segment((x_east, 0.0), width=w_bus)
    cell.add(bus)

    center_y = radius + gap + 0.5 * (w_bus + width)
    start_angle = -0.5 * math.pi
    start = (
        radius * math.cos(start_angle),
        center_y + radius * math.sin(start_angle),
    )
    ring = gdstk.RobustPath(
        start,
        width,
        tolerance=tolerance,
        layer=ring_layer,
    )
    # Four identical quarter-arcs give the constant reference the same
    # well-conditioned polygonization used by segmented modulated rings.
    for quadrant in range(4):
        angle_0 = start_angle + quadrant * 0.5 * math.pi
        angle_1 = angle_0 + 0.5 * math.pi
        ring.arc(radius, angle_0, angle_1, width=width)
    cell.add(ring)

    cell.add(
        gdstk.Label(
            f"CONST-RING R={radius:g} w={width:g} gap={gap:g}",
            (0.0, center_y + radius + 10.0),
            anchor="s",
            layer=text_layer,
        )
    )

    ports = {
        "W": Port("W", x_west, 0.0, math.pi, w_bus, bus_layer),
        "E": Port("E", x_east, 0.0, 0.0, w_bus, bus_layer),
    }
    return cell, ports
