"""Fabrication I/O wrapper for the compact 4 x 4 butterfly ONN core.

The resonator/crossing geometry remains in :mod:`onn_butterfly_network`.
This wrapper adds a deliberately simple optical interface:

* one straight top-apex pump bus addressing all vertical resonators;
* one standardized straight 500 nm-gap drop probe tangent to the requested
  resonator apex (right for horizontal, bottom for vertical);
* circulation-matched drop propagation with every drop routed to a distinct
  lane on the right facet;
* inverse tapers at every exposed facet; and
* an exact, YAML-controlled left-to-right facet span (6000 um by default).

Circulation is uniform within each orientation and opposite between the two
orientations.  Geometry code derives the coupling side from those two values,
so a parameter sweep never needs copied coordinate lists.
"""

from __future__ import annotations

import copy
import math
from typing import Dict, List, Sequence, Tuple

import gdstk
import numpy as np

from ..layer_map import resolve_wg_layer
from ..ports import Port
from .onn_butterfly_network import (
    _add_constant_path,
    _join,
    _line,
    build_onn_butterfly_network,
)


Point = Tuple[float, float]


DEFAULT_CORE = {
    "n": 4,
    "waveguide_width": 1.8,
    "coupling_gap": 0.5,
    "coupling_length": 5.0,
    "min_bend_radius": 30.0,
    "crossing_offset": 150.0,
    "return_arm_offset": 60.0,
    "coupler_offset": 45.0,
    "tile_half_span": 240.0,
    "interaction_group_pitch": 490.0,
    "crossing_taper_length": 5.0,
    "crossing": {"WM": 3.6, "LM": 28.2, "LT": 3.1, "w_in": 1.8},
    "closure_lead": 30.0,
    "flatten": True,
}


DEFAULT_IO = {
    "bus": {
        "width": 1.0,
        "gap": 0.5,
        "bend_radius": 30.0,
    },
    "drop": {
        "coupling_length": 5.0,
        "output_lane_pitch": 70.0,
    },
    "inverse_taper": {
        "tip_width": 0.15,
        "length": 400.0,
    },
    "facets": {
        "span": 6000.0,
        "center_x": 0.0,
    },
    "circulation": {
        "horizontal": "CCW",
        "vertical": "CW",
        "require_opposite": True,
    },
}


def _deep_update(base: dict, updates: dict) -> dict:
    for key, value in updates.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _deep_update(base[key], value)
        else:
            base[key] = copy.deepcopy(value)
    return base


def _defaults(params: dict | None) -> dict:
    design = {
        "name": "ONN_BUTTERFLY_4X4_DEVICE_FINAL",
        "core": copy.deepcopy(DEFAULT_CORE),
        "io": copy.deepcopy(DEFAULT_IO),
        "flatten": True,
    }
    _deep_update(design, params or {})
    return design


def _rotation(point: Sequence[float], theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    x, y = float(point[0]), float(point[1])
    return np.asarray((c * x - s * y, s * x + c * y), dtype=float)


def _transform(point: Sequence[float], origin: Sequence[float], theta: float) -> np.ndarray:
    return _rotation(point, theta) + np.asarray(origin, dtype=float)


def _arc_points(
    start: Sequence[float], heading: float, radius: float, angle: float, samples: int = 81
) -> np.ndarray:
    point = np.asarray(start, dtype=float)
    sign = 1.0 if angle >= 0.0 else -1.0
    left = np.asarray((-math.sin(heading), math.cos(heading)), dtype=float)
    center = point + sign * radius * left
    radial0 = heading - sign * math.pi / 2.0
    phi = np.linspace(radial0, radial0 + angle, samples)
    return center + radius * np.column_stack((np.cos(phi), np.sin(phi)))


def _add_taper(
    cell: gdstk.Cell,
    start: Sequence[float],
    end: Sequence[float],
    width0: float,
    width1: float,
    layer: int,
) -> None:
    path = gdstk.RobustPath(start, width0, layer=layer, tolerance=1e-3)
    path.segment(end, width=width1)
    cell.add(path)


def _all_polygons(cell: gdstk.Cell) -> list:
    return cell.get_polygons(apply_repetitions=True, include_paths=True)


def _polygon_bbox(polygons: list) -> Tuple[float, float, float, float]:
    if not polygons:
        raise ValueError("no polygons available for bounding-box measurement")
    minima = np.min([np.min(poly.points, axis=0) for poly in polygons], axis=0)
    maxima = np.max([np.max(poly.points, axis=0) for poly in polygons], axis=0)
    return float(minima[0]), float(minima[1]), float(maxima[0]), float(maxima[1])


def _normalize_direction(value, label: str) -> str:
    """Accept a scalar or a uniform legacy list and return CW/CCW."""
    if isinstance(value, (list, tuple)):
        normalized = [str(item).upper() for item in value]
        if len(normalized) != 4 or len(set(normalized)) != 1:
            raise ValueError(
                f"{label} must be one CW/CCW value (or four identical legacy values)"
            )
        value = normalized[0]
    direction = str(value).upper()
    if direction not in {"CW", "CCW"}:
        raise ValueError(f"{label} must be CW or CCW")
    return direction


def _validate_circulation(io: dict) -> Dict[str, str]:
    cfg = io["circulation"]
    horizontal = _normalize_direction(cfg["horizontal"], "circulation.horizontal")
    vertical = _normalize_direction(cfg["vertical"], "circulation.vertical")
    if bool(cfg.get("require_opposite", True)) and horizontal == vertical:
        raise ValueError("horizontal and vertical circulation must be opposite")
    return {
        **{f"H{index}": horizontal for index in range(4)},
        **{f"V{index}": vertical for index in range(4)},
    }


def _resonator_records(core_diagnostics: dict, circulation: Dict[str, str]) -> List[dict]:
    canonical = core_diagnostics["canonical"]
    x_left = float(canonical["x_left_um"])
    x_right = float(canonical["x_right_um"])
    y_center = 0.5 * (
        float(canonical["y_active_um"]) + float(canonical["y_return_um"])
    )
    radius = 0.5 * float(canonical["return_arm_offset_um"])
    local_left = np.asarray((x_left, y_center), dtype=float)
    local_right = np.asarray((x_right, y_center), dtype=float)

    records = []
    for placement in core_diagnostics["resonator_placements"]:
        theta = float(placement["rotation_rad"])
        origin = placement["origin_um"]
        closures = [
            _transform(local_left, origin, theta),
            _transform(local_right, origin, theta),
        ]
        center = 0.5 * (closures[0] + closures[1])
        record = {
            **placement,
            "circulation": circulation[placement["id"]],
            "center_um": (float(center[0]), float(center[1])),
            "closure_radius_um": radius,
        }
        if placement["set"] == "horizontal":
            closures.sort(key=lambda point: point[0])
            record["left_closure_um"] = tuple(float(value) for value in closures[0])
            record["right_closure_um"] = tuple(float(value) for value in closures[1])
        else:
            closures.sort(key=lambda point: point[1])
            record["bottom_closure_um"] = tuple(float(value) for value in closures[0])
            record["top_closure_um"] = tuple(float(value) for value in closures[1])
        records.append(record)
    return records


def _facet_coordinates(io: dict) -> Tuple[float, float, float, float]:
    facets = io["facets"]
    taper = io["inverse_taper"]
    span = float(facets["span"])
    center_x = float(facets.get("center_x", 0.0))
    taper_length = float(taper["length"])
    left_x = center_x - 0.5 * span
    right_x = center_x + 0.5 * span
    left_inner = left_x + taper_length
    right_inner = right_x - taper_length
    if left_inner >= right_inner:
        raise ValueError("facet span must exceed twice the inverse-taper length")
    return left_x, right_x, left_inner, right_inner


def _add_left_taper(
    cell: gdstk.Cell,
    y: float,
    left_x: float,
    left_inner: float,
    tip_width: float,
    bus_width: float,
    layer: int,
) -> None:
    # Narrow tip is exactly on the left facet; the guide widens into the die.
    _add_taper(cell, (left_x, y), (left_inner, y), tip_width, bus_width, layer)


def _add_right_taper(
    cell: gdstk.Cell,
    y: float,
    right_inner: float,
    right_x: float,
    bus_width: float,
    tip_width: float,
    layer: int,
) -> None:
    # Full-width guide approaches from the die and narrows at the right facet.
    _add_taper(cell, (right_inner, y), (right_x, y), bus_width, tip_width, layer)


def _add_straight_pump(
    cell: gdstk.Cell,
    vertical: List[dict],
    core_width: float,
    io: dict,
    layer: int,
) -> Tuple[Dict[str, Port], dict]:
    bus = io["bus"]
    taper = io["inverse_taper"]
    bus_width = float(bus["width"])
    gap = float(bus["gap"])
    tip_width = float(taper["tip_width"])
    left_x, right_x, left_inner, right_inner = _facet_coordinates(io)
    separation = 0.5 * (core_width + bus_width) + gap

    vertical = sorted(vertical, key=lambda record: record["center_um"][0])
    direction = vertical[0]["circulation"]
    if any(record["circulation"] != direction for record in vertical):
        raise ValueError("all vertical resonators must have the same circulation")

    # Keep the pump at the top apex, opposite the fixed bottom-apex drops.
    # CW travels right at the top, while CCW travels left.  Reversing the
    # circulation therefore swaps IN and THRU instead of moving the pump onto
    # the drop waveguides.
    tangent = [
        float(record["top_closure_um"][1]) + float(record["closure_radius_um"])
        for record in vertical
    ]
    pump_y = max(tangent) + separation

    _add_constant_path(cell, _line((left_inner, pump_y), (right_inner, pump_y)), bus_width, layer)
    _add_left_taper(cell, pump_y, left_x, left_inner, tip_width, bus_width, layer)
    _add_right_taper(cell, pump_y, right_inner, right_x, bus_width, tip_width, layer)
    if direction == "CW":
        propagation_direction = "right"
        ports = {
            "IN": Port("IN", left_x, pump_y, math.pi, tip_width, layer),
            "THRU": Port("THRU", right_x, pump_y, 0.0, tip_width, layer),
        }
    else:
        propagation_direction = "left"
        ports = {
            "IN": Port("IN", right_x, pump_y, 0.0, tip_width, layer),
            "THRU": Port("THRU", left_x, pump_y, math.pi, tip_width, layer),
        }
    diagnostics = {
        "topology": "single_straight_top_apex",
        "bend_count": 0,
        "coupled_resonators": [record["id"] for record in vertical],
        "resonator_circulation": direction,
        "coupling_side": "top",
        "propagation_direction": propagation_direction,
        "input_facet_side": "left" if direction == "CW" else "right",
        "centerline_y_um": float(pump_y),
        "coupling_edge_gap_um": gap,
        "facet_tip_coordinates_um": [[left_x, pump_y], [right_x, pump_y]],
    }
    return ports, diagnostics


def _add_horizontal_drop(
    cell: gdstk.Cell,
    record: dict,
    core_width: float,
    io: dict,
    layer: int,
) -> Tuple[Port, dict]:
    bus = io["bus"]
    drop = io["drop"]
    taper = io["inverse_taper"]
    bus_width = float(bus["width"])
    gap = float(bus["gap"])
    coupling_length = float(drop["coupling_length"])
    tip_width = float(taper["tip_width"])
    _, right_x, _, right_inner = _facet_coordinates(io)
    separation = 0.5 * (core_width + bus_width) + gap
    bend_radius = float(bus["bend_radius"])
    ring_radius = float(record["closure_radius_um"])
    right_closure_x, center_y = record["right_closure_um"]

    # The probe is tangent to the rightmost apex.  At that point a CW ring
    # travels down and a CCW ring travels up.  The drop follows the same local
    # direction, then makes the single required turn toward the right facet.
    direction_y = -1.0 if record["circulation"] == "CW" else 1.0
    direction_name = "down" if direction_y < 0.0 else "up"
    apex_x = float(right_closure_x) + ring_radius
    probe_x = apex_x + separation
    coupling_start_y = float(center_y) - direction_y * 0.5 * coupling_length
    coupling_end_y = float(center_y) + direction_y * 0.5 * coupling_length
    coupling = _line(
        (probe_x, coupling_start_y), (probe_x, coupling_end_y)
    )
    heading = direction_y * math.pi / 2.0
    turn = _arc_points(
        (probe_x, coupling_end_y),
        heading,
        bend_radius,
        -direction_y * math.pi / 2.0,
    )
    turn_end = turn[-1]
    if right_inner <= float(turn_end[0]):
        raise ValueError("right facet leaves no room for the horizontal drop output")
    trace = _join((coupling, turn, _line(turn_end, (right_inner, turn_end[1]))))
    _add_constant_path(cell, trace, bus_width, layer)
    _add_right_taper(
        cell,
        float(turn_end[1]),
        right_inner,
        right_x,
        bus_width,
        tip_width,
        layer,
    )
    port_name = f"DROP_{record['id']}"
    port = Port(port_name, right_x, float(turn_end[1]), 0.0, tip_width, layer)
    diagnostic = {
        "id": record["id"],
        "set": "horizontal",
        "circulation": record["circulation"],
        "coupling_apex": "right",
        "ring_tangent_direction": direction_name,
        "drop_propagation_direction": direction_name,
        "probe_geometry": "straight",
        "bend_count": 1,
        "minimum_route_bend_radius_um": bend_radius,
        "coupling_edge_gap_um": gap,
        "coupling_length_um": coupling_length,
        "coupling_segment_um": [
            [float(probe_x), float(coupling_start_y)],
            [float(probe_x), float(coupling_end_y)],
        ],
        "apex_centerline_um": [float(apex_x), float(center_y)],
        "facet_side": "right",
        "facet_tip_um": [float(right_x), float(turn_end[1])],
        "port": port_name,
    }
    return port, diagnostic


def _add_vertical_drop(
    cell: gdstk.Cell,
    record: dict,
    lateral_rank: int,
    core_width: float,
    io: dict,
    layer: int,
) -> Tuple[Port, dict]:
    bus = io["bus"]
    drop = io["drop"]
    taper = io["inverse_taper"]
    bus_width = float(bus["width"])
    gap = float(bus["gap"])
    radius = float(bus["bend_radius"])
    coupling_length = float(drop["coupling_length"])
    lane_pitch = float(drop["output_lane_pitch"])
    tip_width = float(taper["tip_width"])
    _, right_x, _, right_inner = _facet_coordinates(io)
    separation = 0.5 * (core_width + bus_width) + gap

    ring_radius = float(record["closure_radius_um"])
    center_x = float(record["center_um"][0])
    bottom_closure_y = float(record["bottom_closure_um"][1])

    # The probe is tangent to the bottommost apex.  At that point a CW ring
    # travels left and a CCW ring travels right.  This local launch direction
    # is fixed by circulation, but every drop must terminate on the right
    # facet.  A left-going (CW) probe therefore makes two same-sense 90 degree
    # turns below the core; a right-going (CCW) probe either remains straight
    # when it is the rightmost probe or uses an S offset to avoid merging with
    # the collinear probes to its right.  ``lateral_rank`` assigns deeper lanes
    # to probes farther left, which prevents route crossings.
    direction_x = -1.0 if record["circulation"] == "CW" else 1.0
    direction_name = "left" if direction_x < 0.0 else "right"
    apex_y = bottom_closure_y - ring_radius
    probe_y = apex_y - separation
    coupling_start_x = center_x - direction_x * 0.5 * coupling_length
    coupling_end_x = center_x + direction_x * 0.5 * coupling_length
    coupling = _line(
        (coupling_start_x, probe_y), (coupling_end_x, probe_y)
    )
    if lateral_rank == 0:
        if direction_x < 0.0:
            raise ValueError("a left-going vertical probe cannot use the straight lane")
        if right_inner <= coupling_end_x:
            raise ValueError("right facet leaves no room for the straight vertical drop")
        trace = _join(
            (coupling, _line((coupling_end_x, probe_y), (right_inner, probe_y)))
        )
        bend_count = 0
        output_y = probe_y
    else:
        output_y = probe_y - lateral_rank * lane_pitch
        first_turn = _arc_points(
            (coupling_end_x, probe_y),
            0.0 if direction_x > 0.0 else math.pi,
            radius,
            -direction_x * math.pi / 2.0,
        )
        first_end = first_turn[-1]
        second_start_y = output_y + radius
        if second_start_y > float(first_end[1]) + 1e-9:
            raise ValueError("drop.output_lane_pitch is too small for 30 um bends")
        vertical_section = _line(
            first_end, (float(first_end[0]), second_start_y)
        )
        second_turn = _arc_points(
            (float(first_end[0]), second_start_y),
            -math.pi / 2.0,
            radius,
            math.pi / 2.0,
        )
        second_end = second_turn[-1]
        if right_inner <= float(second_end[0]):
            raise ValueError("right facet leaves no room for the routed vertical drop")
        trace = _join(
            (
                coupling,
                first_turn,
                vertical_section,
                second_turn,
                _line(second_end, (right_inner, output_y)),
            )
        )
        bend_count = 2

    _add_constant_path(cell, trace, bus_width, layer)
    _add_right_taper(
        cell, output_y, right_inner, right_x, bus_width, tip_width, layer
    )
    port_name = f"DROP_{record['id']}"
    port = Port(port_name, right_x, output_y, 0.0, tip_width, layer)
    diagnostic = {
        "id": record["id"],
        "set": "vertical",
        "circulation": record["circulation"],
        "coupling_apex": "bottom",
        "ring_tangent_direction": direction_name,
        "drop_propagation_direction": direction_name,
        "probe_geometry": "straight",
        "bend_count": bend_count,
        "minimum_route_bend_radius_um": radius if bend_count else None,
        "coupling_edge_gap_um": gap,
        "coupling_length_um": coupling_length,
        "coupling_segment_um": [
            [float(coupling_start_x), float(probe_y)],
            [float(coupling_end_x), float(probe_y)],
        ],
        "apex_centerline_um": [float(center_x), float(apex_y)],
        "facet_side": "right",
        "facet_tip_um": [float(right_x), float(output_y)],
        "port": port_name,
        "lateral_rank": int(lateral_rank),
    }
    return port, diagnostic


def _add_drop_probes(
    cell: gdstk.Cell,
    records: List[dict],
    core_width: float,
    io: dict,
    layer: int,
) -> Tuple[Dict[str, Port], List[dict]]:
    ports: Dict[str, Port] = {}
    diagnostics: List[dict] = []

    horizontal = sorted(
        (record for record in records if record["set"] == "horizontal"),
        key=lambda record: record["center_um"][1],
    )
    for record in horizontal:
        port, diagnostic = _add_horizontal_drop(cell, record, core_width, io, layer)
        ports[port.name] = port
        diagnostics.append(diagnostic)

    vertical = sorted(
        (record for record in records if record["set"] == "vertical"),
        key=lambda record: record["center_um"][0],
    )
    direction = vertical[0]["circulation"]
    if any(record["circulation"] != direction for record in vertical):
        raise ValueError("all vertical resonators must have the same circulation")
    count = len(vertical)
    for index, record in enumerate(vertical):
        # Deeper lanes belong to probes farther left.  For CW every probe must
        # first reverse its left-going tangent, so ranks are N..1.  For CCW the
        # rightmost probe can remain straight, so ranks are N-1..0.
        lateral_rank = count - index if direction == "CW" else count - 1 - index
        port, diagnostic = _add_vertical_drop(
            cell, record, lateral_rank, core_width, io, layer
        )
        ports[port.name] = port
        diagnostics.append(diagnostic)

    by_id = {item["id"]: item for item in diagnostics}
    ordered = [by_id[f"H{index}"] for index in range(4)] + [
        by_id[f"V{index}"] for index in range(4)
    ]
    return ports, ordered


def _validate_parameters(design: dict) -> None:
    core = design["core"]
    io = design["io"]
    numeric = (
        ("bus.width", io["bus"]["width"]),
        ("bus.gap", io["bus"]["gap"]),
        ("bus.bend_radius", io["bus"]["bend_radius"]),
        ("drop.coupling_length", io["drop"]["coupling_length"]),
        ("drop.output_lane_pitch", io["drop"]["output_lane_pitch"]),
        ("inverse_taper.tip_width", io["inverse_taper"]["tip_width"]),
        ("inverse_taper.length", io["inverse_taper"]["length"]),
        ("facets.span", io["facets"]["span"]),
    )
    for label, raw in numeric:
        if float(raw) <= 0.0:
            raise ValueError(f"{label} must be positive")
    if float(io["bus"]["bend_radius"]) < float(core["min_bend_radius"]):
        raise ValueError("bus.bend_radius must be >= core.min_bend_radius")
    if float(io["drop"]["coupling_length"]) > float(core["closure_lead"]):
        raise ValueError("drop.coupling_length must not exceed core.closure_lead")
    minimum_pitch = 2.0 * float(io["bus"]["bend_radius"])
    if float(io["drop"]["output_lane_pitch"]) < minimum_pitch:
        raise ValueError("drop.output_lane_pitch must be at least twice bus.bend_radius")


def build_onn_butterfly_device(params: dict | None, layers: dict):
    """Build the stable ONN core plus the final fabrication I/O."""
    design = _defaults(params)
    _validate_parameters(design)
    name = str(design["name"])
    core_params = copy.deepcopy(design["core"])
    io = design["io"]
    core_params["name"] = f"{name}_CORE"
    core_params["resonator_name"] = f"{name}_RESONATOR"
    core_params.setdefault("crossing", {})["name"] = f"{name}_WX"
    core_params["flatten"] = True

    circulation = _validate_circulation(io)
    core, _, core_diagnostics = build_onn_butterfly_network(core_params, layers)
    core_width = float(core_params["waveguide_width"])
    bus_width = float(io["bus"]["width"])
    tip_width = float(io["inverse_taper"]["tip_width"])
    core_layer = resolve_wg_layer(core_width, layers)
    bus_layer = resolve_wg_layer(bus_width, layers)
    tip_layer = resolve_wg_layer(tip_width, layers)
    text_layer = layers.get("TEXT", 100)

    core_polygons = _all_polygons(core)
    core_bbox = _polygon_bbox(core_polygons)
    resonators = _resonator_records(core_diagnostics, circulation)
    horizontal = [record for record in resonators if record["set"] == "horizontal"]
    vertical = [record for record in resonators if record["set"] == "vertical"]

    io_cell = gdstk.Cell(f"{name}_IO")
    drop_ports, drop_diagnostics = _add_drop_probes(
        io_cell, resonators, core_width, io, bus_layer
    )
    feed_ports, feed_diagnostics = _add_straight_pump(
        io_cell, vertical, core_width, io, bus_layer
    )
    ports = {**feed_ports, **drop_ports}
    for port in ports.values():
        # Match PCellTaper: taper geometry uses the wider bus layer; the
        # exposed port advertises the layer resolved for the narrow tip.
        port.layer = tip_layer

    io_polygons = _all_polygons(io_cell)
    accidental_overlap = gdstk.boolean(
        core_polygons,
        io_polygons,
        "and",
        precision=1e-3,
        layer=bus_layer,
    )
    if accidental_overlap:
        raise ValueError("I/O waveguides overlap the ONN core; increase bus.gap")

    device = gdstk.Cell(name)
    device.add(gdstk.Reference(core), gdstk.Reference(io_cell))
    left_x, right_x, _, _ = _facet_coordinates(io)

    horizontal_direction = horizontal[0]["circulation"]
    vertical_direction = vertical[0]["circulation"]
    device.add(
        gdstk.Label(
            f"H: {horizontal_direction}   V: {vertical_direction}",
            (core_bbox[0], core_bbox[3] + 25.0),
            layer=text_layer,
        )
    )

    physical_bbox = _polygon_bbox(core_polygons + io_polygons)
    physical_span = physical_bbox[2] - physical_bbox[0]
    requested_span = float(io["facets"]["span"])
    if not math.isclose(physical_bbox[0], left_x, abs_tol=1e-6):
        raise ValueError("left inverse-taper tip is not on the requested facet plane")
    if not math.isclose(physical_bbox[2], right_x, abs_tol=1e-6):
        raise ValueError("right inverse-taper tips are not on the requested facet plane")
    if not math.isclose(physical_span, requested_span, abs_tol=1e-6):
        raise ValueError("physical left-to-right span does not match facets.span")

    preflatten_reference_count = len(device.references)
    if bool(design.get("flatten", True)):
        device.flatten()
    device_bbox = device.bounding_box()
    if device_bbox is None:
        raise ValueError("device cell is empty")

    diagnostics = {
        "parameters": design,
        "core": core_diagnostics,
        "circulation": {
            "horizontal": horizontal_direction,
            "vertical": vertical_direction,
            **circulation,
        },
        "resonators": resonators,
        "feed_bus": feed_diagnostics,
        "drop_buses": drop_diagnostics,
        "drop_probe_count": len(drop_diagnostics),
        "straight_drop_probe_count": sum(
            item["probe_geometry"] == "straight" for item in drop_diagnostics
        ),
        "horizontal_drop_bend_count": sum(
            item["bend_count"] for item in drop_diagnostics if item["set"] == "horizontal"
        ),
        "vertical_drop_bend_count": sum(
            item["bend_count"] for item in drop_diagnostics if item["set"] == "vertical"
        ),
        "total_drop_bend_count": sum(item["bend_count"] for item in drop_diagnostics),
        "port_names": sorted(ports),
        "port_count": len(ports),
        "left_facet_port_names": sorted(
            port.name for port in ports.values() if port.x < 0.0
        ),
        "right_facet_port_names": sorted(
            port.name for port in ports.values() if port.x > 0.0
        ),
        "right_facet_drop_port_count": sum(
            port.name.startswith("DROP_") and port.x > 0.0
            for port in ports.values()
        ),
        "facet_count": len(ports),
        "input_port_count": 1,
        "through_port_count": 1,
        "drop_port_count": 8,
        "inverse_taper_count": len(ports),
        "inverse_taper_tip_width_um": tip_width,
        "inverse_taper_length_um": float(io["inverse_taper"]["length"]),
        "facet_left_x_um": left_x,
        "facet_right_x_um": right_x,
        "facet_span_um": requested_span,
        "physical_waveguide_bbox_um": [
            [physical_bbox[0], physical_bbox[1]],
            [physical_bbox[2], physical_bbox[3]],
        ],
        "core_waveguide_bbox_um": [
            [core_bbox[0], core_bbox[1]],
            [core_bbox[2], core_bbox[3]],
        ],
        "device_bbox_um": [
            [float(device_bbox[0][0]), float(device_bbox[0][1])],
            [float(device_bbox[1][0]), float(device_bbox[1][1])],
        ],
        "preflatten_reference_count": preflatten_reference_count,
        "accidental_io_core_overlap_polygon_count": len(accidental_overlap),
        "core_layer": core_layer,
        "bus_layer": bus_layer,
        "facet_tip_layer": tip_layer,
        "fabrication_polygon_layers": sorted(
            {polygon.layer for polygon in _all_polygons(device)}
        ),
        "label_layers": sorted({label.layer for label in device.labels}),
    }
    return device, ports, diagnostics


def PCellONNButterflyDevice(params: dict, layers: dict):
    """Registry-compatible final device wrapper."""
    cell, ports, _ = build_onn_butterfly_device(params, layers)
    return cell, ports
