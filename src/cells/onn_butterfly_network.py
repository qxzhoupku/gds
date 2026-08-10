"""Four-by-four butterfly-coupled resonator network.

The network contains two sets of four identical closed resonators.  Four
butterfly blocks are arranged as a 2 x 2 matrix.  Each butterfly contains two
quasi-horizontal active arms and two quasi-vertical active arms, so it creates
four distinct directional couplers.  Consequently every horizontal/vertical
resonator pair couples exactly once and the complete network has 16 coupling
regions.

Every resonator is a translated/rotated reference to one continuous canonical
loop.  After placement, the resonators are flattened, the ordinary
intersections are opened, and the existing :func:`PCellWx` is inserted.  Each
cross-set resonator pair has four ordinary intersections (active-active,
active-return, return-active, and return-return), giving 64 crossing PCells.
"""

from __future__ import annotations

import math
from typing import Dict, Iterable, List, Sequence, Tuple

import gdstk
import numpy as np

from ..layer_map import resolve_wg_layer
from .wx import PCellWx


Point = Tuple[float, float]


def _unit(theta: float) -> np.ndarray:
    return np.array((math.cos(theta), math.sin(theta)), dtype=float)


def _distance(a: Sequence[float], b: Sequence[float]) -> float:
    return float(np.linalg.norm(np.asarray(b, dtype=float) - np.asarray(a, dtype=float)))


def _polyline_length(points: Sequence[Sequence[float]]) -> float:
    points = np.asarray(points, dtype=float)
    if len(points) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())


def _bezier_points(control: np.ndarray, samples: int) -> np.ndarray:
    degree = len(control) - 1
    t = np.linspace(0.0, 1.0, samples)
    output = np.zeros((samples, 2), dtype=float)
    for index, point in enumerate(control):
        coefficient = (
            math.comb(degree, index)
            * (1.0 - t) ** (degree - index)
            * t**index
        )
        output += coefficient[:, None] * point
    return output


def _bezier_min_radius(control: np.ndarray, samples: int = 501) -> float:
    degree = len(control) - 1
    d1_control = degree * np.diff(control, axis=0)
    d2_control = (degree - 1) * np.diff(d1_control, axis=0)
    d1 = _bezier_points(d1_control, samples)
    d2 = _bezier_points(d2_control, samples)
    speed_squared = np.sum(d1 * d1, axis=1)
    cross = np.abs(d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0])
    valid = (speed_squared > 1e-16) & (cross > 1e-14)
    if not np.any(valid):
        return math.inf
    radius = speed_squared[valid] ** 1.5 / cross[valid]
    return float(np.min(radius))


def _quintic_connector(
    p0: Sequence[float],
    theta0: float,
    p1: Sequence[float],
    theta1: float,
    min_radius: float,
    samples: int = 121,
    label: str = "connector",
) -> Tuple[np.ndarray, float]:
    """Return the shortest sampled zero-end-curvature connector meeting Rmin."""
    p0 = np.asarray(p0, dtype=float)
    p1 = np.asarray(p1, dtype=float)
    chord = _distance(p0, p1)
    if chord <= 1e-9:
        raise ValueError(f"{label}: coincident connector endpoints")

    tangent0 = _unit(theta0)
    tangent1 = _unit(theta1)
    candidates = []
    for factor0 in np.linspace(0.08, 0.95, 36):
        handle0 = chord * float(factor0)
        for factor1 in np.linspace(0.08, 0.95, 36):
            handle1 = chord * float(factor1)
            control = np.vstack(
                (
                    p0,
                    p0 + handle0 * tangent0,
                    p0 + 2.0 * handle0 * tangent0,
                    p1 - 2.0 * handle1 * tangent1,
                    p1 - handle1 * tangent1,
                    p1,
                )
            )
            radius = _bezier_min_radius(control, samples=181)
            if radius + 1e-6 < min_radius:
                continue
            points = _bezier_points(control, samples)
            candidates.append((_polyline_length(points), -radius, points, radius))

    if not candidates:
        raise ValueError(
            f"{label}: no smooth connector satisfies Rmin={min_radius:g} um; "
            "increase crossing_offset or reduce coupler_offset"
        )
    _, _, points, radius = min(candidates, key=lambda item: (item[0], item[1]))
    return points, radius


def _join(parts: Iterable[np.ndarray]) -> np.ndarray:
    output: List[np.ndarray] = []
    for part in parts:
        part = np.asarray(part, dtype=float)
        if len(part) == 0:
            continue
        if output and np.linalg.norm(output[-1][-1] - part[0]) < 1e-8:
            part = part[1:]
        if len(part):
            output.append(part)
    return np.vstack(output)


def _line(a: Sequence[float], b: Sequence[float]) -> np.ndarray:
    return np.asarray((a, b), dtype=float)


def _rotate90(points: np.ndarray) -> np.ndarray:
    points = np.asarray(points, dtype=float)
    return np.column_stack((-points[:, 1], points[:, 0]))


def _arc(
    start: Sequence[float], heading: float, radius: float, angle: float, samples: int = 161
) -> Tuple[np.ndarray, float]:
    point = np.asarray(start, dtype=float)
    sign = 1.0 if angle >= 0.0 else -1.0
    left = np.array((-math.sin(heading), math.cos(heading)))
    center = point + sign * radius * left
    radial0 = heading - sign * math.pi / 2.0
    phi = np.linspace(radial0, radial0 + angle, samples)
    points = center + radius * np.column_stack((np.cos(phi), np.sin(phi)))
    return points, heading + angle


def _add_constant_path(
    cell: gdstk.Cell, points: np.ndarray, width: float, layer: int
) -> None:
    cell.add(
        gdstk.FlexPath(
            points,
            width,
            joins="natural",
            ends="flush",
            tolerance=1e-3,
            layer=layer,
        )
    )


def _add_taper(
    cell: gdstk.Cell,
    p0: Sequence[float],
    p1: Sequence[float],
    width0: float,
    width1: float,
    layer: int,
) -> None:
    path = gdstk.RobustPath(p0, width0, layer=layer, tolerance=1e-3)
    path.segment(p1, width=width1)
    cell.add(path)


def _interaction_template(params: dict, crossing_access: float) -> dict:
    """Build the continuous upper active arm of one butterfly."""
    width = float(params["waveguide_width"])
    gap = float(params["coupling_gap"])
    coupling_length = float(params["coupling_length"])
    min_radius = float(params["min_bend_radius"])
    crossing_offset = float(params["crossing_offset"])
    coupler_offset = float(params["coupler_offset"])
    tile_half_span = float(params["tile_half_span"])
    return_offset = crossing_offset + float(params["return_arm_offset"])

    if tile_half_span <= return_offset + crossing_access:
        raise ValueError(
            "tile_half_span must exceed crossing_offset + return_arm_offset + "
            "WX half-span + crossing_taper_length"
        )
    if crossing_offset <= coupler_offset:
        raise ValueError("crossing_offset must exceed coupler_offset")

    centerline_gap = width + gap
    half_gap_diagonal = centerline_gap / (2.0 * math.sqrt(2.0))
    inner = coupler_offset - half_gap_diagonal
    outer = coupler_offset + half_gap_diagonal
    if inner <= 0.0:
        raise ValueError("coupler_offset is too small for the requested width and gap")

    down = _unit(-math.pi / 4.0)
    up = _unit(math.pi / 4.0)
    left_mid = np.array((-inner, outer))
    right_mid = np.array((inner, outer))
    left_start = left_mid - 0.5 * coupling_length * down
    left_end = left_mid + 0.5 * coupling_length * down
    right_start = right_mid - 0.5 * coupling_length * up
    right_end = right_mid + 0.5 * coupling_length * up

    y_cross = crossing_offset
    left_after_crossing = np.array((-crossing_offset + crossing_access, y_cross))
    right_before_crossing = np.array((crossing_offset - crossing_access, y_cross))

    left_connector, left_radius = _quintic_connector(
        left_after_crossing,
        0.0,
        left_start,
        -math.pi / 4.0,
        min_radius,
        label="crossing_to_left_coupler",
    )
    middle_connector, middle_radius = _quintic_connector(
        left_end,
        -math.pi / 4.0,
        right_start,
        math.pi / 4.0,
        min_radius,
        label="continuous_inner_bend",
    )
    right_connector = left_connector[::-1] * np.array((-1.0, 1.0))

    top_trace = _join(
        (
            _line((-tile_half_span, y_cross), left_after_crossing),
            left_connector,
            _line(left_start, left_end),
            middle_connector,
            _line(right_start, right_end),
            right_connector,
            _line(right_before_crossing, (tile_half_span, y_cross)),
        )
    )

    coupling_pairs = [
        ((-inner, outer), (-outer, inner)),
        ((inner, outer), (outer, inner)),
        ((-inner, -outer), (-outer, -inner)),
        ((inner, -outer), (outer, -inner)),
    ]
    coupling_centers = [
        (
            0.5 * (pair[0][0] + pair[1][0]),
            0.5 * (pair[0][1] + pair[1][1]),
        )
        for pair in coupling_pairs
    ]
    measured_centerline_gaps = [_distance(pair[0], pair[1]) for pair in coupling_pairs]

    return {
        "top_trace": top_trace,
        "coupling_pairs": coupling_pairs,
        "coupling_centers": coupling_centers,
        "measured_centerline_gaps": measured_centerline_gaps,
        "measured_edge_gaps": [value - width for value in measured_centerline_gaps],
        "connector_radii": {
            "crossing_to_coupler": left_radius,
            "continuous_inner_bend": middle_radius,
            "coupler_to_crossing": left_radius,
        },
        "return_offset": return_offset,
    }


def _canonical_resonator(
    params: dict, layer: int, crossing_access: float
) -> Tuple[gdstk.Cell, dict]:
    width = float(params["waveguide_width"])
    tile_half_span = float(params["tile_half_span"])
    group_pitch = float(params["interaction_group_pitch"])
    closure_lead = float(params["closure_lead"])
    return_arm_offset = float(params["return_arm_offset"])
    template = _interaction_template(params, crossing_access)
    crossing_offset = float(params["crossing_offset"])
    return_offset = float(template["return_offset"])
    butterfly_x = np.asarray((-0.5 * group_pitch, 0.5 * group_pitch), dtype=float)

    if group_pitch <= 2.0 * tile_half_span:
        raise ValueError("interaction_group_pitch must exceed 2 * tile_half_span")
    if 0.5 * return_arm_offset < float(params["min_bend_radius"]):
        raise ValueError(
            "return_arm_offset / 2 is the closure bend radius and must be at "
            "least min_bend_radius"
        )

    left_center, right_center = butterfly_x
    active_trace = _join(
        (
            template["top_trace"] + np.array((left_center, 0.0)),
            _line(
                (left_center + tile_half_span, crossing_offset),
                (right_center - tile_half_span, crossing_offset),
            ),
            template["top_trace"] + np.array((right_center, 0.0)),
        )
    )

    active_left = float(left_center - tile_half_span)
    active_right = float(right_center + tile_half_span)
    x_left = active_left - closure_lead
    x_right = active_right + closure_lead
    closure_radius = 0.5 * return_arm_offset
    right_turn, _ = _arc(
        (x_right, crossing_offset), 0.0, closure_radius, math.pi
    )
    left_turn, _ = _arc(
        (x_left, return_offset), math.pi, closure_radius, math.pi
    )

    loop_trace = _join(
        (
            active_trace,
            _line((active_right, crossing_offset), (x_right, crossing_offset)),
            right_turn,
            _line((x_right, return_offset), (x_left, return_offset)),
            left_turn,
            _line((x_left, crossing_offset), (active_left, crossing_offset)),
        )
    )
    if np.linalg.norm(loop_trace[0] - loop_trace[-1]) > 1e-8:
        raise ValueError("Canonical resonator centerline did not close")

    cell = gdstk.Cell(str(params.get("resonator_name", "ONN_CANONICAL_RESONATOR")))
    _add_constant_path(cell, loop_trace, width, layer)

    crossing_offsets = [
        -return_offset,
        -crossing_offset,
        crossing_offset,
        return_offset,
    ]
    relative_crossings = [
        (float(center + offset), y)
        for center in butterfly_x
        for y in (crossing_offset, return_offset)
        for offset in crossing_offsets
    ]
    minimum_radius = min(
        closure_radius,
        *template["connector_radii"].values(),
    )

    diagnostics = {
        "butterfly_x_positions_um": [float(value) for value in butterfly_x],
        "optical_path_length_um": _polyline_length(loop_trace),
        "minimum_bend_radius_um": float(minimum_radius),
        "active_arm_y_um": crossing_offset,
        "return_arm_y_um": return_offset,
        "return_arm_offset_um": return_arm_offset,
        "relative_crossing_centers_um": relative_crossings,
        "crossings_on_single_resonator": len(relative_crossings),
        "local_coupling_edge_gaps_um": template["measured_edge_gaps"],
        "local_coupling_centers_um": template["coupling_centers"],
        "connector_radii_um": template["connector_radii"],
        "x_left_um": x_left,
        "x_right_um": x_right,
        "y_active_um": crossing_offset,
        "y_return_um": return_offset,
        "centerline_point_count": len(loop_trace),
    }
    return cell, diagnostics


def _defaults(params: dict | None) -> dict:
    defaults = {
        "name": "ONN_BUTTERFLY_4X4_CORRECTED",
        "resonator_name": "ONN_CANONICAL_RESONATOR",
        "n": 4,
        "waveguide_width": 1.8,
        "coupling_gap": 0.5,
        "coupling_length": 5.0,
        "min_bend_radius": 50.0,
        "crossing_offset": 300.0,
        "return_arm_offset": 150.0,
        "coupler_offset": 135.0,
        "tile_half_span": 560.0,
        "interaction_group_pitch": 1300.0,
        "crossing_taper_length": 30.0,
        "closure_lead": 100.0,
        "flatten": True,
        "crossing": {"WM": 1.6, "LM": 8.0, "LT": 10.0, "w_in": 0.5},
    }
    params = params or {}
    for key, value in params.items():
        if key == "crossing":
            defaults["crossing"].update(value)
        else:
            defaults[key] = value
    return defaults


def _validate_positive(params: dict) -> None:
    for key in (
        "waveguide_width",
        "coupling_gap",
        "coupling_length",
        "min_bend_radius",
        "crossing_offset",
        "return_arm_offset",
        "coupler_offset",
        "tile_half_span",
        "interaction_group_pitch",
        "crossing_taper_length",
        "closure_lead",
    ):
        if float(params[key]) <= 0.0:
            raise ValueError(f"{key} must be positive")


def _crossing_geometry(
    crossing_params: dict, layers: dict
) -> Tuple[gdstk.Cell, Dict[str, object], float]:
    params = dict(crossing_params)
    params.setdefault("name", "ONN_WX")
    cell, ports = PCellWx(params, layers)
    half_span = float(ports["E"].x)
    if not (
        math.isclose(-float(ports["W"].x), half_span, abs_tol=1e-9)
        and math.isclose(float(ports["N"].y), half_span, abs_tol=1e-9)
        and math.isclose(-float(ports["S"].y), half_span, abs_tol=1e-9)
    ):
        raise ValueError("PCellWx must have symmetric W/E/S/N port positions")
    return cell, ports, half_span


def _resonator_placements(group_centers: np.ndarray) -> List[dict]:
    placements: List[dict] = []
    for row, y in enumerate(group_centers):
        placements.extend(
            (
                {
                    "id": f"H{2 * row}",
                    "set": "horizontal",
                    "arm": "bottom",
                    "origin_um": (0.0, float(y)),
                    "rotation_rad": math.pi,
                },
                {
                    "id": f"H{2 * row + 1}",
                    "set": "horizontal",
                    "arm": "top",
                    "origin_um": (0.0, float(y)),
                    "rotation_rad": 0.0,
                },
            )
        )
    for column, x in enumerate(group_centers):
        placements.extend(
            (
                {
                    "id": f"V{2 * column}",
                    "set": "vertical",
                    "arm": "left",
                    "origin_um": (float(x), 0.0),
                    "rotation_rad": math.pi / 2.0,
                },
                {
                    "id": f"V{2 * column + 1}",
                    "set": "vertical",
                    "arm": "right",
                    "origin_um": (float(x), 0.0),
                    "rotation_rad": -math.pi / 2.0,
                },
            )
        )
    return placements


def _interaction_diagnostics(
    group_centers: np.ndarray, params: dict, template: dict
) -> Tuple[List[Point], List[dict], List[dict]]:
    crossing_offset = float(params["crossing_offset"])
    return_offset = crossing_offset + float(params["return_arm_offset"])
    arm_offsets = (-return_offset, -crossing_offset, crossing_offset, return_offset)
    crossing_centers: List[Point] = []
    pair_records: List[dict] = []
    butterflies: List[dict] = []

    for row, center_y in enumerate(group_centers):
        horizontal = (
            (f"H{2 * row}", -1.0),
            (f"H{2 * row + 1}", 1.0),
        )
        for column, center_x in enumerate(group_centers):
            vertical = (
                (f"V{2 * column}", -1.0),
                (f"V{2 * column + 1}", 1.0),
            )
            block_crossings = [
                (float(center_x + dx), float(center_y + dy))
                for dx in arm_offsets
                for dy in arm_offsets
            ]
            crossing_centers.extend(block_crossings)

            block_pairs = []
            for horizontal_id, horizontal_sign in horizontal:
                for vertical_id, vertical_sign in vertical:
                    pair_crossings = [
                        (
                            float(center_x + vertical_sign * x_offset),
                            float(center_y + horizontal_sign * y_offset),
                        )
                        for x_offset in (crossing_offset, return_offset)
                        for y_offset in (crossing_offset, return_offset)
                    ]
                    center_index = (
                        (0 if horizontal_sign > 0 else 2)
                        + (0 if vertical_sign < 0 else 1)
                    )
                    local_coupling = template["coupling_centers"][center_index]
                    coupling_center = (
                        float(center_x + local_coupling[0]),
                        float(center_y + local_coupling[1]),
                    )
                    record = {
                        "horizontal": horizontal_id,
                        "vertical": vertical_id,
                        "butterfly_index": [row, column],
                        "coupling_center_um": coupling_center,
                        "normal_crossing_centers_um": pair_crossings,
                    }
                    pair_records.append(record)
                    block_pairs.append([horizontal_id, vertical_id])

            butterflies.append(
                {
                    "index": [row, column],
                    "center_um": [float(center_x), float(center_y)],
                    "coupling_pairs": block_pairs,
                    "coupling_region_count": 4,
                    "normal_crossing_count": 16,
                    "normal_crossing_centers_um": block_crossings,
                }
            )

    return crossing_centers, pair_records, butterflies


def _cut_and_insert_crossings(
    resonator_source: gdstk.Cell,
    target: gdstk.Cell,
    crossing_centers: Sequence[Point],
    crossing_cell: gdstk.Cell,
    wx_half_span: float,
    resonator_width: float,
    crossing_width: float,
    taper_length: float,
    waveguide_layer: int,
) -> int:
    access = wx_half_span + taper_length
    source_polygons = resonator_source.get_polygons(
        apply_repetitions=True, include_paths=True, layer=waveguide_layer, datatype=0
    )
    keepouts = [
        gdstk.rectangle(
            (x - access, y - access),
            (x + access, y + access),
            layer=waveguide_layer,
        )
        for x, y in crossing_centers
    ]
    cut_polygons = gdstk.boolean(
        source_polygons,
        keepouts,
        "not",
        precision=1e-3,
        layer=waveguide_layer,
    )
    if cut_polygons is None:
        raise ValueError("Crossing keepout subtraction removed the complete resonator geometry")
    target.add(*cut_polygons)

    for x, y in crossing_centers:
        _add_taper(
            target,
            (x - access, y),
            (x - wx_half_span, y),
            resonator_width,
            crossing_width,
            waveguide_layer,
        )
        _add_taper(
            target,
            (x + wx_half_span, y),
            (x + access, y),
            crossing_width,
            resonator_width,
            waveguide_layer,
        )
        _add_taper(
            target,
            (x, y - access),
            (x, y - wx_half_span),
            resonator_width,
            crossing_width,
            waveguide_layer,
        )
        _add_taper(
            target,
            (x, y + wx_half_span),
            (x, y + access),
            crossing_width,
            resonator_width,
            waveguide_layer,
        )
        target.add(gdstk.Reference(crossing_cell, origin=(x, y)))
    return len(cut_polygons)


def build_onn_butterfly_resonator(params: dict | None, layers: dict):
    """Build one continuous review resonator and return diagnostics."""
    design = _defaults(params)
    _validate_positive(design)
    _, _, wx_half_span = _crossing_geometry(design["crossing"], layers)
    crossing_access = wx_half_span + float(design["crossing_taper_length"])
    waveguide_layer = resolve_wg_layer(float(design["waveguide_width"]), layers)
    resonator, diagnostics = _canonical_resonator(
        design, waveguide_layer, crossing_access
    )
    diagnostics["parameters"] = design
    diagnostics["crossing_port_to_port_span_um"] = 2.0 * wx_half_span
    return resonator, {}, diagnostics


def build_onn_butterfly_network(params: dict | None, layers: dict):
    """Build the corrected 4 x 4 ONN core and return diagnostics."""
    design = _defaults(params)
    if int(design["n"]) != 4:
        raise ValueError("This verified device is fixed to n=4 (a 4 x 4 network).")
    _validate_positive(design)

    crossing_cell, crossing_ports, wx_half_span = _crossing_geometry(
        design["crossing"], layers
    )
    crossing_access = wx_half_span + float(design["crossing_taper_length"])
    waveguide_layer = resolve_wg_layer(float(design["waveguide_width"]), layers)
    text_layer = layers.get("TEXT", 100)
    canonical, canonical_diagnostics = _canonical_resonator(
        design, waveguide_layer, crossing_access
    )
    template = _interaction_template(design, crossing_access)

    group_pitch = float(design["interaction_group_pitch"])
    group_centers = np.asarray((-0.5 * group_pitch, 0.5 * group_pitch), dtype=float)
    placements = _resonator_placements(group_centers)
    crossing_centers, pair_records, butterflies = _interaction_diagnostics(
        group_centers, design, template
    )
    if len(crossing_centers) != 64 or len(set(crossing_centers)) != 64:
        raise ValueError("The corrected topology must produce 64 unique crossings")

    source = gdstk.Cell(f"{design['name']}_CONTINUOUS_RESONATORS")
    for placement in placements:
        source.add(
            gdstk.Reference(
                canonical,
                origin=placement["origin_um"],
                rotation=placement["rotation_rad"],
            )
        )

    network = gdstk.Cell(str(design["name"]))
    polygon_count = _cut_and_insert_crossings(
        source,
        network,
        crossing_centers,
        crossing_cell,
        wx_half_span,
        float(design["waveguide_width"]),
        float(crossing_ports["E"].width),
        float(design["crossing_taper_length"]),
        waveguide_layer,
    )
    label_position = (
        float(group_centers[0] - design["tile_half_span"]),
        float(canonical_diagnostics["y_return_um"] + group_centers[-1] + 130.0),
    )
    network.add(
        gdstk.Label(
            "4x4 ONN: 8 identical resonators / 4 butterflies / 16 couplers / 64 WX",
            label_position,
            layer=text_layer,
        )
    )

    crossing_reference_count = len(network.references)
    if crossing_reference_count != 64:
        raise ValueError(
            f"Expected 64 crossing references, found {crossing_reference_count}"
        )
    if bool(design["flatten"]):
        network.flatten()

    pair_keys = [(record["horizontal"], record["vertical"]) for record in pair_records]
    path_length = float(canonical_diagnostics["optical_path_length_um"])
    same_set_arm_coordinates = sorted(
        {
            float(center + offset)
            for center in group_centers
            for offset in (
                -template["return_offset"],
                -float(design["crossing_offset"]),
                float(design["crossing_offset"]),
                template["return_offset"],
            )
        }
    )
    minimum_same_set_clearance = min(
        b - a for a, b in zip(same_set_arm_coordinates[:-1], same_set_arm_coordinates[1:])
    )

    diagnostics = {
        "parameters": design,
        "resonator_count": 8,
        "horizontal_resonator_count": 4,
        "vertical_resonator_count": 4,
        "butterfly_count": 4,
        "interaction_tile_count": 4,
        "couplers_per_butterfly": 4,
        "coupling_region_count": 16,
        "crossings_per_butterfly": 16,
        "normal_crossing_count": 64,
        "crossing_centers_um": crossing_centers,
        "crossing_port_to_port_span_um": 2.0 * wx_half_span,
        "crossing_keepout_half_span_um": crossing_access,
        "crossing_references_before_flatten": crossing_reference_count,
        "cut_resonator_polygon_count": polygon_count,
        "resonator_placements": placements,
        "resonator_path_lengths_um": [path_length] * 8,
        "path_length_spread_nm": 0.0,
        "minimum_bend_radius_um": float(canonical_diagnostics["minimum_bend_radius_um"]),
        "same_set_minimum_centerline_clearance_um": minimum_same_set_clearance,
        "coupling_pair_count": len(pair_keys),
        "unique_coupling_pair_count": len(set(pair_keys)),
        "couplings_per_cross_set_pair": 1,
        "normal_crossings_per_cross_set_pair": 4,
        "cross_set_pair_records": pair_records,
        "butterflies": butterflies,
        "canonical": canonical_diagnostics,
    }
    return network, {}, diagnostics


def PCellONNButterflyNetwork(params: dict, layers: dict):
    """Registry-compatible full-network PCell wrapper."""
    cell, ports, _ = build_onn_butterfly_network(params, layers)
    return cell, ports


def PCellONNButterflyResonator(params: dict, layers: dict):
    """Registry-compatible continuous single-resonator PCell wrapper."""
    cell, ports, _ = build_onn_butterfly_resonator(params, layers)
    return cell, ports
