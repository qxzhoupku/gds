"""YAML-driven photonic layout builder.

Usage:
    python build.py [profile.yaml]

Reads a design profile, instantiates parametric cells, places and connects
them via ports, applies routing, and writes a GDS-II file.
"""

import os
import math
import yaml
import gdstk

from src.ports import Port
from src.layer_map import resolve_wg_layer
from src.place import (
    place_by_ports, route_straight, route_manhattan,
    route_euler_bend, transform_ports,
)
from src.cells.wx import PCellWx
from src.cells.taper import PCellTaper
from src.cells.ring import PCellRingCoupler
from src.cells.racetrack import PCellRacetrack
from src.cells.any_arc import PCellAnyArc
from src.cells.pulley_ring import PCellPulleyRing, PCellADDDROPPulleyRing
from src.cells.width_varying_ring import (
    PCellConstantWidthRing,
    PCellWidthVaryingRing,
)
from src.cells.onn_butterfly_network import (
    PCellONNButterflyNetwork,
    PCellONNButterflyResonator,
)
from src.cells.onn_butterfly_device import PCellONNButterflyDevice


# ---------------------------------------------------------------------------
# PCell registry — every entry has the same signature: (params, layers) → (cell, ports)
# ---------------------------------------------------------------------------

REGISTRY = {
    "WX":                  PCellWx,
    "TAPER":               PCellTaper,
    "RING":                PCellRingCoupler,
    "RACETRACK":           PCellRacetrack,
    "ARC":                 PCellAnyArc,
    "PULLEY_RING":         PCellPulleyRing,
    "PULLEY_ADD_DROP_RING": PCellADDDROPPulleyRing,
    "WIDTH_VARYING_RING":  PCellWidthVaryingRing,
    "CONSTANT_WIDTH_RING": PCellConstantWidthRing,
    "ONN_BUTTERFLY_NETWORK": PCellONNButterflyNetwork,
    "ONN_BUTTERFLY_RESONATOR": PCellONNButterflyResonator,
    "ONN_BUTTERFLY_DEVICE": PCellONNButterflyDevice,
}


# ---------------------------------------------------------------------------
# YAML helpers
# ---------------------------------------------------------------------------

def load_yaml(path: str):
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def deep_update(base: dict, upd: dict) -> dict:
    """Recursively merge *upd* into *base* (mutates *base*)."""
    for k, v in upd.items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            deep_update(base[k], v)
        else:
            base[k] = v
    return base


def resolve_design(profile_path: str) -> dict:
    """Load a design profile, resolving single-level ``extends`` inheritance."""
    cfg = load_yaml(profile_path)
    if "extends" in cfg:
        base = load_yaml(cfg["extends"])
        cfg = deep_update(base, {k: v for k, v in cfg.items() if k != "extends"})
    return cfg


# ---------------------------------------------------------------------------
# Validation helpers
# ---------------------------------------------------------------------------

class DesignError(Exception):
    """Raised when a design profile references missing instances or ports."""


def _require_instance(name, inst_cells):
    if name not in inst_cells:
        raise DesignError(f"Instance '{name}' not found. Available: {sorted(inst_cells)}")


def _require_port(port_name, alias, placed_ports):
    if alias not in placed_ports:
        raise DesignError(f"Alias '{alias}' has not been placed yet.")
    if port_name not in placed_ports[alias]:
        available = sorted(placed_ports[alias])
        raise DesignError(f"Port '{port_name}' not found on '{alias}'. Available: {available}")


def _resolve_port_ref(ref_str, placed_ports):
    """Split 'inst.port' and return the Port, with validation."""
    inst, port = ref_str.split(".")
    _require_port(port, inst, placed_ports)
    return placed_ports[inst][port]


# ---------------------------------------------------------------------------
# Placement
# ---------------------------------------------------------------------------

def place_instance(top, spec, inst_cells, inst_ports, placed_ports):
    """Absolute placement: put *inst* at (at, rot)."""
    name  = spec["inst"]
    alias = spec.get("as", name)
    at    = spec.get("at", [0, 0])
    rot   = math.radians(float(spec.get("rot", 0.0)))

    _require_instance(name, inst_cells)

    ref = gdstk.Reference(inst_cells[name], origin=(at[0], at[1]), rotation=rot)
    top.add(ref)
    placed_ports[alias] = transform_ports(inst_ports[name], origin=(at[0], at[1]), rotation=rot)


def connect_instance(top, spec, inst_cells, inst_ports, placed_ports):
    """Port-connected placement: snap *inst.port* to *target_inst.target_port*."""
    inst  = spec["inst"]
    port  = spec["port"]
    alias = spec.get("as", inst)
    target_inst, target_port = spec["to"].split(".")

    _require_instance(inst, inst_cells)

    if target_inst not in placed_ports:
        raise DesignError(
            f"Cannot connect to '{target_inst}' — it has not been placed yet. "
            f"Placed so far: {sorted(placed_ports)}"
        )

    ref = place_by_ports(top, inst_cells[inst], inst_ports[inst][port],
                         placed_ports[target_inst][target_port])
    rot = float(ref.rotation or 0.0)
    ox, oy = ref.origin
    placed_ports[alias] = transform_ports(inst_ports[inst], origin=(ox, oy), rotation=rot)


# ---------------------------------------------------------------------------
# Routing
# ---------------------------------------------------------------------------

def _route_layer(A, B, layers):
    """Determine the GDS layer for a route connecting ports A and B."""
    return resolve_wg_layer(min(A.width, B.width), layers)


def apply_routes(cfg, top, placed_ports, layers):
    for r in cfg.get("routes", []):
        if "straight" in r:
            A = _resolve_port_ref(r["straight"]["from"], placed_ports)
            B = _resolve_port_ref(r["straight"]["to"],   placed_ports)
            route_straight(top, A, B, layer=_route_layer(A, B, layers))

        elif "manhattan" in r:
            A = _resolve_port_ref(r["manhattan"]["from"], placed_ports)
            B = _resolve_port_ref(r["manhattan"]["to"],   placed_ports)
            route_manhattan(top, A, B, float(r["manhattan"]["r"]),
                            layer=_route_layer(A, B, layers))

        elif "euler" in r:
            A = _resolve_port_ref(r["euler"]["from"], placed_ports)
            B = _resolve_port_ref(r["euler"]["to"],   placed_ports)
            route_euler_bend(top, A, B, float(r["euler"]["Rmin"]),
                             layer=_route_layer(A, B, layers))


# ---------------------------------------------------------------------------
# Macro (block) expansion
# ---------------------------------------------------------------------------

def _macro_alias(alias_prefix, inner_alias):
    return f"{alias_prefix}.{inner_alias}" if alias_prefix else inner_alias


def apply_macro_placement(top, macro, inst_cells, inst_ports, placed_ports,
                          alias_prefix, base_offset,
                          substitutions=None, individual_offsets=None):
    substitutions = substitutions or {}
    individual_offsets = individual_offsets or {}

    for step in macro.get("placement", []):
        if "place" in step:
            spec = dict(step["place"])
            inst = substitutions.get(spec["inst"], spec["inst"])
            inner_alias = spec.get("as", inst)
            full_alias  = _macro_alias(alias_prefix, inner_alias)

            at  = spec.get("at", [0, 0])
            rot = math.radians(float(spec.get("rot", 0.0)))
            offset = individual_offsets.get(inner_alias, [0, 0])
            origin = [at[0] + base_offset[0] + offset[0],
                      at[1] + base_offset[1] + offset[1]]

            ref = gdstk.Reference(inst_cells[inst], origin=origin, rotation=rot)
            top.add(ref)
            placed_ports[full_alias] = transform_ports(
                inst_ports[inst], origin=origin, rotation=rot,
            )

        elif "connect" in step:
            spec = dict(step["connect"])
            inst = substitutions.get(spec["inst"], spec["inst"])
            inner_alias = spec.get("as", inst)
            full_alias  = _macro_alias(alias_prefix, inner_alias)

            target_raw, target_port = spec["to"].split(".")
            target_inst = substitutions.get(target_raw, target_raw)
            target_full = _macro_alias(alias_prefix, target_inst)

            if target_full not in placed_ports:
                raise DesignError(
                    f"Macro connect: target '{target_full}' not placed. "
                    f"Placed: {sorted(placed_ports)}"
                )

            ref = place_by_ports(
                top, inst_cells[inst],
                inst_ports[inst][spec["port"]],
                placed_ports[target_full][target_port],
            )
            rot = float(ref.rotation or 0.0)
            ox, oy = ref.origin
            placed_ports[full_alias] = transform_ports(
                inst_ports[inst], origin=(ox, oy), rotation=math.radians(rot),
            )


def apply_macro_routes(top, macro, placed_ports, layers, alias_prefix):
    for r in macro.get("routes", []):
        def resolve(port_ref):
            inst, port = port_ref.split(".")
            full = _macro_alias(alias_prefix, inst)
            return placed_ports[full][port]

        if "straight" in r:
            A = resolve(r["straight"]["from"])
            B = resolve(r["straight"]["to"])
            route_straight(top, A, B, layer=_route_layer(A, B, layers))
        elif "manhattan" in r:
            A = resolve(r["manhattan"]["from"])
            B = resolve(r["manhattan"]["to"])
            route_manhattan(top, A, B, float(r["manhattan"]["r"]),
                            layer=_route_layer(A, B, layers))
        elif "euler" in r:
            A = resolve(r["euler"]["from"])
            B = resolve(r["euler"]["to"])
            route_euler_bend(top, A, B, float(r["euler"]["Rmin"]),
                             layer=_route_layer(A, B, layers))


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main(profile="designs/profiles/demo_small.yaml"):
    cfg = resolve_design(profile)

    defaults = cfg.get("defaults", {})
    layers   = defaults.get("layers", {"WG": 1, "PORT": 99, "TEXT": 100})
    # Merge width_layers into the layers dict so resolve_wg_layer can find it
    if "width_layers" in defaults:
        layers["width_layers"] = defaults["width_layers"]
    out_path = cfg.get("chip", {}).get("out", "out/chip_demo.gds")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    top_name = str(cfg.get("chip", {}).get("name", "TOP") or "TOP")
    top = gdstk.Cell(top_name)

    # ---- Unique cell-name allocator ----
    used_names = {top_name}

    def unique_name(base: str) -> str:
        if base not in used_names:
            used_names.add(base)
            return base
        i = 1
        while f"{base}_{i}" in used_names:
            i += 1
        used_names.add(f"{base}_{i}")
        return f"{base}_{i}"

    # Chip boundaries are intentionally not emitted.  BEAMER must receive
    # only device geometry; chip dimensions remain documentation in YAML.

    # ---- Instantiate PCells ----
    lib = gdstk.Library(unit=1e-6, precision=1e-9)
    inst_cells, inst_ports = {}, {}

    for inst_name, node in cfg.get("instances", {}).items():
        type_key = node["type"]
        if type_key not in REGISTRY:
            raise DesignError(
                f"Unknown type '{type_key}' for instance '{inst_name}'. "
                f"Available: {sorted(REGISTRY)}"
            )
        params = node.get("params", {})
        cell, ports = REGISTRY[type_key](params, layers)
        cell.name = unique_name(inst_name)
        lib.add(cell)
        inst_cells[inst_name] = cell
        inst_ports[inst_name] = ports

    # ---- Placement ----
    placed_ports = {}
    for step in cfg.get("placement", []) or []:
        if "place" in step:
            place_instance(top, step["place"], inst_cells, inst_ports, placed_ports)
        elif "connect" in step:
            connect_instance(top, step["connect"], inst_cells, inst_ports, placed_ports)

    # ---- Top-level routes ----
    apply_routes(cfg, top, placed_ports, layers)

    # ---- Macro / block expansion ----
    macro_defs = {m["name"]: m for m in cfg.get("macros", [])}
    for block in cfg.get("blocks", []):
        macro = macro_defs[block["use"]]
        apply_macro_placement(
            top, macro, inst_cells, inst_ports, placed_ports,
            alias_prefix=block.get("as"),
            base_offset=block.get("at", [0, 0]),
            substitutions=block.get("substitutions", {}),
            individual_offsets=block.get("offsets", {}),
        )
        apply_macro_routes(top, macro, placed_ports, layers, block.get("as"))

    # ---- Write GDS ----
    lib.add(top)
    lib.write_gds(out_path)
    print(f"Wrote {out_path}")


if __name__ == "__main__":
    import sys
    profile = sys.argv[1] if len(sys.argv) > 1 else "designs/profiles/demo_small.yaml"
    main(profile)
