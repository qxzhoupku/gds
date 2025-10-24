import os
import uuid
import yaml
import gdstk
import math

from src.ports import Port
from src.place import place_by_ports, route_straight, route_manhattan, route_euler_bend, transform_ports
from src.cells.wx import PCellWx
from src.cells.taper import PCellTaper
from src.cells.ring import PCellRingCoupler
from src.cells.racetrack import PCellRacetrack
from src.cells.any_arc import PCellAnyArc

def load_yaml(path: str):
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)

def deep_update(base: dict, upd: dict) -> dict:
    for k, v in upd.items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            deep_update(base[k], v)
        else:
            base[k] = v
    return base

def resolve_design(profile_path: str) -> dict:
    cfg = load_yaml(profile_path)
    if "extends" in cfg:
        base = load_yaml(cfg["extends"])
        cfg = deep_update(base, {k: v for k, v in cfg.items() if k != "extends"})
    return cfg

def place_instance(top, spec, inst_cells, inst_ports, placed_ports):
    n = spec["inst"]
    alias = spec.get("as", n)
    at = spec.get("at", [0, 0])
    rot = float(spec.get("rot", 0.0))
    ref = gdstk.Reference(inst_cells[n], origin=(at[0], at[1]), rotation=math.radians(rot))
    top.add(ref)
    ports = transform_ports(inst_ports[n], origin=(at[0], at[1]), rotation=math.radians(rot))
    placed_ports[alias] = ports

def connect_instance(top, spec, inst_cells, inst_ports, placed_ports):
    inst = spec["inst"]
    port = spec["port"]
    alias = spec.get("as", inst)
    target_inst, target_port = spec["to"].split(".")

    if target_inst not in placed_ports:
        ref = gdstk.Reference(inst_cells[target_inst], origin=(0, 0), rotation=0.0)
        top.add(ref)
        placed_ports[target_inst] = transform_ports(inst_ports[target_inst], origin=(0, 0), rotation=0.0)

    ref = place_by_ports(top, inst_cells[inst], inst_ports[inst][port], placed_ports[target_inst][target_port])
    rot = float(ref.rotation or 0.0)
    ox, oy = ref.origin
    placed_ports[alias] = transform_ports(inst_ports[inst], origin=(ox, oy), rotation=rot)

def apply_routes(cfg, top, placed_ports, layers):
    for r in cfg.get("routes", []):
        if "straight" in r:
            f_inst, f_port = r["straight"]["from"].split(".")
            t_inst, t_port = r["straight"]["to"].split(".")
            A = placed_ports[f_inst][f_port]
            B = placed_ports[t_inst][t_port]
            route_straight(top, A, B, layer=layers["WG"])
        elif "manhattan" in r:
            f_inst, f_port = r["manhattan"]["from"].split(".")
            t_inst, t_port = r["manhattan"]["to"].split(".")
            radius = float(r["manhattan"]["r"])
            A = placed_ports[f_inst][f_port]
            B = placed_ports[t_inst][t_port]
            route_manhattan(top, A, B, radius, layer=layers["WG"])
        elif "euler" in r:
            f_inst, f_port = r["euler"]["from"].split(".")
            t_inst, t_port = r["euler"]["to"].split(".")
            Rmin = float(r["euler"]["Rmin"])
            A = placed_ports[f_inst][f_port]
            B = placed_ports[t_inst][t_port]
            route_euler_bend(top, A, B, Rmin, layer=layers["WG"])

def apply_macro_placement(top, macro, inst_cells, inst_ports, placed_ports, alias_prefix, base_offset, substitutions={}, individual_offsets={}):
    substitutions = substitutions or {}
    individual_offsets = individual_offsets or {}
    for step in macro.get("placement", []):
        if "place" in step:
            spec = dict(step["place"])  # shallow copy
            raw_inst = spec["inst"]
            inst = substitutions.get(raw_inst, raw_inst)

            inner_alias = spec.get("as", inst)
            full_alias = f"{alias_prefix}.{inner_alias}" if alias_prefix else inner_alias

            at = spec.get("at", [0, 0])
            rot = float(spec.get("rot", 0.0))

            offset = individual_offsets.get(inner_alias, [0, 0])
            origin = [
                at[0] + base_offset[0] + offset[0],
                at[1] + base_offset[1] + offset[1]
            ]

            ref = gdstk.Reference(inst_cells[inst], origin=origin, rotation=math.radians(rot))
            top.add(ref)

            ports = transform_ports(inst_ports[inst], origin=origin, rotation=math.radians(rot))
            placed_ports[full_alias] = ports

def apply_macro_routes(top, macro, placed_ports, layers, alias_prefix):
    for r in macro.get("routes", []):
        def resolve(port_ref):
            inst, port = port_ref.split(".")
            full_inst = f"{alias_prefix}.{inst}" if alias_prefix else inst
            return placed_ports[full_inst][port]

        if "straight" in r:
            A, B = resolve(r["straight"]["from"]), resolve(r["straight"]["to"])
            route_straight(top, A, B, layer=layers["WG"])
        elif "manhattan" in r:
            A, B = resolve(r["manhattan"]["from"]), resolve(r["manhattan"]["to"])
            radius = float(r["manhattan"]["r"])
            route_manhattan(top, A, B, radius, layer=layers["WG"])
        elif "euler" in r:
            A, B = resolve(r["euler"]["from"]), resolve(r["euler"]["to"])
            Rmin = float(r["euler"]["Rmin"])
            route_euler_bend(top, A, B, Rmin, layer=layers["WG"])



REGISTRY = {
    "WX": lambda p, L: PCellWx(
        p.get("WM", 1.6), p.get("LM", 8.0), p.get("LT", 10.0), p.get("w_in", 0.5), L, name=p.get("name", "WX")
    ),
    "TAPER": lambda p, L: PCellTaper(
        p.get("w0", 0.5), p.get("w1", 3.0), p.get("L", 150.0), L, name=p.get("name", "TP")
    ),
    "RING": lambda p, L: PCellRingCoupler(
        p.get("R", 20.0), p.get("gap", 0.25), p.get("w_ring", 0.5), p.get("w_bus", 0.5), p.get("L_bus", None), L, name=p.get("name", "RING")
    ),
    "RACETRACK": lambda p, L: PCellRacetrack(
        p.get("R", 50.0), p.get("L_straight", 30.0), p.get("gap", 0.2), p.get("w_ring", 0.5), p.get("w_bus", 0.5), p.get("L_bus", None), L, name=p.get("name", "RT")
    ),
    "ARC": lambda p, L: PCellAnyArc(
        p.get("radius", 10.0), p.get("width", 0.5), p.get("angle_deg", 90.0), L.get("WG", 1), name=p.get("name", "ARC")
    ),
}




def main(profile="designs/profiles/demo_small.yaml"):
    cfg = resolve_design(profile)
    layers = cfg.get("defaults", {}).get("layers", {"WG": 1, "PORT": 99, "TEXT": 100})
    out_path = cfg.get("chip", {}).get("out", "out/chip_demo.gds")

    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    requested_top_name = cfg.get("chip", {}).get("name", "TOP")
    top_name = str(requested_top_name) if requested_top_name else "TOP"

    lib = gdstk.Library(unit=1e-6, precision=1e-9)
    top = lib.new_cell(top_name)

    used_names = {top_name}
    def unique_name(base: str) -> str:
        if base not in used_names:
            used_names.add(base); return base
        i = 1
        while True:
            cand = f"{base}_{i}"
            if cand not in used_names:
                used_names.add(cand); return cand
            i += 1

    die_size = cfg.get("chip", {}).get("die", {}).get("size_um", None)
    die_margin = cfg.get("chip", {}).get("die", {}).get("margin_um", 0)
    die_disp = cfg.get("chip", {}).get("die", {}).get("at", [0, 0])
    if die_size is not None:
        w, h = die_size
        rect = gdstk.rectangle(
            ( -die_margin,       -die_margin),
            ( w + die_margin,  h + die_margin),
            layer=layers.get("DIE", 10)
        )
        # offset by die_disp
        rect.translate(die_disp[0], die_disp[1])
        top.add(rect)

    inst_cells, inst_ports = {}, {}
    for inst_name, node in cfg.get("instances", {}).items():
        maker = REGISTRY[node["type"]]
        params = node.get("params", {})
        cell, ports = maker(params, layers)
        safe_name = unique_name(inst_name)
        cell.name = safe_name
        lib.add(cell)
        inst_cells[inst_name] = cell
        inst_ports[inst_name] = ports

    placed_ports = {}
    if cfg.get("placement") is None:
        cfg["placement"] = []
    for step in cfg.get("placement", []):
        if "place" in step:
            place_instance(top, step["place"], inst_cells, inst_ports, placed_ports)
        elif "connect" in step:
            connect_instance(top, step["connect"], inst_cells, inst_ports, placed_ports)
    if cfg.get("routes") is None:
        cfg["routes"] = []
    apply_routes(cfg, top, placed_ports, layers)

    macro_defs = {m["name"]: m for m in cfg.get("macros", [])}

    for block in cfg.get("blocks", []):
        macro = macro_defs[block["use"]]
        alias = block.get("as")
        offset = block.get("at", [0, 0])
        substitutions = block.get("substitutions", {})
        individual_offsets = block.get("offsets", {})
        apply_macro_placement(top, macro, inst_cells, inst_ports, placed_ports, alias, offset, substitutions, individual_offsets)
        apply_macro_routes(top, macro, placed_ports, layers, alias)


    lib.write_gds(out_path)
    print(f"Wrote {out_path}")

if __name__ == "__main__":
    import sys
    profile = sys.argv[1] if len(sys.argv) > 1 else "designs/profiles/demo_small.yaml"
    main(profile)
