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
            spec = step["place"]
            n = spec["inst"]
            alias = spec.get("as", None)  # optional alias for place
            at = spec.get("at", [0, 0])
            rot = float(spec.get("rot", 0.0)) / 180.0 * math.pi
            ref = gdstk.Reference(inst_cells[n], origin=(at[0], at[1]), rotation=rot)
            top.add(ref)

            ports_world = transform_ports(inst_ports[n], origin=(at[0], at[1]), rotation=rot)
            placed_ports[alias or n] = ports_world
        elif "connect" in step:
            spec = step["connect"]
            n = spec["inst"]; p = spec["port"]
            alias = spec.get("as", None)  # <-- new
            t_name, t_port = spec["to"].split(".")

            if t_name not in placed_ports:
                refT = gdstk.Reference(inst_cells[t_name], origin=(0, 0), rotation=0.0)
                top.add(refT)
                placed_ports[t_name] = transform_ports(inst_ports[t_name], origin=(0, 0), rotation=0.0)

            ref = place_by_ports(top, inst_cells[n], inst_ports[n][p], placed_ports[t_name][t_port])
            rot = float(ref.rotation or 0.0); ox, oy = ref.origin
            placed_dict = transform_ports(inst_ports[n], origin=(ox, oy), rotation=rot)

            key = alias or n  # if alias provided, store under alias; else overwrite n
            placed_ports[key] = placed_dict


    if cfg.get("routes") is None:
        cfg["routes"] = []
    for r in cfg.get("routes", []):
        if "straight" in r:
            f_inst, f_port = r["straight"]["from"].split(".")
            t_inst, t_port = r["straight"]["to"].split(".")
            A = placed_ports[f_inst][f_port]; B = placed_ports[t_inst][t_port]
            route_straight(top, A, B, layer=layers["WG"])
        elif "manhattan" in r:
            f_inst, f_port = r["manhattan"]["from"].split(".")
            t_inst, t_port = r["manhattan"]["to"].split(".")
            radius = float(r["manhattan"]["r"])
            A = placed_ports[f_inst][f_port]; B = placed_ports[t_inst][t_port]
            route_manhattan(top, A, B, radius, layer=layers["WG"])
        elif "euler" in r:
            f_inst, f_port = r["euler"]["from"].split(".")
            t_inst, t_port = r["euler"]["to"].split(".")
            Rmin = float(r["euler"]["Rmin"])
            A = placed_ports[f_inst][f_port]
            B = placed_ports[t_inst][t_port]
            route_euler_bend(top, A, B, Rmin, layer=layers["WG"])


    lib.write_gds(out_path)
    print(f"Wrote {out_path}")

if __name__ == "__main__":
    import sys
    profile = sys.argv[1] if len(sys.argv) > 1 else "designs/profiles/demo_small.yaml"
    main(profile)
