"""The build engine: design profile in, ``gdstk.Library`` out.

Kept separate from the ``build.py`` command-line entry point so a design can
be built, inspected and asserted on from Python — a sweep script or a test can
call :func:`build_library` and examine the geometry without writing a file.

The pipeline, in the order geometry is emitted (that order is what makes the
written bytes reproducible, so it must not be rearranged):

1. instantiate every entry of ``instances`` via the PCell registry
2. walk ``placement``, absolute (``place``) or port-mated (``connect``)
3. draw top-level ``routes``
4. expand each ``blocks`` entry against its ``macros`` definition
5. add the top cell last

Placement is handled by one :class:`Placer` for both top-level steps and macro
bodies.  A macro body is the general case — aliases carry a prefix, instance
names go through a substitution table, and a per-instance offset is added — and
the top level is that same case with no prefix, no substitutions and no
offsets.
"""

from __future__ import annotations

import math
from typing import Dict, Mapping

import gdstk

from . import registry, router
from .design import DesignError, steps
from .place import place_by_ports, transform_ports
from .ports import Port

# GDS database settings.  unit=1 um with precision=1 nm gives the 0.001 um
# grid that profiles declare as ``defaults.grid_um``.
GDS_UNIT = 1e-6
GDS_PRECISION = 1e-9


class NameAllocator:
    """Hands out unique GDS cell names, suffixing ``_1``, ``_2``, ... on clash."""

    def __init__(self, reserved: str | None = None):
        self._used: set[str] = {reserved} if reserved else set()

    def take(self, base: str) -> str:
        if base not in self._used:
            self._used.add(base)
            return base
        i = 1
        while f"{base}_{i}" in self._used:
            i += 1
        name = f"{base}_{i}"
        self._used.add(name)
        return name


class Placer:
    """Places instances into a cell and tracks where their ports ended up.

    Parameters
    ----------
    top
        Cell that references are added to.
    inst_cells, inst_ports
        The instantiated PCells and their local ports, keyed by profile
        instance name.
    placed_ports
        Alias -> {port name -> placed :class:`Port`}.  Shared across the whole
        build so a macro can connect to something placed at the top level.
    alias_prefix
        Prefix joined with ``.`` onto aliases produced here (macro scoping).
    base_offset, offsets
        Translation applied to ``place`` steps: every instance shifts by
        *base_offset*, plus a per-alias extra from *offsets*.
    substitutions
        Rewrites instance names, letting one macro body be reused with
        different components.
    """

    def __init__(
        self,
        top: gdstk.Cell,
        inst_cells: Mapping[str, gdstk.Cell],
        inst_ports: Mapping[str, Dict[str, Port]],
        placed_ports: Dict[str, Dict[str, Port]],
        *,
        alias_prefix: str | None = None,
        base_offset=(0.0, 0.0),
        offsets: Mapping[str, object] | None = None,
        substitutions: Mapping[str, str] | None = None,
    ):
        self.top = top
        self.inst_cells = inst_cells
        self.inst_ports = inst_ports
        self.placed_ports = placed_ports
        self.alias_prefix = alias_prefix
        self.base_offset = base_offset
        self.offsets = offsets or {}
        self.substitutions = substitutions or {}

    # -- name/alias plumbing -------------------------------------------------

    def _alias(self, inner: str) -> str:
        return f"{self.alias_prefix}.{inner}" if self.alias_prefix else inner

    def _instance(self, name: str) -> str:
        return self.substitutions.get(name, name)

    def _cell(self, inst: str) -> gdstk.Cell:
        try:
            return self.inst_cells[inst]
        except KeyError:
            raise DesignError(
                f"Instance {inst!r} is not declared under 'instances'. "
                f"Available: {sorted(self.inst_cells)}"
            ) from None

    def _local_port(self, inst: str, port: str) -> Port:
        ports = self.inst_ports[inst]
        try:
            return ports[port]
        except KeyError:
            raise DesignError(
                f"Port {port!r} not found on instance {inst!r}. "
                f"Available: {sorted(ports)}"
            ) from None

    def placed_port(self, ref: str) -> Port:
        """Resolve an ``"alias.port"`` reference against what has been placed."""
        alias, _, port = ref.rpartition(".")
        if not alias:
            raise DesignError(
                f"Port reference {ref!r} must be of the form 'instance.port'."
            )
        alias = self._alias(self._instance(alias))
        if alias not in self.placed_ports:
            raise DesignError(
                f"Cannot reference {ref!r}: {alias!r} has not been placed yet. "
                f"Placed so far: {sorted(self.placed_ports)}"
            )
        ports = self.placed_ports[alias]
        if port not in ports:
            raise DesignError(
                f"Port {port!r} not found on {alias!r}. "
                f"Available: {sorted(ports)}"
            )
        return ports[port]

    # -- placement steps -----------------------------------------------------

    def place(self, spec: dict) -> None:
        """Absolute placement at ``at`` rotated by ``rot`` degrees."""
        inst = self._instance(spec["inst"])
        inner_alias = spec.get("as", inst)
        at = spec.get("at", [0, 0])
        extra = self.offsets.get(inner_alias, [0, 0])
        origin = (
            at[0] + self.base_offset[0] + extra[0],
            at[1] + self.base_offset[1] + extra[1],
        )
        rotation = math.radians(float(spec.get("rot", 0.0)))

        cell = self._cell(inst)
        self.top.add(gdstk.Reference(cell, origin=origin, rotation=rotation))
        self.placed_ports[self._alias(inner_alias)] = transform_ports(
            self.inst_ports[inst], origin=origin, rotation=rotation,
        )

    def connect(self, spec: dict) -> None:
        """Port-mated placement: snap ``inst.port`` onto ``to``."""
        inst = self._instance(spec["inst"])
        inner_alias = spec.get("as", inst)
        cell = self._cell(inst)
        child_port = self._local_port(inst, spec["port"])
        target = self.placed_port(spec["to"])

        ref = place_by_ports(self.top, cell, child_port, target)
        # gdstk.Reference.rotation is in radians, which is what transform_ports
        # expects — do not convert.
        self.placed_ports[self._alias(inner_alias)] = transform_ports(
            self.inst_ports[inst],
            origin=tuple(ref.origin),
            rotation=float(ref.rotation or 0.0),
        )

    def run(self, placement_steps) -> None:
        """Execute an ordered list of ``{place: ...}`` / ``{connect: ...}``."""
        handlers = {"place": self.place, "connect": self.connect}
        for step in placement_steps:
            known = [k for k in step if k in handlers]
            if not known:
                raise DesignError(
                    f"Placement step {step!r} names neither 'place' nor "
                    f"'connect'."
                )
            if len(known) > 1:
                raise DesignError(
                    f"Placement step names several actions {known!r}; "
                    f"use one per entry."
                )
            handlers[known[0]](step[known[0]] or {})


def instantiate(cfg: dict, layers: dict, lib: gdstk.Library,
                names: NameAllocator):
    """Build every PCell declared under ``instances``.

    Cells are added to *lib* in profile order; each is renamed to a unique
    name derived from its instance name.
    """
    inst_cells: Dict[str, gdstk.Cell] = {}
    inst_ports: Dict[str, Dict[str, Port]] = {}

    for inst_name, node in (cfg.get("instances") or {}).items():
        if not isinstance(node, dict) or "type" not in node:
            raise DesignError(
                f"Instance {inst_name!r} must be a mapping with a 'type' key."
            )
        factory = registry.get(node["type"])
        try:
            cell, ports = factory(node.get("params") or {}, layers)
        except DesignError:
            raise
        except Exception as exc:
            raise DesignError(
                f"Instance {inst_name!r} (type {node['type']!r}) failed to "
                f"build: {type(exc).__name__}: {exc}"
            ) from exc

        cell.name = names.take(inst_name)
        lib.add(cell)
        inst_cells[inst_name] = cell
        inst_ports[inst_name] = ports

    return inst_cells, inst_ports


def expand_blocks(cfg: dict, top: gdstk.Cell, inst_cells, inst_ports,
                  placed_ports: dict, layers: dict) -> None:
    """Expand every ``blocks`` entry against its ``macros`` definition."""
    macro_defs = {}
    for macro in steps(cfg, "macros"):
        if "name" not in macro:
            raise DesignError(f"Macro {macro!r} has no 'name'.")
        macro_defs[macro["name"]] = macro

    for block in steps(cfg, "blocks"):
        use = block.get("use")
        if use not in macro_defs:
            raise DesignError(
                f"Block references unknown macro {use!r}. "
                f"Defined macros: {sorted(macro_defs)}"
            )
        macro = macro_defs[use]
        placer = Placer(
            top, inst_cells, inst_ports, placed_ports,
            alias_prefix=block.get("as"),
            base_offset=block.get("at", [0, 0]),
            offsets=block.get("offsets", {}),
            substitutions=block.get("substitutions", {}),
        )
        placer.run(steps(macro, "placement"))
        router.apply_routes(top, steps(macro, "routes"),
                            placer.placed_port, layers)


def build_library(cfg: dict, layers: dict, top_name: str) -> gdstk.Library:
    """Assemble a design into a ``gdstk.Library`` without writing it out.

    Chip boundaries are intentionally never emitted: BEAMER must receive
    device geometry only, so ``chip.die`` stays documentation in the profile.
    """
    lib = gdstk.Library(unit=GDS_UNIT, precision=GDS_PRECISION)
    top = gdstk.Cell(top_name)
    names = NameAllocator(reserved=top_name)

    inst_cells, inst_ports = instantiate(cfg, layers, lib, names)

    placed_ports: Dict[str, Dict[str, Port]] = {}
    placer = Placer(top, inst_cells, inst_ports, placed_ports)
    placer.run(steps(cfg, "placement"))

    router.apply_routes(top, steps(cfg, "routes"), placer.placed_port, layers)

    expand_blocks(cfg, top, inst_cells, inst_ports, placed_ports, layers)

    lib.add(top)
    return lib
