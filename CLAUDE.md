# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A YAML-driven GDS-II layout generator for photonic chips (`gdstk` + `numpy` + `PyYAML`).
A *design profile* (`designs/profiles/*.yaml`) declares parametric cells, where to
place them and how to route between them; `build.py` emits a GDS for e-beam
lithography. [README.md](README.md) documents the profile format, the component
table and the fabrication rationale — read it before editing a profile.

## Commands

```bash
source .venv/bin/activate                          # Python 3.14, pinned deps in requirements.txt

python build.py designs/profiles/<name>.yaml       # build; output path comes from chip.out
python build.py <profile> -o out/elsewhere.gds     # override chip.out
python build.py <profile> --dry-run                # assemble + validate, write nothing
python build.py --list-types                       # registered instance 'type' keys
```

There is no unit-test suite. The 35 committed profiles *are* the test suite, and
`tools/gdscheck.py` is the harness:

```bash
python tools/gdscheck.py check                     # build all profiles, diff against tools/reference.json
python tools/gdscheck.py update-reference          # accept an intentional geometry change
python tools/gdscheck.py snapshot -o /tmp/before.json   # before/after around a risky refactor
python tools/gdscheck.py compare /tmp/before.json /tmp/after.json
```

`check` takes well under a minute (the ONN profiles are the slow ones, ~4 s each)
and builds into a temp dir, so `out/` is never touched. The closest thing to
"run one test" is `python build.py <one profile> --dry-run`.

**Run `check` after any change to `src/`.** It reports changes in geometry, in
*emission order*, in cell names and in file size. A new profile shows up as
`note: new profile not in reference` — that needs `update-reference` too.

## Architecture

`build.py` is a thin CLI; the engine is importable so a sweep script can build and
inspect a design without writing a file (see the README's Python snippet).

Pipeline, in [src/builder.py](src/builder.py) `build_library()`:

1. `instances` → PCell factories via [src/registry.py](src/registry.py)
2. `placement` → `Placer.run()`: `place` (absolute) or `connect` (port-mated)
3. top-level `routes` → [src/router.py](src/router.py)
4. `blocks` expanded against `macros` (each gets its own `Placer`)
5. the top cell is added **last**

**This order is load-bearing.** It determines the byte layout of the written file,
which `gdscheck` pins; rearranging it is a compatibility break even when the
geometry is identical.

Supporting modules:

- [src/design.py](src/design.py) — profile loading. `extends` is followed
  transitively with the child always winning; unknown top-level and `chip` keys
  *warn* rather than being dropped silently. `resolve_layers()` folds
  `defaults.width_layers` into the `layers` dict because that is where
  `resolve_wg_layer` looks for it. `steps()` exists because a section whose every
  entry is commented out parses as `None`, not `[]`.
- [src/place.py](src/place.py) — geometry primitives: port transforms,
  `place_by_ports`, and the three routers.
- [src/layer_map.py](src/layer_map.py) — `resolve_wg_layer(width, layers)`, the
  single source of truth for which GDS layer a given width lands on.
- [src/catalog.py](src/catalog.py) — the one place a new component gets wired up.

### Ports

`Port` ([src/ports.py](src/ports.py)) angles are **radians** and point **outward**,
away from the component body; `connect` works by rotating the child so its port
faces the target's. A PCell returns ports in its own **local** coordinates and the
builder transforms them on placement. Ports live only in memory and are never
drawn — `TEXT` labels (layer 100) *are* written into the GDS.

### One Placer for two cases

A macro body is the general case — aliases carry a prefix, instance names go
through a `substitutions` table, and a per-alias offset is added. The top level is
that same class with no prefix, no substitutions and no offsets. Don't grow a
second placement path; extend `Placer`.

`placed_ports` is shared across the whole build, so a macro can `connect` to
something placed at the top level.

## Conventions that matter here

- **Never hardcode a waveguide layer.** Call `resolve_wg_layer(width, layers)`, or
  width-based dose splitting silently stops working. Routes are drawn on the layer
  implied by the *narrower* of the two ports, because width sets the dose recipe.
- **Never emit a chip or die outline.** BEAMER must receive device geometry only —
  a rectangle would become a real exposure. `chip.die` is parsed, accepted and
  deliberately ignored; chip dimensions stay in the profile as documentation.
- **A malformed profile is a user error, not a crash.** Raise `DesignError` (from
  `src.design`) with the available names in the message; the CLI prints
  `error: ...` and returns 1. `instantiate()` wraps any other exception from a
  factory into a `DesignError` naming the instance.
- Adding a route kind = one entry in `router.ROUTERS`; both the top-level and
  macro route paths pick it up automatically.
- Adding a component = write `src/cells/<thing>.py` exposing
  `PCellThing(params, layers) -> (cell, ports)`, then add one `CATALOG` entry.
  Read every param out of `params` with an explicit default, in microns.
  `CATALOG` keys are part of the profile format and every committed design depends
  on them — they must stay stable.
- The database unit is 1 um with 1 nm precision, matching `defaults.grid_um: 0.001`.

## Things that will surprise you

- `designs/profiles/Fabricated/` corresponds to chips that have physically been
  written. Their geometry must not drift. `Final/` means frozen, and the two
  folders **overlap rather than partition**: 4 of the 11 `Final/` profiles also
  sit in `Fabricated/` as byte-identical copies, 4 more are only in
  `Fabricated/`, and 7 are only in `Final/`. So "it's in `Final/`" does not
  mean "not yet fabricated" — check both. `gdscheck` covers every profile
  including `Archive/`.
- `.gitignore` ignores `out/*.gds` at the top level only, so the GDS files under
  `out/Final/`, `out/Fabricated/` and `out/Archive/` are committed while fresh
  builds at the root of `out/` are not.
- The ONN cells are a different shape from the rest of the catalog. `params` is a
  *nested* dict deep-merged over `DEFAULT_CORE` / `DEFAULT_IO`, and the underlying
  `build_onn_butterfly_*()` functions return `(cell, ports, diagnostics)` — the
  `PCell*` wrappers exist only to drop the diagnostics for the registry. Call the
  `build_*` function directly when you want the diagnostics.
- The sweep profiles keep YAML anchors under a `_templates:` key so they can be
  reused via `*` aliases further down. It is not a builder section, and
  `warn_unknown_keys` skips any top-level key starting with `_` for exactly
  that reason — use that prefix for a deliberate non-section rather than
  widening `KNOWN_TOP_LEVEL`.
- `route_euler_bend` draws circular and raised-cosine curves, not true clothoids.
  `Rmin` is a *constraint that warns* when the endpoint geometry forces a tighter
  bend, not a shape parameter — between two fixed ports the curve is determined by
  the endpoints.
- `config/crossing_default.json` is not read by anything.
- `gdscheck` digests geometry rather than hashing bytes because `gdstk` stamps
  wall-clock time into every file it writes.

## Pre-existing drift — not regressions

Do not "fix" these silently; they predate the current builder and only the
author knows the intent.

- `Final/StWG_Ring_Coupler.yaml` builds but reproduces neither committed
  output: it matches the polygon count (5) and exact x-extent of
  `out/Final/StWG_Ring_no_Coupler.gds` but spans y −93.19..100.00 against that
  file's −0.90..11.82, and `out/Final/StWG_Ring_Coupler.gds` has 6 polygons.
  The profile has an instance and a `place` step commented out, so the YAML was
  edited after its GDS was written.
- `Archive/WX_demo_01.yaml` and `Archive/demo_small.yaml` set no `chip.out`, so
  they inherit `designs/base.yaml`'s `out: out/demo_small` and write a file with
  **no `.gds` extension**. `gdscheck` redirects output, so it does not care.
- `defaults.width_layers` is declared in only 3 of 35 profiles, all `dose_test`
  designs. Every other profile puts all its geometry on layer 1 — the
  width-based layer split is a dose-calibration tool, not the normal path.
