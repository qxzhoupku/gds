# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A YAML-driven GDS-II layout generator for photonic chips (`gdstk` + `numpy` + `PyYAML`).
A *design profile* (`designs/profiles/**/*.yaml`) declares parametric cells, where to
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

There is no unit-test suite. The 37 committed profiles *are* the test suite, and
`tools/gdscheck.py` is the harness:

```bash
python tools/gdscheck.py check                     # build all profiles, diff against tools/reference.json
python tools/gdscheck.py update-reference          # accept an intentional geometry change
python tools/gdscheck.py snapshot -o /tmp/before.json   # before/after around a risky refactor
python tools/gdscheck.py compare /tmp/before.json /tmp/after.json
```

`check` takes about 24 s (the three ONN *gap-length sweep* profiles are the slow
ones, ~4.3 s each; the other three ONN profiles build in 0.6-1.0 s, and nothing
else exceeds 0.4 s) and builds into a temp dir, so `out/` is never touched. The closest thing to
"run one test" is `python build.py <one profile> --dry-run`.

**Run `check` after any change to `src/`.** It reports changes in geometry, in
*emission order*, in cell names and in file size. A new profile shows up as
`note: new profile not in reference` — a note, not a break, so `check` still
prints "OK" and exits 0; it needs `update-reference` too.

## Architecture

`build.py` is a thin CLI; the engine is importable so a sweep script can build and
inspect a design without writing a file (see the README's Python snippet).

Pipeline, in [src/builder.py](src/builder.py) `build_library()`:

1. `instances` → PCell factories via [src/registry.py](src/registry.py)
2. `placement` → `Placer.run()`: `place` (absolute) or `connect` (port-mated)
3. top-level `routes` → [src/router.py](src/router.py)
4. `blocks` expanded against `macros` — each gets its own `Placer`, and each
   block's own `routes` are drawn immediately after that block's placement, not
   batched with step 3
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
  `place_by_ports`, and the four routers.
- [src/clothoid.py](src/clothoid.py) — Euler-spiral geometry, no `gdstk`: the
  bend primitive, and `plan_route()`, which decides which segments join two
  ports *and at what radius*. `route_clothoid` in `place.py` is a thin wrapper
  that emits the plan. The bend primitive has a second caller outside the
  routers: `src/cells/onn_butterfly_network.py` uses `unit_chord`,
  `bend_samples` and `bend_points` for the `closure_style: euler` closure, so
  touching it moves ONN *cell* geometry too. `plan_route()` stays route-only.
  Note the split: `src/clothoid.py` has nothing to do
  with `route_euler_bend`, which is the older approximation and lives entirely
  in `place.py`.
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
through a `substitutions` table, and two offsets are added: the block's `at`
shifts every instance in the body, and the block's `offsets` add a per-alias extra
on top. Both apply to `place` only — `connect` takes its position from the target
port and ignores them. The top level is that same class with no prefix, no
substitutions and no offsets. Don't grow a
second placement path; extend `Placer`.

`placed_ports` is shared across the whole build, so a macro can `connect` to
something placed at the top level — but only from a block with no `as:`.
`placed_port` runs every reference through `_alias`, so in a block declared
`as: blk` the reference `TOPA.E` is looked up as `blk.TOPA` and raises; that is
also why one block cannot reach another block's aliases. Every committed block
uses `as:`.

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
  factory into a `DesignError` naming the instance. Placement steps are the hole:
  `Placer.place` reads `spec["inst"]` and `Placer.connect` reads
  `spec["port"]`/`spec["to"]` bare, so `- place: {at: [0, 0]}` still dies on a
  `KeyError` traceback. Fix that where the key is read, not by widening the CLI's
  `except`. Route entries do check (`apply_route`'s `missing` list).
- Adding a route kind = one entry in `router.ROUTERS` — `(handler, required
  keys, optional keys)`, the handler taking `(parent, a, b, spec, layer)` with the
  layer already resolved; both the top-level and macro route paths pick it up
  automatically. Every key the kind reads must appear in one of those two tuples:
  anything else warns and is ignored, so an undeclared key looks accepted and
  does nothing.
- Adding a component = write `src/cells/<thing>.py` exposing
  `PCellThing(params, layers) -> (cell, ports)`, then import it in
  [src/catalog.py](src/catalog.py) and add one `CATALOG` entry. Read *optional*
  params out of `params` with an explicit default, in microns; a required
  dimension is indexed directly (`params["ring_radius"]`, as `pulley_ring.py` and
  the ONN cells do). A cell raises plain `ValueError` and never imports
  `DesignError` — `instantiate()` turns either into a `DesignError` naming the
  instance.
  `CATALOG` keys are part of the profile format and every committed design depends
  on them — they must stay stable.
- The database unit is 1 um with 1 nm precision — `GDS_UNIT`/`GDS_PRECISION` in
  [src/builder.py](src/builder.py), hardcoded. `defaults.grid_um: 0.001` records
  the same grid but is **never read**: `Final/StWG_Ring_Coupler.yaml` declares
  `0.0001` and still gets 1 nm. Changing the real grid moves every polygon and
  breaks every `gdscheck` reference at once.

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
  builds at the root of `out/` are not. Nothing ever *writes* into those folders
  — every resolved `chip.out` is a bare `out/<name>` — so promotion is a manual
  copy, often under a different name: `pulley_400nm.yaml` writes
  `out/chip_pulley_400nm.gds` while its geometry actually matches
  `chip_pulley_400nm_wide_drop_v1.gds`. Match a profile to a committed GDS by
  geometry, never by filename.
- The ONN cells are a different shape from the rest of the catalog, and the three
  of them are not shaped alike. `ONN_BUTTERFLY_DEVICE` takes a *nested* `params`
  deep-merged over `DEFAULT_CORE` / `DEFAULT_IO`, which exist **only** in
  [src/cells/onn_butterfly_device.py](src/cells/onn_butterfly_device.py), and
  hands its `core` sub-dict to the network cell. `ONN_BUTTERFLY_NETWORK` and
  `ONN_BUTTERFLY_RESONATOR` take a **flat** `params` merged over a private
  `_defaults()` in
  [src/cells/onn_butterfly_network.py](src/cells/onn_butterfly_network.py) —
  only `crossing` merges, one level — and that `_defaults()` carries its own
  larger numbers (`min_bend_radius` 50, `crossing_offset` 300,
  `return_arm_offset` 150, `tile_half_span` 560, `interaction_group_pitch` 1300,
  `closure_lead` 100) which `DEFAULT_CORE` overrides on the device path. All
  three `build_onn_butterfly_*()` functions return `(cell, ports, diagnostics)`
  — the `PCell*` wrappers exist only to drop the diagnostics for the registry.
  Call the `build_*` function directly when you want the diagnostics. The network
  and resonator return `{}` for ports, so nothing can `connect` to them; only the
  device exposes ports, and every committed profile uses the device.
- The ONN resonator closure is a curvature step *inside* a recirculating loop, so
  it is paid every pass and caps the loaded Q. Inside the ONN core it is the only
  one — everything else in that loop is a `_quintic_connector`,
  curvature-continuous by construction (control points 1-2 collinear with the
  endpoint make the second derivative vanish), though *not* a clothoid: its
  d(kappa)/ds is non-constant and even changes sign. Its radii (31.60 / 34.11 um)
  are looser than the closure's 30, so **the author has decided these stay as
  they are — do not "upgrade" them to clothoids.** The closure is also not the
  only in-loop step in the repo: `RT_Lattice_4x4.yaml` and its
  `Final/`/`Fabricated/` copies butt 112 180-degree `ARC`s at R = 30 um onto
  `straight` routes, and `PCellRacetrack` butts two semicircles onto two
  straights. Those predate this work; leave them.
  `core.closure_style` picks the shape: `arc` (default, the published
  semicircle) or `euler`, a clothoid whose curvature ramps over `core.closure_p`
  of the turn. **`return_arm_offset` is the closure's lateral span, not its
  diameter** — it equals `unit_chord(pi, p) * R`, which is `2*R` only for a
  semicircle, so switching to `euler` at a fixed offset drives the radius *down*
  (60 um gives 21.79 um at p=1, below `min_bend_radius`, and the cell raises
  saying so). Widen the offset to keep the radius, and `tile_half_span` and
  `interaction_group_pitch` must then follow their own two guards. Note the
  guard on the pitch is strict (`pitch <= 2 * tile_half_span` raises), so
  `2 * tile_half_span` exactly is not a legal pitch. The apex also moves:
  it sits `closure_apex_extent_um` past the arm, which is `R` for the
  semicircle but `2.4501 * R` at p=1 — `onn_butterfly_device.py` places its
  pump and drop probes off that diagnostic alone; `closure_radius_um` rides along
  in the same record as the number that sets the curvature *at* the apex, and so
  the coupling, but no geometry is placed off it. Conflating the two is what made
  the device's own overlap guard fire.
  `designs/profiles/onn_butterfly_4x4_euler_closure.yaml` is the worked example
  and the only committed user of `closure_style: euler`; its header carries the
  widened offset, span and pitch. Watch the footprint when you copy it: the euler
  device stands 1569 um tall against the arc version's 1375 (flattened
  polygons; `bounding_box()` says 1592.78 and 1412.40 because a layer-100
  label sits above the topmost waveguide, and those inflated figures are what
  an earlier version of this file and the profile header both quoted). The
  1450 um y-pitch the gap-length sweep stacks devices on is smaller than the
  euler device either way — nothing validates device-to-device spacing, so
  copies merge silently instead of raising.
- The ONN profiles keep YAML anchors under a `_templates:` key so they can be
  reused via `*` aliases further down. It is not a builder section, and
  `warn_unknown_keys` skips any top-level key starting with `_` for exactly
  that reason — use that prefix for a deliberate non-section rather than
  widening `KNOWN_TOP_LEVEL`. `_platform` is the other user of it: the process
  platform (400 nm Si3N4 core, SiO2 clad) lives in `designs/platforms/*.yaml`,
  one fragment per platform, and **every profile names its platform in
  `extends`** — so the chain is now `profile → platform → base`, and
  `designs/base.yaml` is no longer what a profile extends directly. Those
  fragments must stay out of `designs/profiles/`, which `gdscheck` globs.
  Never override `_platform` partially in a profile: `deep_update` merges
  key-by-key, so the omitted fields survive silently — that is why the block
  lives in the fragment and not in `base.yaml`. The three `*AlN*` profiles
  point at a separate fragment that asserts only the core material, and
  `clothoid_demo.yaml` + `Archive/demo_small.yaml` point at
  `designs/platforms/unspecified.yaml`, which is non-binding on purpose — it
  marks a fixture as illustrative rather than as a fab record, so "no platform"
  is distinguishable from "platform not set yet".
  Note the `_` exemption is **top-level only**: `chip._platform` warns, and
  `defaults._platform` is silent only because nothing nested under `defaults`
  is ever checked.
- There are **two** Euler-ish route kinds and they behave nothing alike.
  `euler` (`route_euler_bend`) draws circular and raised-cosine curves, not true
  clothoids, and `Rmin` means two different things inside it. In the L-bend
  branch it *is* the shape parameter — the quarter arc is drawn at exactly `Rmin`
  and nothing warns, because that branch is only taken when both offsets already
  clear `Rmin`. In the raised-cosine S-bend fallback the curve is fixed by the
  endpoints alone, so there `Rmin` is a *constraint that warns* when the implied
  `2*x_rel^2 / (pi^2*|y_rel|)` comes out tighter. (`_euler_local_points`'s own
  docstring claims only the second, and is wrong.) `clothoid`
  (`route_clothoid`) is the real thing: a true Euler spiral whose curvature
  ramps linearly with arc length. `euler` is kept only because it is the
  documented, published behaviour; prefer `clothoid` for anything new. No
  committed profile uses the `euler` *route kind* — the single route reference to
  it is a commented-out line in `Archive/demo_small.yaml`. Don't read a
  `grep euler designs/` hit as one: `onn_butterfly_4x4_euler_closure.yaml` sets
  `core.closure_style: euler`, which is the ONN closure's clothoid option and has
  nothing to do with `route_euler_bend`.
- `clothoid` requires both ports at the *same width* and raises rather than
  quietly drawing at the narrower, unlike every other route kind. An Euler bend
  has one width; a taper is a different component.
- **`clothoid`'s `Rmin` is a floor, not the radius you get.** Both drivers of
  mode conversion fall with radius, so `plan_route()` grows every bend to the
  largest radius the two ports admit. How far above the floor is set almost
  entirely by how much room the ports have: median 1.0–1.7× `Rmin` while the
  separation is under 10× `Rmin`, rising to 4.5× at 25× separation and 10.9× at
  62.5×, with the largest sampled bend 75× `Rmin`. README.md carries the
  measured table. Nothing caps the radius as a multiple of `Rmin` — the only
  ceiling is `_MAX_RADIUS_FACTOR = 10.0` times the *port separation* (floored
  at `Rmin`), which `Rmax` can only tighten, never raise. So a widely-spaced
  pair grows further still, and the sampled maxima track 10× the separation
  rather than any multiple of `Rmin`.
  Read `plan.radius` for what was actually drawn. Two things follow and have
  bitten before: the bend sweeps the *middle* of the rectangle its ports span
  rather than hugging the edges, and two identical-looking connections at
  different spacings get different radii and lengths, so arms do not
  path-length match by construction. `Rmax` caps the sweep; setting
  `Rmax: <same as Rmin>` pins a bend for matched arms — the two-bend solves
  root-find `R(alpha) = Rmax` rather than filtering on it, because a search can
  never *hit* an exact radius by scanning, while `_corner` is affine in `R` and
  reaches the cap in closed form. Pinning costs reachability: over half the random
  poses that route at a grown radius come back a `DesignError` once
  `Rmax = Rmin`, and the pinned solve can land on a different *shape* than the
  free one — so check that both arms solved and agree on the shape.
- Growing the radius is also what tames the near-antiparallel `corner`, whose
  `1/sin(Δ)` solve used to run to kilometres while reporting a zero endpoint
  residual. Don't "simplify" `_corner` back to a fixed radius.
- A pure clothoid is exactly twice as long as the circular arc of the same
  radius and turn, and a 90° one spans the corner box of a 1.87·R circular
  bend. That is inherent, not a bug — lower `p` to trade gradualness back for
  footprint.
- `plan_route()` maximises the radius over the turn split by enumerating exact
  candidates — where the straight reaches zero, where the radius hits a bound,
  and the ends of the split bracket — *not* by assuming the straight goes to
  zero. 59% of sampled two-bend optima sit at a bracket end (67% at wide
  separations), every one of them with one bend saturating a half turn *and* a
  strictly positive straight, so that shortcut is wrong. The candidates are found on the smooth quantities `den`, `N` and `M`
  rather than on `R = N/den`, which has poles where the radius and the straight
  blow up together; a search on `R` itself gets dragged onto one every time.
- Every shape is offered at a **ladder** of radii, gentlest down to `Rmin`, and
  the screens in `plan_route()` take the gentlest that survives. Returning only
  the gentlest candidate made a screen cost *reachability* instead of
  footprint, which pinning at `Rmin` never did.
- `_C1_TOL` in `src/clothoid.py` looks unused-ish and is not. At `|Δ| = π`
  exactly, IEEE gives `c1 = (0.0, 1.22e-16)` — `1+cos(π)` is exactly zero but
  `sin(π)` is not — so the S-bend denominator lands near `1e-17`, a
  `den == 0.0` test never fires, and a radius gets built out of pure round-off.
  I removed this guard once as an orphan; an audit caught it. Don't.
- The `_bulge` screen is load-bearing, not belt-and-braces. A free radius makes
  pairs of near-half-turn bends reachable that close on the ports exactly and
  pass every other check while sweeping millimetres — 15 mm of waveguide
  between ports 224 µm apart was a measured case — 24.5 mm with the screen
  disabled — and poses that used to raise `DesignError` came back as wafer
  geometry. Don't read the threshold as slack, though: measured over the
  sampled poses, accepted corners bulge a median 0.14 of the port separation
  and reach 0.855, and both two-bend shapes run right up to 1.499, because the
  cap is what selected them. A corner or S-bend measures exactly 0 only when
  the target is ahead of the start port; `_bulge` is an axis-aligned box, so
  reaching backwards leaves it immediately. The screen does turn 12.8% of
  sampled poses into a `DesignError`, but it costs no *usable* route: those
  poses would have drawn a bulge of at least 2.53× the separation (median
  6.9×) at 9.5–70× the separation in length, and none of them route with
  `Rmax = Rmin` either.
- `src/cells/racetrack.py` closes its loop **only when `L_straight == 2*R`**.
  The return leg is a `segment()` to an absolute point, so any other pair leaves
  the ring open — 92.39 um apart at the factory's own defaults (`R: 50.0`,
  `L_straight: 30.0`), silently, with no warning. The only committed user,
  `Archive/demo_small.yaml`, sets `R: 20, L_straight: 40`, so `check` never
  exercises the broken case. Treat `L_straight` as pinned to `2*R` until the
  return leg is built from the turn's exit.
- **A green `check` is thin cover for the routers.** Of the four kinds in
  `router.ROUTERS`, committed profiles use only `straight` and `clothoid` (the
  latter in `clothoid_demo.yaml` alone); `manhattan` appears in no profile and
  `euler` only in that commented line. Everything else curved is a placed `ARC`
  cell. So a change to `src/clothoid.py` is pinned by one route profile plus the
  euler-closure ONN profile, and a change to `_manhattan` by nothing — add a demo
  profile rather than trusting `check`.
- `width_varying_ring_sweep_16dev.yaml` and its `Final/` copy declare
  `chip.size_um`, which is not a known `chip` key, so both print
  `UserWarning: Unknown 'chip' key 'size_um' — ignored.` on every build. They are
  the only two profiles that warn at all; the geometry is fine. Don't chase it,
  and don't widen `KNOWN_CHIP_KEYS` to hide it — chip dimensions go in `chip.die`.
- `config/crossing_default.json` is not read by anything.
- `gdscheck` digests geometry rather than hashing bytes because `gdstk` stamps
  wall-clock time into every file it writes.

## Pre-existing drift — not regressions

Do not "fix" these silently; they predate the current builder and only the
author knows the intent.

- `Final/StWG_Ring_Coupler.yaml` does not build the output its name and
  `chip.out` point at, but it is **not** the mystery an earlier version of this
  file claimed. Its 5 waveguide polygons are identical to the nanometre to
  `out/Final/StWG_Ring_no_Coupler.gds`; uncomment the `ST_WG` instance and its
  `place` step and it reproduces the 6 polygons of
  `out/Final/StWG_Ring_Coupler.gds` exactly (the extra one is the 1.0 x 120 um
  straight). So the YAML was edited to drop that straight *after* the Coupler GDS
  was written, and the committed no_Coupler file is that edited state. The only
  other difference is labels: the current builder also labels ARCs, so it emits 5
  layer-100 labels against the file's 2. Don't measure this with
  `Cell.bounding_box()` — label origins and gdstk's rotation-inflated reference
  boxes make it report y −93.19..100.00 for geometry that spans y −0.90..7.72,
  which is where the old "reproduces neither" conclusion came from. Compare
  flattened, quantised polygons the way `gdscheck` does.
- `Archive/WX_demo_01.yaml` and `Archive/demo_small.yaml` set no `chip.out`, so
  they inherit `designs/base.yaml`'s `out: out/demo_small` and write a file with
  **no `.gds` extension**, and both write the same file. `gdscheck` redirects
  output, so it does not care. `.gitignore` does: `out/*.gds` does not match
  `out/demo_small`, so a real build of either leaves an untracked file behind.
- **The committed GDS under `out/Final/` and `out/Fabricated/` are not a rebuild
  reference; `tools/reference.json` is.** Several of those profiles no longer
  reproduce the GDS committed beside them. Most differences are cosmetic — identical union area, but
  paths cut into more polygons, more labels, and no layer-10 outline. Two are
  real: `Final/pulley_400nm_v1` builds 760 polygons against the committed 711, and
  `pulley_400nm_v2` 923 against 918. The profiles were edited after those chips
  were written and only the author knows which state was exposed, so don't
  reconcile them. Nine committed files under `out/` also carry one layer-10
  polygon — the die outline the current builder refuses to emit. They predate that
  rule; they are not licence to draw one.
- **Nine `chip.out` paths are claimed by two profiles each.** Six pairs are
  byte-identical YAML and one is the inherited `out/demo_small`, but two are
  different designs sharing a file: `Final/pulley_400nm_v1.yaml` and
  `pulley_400nm.yaml` both write `out/chip_pulley_400nm.gds`, and
  `Archive/pulley_400nm_dose_test.yaml` and `pulley_400nm_dose_test.yaml` both
  write `out/chip_pulley_400nm_dose_test.gds`. Building one silently destroys the
  other's output, and `gdscheck` cannot see it because it redirects every build
  with `--out`. Pass `-o` when you build these.
- `defaults.width_layers` is declared in only 3 of 37 profiles:
  `pulley_400nm_dose_test.yaml`, `Final/pulley_400nm_dose_test_v2.yaml` and
  `demo_multi_layer.yaml` — the other three `dose_test`-named profiles do not
  declare it. Every other profile puts all its
  geometry on layer 1 — the width-based layer split is a dose-calibration tool,
  not the normal path.
