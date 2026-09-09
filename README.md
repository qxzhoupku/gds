# Photonic Chip Builder — YAML-driven GDS assembly (Python + gdstk)

Build reproducible photonic chip layouts from a single YAML design profile.
The profile declares which parametric cells to instantiate, where to put them,
and how to route between them; `build.py` emits a GDS-II file for e-beam
lithography.

## Quick start

```bash
python -m venv .venv
source .venv/bin/activate          # Windows: .venv\Scripts\activate
pip install -r requirements.txt

python build.py designs/profiles/demo_multi_layer.yaml
klayout out/demo_multi_layer.gds
```

The output path comes from `chip.out` in the profile. Useful flags:

```bash
python build.py <profile> -o out/elsewhere.gds   # override chip.out
python build.py <profile> --dry-run              # validate without writing
python build.py --list-types                     # available instance types
```

## Components

| `type` | Cell | What it is |
| --- | --- | --- |
| `WX` | [wx.py](src/cells/wx.py) | Waveguide crossing with tapered multimode centre |
| `TAPER` | [taper.py](src/cells/taper.py) | Linear width taper |
| `ARC` | [any_arc.py](src/cells/any_arc.py) | Circular arc of arbitrary sweep |
| `RING` | [ring.py](src/cells/ring.py) | Point-coupled ring over a straight bus |
| `RACETRACK` | [racetrack.py](src/cells/racetrack.py) | Point-coupled racetrack — the loop closes **only** when `L_straight == 2*R` |
| `PULLEY_RING` | [pulley_ring.py](src/cells/pulley_ring.py) | Ring with a wrapped (pulley) bus |
| `PULLEY_ADD_DROP_RING` | [pulley_ring.py](src/cells/pulley_ring.py) | Pulley ring with add and drop buses |
| `WIDTH_VARYING_RING` | [width_varying_ring.py](src/cells/width_varying_ring.py) | Ring with periodic triangular width modulation |
| `CONSTANT_WIDTH_RING` | [width_varying_ring.py](src/cells/width_varying_ring.py) | Constant-width control for the above |
| `ONN_BUTTERFLY_NETWORK` | [onn_butterfly_network.py](src/cells/onn_butterfly_network.py) | 4x4 butterfly-coupled resonator core |
| `ONN_BUTTERFLY_RESONATOR` | [onn_butterfly_network.py](src/cells/onn_butterfly_network.py) | One canonical resonator from that core |
| `ONN_BUTTERFLY_DEVICE` | [onn_butterfly_device.py](src/cells/onn_butterfly_device.py) | The core plus pump bus, drop probes and facet tapers |

## Profile format

```yaml
extends: designs/platforms/SiN400_SiO2.yaml   # the process platform;
                                              # itself extends
                                              # designs/base.yaml

defaults:
  layers: { WG: 1, PORT: 99, TEXT: 100 }
  width_layers:                   # optional: width -> layer, for dose splitting
    - { max_width: 0.6, layer: 1 }
    - { max_width: 2.0, layer: 2 }
    - { layer: 3 }                # catch-all, must be last
  grid_um: 0.001

chip:
  name: CHIP_1                    # top cell name
  out: out/my_chip.gds            # GDS output path
  # die: { size_um: [6000, 6000] }  # documentation only, never drawn

instances:                        # name -> parametric cell
  R1:
    type: PULLEY_RING
    params: { ring_radius: 100.0, ring_width: 1.8, coupler_width: 1.0,
              gap: 0.5, pulley_angle_deg: 5.0 }
  T1:
    type: TAPER
    params: { w0: 1.8, w1: 0.15, L: 400.0 }

placement:                        # ordered
  - place:   { inst: R1, at: [0, 0], rot: 0, as: R1_1 }
  - connect: { inst: T1, port: E, to: R1_1.W, as: T1_1 }

routes:                           # ordered
  - straight:    { from: R1_1.E, to: T1_1.W }
  - manhattan:   { from: A.E, to: B.W, r: 50 }
  - euler:       { from: A.E, to: B.W, Rmin: 80 }   # legacy approximation
  - clothoid:    { from: A.E, to: B.W, Rmin: 80 }   # true Euler spiral
```

`place` positions a cell absolutely (`rot` in degrees). `connect` mates
`inst.port` face-to-face against an already-placed `to` port. Routes are drawn
on the layer implied by the narrower of the two port widths.

Of the two Euler-ish kinds, prefer `clothoid`. `euler` is kept only because it
is the published behaviour of designs already written: it draws circular and
raised-cosine curves rather than true Euler spirals, and its `Rmin` means two
different things — the shape parameter in its L-bend branch, a
warning-only constraint in its S-bend fallback. No committed profile routes
with it.

**Platform metadata.** Every profile names the process platform it was drawn for
in its `extends` line. The fragments live in `designs/platforms/` — deliberately
*not* under `designs/profiles/`, which `tools/gdscheck.py` globs for buildable
profiles — and each holds a single `_platform:` block plus
`extends: designs/base.yaml`. Top-level keys starting with `_` are skipped by
the loader, so `_platform` is documentation only: the builder never reads it and
it emits no geometry (`_templates`, which parks YAML anchors in the sweep
profiles, is the other user of that convention). A design changes platform by
repointing `extends` at a different fragment, **never** by copying the block
into a profile — `extends` deep-merges key by key, so a partial copy would
silently keep the fields it omitted and quietly falsify the record.

### Clothoid (Euler spiral) routes

`clothoid` connects two ports with a **true Euler spiral**: curvature ramps
linearly with arc length, so it is continuous along the whole route and is zero
where the path meets each port. A circular bend instead steps curvature from 0
to 1/R at the straight-to-arc junction, and that step has to be absorbed by an
abrupt field mismatch, which converts power into higher-order modes. Ramping
the curvature lets the mode deform continuously instead.

```yaml
routes:
  - clothoid: { from: A.E, to: B.W, Rmin: 80 }
  - clothoid: { from: C.E, to: D.W, Rmin: 80, Rmax: 250, p: 0.4 }
```

| key | default | meaning |
| --- | --- | --- |
| `Rmin` | required | minimum tolerable bend radius, um. A hard floor: no point on the route is tighter |
| `Rmax` | none | cap on how gentle a bend may become. Use it when a sweeping bend would collide with what sits between the ports, or set it equal to `Rmin` to pin the radius exactly |
| `p` | `1.0` | fraction of each turn spent ramping curvature. `1.0` is the pure clothoid |
| `tolerance` | `0.001` | chord-sagitta budget for the emitted polyline, um (the database grid) |

**Both ports must have the same width** — an Euler bend has one width — and a
mismatch is a profile error rather than something silently drawn at the
narrower. Taper one side first.

#### Rmin is a floor, not the radius you get

Both drivers of mode conversion fall with radius: peak curvature as `1/Rmin`,
and the curvature ramp rate as `1/(p * t * Rmin^2)`. So the solver grows every
bend to the **largest radius the two ports admit**, and the straight sections
absorb whatever is left.

How far above the floor that lands depends almost entirely on how much room the
ports have. Measured over 800 poses per row — port separation fixed, direction
of the target uniform on the circle, arrival heading uniform on (-180, 180],
`Rmin: 80`, `p: 1`:

| separation / `Rmin` | poses that route | median `R`/`Rmin` | largest seen |
| --- | --- | --- | --- |
| 1 | 0.2% | 1.3 | 1.4 |
| 2.5 | 9% | 1.1 | 3.0 |
| 5 | 47% | 1.4 | 6.0 |
| 7.5 | 73% | 1.7 | 9.0 |
| 12.5 | 85% | 2.4 | 15 |
| 25 | 92% | 4.5 | 30 |
| 62.5 | 97% | 11 | 75 |

The ramp rate goes as `1/R^2`, so the median 2.75x radius across that whole
sample is a 7.6x gentler curvature ramp than bending at `Rmin` would give. The
growth is bounded, but not as a multiple of `Rmin`: the ceiling is ten times the
*port separation*, floored at `Rmin`, which is why the last column tracks the
first.

Two consequences worth planning around:

- the bend sweeps the middle of the rectangle the two ports span instead of
  hugging its edges, so anything placed there will collide — that is what
  `Rmax` is for
- two identical-looking connections at different spacings get different radii
  and different lengths, so interferometer arms no longer path-length match by
  construction. Setting `Rmax: <the same value as Rmin>` on both arms pins the
  radius exactly and restores that.

Read the achieved radius back from Python when it matters:

```python
plan = plan_route(x_rel, y_rel, delta, r_min=80)
print(plan.shape, plan.radius, plan.length)
```

#### Shapes

The solver works out every shape that reaches the ports and prefers them in
this order — fewest bends first, then the form that keeps straights parallel to
the ports. Within a shape the gentlest bend wins:

| shape | segments | when |
| --- | --- | --- |
| `straight` | one straight | ports collinear and facing each other |
| `corner` | straight, bend, straight | one bend of the whole turn fits — any turn angle except 0 and 180 degrees |
| `sbend` | straight, bend, bend, straight | a lateral offset a single bend cannot absorb, including the 0-turn S-bend |
| `uturn` | bend, straight, bend | the general fallback; the only one that works when the ports face the same way |

Which straights survive depends on what stopped the radius from growing. A
`corner` grows until the shorter of its two straights reaches zero, so at *that*
radius it keeps one — but when the radius ceiling binds first, both survive, and
across separations the split is roughly even (45-52% keep both). An `sbend` is
the opposite of what you might expect: about three quarters of the sampled ones
collapse both end straights to zero and are two bends and nothing else. A
`uturn` has exactly one straight by construction, and it was positive in every
sampled case. Read `plan.segments` rather than assuming a shape's general form.

`p` trades footprint against gradualness. A pure clothoid (`p = 1`) is exactly
twice as long as the circular arc of the same radius and turn, and a 90 degree
one spans the corner box of a *1.87 R* circular bend. Lowering `p` spends part
of the turn at constant curvature `1/R` and buys that footprint back; curvature
stays continuous for any `p > 0`. `p = 0` would be a plain arc and is rejected.

#### Limits

A shape that reaches the ports is not automatically used. `corner` solves
through `1/sin(Delta)`, so as two ports approach antiparallel its straights run
away. Growing the radius removes most of that on its own. Take ports 500 um
apart with the target directly abeam, a turn one microradian short of 180
degrees, `Rmin: 80`: with the radius pinned there the corner closes on the ports
exactly and asks for 559 *metres* of waveguide (a nanoradian short, 559 km).
Freed, the same pose comes back as a corner at R = 181.6 um and 1141 um long.
Candidates whose straights still double back for more than three times the port
separation are discarded, as are any that cross themselves, and so is anything
sweeping more than 1.5x the port separation outside the rectangle the two ports
span. That last screen matters most: a free radius makes pairs of near-half-turn
bends reachable, which close on the ports exactly and pass every other check
while running millimetres across the die. With that screen disabled, the worst
route found between ports 224 um apart was 24.5 mm long — an S-bend at
R = 1996 um between two ports a fifth of a millimetre apart.

Each shape is therefore offered at a ladder of radii from the gentlest down to
`Rmin`, and the screens take the gentlest that survives, so a screen costs
footprint rather than reachability. 12.8% of sampled poses do route only with
the bulge screen switched off — but none of those are near misses. The routes
they would have drawn bulge at least 2.53x the port separation (median 6.9x) and
run 9.5x to 70x the separation in length, and not one of them routes with the
radius pinned at `Rmin` either. Those poses have no sensible route at any
radius, which is the answer a `DesignError` gives you.

Two bends do not reach every pose:

- each bend turns at most 180 degrees, which keeps it a simple curve
- a pose needing three bends — arriving at a port from behind, so the route has
  to overshoot and come back — is not routable
- ports crowded close together often have no solution — the first two columns
  of the table above are the same measurement: at 2.5x `Rmin` of separation only
  9% of poses route, against 92% at 25x

Each of these is a `DesignError` listing what every shape would have needed,
rather than a badly routed waveguide. Split the connection with an intermediate
port, or lower `Rmin`.

[clothoid_demo.yaml](designs/profiles/clothoid_demo.yaml) draws one of each
shape, plus an `Rmax`-capped corner.

### The ONN butterfly closure

`ONN_BUTTERFLY_DEVICE` builds eight canonical resonators, and each closes its
loop with two 180 degree turns. `core.closure_style` picks their shape:

| value | closure |
| --- | --- |
| `arc` (default) | a semicircle, as published — curvature steps 0 to 1/R where it meets each straight |
| `euler` | a clothoid, curvature ramping over `core.closure_p` of the turn |

Those junctions sit *inside a recirculating loop*, so unlike a bend on a bus
they are paid on every pass and cap the loaded Q. That is the whole reason
`euler` exists here.

**`return_arm_offset` is the closure's lateral span, not its diameter.** It
equals `unit_chord(pi, p) * R`, which is `2*R` only for a semicircle and
`2.753663*R` for a pure clothoid. So switching an existing device to `euler`
without touching the offset drives the radius *down* — 60 um yields 21.79 um at
`p: 1`, below the design's own `min_bend_radius`, and the cell raises saying so.
Widen the offset instead, and `tile_half_span` and `interaction_group_pitch`
then have to follow their own two guards.
[onn_butterfly_4x4_euler_closure.yaml](designs/profiles/onn_butterfly_4x4_euler_closure.yaml)
is the worked example and carries the arithmetic in its header.

The apex moves further than the radius does: it sits `2.4501*R` past the arm it
leaves at `p: 1`, against exactly `R` for the semicircle, which makes the euler
device 1569 um tall against the arc version's 1375. Nothing validates
device-to-device spacing, so check the pitch before stacking copies — the
1450 um y pitch in `onn_butterfly_4x4_gap_length_sweep.yaml` is *smaller* than
the euler device, and adjacent copies would merge silently rather than raise.

### Macros and blocks

A macro is a reusable placement + route group; a block instantiates it at an
offset. `substitutions` swaps an instance name inside the body, so one macro
serves many variants — see [WX_Taper_400_SiN.yaml](designs/profiles/Fabricated/WX_Taper_400_SiN.yaml).

```yaml
macros:
  - name: FOUR_CROSS
    placement:
      - place: { inst: __CROSS_CELL__, at: [120, 100], as: X_1 }
      - place: { inst: SemiArc01, at: [0, 0], rot: 90, as: SemiArc01_1 }
    routes:
      - straight: { from: SemiArc01_1.W, to: X_1.W }

blocks:
  - use: FOUR_CROSS
    as: BLOCK_1                   # aliases become BLOCK_1.X_1, ...
    at: [-200, -100]              # offset for every place step
    substitutions: { __CROSS_CELL__: X_1800_01 }
    offsets: { T_E_1: [200, 0] }  # extra per-alias offset
```

## Adding a component

A PCell is any function with this signature:

```python
def PCellMyThing(params: dict, layers: dict) -> tuple[gdstk.Cell, dict[str, Port]]:
```

1. Read *optional* parameters out of `params` with an explicit default, in
   microns. Index a *required* dimension directly (`params["ring_radius"]`) so
   omitting it raises instead of silently building the wrong thing.
2. Get the layer from `resolve_wg_layer(width, layers)` rather than hardcoding,
   so width-based dose splitting keeps working.
3. Return ports in the cell's **local** coordinates. `Port.angle` is in
   radians and points **outward**, away from the component body — the builder
   mates ports by rotating a child so its port faces the target's.
4. Register it in [src/catalog.py](src/catalog.py).

## Layout of the repo

```
build.py            command-line entry point
src/design.py       profile loading, extends resolution, layer/output accessors
src/registry.py     type key -> PCell factory
src/catalog.py       the standard component catalog
src/builder.py      the build engine (build_library returns a gdstk.Library)
src/router.py       route dispatch table
src/place.py        placement and routing primitives
src/clothoid.py     Euler-spiral geometry and the port-to-port solver
src/layer_map.py    width -> GDS layer mapping
src/ports.py        the Port dataclass
src/cells/          the parametric cells
designs/base.yaml   builder defaults (layers, grid_um) - no platform data
designs/platforms/  process platforms; each holds one `_platform` block
designs/profiles/   design profiles (Final/ and Fabricated/ are promoted)
out/                build output (Final/, Fabricated/ and Archive/ are committed)
tools/gdscheck.py   compatibility harness — see below
```

The engine is importable, so a sweep script or a check can build a design
without writing a file:

```python
import src.catalog                     # registers the standard components
from src.builder import build_library
from src.design import resolve_design, resolve_layers, resolve_top_name

cfg = resolve_design("designs/profiles/pulley_400nm.yaml")
lib = build_library(cfg, resolve_layers(cfg), resolve_top_name(cfg))
top = lib.top_level()[0]

# Measure from flattened polygons, never from bounding_box() - see below.
pts = [pt for poly in top.get_polygons(depth=None) for pt in poly.points]
print(min(x for x, _ in pts), max(x for x, _ in pts),
      min(y for _, y in pts), max(y for _, y in pts))
```

**Do not measure this repo's geometry with `Cell.bounding_box()`.** It counts
label origins and inflates the box of a rotated reference, and both are
everywhere here — the builder labels every ARC on layer 100. On the ONN
butterfly device it reports 1592.78 um of height where the waveguides really
span 1569.28, because a label sits 23.5 um above the topmost polygon; on
`Final/StWG_Ring_Coupler.yaml` it reports y -93.19..100.00 for geometry that
spans y -0.90..7.72. Flattened polygons quantised to the 1 nm grid are the
measurement of record, and that is what `tools/gdscheck.py` digests.

## Not breaking fabricated layouts

Several profiles under `designs/profiles/Fabricated/` correspond to chips that
have been written, so their geometry must not drift when the builder changes.
`tools/gdscheck.py` builds every profile and digests the result:

```bash
python tools/gdscheck.py check              # compare against tools/reference.json
```

It reports any change in geometry, in emission order, in cell names or in file
size, and it builds into a temporary directory so `out/` is never touched.
After an *intentional* change, refresh the stored reference:

```bash
python tools/gdscheck.py update-reference
```

Metadata-only keys are the one edit class allowed on a promoted profile: a
`_`-prefixed top-level key never reaches the builder, so `check` must still
report every profile identical afterwards. If it does not, the edit touched
more than the metadata.

The digest covers polygon layer/datatype/vertices and label
layer/texttype/text/origin rather than the raw bytes, because `gdstk` stamps
the current time into every file it writes.

**Nine `chip.out` paths are each claimed by two profiles.** Most of those pairs
are byte-identical YAML, but two are genuinely different designs sharing one
output file, so building one overwrites the other's GDS. `gdscheck` cannot see
it, because it redirects every build with `--out`. Pass `-o` explicitly when
you build `pulley_400nm.yaml`, `Final/pulley_400nm_v1.yaml`,
`pulley_400nm_dose_test.yaml` or `Archive/pulley_400nm_dose_test.yaml`.

## Fabrication notes

- Chip outlines are never emitted. BEAMER must receive device geometry only,
  so a `die` rectangle would become a real exposure; chip dimensions stay in
  the profile as documentation.
- `width_layers` exists because e-beam dose depends on feature width. Splitting
  widths onto separate layers lets each get its own dose recipe — used by the
  `dose_test` profiles.
- Every design here is drawn for one platform: a 400 nm Si3N4 core clad in
  SiO2, recorded machine-readably in `designs/platforms/SiN400_SiO2.yaml` and
  reached through each profile's `extends`. Refractive indices, deposition
  method and design wavelength are **not** recorded anywhere in this repo —
  they are left commented out in that file — so any loss, Q or FSR figure
  quoted about these designs is unanchored until they are measured. Do not fill
  them in from a textbook; SiN's index depends strongly on deposition. The
  three `*AlN*` profiles point at `designs/platforms/AlN_SiO2.yaml`, which
  asserts only the core material: that stack's thickness and cladding were
  never recorded. `clothoid_demo.yaml` and `Archive/demo_small.yaml` point at
  `designs/platforms/unspecified.yaml`, which is non-binding by design — they
  are builder/router fixtures whose geometry is illustrative, so no loss or Q
  number should be quoted from them. That fragment distinguishes "never
  dimensioned for a real stack" from "platform not set yet"; do not point a
  real design at it to dodge an unknown value, comment the field out in a real
  fragment instead.
- Ports live only in memory; they are never drawn. `TEXT` labels (layer 100)
  *are* written into the GDS.
- Bends want continuous curvature. A straight-to-arc junction steps curvature
  from 0 to 1/R and converts power into higher-order modes; `clothoid` ramps it
  linearly with arc length instead, and grows the radius as far above `Rmin` as
  the ports allow, since both peak curvature and curvature rate fall with it.
- The database unit is 1 um with 1 nm precision. Both are hardcoded as
  `GDS_UNIT`/`GDS_PRECISION` in [src/builder.py](src/builder.py);
  `defaults.grid_um` records the same grid for the reader but is never read
  by anything, so changing it changes nothing.
