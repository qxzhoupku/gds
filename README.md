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
| `RACETRACK` | [racetrack.py](src/cells/racetrack.py) | Point-coupled racetrack |
| `PULLEY_RING` | [pulley_ring.py](src/cells/pulley_ring.py) | Ring with a wrapped (pulley) bus |
| `PULLEY_ADD_DROP_RING` | [pulley_ring.py](src/cells/pulley_ring.py) | Pulley ring with add and drop buses |
| `WIDTH_VARYING_RING` | [width_varying_ring.py](src/cells/width_varying_ring.py) | Ring with periodic triangular width modulation |
| `CONSTANT_WIDTH_RING` | [width_varying_ring.py](src/cells/width_varying_ring.py) | Constant-width control for the above |
| `ONN_BUTTERFLY_NETWORK` | [onn_butterfly_network.py](src/cells/onn_butterfly_network.py) | 4x4 butterfly-coupled resonator core |
| `ONN_BUTTERFLY_RESONATOR` | [onn_butterfly_network.py](src/cells/onn_butterfly_network.py) | One canonical resonator from that core |
| `ONN_BUTTERFLY_DEVICE` | [onn_butterfly_device.py](src/cells/onn_butterfly_device.py) | The core plus pump bus, drop probes and facet tapers |

## Profile format

```yaml
extends: designs/base.yaml        # optional, resolved transitively

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
absorb whatever is left. On random port pairs at `Rmin: 80` the median bend
comes out 1.2x that floor where the ports are crowded and 3.9x where they have
room, reaching 26x — a 1.5x to 15x gentler curvature ramp than bending at
`Rmin` would give, and up to 680x at the extreme.

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

At the maximum radius a `corner` usually loses one of its two straights — that
is precisely where growing the radius stops. The two-bend shapes often keep
theirs: an interior optimum with both straights positive is common, so the
segment list is worth reading rather than assuming.

`p` trades footprint against gradualness. A pure clothoid (`p = 1`) is exactly
twice as long as the circular arc of the same radius and turn, and a 90 degree
one spans the corner box of a *1.87 R* circular bend. Lowering `p` spends part
of the turn at constant curvature `1/R` and buys that footprint back; curvature
stays continuous for any `p > 0`. `p = 0` would be a plain arc and is rejected.

#### Limits

A shape that reaches the ports is not automatically used. `corner` solves
through `1/sin(Delta)`, so as two ports approach antiparallel its straights run
away. Growing the radius removes most of that on its own — a pinned-radius
corner a microradian short of 180 degrees solved to 359 *metres* of waveguide
between ports 500 um apart; the same pose now comes back as a 138 um bend.
Candidates whose straights still double back for more than three times the port
separation are discarded, as are any that cross themselves, and so is anything
sweeping more than 1.5x the port separation outside the rectangle the two ports
span. That last screen matters most: a free radius makes pairs of near-half-turn
bends reachable, which close on the ports exactly and pass every other check
while running millimetres across the die — 15 mm of waveguide between ports
224 um apart, in one measured case. Each shape is therefore offered at a ladder
of radii from the gentlest down to `Rmin`, and the screens take the gentlest
that survives, so a screen costs footprint rather than reachability.

Two bends do not reach every pose:

- each bend turns at most 180 degrees, which keeps it a simple curve
- a pose needing three bends — arriving at a port from behind, so the route has
  to overshoot and come back — is not routable
- ports crowded close together often have no solution: of random poses at
  `Rmin: 80`, 92% route inside a 2 mm box, 62% inside 600 um and only 9%
  inside 200 um

Each of these is a `DesignError` listing what every shape would have needed,
rather than a badly routed waveguide. Split the connection with an intermediate
port, or lower `Rmin`.

[clothoid_demo.yaml](designs/profiles/clothoid_demo.yaml) draws one of each
shape, plus an `Rmax`-capped corner.

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

1. Read every parameter out of `params` with an explicit default, in microns.
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
designs/profiles/   design profiles (Final/ and Fabricated/ are promoted)
out/                build output (only Final/ and Fabricated/ are committed)
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
print(lib.top_level()[0].bounding_box())
```

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

The digest covers polygon layer/datatype/vertices and label
layer/texttype/text/origin rather than the raw bytes, because `gdstk` stamps
the current time into every file it writes.

## Fabrication notes

- Chip outlines are never emitted. BEAMER must receive device geometry only,
  so a `die` rectangle would become a real exposure; chip dimensions stay in
  the profile as documentation.
- `width_layers` exists because e-beam dose depends on feature width. Splitting
  widths onto separate layers lets each get its own dose recipe — used by the
  `dose_test` profiles.
- Ports live only in memory; they are never drawn. `TEXT` labels (layer 100)
  *are* written into the GDS.
- Bends want continuous curvature. A straight-to-arc junction steps curvature
  from 0 to 1/R and converts power into higher-order modes; `clothoid` ramps it
  linearly with arc length instead, and grows the radius as far above `Rmin` as
  the ports allow, since both peak curvature and curvature rate fall with it.
- The database unit is 1 um with 1 nm precision, matching `grid_um: 0.001`.
