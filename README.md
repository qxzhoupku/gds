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
  - straight:  { from: R1_1.E, to: T1_1.W }
  - manhattan: { from: A.E, to: B.W, r: 50 }
  - euler:     { from: A.E, to: B.W, Rmin: 80 }
```

`place` positions a cell absolutely (`rot` in degrees). `connect` mates
`inst.port` face-to-face against an already-placed `to` port. Routes are drawn
on the layer implied by the narrower of the two port widths.

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
- The database unit is 1 um with 1 nm precision, matching `grid_um: 0.001`.
