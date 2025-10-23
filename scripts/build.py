# scripts/build_wx.py
# Build a single Waveguide Crossing (WX) from two identical units.
import argparse
from src.cells import PCellCrossingUnits
from src.export import write_gds

"""
Parameters for the Waveguide Crossing (WX) unit cell:
  WM: MMW width (um)
  LM: MMW length (um)
  LT: Linear taper length (um)
  w_in: Singlemode waveguide width (um)
  die_size: Size of the die outline (um)
  add_die: Whether to add the die outline
  name: Name of the cell
"""

WM = 1.6
LM = 8.0
LT = 10.0
w_in = 0.5
die_size = 150.0
add_die = True
name = "WX_UNITS"




def main():
    ap = argparse.ArgumentParser(description="Generate a Waveguide Crossing (two identical units).")
    ap.add_argument("--WM", type=float, default=1.6, help="MMW width (um)")
    ap.add_argument("--LM", type=float, default=8.0, help="MMW length (um)")
    ap.add_argument("--LT", type=float, default=10.0, help="Linear taper length (um)")
    ap.add_argument("--w_in", type=float, default=0.5, help="Singlemode waveguide width (um)")
    ap.add_argument("--die", type=float, default=150.0, help="Die size (um)")
    ap.add_argument("--no_die", action="store_true", help="Do not draw the die outline")
    ap.add_argument("--out", type=str, default="out/wx_units_demo.gds", help="Output GDS path")
    args = ap.parse_args()

    lib, cell = PCellCrossingUnits(WM=args.WM, LM=args.LM, LT=args.LT, w_in=args.w_in,
                                   die_size=args.die, add_die=not args.no_die, name="WX_UNITS")
    out = write_gds(lib, args.out)
    print("Wrote", out)

if __name__ == "__main__":
    main()
