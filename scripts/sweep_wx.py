# scripts/sweep_wx.py
# Array sweep of WM and LM for the two-unit crossing.
import argparse
import gdstk
from src.cells import PCellCrossingUnits
from src.export import write_gds

def main():
    ap = argparse.ArgumentParser(description="Sweep WM and LM to produce a GDS array of two-unit crossings.")
    ap.add_argument("--WM", type=float, nargs="+", default=[1.4, 1.6, 1.8, 2.0], help="List of WM values (um)")
    ap.add_argument("--LM", type=float, nargs="+", default=[6.0, 8.0, 10.0, 12.0], help="List of LM values (um)")
    ap.add_argument("--LT", type=float, default=10.0, help="Linear taper length (um)")
    ap.add_argument("--w_in", type=float, default=0.5, help="Singlemode waveguide width (um)")
    ap.add_argument("--pitch", type=float, default=200.0, help="Array pitch (um)")
    ap.add_argument("--out", type=str, default="out/wx_units_sweep.gds", help="Output GDS path")
    args = ap.parse_args()

    top = gdstk.Library(unit=1e-6, precision=1e-9)
    top_cell = top.new_cell("WX_UNITS_SWEEP")

    for i, WM in enumerate(args.WM):
        for j, LM in enumerate(args.LM):
            child_lib, child_cell = PCellCrossingUnits(WM=WM, LM=LM, LT=args.LT, w_in=args.w_in,
                                                       add_die=False, name=f"WX_UNITS_WM{WM}_LM{LM}")
            ref = gdstk.Reference(child_cell, (i * args.pitch, j * args.pitch))
            top_cell.add(ref)

    top.add(top_cell)
    out = write_gds(top, args.out)
    print("Wrote", out)

if __name__ == "__main__":
    main()
