# gdsgen_wx — Waveguide Crossing (WX) GDS generator

This project generates a **waveguide crossing** (WX) as a parametric GDSII cell using **gdstk**.
Designed for easy use in **VS Code**.

## Quick start

```bash
# 1) Create venv (optional)
python -m venv .venv
# 2) Activate it
# Windows: .venv\Scripts\activate
# macOS/Linux:
source .venv/bin/activate
# 3) Install deps
pip install -r requirements.txt
# 4) Build a single crossing (WX)
python scripts/build_wx.py --WM 1.6 --LM 8.0 --LT 10.0 --w_in 0.5 --out out/wx_demo.gds
# 5) Sweep examples
python scripts/sweep_wx.py --out out/wx_sweep.gds
```

Open any generated GDS in **KLayout**:

```bash
klayout out/wx_demo.gds
```

## What is generated

- A **Waveguide Crossing** cell with:
  - Input waveguide width `w_in` (default 0.5 µm)
  - Multi-mode window width `WM`
  - Length `LM`
  - Tapers of length `LT` (from `w_in` to `WM` and back)
  - 4 labeled ports: `PORT_W/E/S/N`
  - Optional die outline, alignment marks (you can toggle in `cells.py`)
- Layers default: WG=1, MARK=10, DIE=20, PORT=99, TEXT=100

## Files

- `src/cells.py`     — PCell definitions (including `PCellCrossing` for WX)
- `src/export.py`    — GDS writer helper
- `src/rules.py`     — Foundry/layer constants
- `scripts/build_wx.py`  — CLI to generate one WX
- `scripts/sweep_wx.py`  — CLI to generate an array sweep of WX variants
- `config/crossing_default.json` — Example parameter set

## Notes

- For **SiN vs SOI**, recompute the **beat length** `LB = λ / [2 (n0 - n1)]` for your stack to choose `LM`.
- Keep the Python as **source of truth**; use KLayout to **view/measure** or do minor edits/merges.
