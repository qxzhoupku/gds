# Photonic Chip Starter — YAML-driven GDS assembly (Python + gdstk)

Build reproducible photonic chips from a single YAML file in VS Code.
Components included: Waveguide Crossing (two identical units), Taper, Ring coupler, Racetrack coupler.

## Quick start (Windows 11 / PowerShell)
```powershell
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt

# Build demo chip (uses designs/profiles/demo_small.yaml)
python build.py

# Open the result in KLayout
klayout out\chip_demo.gds
```
