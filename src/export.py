# src/export.py
import gdstk
from pathlib import Path

def write_gds(library: gdstk.Library, out_path: str):
    p = Path(out_path)
    p.parent.mkdir(parents=True, exist_ok=True)
    library.write_gds(str(p))
    return str(p.resolve())
