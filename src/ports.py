from dataclasses import dataclass

@dataclass
class Port:
    name: str
    x: float
    y: float
    angle: float   # degrees, outward from the cell
    width: float
    layer: int
