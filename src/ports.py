"""Port — the fundamental connection point for photonic components.

All angles are in **radians**.  The angle describes the direction the port
faces *outward* (i.e. away from the component body).
"""

from dataclasses import dataclass


@dataclass
class Port:
    name: str
    x: float
    y: float
    angle: float  # radians — outward-facing direction
    width: float
    layer: int = 1
