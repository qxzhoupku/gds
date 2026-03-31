"""Width-based layer mapping for multi-process e-beam lithography.

In e-beam fabrication, waveguides of different widths often need different
dose/current/resist recipes.  This module maps a waveguide width to the
correct GDS layer number so that each process step can be separated.

Configuration in YAML (inside ``defaults``)::

    defaults:
      layers:
        WG: 1          # fallback waveguide layer (still required)
        TEXT: 100
        PORT: 99
      width_layers:    # optional — omit to put everything on WG
        - max_width: 0.6
          layer: 1     # narrow single-mode guides
        - max_width: 2.0
          layer: 2     # tapers and multimode regions
        - layer: 3     # catch-all for anything wider

Rules are evaluated in order; the first rule whose ``max_width`` is ≥ the
query width wins.  A rule without ``max_width`` is the catch-all (must be
last).  If no ``width_layers`` key exists, every width maps to ``layers["WG"]``.
"""

from __future__ import annotations


def resolve_wg_layer(width: float, layers: dict) -> int:
    """Return the GDS layer number appropriate for a waveguide of *width*.

    Parameters
    ----------
    width : float
        Waveguide width in microns.
    layers : dict
        The full ``layers`` dict from the design config, optionally containing
        a ``width_layers`` list of rules.

    Returns
    -------
    int
        GDS layer number.
    """
    rules = layers.get("width_layers")
    if not rules:
        return layers.get("WG", 1)

    for rule in rules:
        max_w = rule.get("max_width")
        if max_w is None or width <= max_w:
            return rule["layer"]

    # No rule matched (shouldn't happen if a catch-all is present)
    return layers.get("WG", 1)
