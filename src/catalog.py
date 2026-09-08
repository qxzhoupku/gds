"""The standard component catalog — every PCell type a profile can name.

Importing this module populates :mod:`src.registry`.  It is the single place
to wire up a new component:

1. write ``src/cells/my_thing.py`` exposing
   ``PCellMyThing(params, layers) -> (cell, ports)``
2. import it below and add one ``CATALOG`` entry

Nothing else needs to change — the builder, the CLI's ``--list-types`` and the
error message for an unknown type all read from the registry.
"""

from __future__ import annotations

from . import registry
from .cells.any_arc import PCellAnyArc
from .cells.onn_butterfly_device import PCellONNButterflyDevice
from .cells.onn_butterfly_network import (
    PCellONNButterflyNetwork,
    PCellONNButterflyResonator,
)
from .cells.pulley_ring import PCellADDDROPPulleyRing, PCellPulleyRing
from .cells.racetrack import PCellRacetrack
from .cells.ring import PCellRingCoupler
from .cells.taper import PCellTaper
from .cells.width_varying_ring import (
    PCellConstantWidthRing,
    PCellWidthVaryingRing,
)
from .cells.wx import PCellWx

#: YAML ``type`` string -> factory.  Keys are part of the profile format and
#: must stay stable; every committed design depends on them.
CATALOG = {
    "WX": PCellWx,
    "TAPER": PCellTaper,
    "RING": PCellRingCoupler,
    "RACETRACK": PCellRacetrack,
    "ARC": PCellAnyArc,
    "PULLEY_RING": PCellPulleyRing,
    "PULLEY_ADD_DROP_RING": PCellADDDROPPulleyRing,
    "WIDTH_VARYING_RING": PCellWidthVaryingRing,
    "CONSTANT_WIDTH_RING": PCellConstantWidthRing,
    "ONN_BUTTERFLY_NETWORK": PCellONNButterflyNetwork,
    "ONN_BUTTERFLY_RESONATOR": PCellONNButterflyResonator,
    "ONN_BUTTERFLY_DEVICE": PCellONNButterflyDevice,
}

registry.register_all(CATALOG)
