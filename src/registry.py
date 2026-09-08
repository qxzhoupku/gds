"""PCell registry — maps a YAML ``type`` string to a cell factory.

Every factory shares one contract::

    factory(params: dict, layers: dict) -> (gdstk.Cell, dict[str, Port])

``params`` is the profile's ``params`` block for that instance, ``layers`` is
the resolved layer table (see :func:`src.design.resolve_layers`), and the
returned ports are in the cell's own local coordinates — the builder
transforms them on placement.

To add a component, write the factory and register it::

    from src.registry import register

    @register("MY_THING")
    def PCellMyThing(params, layers):
        ...
        return cell, ports

Registering a name that is already taken raises, so two components cannot
silently shadow each other.
"""

from __future__ import annotations

from typing import Callable, Dict, Iterable, Tuple

import gdstk

from .design import DesignError
from .ports import Port

PCellFactory = Callable[[dict, dict], Tuple[gdstk.Cell, Dict[str, Port]]]

_REGISTRY: Dict[str, PCellFactory] = {}


class UnknownPCellError(DesignError, KeyError):
    """Raised when a profile names a ``type`` that is not registered.

    Subclasses :class:`~src.design.DesignError` so the CLI reports it as a
    profile error rather than a traceback.
    """


def register(type_key: str, factory: PCellFactory | None = None):
    """Register *factory* under *type_key*.  Usable as a decorator."""
    def _do(f: PCellFactory) -> PCellFactory:
        if type_key in _REGISTRY and _REGISTRY[type_key] is not f:
            raise ValueError(
                f"PCell type {type_key!r} is already registered to "
                f"{_REGISTRY[type_key].__name__}."
            )
        _REGISTRY[type_key] = f
        return f

    return _do if factory is None else _do(factory)


def register_all(mapping: Dict[str, PCellFactory]) -> None:
    """Register every ``{type_key: factory}`` pair in *mapping*."""
    for type_key, factory in mapping.items():
        register(type_key, factory)


def get(type_key: str) -> PCellFactory:
    """Look up a factory, with the available types in the error message."""
    try:
        return _REGISTRY[type_key]
    except KeyError:
        raise UnknownPCellError(
            f"Unknown PCell type {type_key!r}. Available: {available()}"
        ) from None


def available() -> list[str]:
    """Sorted list of registered type keys."""
    return sorted(_REGISTRY)


def items() -> Iterable[Tuple[str, PCellFactory]]:
    """Iterate over ``(type_key, factory)`` pairs."""
    return tuple(_REGISTRY.items())
