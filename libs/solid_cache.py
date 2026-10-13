"""In-memory solids of one design, built once and shared by every CAD consumer (headless).

Every expensive solid (body lofts, panels, printable parts, access material, the whole-airframe pieces, drawing
projections) is built through cached(): the first consumer builds it, every later one (section export, STEP, STL,
drawings, exact mass) reuses it while the design is unchanged. One design is held at a time; the store is dropped
when a different design asks. The editor never builds solids, so editing alone never invalidates it: returning to a
built design finds it still there. Nothing here is written to or read from disk.

The design digest covers the parsed definition, the resolved motors (their library dimensions) and the source of the
modules that build solids, so a code change cannot serve a stale solid.
"""

from __future__ import annotations

import hashlib
from collections.abc import Callable, Hashable
from pathlib import Path
from typing import TypeVar

from libs.geometry import ResolvedAirframe

T = TypeVar("T")

SOLID_SOURCES = ("airframe.py", "geometry.py", "print_features.py", "body_solids.py", "access.py", "cad.py", "drawings.py")
_SOURCE_DIGEST = hashlib.sha256(b"".join((Path(__file__).parent / name).read_bytes() for name in SOLID_SOURCES)).hexdigest()

_store: dict[Hashable, object] = {}
_store_digest: str | None = None
_last_digest: tuple[ResolvedAirframe, str] | None = None  # the resolved airframe asked last and its digest


def design_digest(resolved: ResolvedAirframe) -> str:
    """The identity of the solids this resolved airframe builds."""
    global _last_digest
    if _last_digest is not None and _last_digest[0] is resolved:
        return _last_digest[1]
    text = repr((resolved.airframe, resolved.motors)) + _SOURCE_DIGEST
    digest = hashlib.sha256(text.encode()).hexdigest()
    _last_digest = (resolved, digest)
    return digest


def cached(resolved: ResolvedAirframe, key: Hashable, build: Callable[[], T]) -> T:
    """The design's `key` solid, built by build() the first time it is asked for."""
    global _store_digest
    digest = design_digest(resolved)
    if digest != _store_digest:
        _store.clear()
        _store_digest = digest
    if key not in _store:
        _store[key] = build()
    return _store[key]  # type: ignore[return-value]


def is_built(resolved: ResolvedAirframe, key: Hashable) -> bool:
    """True when the design's `key` solid is already held (no building)."""
    return design_digest(resolved) == _store_digest and key in _store
