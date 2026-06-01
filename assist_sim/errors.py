"""Shared error helpers: raise-by-default with 'did you mean' suggestions.

Centralizes the message format used across the preprocess layer, the
combiner, and the standalone validator so unresolved-name errors read the
same everywhere.
"""

from __future__ import annotations

import difflib
from typing import Iterable, List, Optional


def closest_matches(name: str, candidates: Iterable[str], limit: int = 3) -> List[str]:
    """Return up to ``limit`` candidate names closest to ``name``.

    Uses difflib for fuzzy matching and also includes simple substring
    matches, which difflib can miss for short typos in long names.
    """
    candidates = [c for c in candidates if c]
    fuzzy = difflib.get_close_matches(name, candidates, n=limit, cutoff=0.6)

    lowered = name.lower()
    substring = [c for c in candidates if lowered in c.lower() or c.lower() in lowered]

    ordered: List[str] = []
    for c in fuzzy + substring:
        if c not in ordered:
            ordered.append(c)
    return ordered[:limit]


def unknown_reference(
    name: str,
    candidates: Iterable[str],
    *,
    section: str,
    kind: str,
) -> ValueError:
    """Build a ValueError for an unresolved name reference.

    Args:
        name: the offending name that could not be resolved.
        candidates: the set of valid names to suggest from.
        section: the YAML section (or context) the name came from.
        kind: the kind of object (e.g. "body", "actuator", "tendon").
    """
    suggestions = closest_matches(name, candidates)
    msg = f"{section} references unknown {kind} '{name}'."
    if suggestions:
        joined = ", ".join(f"'{s}'" for s in suggestions)
        msg += f" Did you mean: {joined}?"
    return ValueError(msg)
