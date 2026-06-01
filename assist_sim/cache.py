"""Opt-in local caching of combined models.

Caching is **off by default**.  Callers pass ``cache_dir=Path(...)`` to opt in.
The cache key is a hash of every input file's absolute path + mtime plus the
pipeline ``__version__``, so editing any source model/config or upgrading the
pipeline invalidates stale entries automatically.

Layout: ``<cache_dir>/<key>.xml`` (the exported combined model) and
``<key>.meta.json`` (provenance).  No global / ``~/.cache`` magic.
"""

from __future__ import annotations

import hashlib
import json
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import List, Optional, Tuple

import mujoco as mj


def _resolve_includes(xml_path: Path, seen: Optional[set] = None) -> List[Path]:
    """Return *xml_path* plus all XMLs it pulls in via ``<include>``."""
    seen = seen if seen is not None else set()
    xml_path = xml_path.resolve()
    if xml_path in seen or not xml_path.exists():
        return list(seen)
    seen.add(xml_path)
    try:
        root = ET.parse(str(xml_path)).getroot()
    except ET.ParseError:
        return list(seen)
    for inc in root.iter("include"):
        f = inc.get("file")
        if f:
            _resolve_includes((xml_path.parent / f), seen)
    return list(seen)


def input_paths(human_xml: str, device_config_path: str, device_model_xml: str) -> List[Path]:
    """Collect every input file whose change should invalidate the cache."""
    paths: set = set()
    paths.update(_resolve_includes(Path(human_xml)))
    paths.add(Path(device_config_path).resolve())
    paths.update(_resolve_includes(Path(device_model_xml)))
    return sorted(paths, key=str)


def compute_key(paths: List[Path], version: str, msk_key: Optional[str] = None) -> str:
    """Hash input paths + their mtimes + pipeline version into a cache key."""
    h = hashlib.sha1()
    h.update(version.encode())
    h.update((msk_key or "").encode())
    for p in sorted(paths, key=str):
        p = Path(p)
        h.update(str(p).encode())
        if p.exists():
            h.update(str(p.stat().st_mtime_ns).encode())
        else:
            h.update(b"<missing>")
    return h.hexdigest()


def cached_xml_path(cache_dir: Path, key: str) -> Path:
    return Path(cache_dir) / f"{key}.xml"


def try_load(cache_dir: Path, key: str) -> Optional[Tuple[mj.MjModel, mj.MjData]]:
    """Load a cached combined model if present; otherwise return None."""
    xml = cached_xml_path(cache_dir, key)
    if not xml.exists():
        return None
    model = mj.MjModel.from_xml_path(str(xml))
    return model, mj.MjData(model)


def write_meta(cache_dir: Path, key: str, meta: dict) -> None:
    """Write provenance for a cache entry next to its XML."""
    (Path(cache_dir) / f"{key}.meta.json").write_text(
        json.dumps(meta, indent=2), encoding="utf-8"
    )
