"""Opt-in local caching of combined models.

Caching is **off by default**.  Callers pass ``cache_dir=Path(...)`` to opt in.

The cache key is a hash of every input file's absolute path + mtime, plus a token per
package (see ``loading._package_token``) that folds in both the release version *and* the
newest source mtime.  The token matters: keying on the version alone meant an editable
install -- the normal development setup -- never invalidated, because the version is fixed at
install time.  Editing a device config, a device XML, assist_sim, or myo_sim all invalidate.

A hit skips the compose as well as the combine.  Whether that is a win depends on the model:
the reload is dominated by XML text parsing, so it is ~9x faster for ``myolegs22`` and ~1.6x
*slower* for ``myofullbody``.  See :func:`assist_sim.loading.load_combined` for the numbers.

Layout: ``<cache_dir>/<key>.xml`` (the exported combined model) and ``<key>.meta.json``
(provenance).  An entry is built under a per-writer ``<key>.<pid>.<rand>.partial`` name and
published with :func:`commit`, so neither an interrupted run nor N parallel workers racing a
cold cache can leave a half-written file that later reads as a hit.  No global / ``~/.cache`` magic, and no eviction: the directory is the caller's to
delete.
"""

from __future__ import annotations

import hashlib
import json
import os
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import List, Optional, Tuple
from uuid import uuid4

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


def input_paths_composed(device_config_path: str, device_model_xml: str) -> List[Path]:
    """Collect cache-invalidating input files for a myo_sim-composed MSK.

    Unlike :func:`input_paths`, there is no human XML on disk -- the MSK is
    composed in-memory from the ``myo_sim`` package.  The myo_sim package
    identity is folded into the cache key by the caller (via the ``version``
    string), so only the device files are collected here.
    """
    paths: set = set()
    paths.add(Path(device_config_path).resolve())
    paths.update(_resolve_includes(Path(device_model_xml)))
    return sorted(paths, key=str)


def input_paths_msk() -> List[Path]:
    """Collect cache-invalidating input files for a device-less MSK: none.

    A myo_sim-composed MSK has no source file on disk and no device is involved,
    so its identity is entirely ``(msk_key, assist_sim token, myo_sim token)``
    -- all three of which :func:`compute_key` already folds in via its
    ``version`` and ``msk_key`` arguments.  This exists so the MSK-only caller
    reads the same way as the combined ones rather than passing a bare ``[]``.
    """
    return []


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


def staging_xml_path(cache_dir: Path, key: str) -> Path:
    """A private path to build an entry in, before :func:`commit` publishes it.

    The name is unique per call, not just per key.  The cache's main beneficiary is a
    multi-process launch -- an RL run starts N ``SubprocVecEnv`` workers that all miss a cold
    cache at once -- and a shared staging name would have them interleave writes into one
    half-built file.  Each writer now builds its own copy and the last ``commit`` wins, which
    is harmless because they all produce the same model.
    """
    return Path(cache_dir) / f"{key}.{os.getpid()}.{uuid4().hex[:8]}.partial"


def commit(cache_dir: Path, key: str, staged: Path) -> Path:
    """Atomically publish *staged* as the entry for *key*, and return the final path.

    Entries used to be written straight to their final name, so an interrupted run left a
    truncated XML that :func:`try_load` would then treat as a valid hit forever.
    ``os.replace`` is atomic on the same filesystem, so an entry is either absent or complete.
    """
    final = cached_xml_path(cache_dir, key)
    os.replace(staged, final)
    return final


def try_load(cache_dir: Path, key: str) -> Optional[Tuple[mj.MjModel, mj.MjData]]:
    """Load a cached combined model if present and readable; otherwise return None.

    A cache is an optimisation, so an entry that will not load is treated as a miss and
    discarded rather than raised: the caller then rebuilds it.  Without this, one corrupt
    file turned every later call into a MuJoCo parse error with no hint that deleting the
    cache would fix it.
    """
    xml = cached_xml_path(cache_dir, key)
    if not xml.exists():
        return None
    try:
        model = mj.MjModel.from_xml_path(str(xml))
    except Exception:  # noqa: BLE001 - unreadable entry is a miss, not a failure
        xml.unlink(missing_ok=True)
        (Path(cache_dir) / f"{key}.meta.json").unlink(missing_ok=True)
        return None
    return model, mj.MjData(model)


def write_meta(cache_dir: Path, key: str, meta: dict) -> None:
    """Write provenance for a cache entry next to its XML."""
    (Path(cache_dir) / f"{key}.meta.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
