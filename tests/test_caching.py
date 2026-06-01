"""Tests for opt-in local caching of combined models."""

from __future__ import annotations

import os
import time
from pathlib import Path

from assist_sim import load_combined_model
from assist_sim.cache import compute_key, input_paths


def test_first_call_writes_cache(minimal_human, minimal_device_config, tmp_path):
    load_combined_model(minimal_human, minimal_device_config, cache_dir=tmp_path)
    xmls = list(tmp_path.glob("*.xml"))
    metas = list(tmp_path.glob("*.meta.json"))
    assert len(xmls) == 1
    assert len(metas) == 1


def test_second_call_hits_cache(minimal_human, minimal_device_config, tmp_path):
    m1, _ = load_combined_model(minimal_human, minimal_device_config, cache_dir=tmp_path)
    mtime_after_first = next(tmp_path.glob("*.xml")).stat().st_mtime_ns
    m2, _ = load_combined_model(minimal_human, minimal_device_config, cache_dir=tmp_path)
    # cache hit -> the XML is not rewritten
    assert next(tmp_path.glob("*.xml")).stat().st_mtime_ns == mtime_after_first
    assert m1.nq == m2.nq and m1.nbody == m2.nbody


def test_mtime_bump_invalidates_key(minimal_human, minimal_device_config, tmp_path):
    paths = input_paths(
        minimal_human,
        minimal_device_config,
        str(Path(minimal_human)),  # any extra path; not important here
    )
    key1 = compute_key(paths, "0.1.0")

    # bump mtime of one input
    target = Path(minimal_human)
    os.utime(target, (time.time() + 10, time.time() + 10))
    key2 = compute_key(input_paths(minimal_human, minimal_device_config, str(target)), "0.1.0")
    assert key1 != key2


def test_version_bump_invalidates_key(minimal_human, minimal_device_config):
    paths = input_paths(minimal_human, minimal_device_config, minimal_human)
    assert compute_key(paths, "0.1.0") != compute_key(paths, "0.2.0")


def test_msk_key_part_of_cache_key(minimal_human, minimal_device_config):
    paths = input_paths(minimal_human, minimal_device_config, minimal_human)
    assert compute_key(paths, "0.1.0", "msk_a") != compute_key(paths, "0.1.0", "msk_b")
