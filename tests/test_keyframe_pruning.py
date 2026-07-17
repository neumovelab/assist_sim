"""The keyframe pipeline gracefully handles keyframe-less composed MSKs.

Both composed leg models (``myolegs`` and ``myolegs26``) now ship without a
keyframe, so a device's ``keyframe_overrides`` have no base keyframe to modify.
The pipeline must no-op cleanly -- no crash, no fabricated keyframe -- rather than
the old behavior where a MSK ``stand`` keyframe was decomposed by joint name and
rebuilt after surgery.
"""

from __future__ import annotations


from assist_sim import load_combined

from .conftest import needs_myo_sim


@needs_myo_sim
def test_keyframe_overrides_noop_without_base_keyframe():
    """DephyExoBoot declares per-keyframe ``keyframe_overrides`` (e.g. pelvis_ty),
    but the composed MSK is keyframe-less, so the overrides have nothing to attach
    to.  The combine must still succeed and emit no keyframe -- exercising the
    keyframe-handling path's no-base branch (graceful degradation now that no
    composed MSK ships a keyframe)."""
    for msk in ("myolegs26", "myolegs"):
        model, _ = load_combined(msk, "DephyExoBoot_L1")
        assert model.nkey == 0, f"{msk}: unexpected keyframe from a keyframe-less base"
