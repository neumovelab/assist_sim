"""``DeviceConfig.from_yaml`` rejects what it used to ignore.

Each case below is a config that previously loaded clean and then did nothing -- a typo'd
section, a typo'd key inside an entry, a per-MSK block keyed by a misspelled MSK name, an
unimplemented actuator type. The package's stated rule is errors over warnings, so these
are ValueErrors with a "did you mean" rather than silent no-ops.

The shipped device configs are all clean against these checks; see
``test_shipped_configs_are_strict_clean``.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from assist_sim.config import DeviceConfig
from assist_sim.registry import DEVICE_CONFIGS

BASE = """
device:
  name: "Probe"
  model_xml: "minimal_device.xml"
attachments:
  - device_body: "dev_a"
    parent_body: "pelvis"
"""


def _write(tmp_path: Path, fixtures: Path, body: str) -> str:
    """Write a config next to a copy of the device XML it references."""
    (tmp_path / "minimal_device.xml").write_bytes((fixtures / "minimal_device.xml").read_bytes())
    path = tmp_path / "L1config.yaml"
    path.write_text(body, encoding="utf-8")
    return str(path)


@pytest.fixture
def fixtures() -> Path:
    return Path(__file__).resolve().parent / "fixtures"


@pytest.mark.parametrize(
    "body,expected",
    [
        # An unknown top-level section: 'bodyremovals' used to be dropped in silence.
        (BASE + "bodyremovals: [shank]\n", "bodyremovals"),
        # An unknown key inside the device block.
        (
            'device:\n  name: "Probe"\n  model_xml: "minimal_device.xml"\n  compatible_mks: [myolegs26]\n'
            'attachments:\n  - device_body: "dev_a"\n    parent_body: "pelvis"\n',
            "compatible_mks",
        ),
        # An unknown key inside an entry: 'position' for 'pos' seats the device wrong.
        (
            'device:\n  name: "Probe"\n  model_xml: "minimal_device.xml"\n'
            'attachments:\n  - device_body: "dev_a"\n    parent_body: "pelvis"\n    position: [0, 0, 0]\n',
            "position",
        ),
        # A per-MSK block keyed by a misspelled MSK name never applied.
        (BASE + "body_removals:\n  myolegz26: [shank]\n", "myolegz26"),
        # Same, for a section whose per-MSK entries are dicts.
        (BASE + "joint_overrides:\n  myolegz26:\n    - name: knee\n      range: [-1, 0]\n", "myolegz26"),
        # An actuator type the pipeline does not implement would silently become a motor.
        (BASE + 'actuators:\n  - name: a1\n    joint: "dev_joint"\n    type: position\n', "position"),
        # A declared motor whose parameters contradict the declaration.
        (
            BASE + 'actuators:\n  - name: a1\n    joint: "dev_joint"\n    type: motor\n    biastype: affine\n',
            "motor",
        ),
        # reposition_* ignores new_body, so accepting one would silently not re-anchor.
        (
            BASE
            + "tendon_modifications:\n  - name: t\n    wraps:\n      - reposition_site: s\n        new_body: b\n        pos: [0, 0, 0]\n",
            "ignores",
        ),
        # An equality carrying keys from the other form: they are read by neither.
        (
            BASE + "equality:\n  - type: joint\n    joint1: a\n    device_body: d\n",
            "belong to the connect/weld form",
        ),
        (
            BASE
            + "equality:\n  - type: connect\n    device_body: d\n    parent_body: p\n    anchor: [0, 0, 0]\n    polycoef: [1]\n",
            "belong to the joint form",
        ),
        # A wrap edit with an unknown key.
        (
            BASE
            + "tendon_modifications:\n  - name: t\n    wraps:\n      - replace_site: s\n        newbody: b\n        pos: [0, 0, 0]\n",
            "newbody",
        ),
    ],
)
def test_silently_ignored_keys_now_raise(tmp_path, fixtures, body, expected):
    with pytest.raises(ValueError, match=expected):
        DeviceConfig.from_yaml(_write(tmp_path, fixtures, body))


def test_flat_only_section_rejects_the_per_msk_form(tmp_path, fixtures):
    """``actuators`` takes a flat list; the dict form used to raise a bare TypeError.

    ``TypeError: string indices must be integers`` named neither the section nor the file.
    """
    body = BASE + 'actuators:\n  default:\n    - name: a1\n      joint: "dev_joint"\n'
    with pytest.raises(ValueError, match="actuators.*flat list"):
        DeviceConfig.from_yaml(_write(tmp_path, fixtures, body))


def test_suggestion_points_at_the_intended_key(tmp_path, fixtures):
    """The error carries a 'did you mean', not just a rejection."""
    with pytest.raises(ValueError, match="did you mean.*body_removals"):
        DeviceConfig.from_yaml(_write(tmp_path, fixtures, BASE + "bodyremovals: [shank]\n"))


def test_valid_config_still_loads(tmp_path, fixtures):
    """The strict checks must not reject a well-formed config."""
    body = BASE + (
        "body_removals:\n  default: [shank]\n  myolegs: [shank, patella_r]\n"
        'actuators:\n  - name: a1\n    joint: "dev_joint"\n    type: motor\n    gear: [1.0]\n'
    )
    config = DeviceConfig.from_yaml(_write(tmp_path, fixtures, body))
    assert config.name == "Probe"
    assert config.resolve_body_removals("myolegs") == ["shank", "patella_r"]
    assert config.resolve_body_removals("myolegs26") == ["shank"]


def test_legacy_keyframes_section_still_loads(tmp_path, fixtures):
    """The legacy ``keyframes`` section is a mapping, and must not trip the flat-only check.

    No shipped config uses it, so a shape-based rejection here would have gone unnoticed
    until someone relied on the documented legacy mode.
    """
    body = BASE + "keyframes:\n  home:\n    time: 0.0\n    qpos: [0.0, 0.1]\n"
    config = DeviceConfig.from_yaml(_write(tmp_path, fixtures, body))
    assert set(config.keyframes) == {"home"}
    assert config.keyframes["home"].qpos == [0.0, 0.1]


def test_shipped_configs_are_strict_clean():
    """Every bundled device config passes the strict loader."""
    failures = []
    for key, path in sorted(DEVICE_CONFIGS.items()):
        try:
            DeviceConfig.from_yaml(str(path))
        except Exception as exc:  # noqa: BLE001 - collect all, report together
            failures.append(f"{key}: {type(exc).__name__}: {exc}")
    assert not failures, "shipped configs rejected by the strict loader:\n" + "\n".join(failures)
