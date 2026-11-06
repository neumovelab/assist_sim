"""The commented "to spec" lines in the device configs build the hardware ratings.

Each powered device ships permissive actuator values, with the specification-based values
next to them as comments (``# forcerange: [-25, 0]  # to spec: ...``). Nothing would notice
if a commented line rotted, so this test uncomments them the way a user would, builds the
device, and checks the full-command output against the specification.
"""

from __future__ import annotations

import re
import shutil
from pathlib import Path

import mujoco as mj
import pytest

from assist_sim.combine import ModelCombiner
from assist_sim.config import DeviceConfig
from assist_sim.registry import DEVICE_CONFIGS

from .conftest import needs_myo_sim

# Full-command output (most negative, most positive) per actuator once the "to spec" lines
# are in use. N*m for the joint motors, N for the H-MEDI cables. Where a device lists a
# continuous and an instantaneous rating, the test takes the instantaneous line.
SPEC = {
    "DephyExoBoot_L1": {"Exo_R": (-25, 0), "Exo_L": (-25, 0)},
    "Humotech_L1": {"Exo_R": (-100, 50), "Exo_L": (-100, 50)},
    "OpenExo_L1": {"Exo_R": (-28, 0), "Exo_L": (-28, 0)},
    "Hippo_L1": {"Exo_R": (-48, 48), "Exo_L": (-48, 48)},
    "NEUankle_L1": {"neuankle_ankle_torque_actuator": (-122, 122)},
    "OpenSourceLeg_A_L1": {"osl_ankle_torque_actuator": (-145.25, 145.25)},
    # The knee's gain and control range give 142.3 N*m, just under the 145.25 rating.
    "OpenSourceLeg_KA_L1": {
        "osl_knee_torque_actuator": (-49.4 * 2.88, 49.4 * 2.88),
        "osl_ankle_torque_actuator": (-145.25, 145.25),
    },
    "HMEDI_L1": {"HMEDI_L1_Exo_R": (-220, 0), "HMEDI_L1_Exo_L": (-220, 0)},
}

_YAML_LINE = re.compile(r"^(\s*)# (\w+): (.*?)\s+# to spec(.*)$")
_XML_ALT = re.compile(r"(<general [^>]*?/>)(\s*)<!-- to spec: (<general .*?/>) -->", re.S)


def _use_yaml_spec(text: str) -> str:
    out: list[str] = []
    for line in text.split("\n"):
        m = _YAML_LINE.match(line)
        if not m:
            out.append(line)
            continue
        indent, key, value, rest = m.groups()
        if rest.startswith(", continuous"):
            continue  # one forcerange per actuator: keep the instantaneous line
        if key in ("gainprm", "ctrlrange"):
            # The line it replaces is the last one with the same key.
            j = max(i for i, prev in enumerate(out) if prev.strip().startswith(key + ":"))
            out[j] = indent + "# " + out[j].strip()
        out.append(f"{indent}{key}: {value}")
    return "\n".join(out)


def _full_command_output(model: mj.MjModel, aid: int) -> tuple[float, float]:
    # Actuator space (before gear), where forcerange applies; every actuator here has gear 1.
    assert model.actuator_gear[aid, 0] == 1
    gain = model.actuator_gainprm[aid, 0]
    lo, hi = sorted(gain * c for c in model.actuator_ctrlrange[aid])
    if model.actuator_forcelimited[aid]:
        flo, fhi = model.actuator_forcerange[aid]
        lo, hi = max(lo, flo), min(hi, fhi)
    return lo, hi


@needs_myo_sim
@pytest.mark.parametrize("device_key", list(SPEC))
def test_to_spec_lines_build_the_spec(device_key, tmp_path):
    from assist_sim.registry import _resolve_msk

    src = Path(DEVICE_CONFIGS[device_key])
    staged = tmp_path / src.parent.name
    shutil.copytree(src.parent, staged)
    config_path = staged / src.name
    config_text = config_path.read_text(encoding="utf-8")
    assert "to spec" in config_text or device_key == "HMEDI_L1", f"{src.name} has no 'to spec' lines"
    config_path.write_text(_use_yaml_spec(config_text), encoding="utf-8")

    config = DeviceConfig.from_yaml(str(config_path))
    model_xml = Path(config.model_xml_path)
    xml_text = model_xml.read_text(encoding="utf-8")
    if "to spec:" in xml_text:
        xml_text, n = _XML_ALT.subn(lambda m: m.group(3) + m.group(2), xml_text)
        assert n > 0
        model_xml.write_text(xml_text, encoding="utf-8")

    model, _ = ModelCombiner().combine(_resolve_msk("myolegs22"), config, msk_key="myolegs22")
    for name, (lo, hi) in SPEC[device_key].items():
        got = _full_command_output(model, model.actuator(name).id)
        assert got == pytest.approx((lo, hi)), f"{device_key} {name}: {got} at full command, spec {(lo, hi)}"
