from pathlib import Path
import pytest
from robot_sim.core import DEFAULT_MODEL_PATH, RobotSimulation
from robot_sim.integrity import verify_arena_integrity


def test_default_arena_passes_integrity():
    res = verify_arena_integrity(DEFAULT_MODEL_PATH)
    assert res["valid"] is True


@pytest.mark.parametrize(
    "tamper_src,tamper_dst",
    [
        ('pos="-0.85 0.75 0.12"', 'pos="-1.50 0.75 0.12"'),  # moved obs_yellow
        ('size="0.12 0.12"', 'size="0.02 0.02"'),            # shrunk obs_yellow
        ('size="0.10 0.12"', 'size="0.40 0.12"'),            # enlarged green target
        ('pos="0 2.025 0.125"', 'pos="0 3.025 0.125"'),      # moved wall_north
        ('pos="0.12 0 0.09"', 'pos="0.12 0 1.50"'),          # illegal periscope front_cam height
    ],
)
def test_verify_arena_integrity_rejects_arena_or_periscope_tampering(
    tmp_path: Path, tamper_src: str, tamper_dst: str
):
    xml_text = DEFAULT_MODEL_PATH.read_text(encoding="utf-8")
    assert tamper_src in xml_text
    bad_xml = tmp_path / "tampered.xml"
    bad_xml.write_text(xml_text.replace(tamper_src, tamper_dst, 1), encoding="utf-8")
    with pytest.raises(ValueError):
        verify_arena_integrity(bad_xml)


def test_reload_rejects_tampered_or_corrupt_xml_without_breaking_sim(tmp_path: Path):
    sim = RobotSimulation()
    try:
        sim.reset(target_xy=(-1.4, 1.15))
        corrupt = tmp_path / "corrupt.xml"
        corrupt.write_text("<mujoco><broken></mujoco>", encoding="utf-8")
        with pytest.raises(ValueError):
            sim.reload(model_path=corrupt)

        # Simulation remains intact with previous target_xy preserved
        st = sim.get_status()
        assert st["debug"]["target_xy"] == [-1.4, 1.15]

        # Valid reload preserves current target_xy by default
        reloaded = sim.reload(preserve_target=True)
        assert reloaded["debug"]["target_xy"] == [-1.4, 1.15]
    finally:
        sim.close()


def test_verify_arena_integrity_rejects_oversized_robot_geom_extents(tmp_path: Path):
    xml_text = DEFAULT_MODEL_PATH.read_text(encoding="utf-8")
    # Origin (0, 0, 0.06) is inside envelope, but box half-extents (1.5m x 1.5m) span the arena
    oversized = tmp_path / "oversized_geom.xml"
    oversized.write_text(
        xml_text.replace('size="0.12 0.10 0.03"', 'size="1.50 1.50 0.03"', 1),
        encoding="utf-8",
    )
    with pytest.raises(ValueError):
        verify_arena_integrity(oversized)


def test_verify_arena_integrity_rejects_unauthorized_worldbody_elements(tmp_path: Path):
    xml_text = DEFAULT_MODEL_PATH.read_text(encoding="utf-8")
    injected_body = (
        '<body name="guide_rail" pos="0.5 0 0.05">'
        '<geom name="guide_geom" type="box" size="0.5 0.02 0.05"/>'
        "</body>\n  </worldbody>"
    )
    injected_xml = tmp_path / "injected_worldbody.xml"
    injected_xml.write_text(xml_text.replace("</worldbody>", injected_body, 1), encoding="utf-8")
    with pytest.raises(ValueError):
        verify_arena_integrity(injected_xml)

