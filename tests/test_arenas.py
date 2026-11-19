from collections import deque
import math
from pathlib import Path
import numpy as np
import pytest

from robot_sim.arenas import MODELS_DIR, identify_arena_id, list_arenas, resolve_arena_model
from robot_sim.core import RobotSimulation
from robot_sim.integrity import verify_arena_integrity


@pytest.mark.parametrize(
    "key,expected_id,expected_file",
    [
        (None, "2wheel", "arena_2wheel.xml"),
        ("2wheel", "2wheel", "arena_2wheel.xml"),
        ("default", "2wheel", "arena_2wheel.xml"),
        ("basic", "2wheel", "arena_2wheel.xml"),
        ("maze", "maze_a", "arena_maze_a.xml"),
        ("maze_a", "maze_a", "arena_maze_a.xml"),
        ("maze_b", "maze_b", "arena_maze_b.xml"),
        ("maze_c", "maze_c", "arena_maze_c.xml"),
        ("dynamic", "dynamic_a", "arena_dynamic_a.xml"),
        ("dynamic_a", "dynamic_a", "arena_dynamic_a.xml"),
        ("dynamic_b", "dynamic_b", "arena_dynamic_b.xml"),
    ],
)
def test_resolve_arena_aliases_and_verify_integrity(key, expected_id, expected_file):
    path = resolve_arena_model(key)
    assert path.name == expected_file
    assert identify_arena_id(path) == expected_id
    res = verify_arena_integrity(path)
    assert res["valid"] is True
    assert res["arena"] == expected_id


def test_resolve_arena_rejects_unknown_name_and_outside_models_when_restricted(tmp_path: Path):
    with pytest.raises(ValueError):
        resolve_arena_model("nonexistent_arena")

    outside = tmp_path / "arena_2wheel.xml"
    outside.write_text("<mujoco/>", encoding="utf-8")
    with pytest.raises(ValueError):
        resolve_arena_model(str(outside), restrict_to_models_dir=True)

    # Unrestricted explicit file path is allowed and non-canonical filename maps to 'custom'
    custom_file = tmp_path / "my_custom_map.xml"
    custom_file.write_text("<mujoco/>", encoding="utf-8")
    assert resolve_arena_model(custom_file, restrict_to_models_dir=False) == custom_file.resolve()
    assert identify_arena_id(custom_file) == "custom"


def test_list_arenas_contains_all_six_maps():
    catalog = list_arenas()
    assert [item["id"] for item in catalog] == ["2wheel", "maze_a", "maze_b", "maze_c", "dynamic_a", "dynamic_b"]
    assert "maze" in catalog[1]["aliases"]
    assert "dynamic" in catalog[4]["aliases"]
    for item in catalog:
        assert "model_path" not in item
        assert "spawn_xy" not in item
        assert "default_target_xy" not in item

    debug_catalog = list_arenas(include_debug=True)
    for item in debug_catalog:
        assert Path(item["model_path"]).parent == MODELS_DIR.resolve()
        assert len(item["spawn_xy"]) == 2
        assert len(item["default_target_xy"]) == 2


@pytest.mark.parametrize(
    "arena_id,exp_spawn,exp_yaw,exp_target,pre_goal_xy,pre_goal_yaw",
    [
        ("maze_a", [-1.5, -1.5], 0.0, [1.5, 1.5], (0.5, 1.5), 0.0),
        ("maze_b", [0.0, 0.0], 0.0, [1.6, 1.6], (0.8, 1.6), 0.0),
        ("maze_c", [0.0, 0.0], -90.0, [1.6, 1.6], (0.8, 1.6), 0.0),
    ],
)
def test_maze_spawn_occludes_target_360_has_clear_corridor_and_reveals_at_final_turn(
    arena_id, exp_spawn, exp_yaw, exp_target, pre_goal_xy, pre_goal_yaw
):
    sim = RobotSimulation(model_path=arena_id)
    try:
        st = sim.get_status(include_vision=True)
        assert st["debug"]["arena"] == arena_id
        assert st["debug"]["robot_xy"] == pytest.approx(exp_spawn, abs=1e-3)
        assert st["debug"]["robot_yaw_deg"] == pytest.approx(exp_yaw, abs=0.5)
        assert st["debug"]["target_xy"] == pytest.approx(exp_target, abs=1e-3)

        # 1. Target must be 100% occluded across full 360° spin at spawn, with zero false obstacle colors
        for deg in range(0, 360, 30):
            rad = math.radians(deg)
            sim.data.qpos[0:2] = exp_spawn
            sim.data.qpos[3:7] = [math.cos(rad / 2.0), 0.0, 0.0, math.sin(rad / 2.0)]
            vis = sim.analyze_vision("front_cam")
            assert vis["target_visible"] is False, f"{arena_id} leaked target at yaw={deg}°"
            assert vis["obstacles"] == []

        # 2. Continuous 2D clearance check (robot clearance radius 0.35m => >= 0.70m corridor width)
        step = 0.10
        xs = np.round(np.arange(-1.8, 1.801, step), 3)
        ys = np.round(np.arange(-1.8, 1.801, step), 3)
        wall_boxes = []
        for gid in range(sim.model.ngeom):
            gname = sim.model.geom(gid).name
            if gname.startswith(("wall_", "maze_w")):
                cx, cy, _ = sim.model.geom_pos[gid]
                sx, sy, _ = sim.model.geom_size[gid]
                wall_boxes.append((float(cx), float(cy), float(sx), float(sy)))

        def is_clear(px: float, py: float, radius: float = 0.35) -> bool:
            for cx, cy, sx, sy in wall_boxes:
                dx = max(abs(px - cx) - sx, 0.0)
                dy = max(abs(py - cy) - sy, 0.0)
                if math.hypot(dx, dy) < radius - 1e-6:
                    return False
            return True

        start_idx = (int(round((exp_spawn[0] - xs[0]) / step)), int(round((exp_spawn[1] - ys[0]) / step)))
        goal_idx = (int(round((exp_target[0] - xs[0]) / step)), int(round((exp_target[1] - ys[0]) / step)))
        assert is_clear(xs[start_idx[0]], ys[start_idx[1]])
        assert is_clear(xs[goal_idx[0]], ys[goal_idx[1]])

        visited = {start_idx}
        q = deque([start_idx])
        while q:
            ix, iy = q.popleft()
            if (ix, iy) == goal_idx:
                break
            for nix, niy in ((ix + 1, iy), (ix - 1, iy), (ix, iy + 1), (ix, iy - 1)):
                if 0 <= nix < len(xs) and 0 <= niy < len(ys) and (nix, niy) not in visited:
                    if is_clear(float(xs[nix]), float(ys[niy])):
                        visited.add((nix, niy))
                        q.append((nix, niy))
        assert goal_idx in visited, f"{arena_id} lacks a >=0.70m wide continuous path from spawn to target"

        # 3. At the final turn cell facing East toward target, target becomes clearly visible
        rad_final = math.radians(pre_goal_yaw)
        sim.data.qpos[0:2] = list(pre_goal_xy)
        sim.data.qpos[3:7] = [math.cos(rad_final / 2.0), 0.0, 0.0, math.sin(rad_final / 2.0)]
        vis_final = sim.analyze_vision("front_cam")
        assert vis_final["target_visible"] is True

        # 4. sim.reset() restores the arena's robot_spawn pose and default target
        st_reset = sim.reset()
        assert st_reset["debug"]["arena"] == arena_id
        assert st_reset["debug"]["robot_xy"] == pytest.approx(exp_spawn, abs=1e-3)
        assert st_reset["debug"]["robot_yaw_deg"] == pytest.approx(exp_yaw, abs=0.5)
        assert st_reset["debug"]["target_xy"] == pytest.approx(exp_target, abs=1e-3)
    finally:
        sim.close()
