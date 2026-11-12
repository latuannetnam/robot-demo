from collections import deque
import math
from pathlib import Path
import pytest

from robot_sim.arenas import ARENAS, MODELS_DIR, identify_arena_id, list_arenas, resolve_arena_model
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


def test_list_arenas_contains_all_four_maps():
    catalog = list_arenas()
    assert [item["id"] for item in catalog] == ["2wheel", "maze_a", "maze_b", "maze_c"]
    assert "maze" in catalog[1]["aliases"]
    for item in catalog:
        assert "model_path" not in item
        assert "spawn_xy" not in item
        assert "default_target_xy" not in item

    debug_catalog = list_arenas(include_debug=True)
    for item in debug_catalog:
        assert Path(item["model_path"]).parent == MODELS_DIR.resolve()
        assert len(item["spawn_xy"]) == 2
        assert len(item["default_target_xy"]) == 2


def _point_clears_boxes(x: float, y: float, boxes: list[tuple[float, float, float, float]], margin: float) -> bool:
    for cx, cy, hx, hy in boxes:
        dx = max(abs(x - cx) - hx, 0.0)
        dy = max(abs(y - cy) - hy, 0.0)
        if math.hypot(dx, dy) < margin:
            return False
    return True


def _has_wide_corridor_path(arena_id: str, min_corridor_width: float = 0.70) -> bool:
    spec = ARENAS[arena_id]
    margin = min_corridor_width / 2.0
    boxes: list[tuple[float, float, float, float]] = []
    for pos, size in spec.walls.values():
        boxes.append((pos[0], pos[1], size[0], size[1]))
    for pos, size, _rgba in spec.maze_walls.values():
        boxes.append((pos[0], pos[1], size[0], size[1]))

    step = 0.05
    n = int(round(4.0 / step)) + 1
    xs = [-2.0 + i * step for i in range(n)]

    def to_idx(val: float) -> int:
        return int(round((val + 2.0) / step))

    start = (to_idx(spec.spawn_xy[0]), to_idx(spec.spawn_xy[1]))
    goal = (to_idx(spec.default_target_xy[0]), to_idx(spec.default_target_xy[1]))
    assert _point_clears_boxes(xs[start[0]], xs[start[1]], boxes, margin)
    assert _point_clears_boxes(xs[goal[0]], xs[goal[1]], boxes, margin)

    visited = {start}
    queue = deque([start])
    while queue:
        ix, iy = queue.popleft()
        if (ix, iy) == goal:
            return True
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = ix + dx, iy + dy
            if 0 <= nx < n and 0 <= ny < n and (nx, ny) not in visited:
                if _point_clears_boxes(xs[nx], xs[ny], boxes, margin):
                    visited.add((nx, ny))
                    queue.append((nx, ny))
    return False


@pytest.mark.parametrize(
    "arena_id,final_turn_xy",
    [
        ("maze_a", (0.5, 1.5)),
        ("maze_b", (0.8, 1.6)),
        ("maze_c", (0.8, 1.6)),
    ],
)
def test_maze_maps_occlude_target_at_spawn_have_clear_corridor_and_reveal_at_final_turn(
    arena_id: str, final_turn_xy: tuple[float, float]
):
    spec = ARENAS[arena_id]
    assert _has_wide_corridor_path(arena_id, min_corridor_width=0.70)

    sim = RobotSimulation(resolve_arena_model(arena_id))
    try:
        # 1. From robot_spawn across full 360 deg sweep, green target is 100% occluded and gray walls trigger 0 false colors
        sim.data.qpos[0:2] = [spec.spawn_xy[0], spec.spawn_xy[1]]
        for step_idx in range(16):
            yaw = (2.0 * math.pi * step_idx) / 16.0
            sim.data.qpos[3:7] = [math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)]
            vis = sim.analyze_vision("front_cam")
            assert vis["target_visible"] is False, f"{arena_id} leaked target at spawn yaw_step={step_idx}: {vis}"
            assert vis["obstacles"] == [], f"{arena_id} gray walls triggered false obstacle mask: {vis}"

        # 2. At final turn facing +X (yaw=0), green target is clearly visible ahead
        sim.data.qpos[0:2] = [final_turn_xy[0], final_turn_xy[1]]
        sim.data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
        vis_final = sim.analyze_vision("front_cam")
        assert vis_final["target_visible"] is True
        assert vis_final["obstacles"] == []
    finally:
        sim.close()
