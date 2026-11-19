from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
MODELS_DIR: Path = PROJECT_ROOT / "models"
ROBOT_INCLUDE_FILENAME = "robot_2wheel.xml"

_BASIC_WALLS: dict[str, tuple[list[float], list[float]]] = {
    "wall_north": ([0.0, 2.025, 0.125], [2.05, 0.025, 0.125]),
    "wall_south": ([0.0, -2.025, 0.125], [2.05, 0.025, 0.125]),
    "wall_east": ([2.025, 0.0, 0.125], [0.025, 2.00, 0.125]),
    "wall_west": ([-2.025, 0.0, 0.125], [0.025, 2.00, 0.125]),
}

_MAZE_OUTER_WALLS: dict[str, tuple[list[float], list[float]]] = {
    "wall_north": ([0.0, 2.025, 0.14], [2.05, 0.025, 0.14]),
    "wall_south": ([0.0, -2.025, 0.14], [2.05, 0.025, 0.14]),
    "wall_east": ([2.025, 0.0, 0.14], [0.025, 2.00, 0.14]),
    "wall_west": ([-2.025, 0.0, 0.14], [0.025, 2.00, 0.14]),
}

_STONE_GRAY = [0.48, 0.52, 0.57, 1.0]
_ISLAND_GRAY = [0.58, 0.60, 0.64, 1.0]


@dataclass(frozen=True)
class MovingObstacleSpec:
    body_name: str
    joint_name: str
    geom_name: str
    geom_type: str
    base_pos: list[float]
    geom_size: list[float]
    rgba: list[float]
    slide_axis: list[float]
    slide_range: list[float]
    amplitude_m: float
    freq_hz: float
    phase_rad: float
    mass_kg: float = 8.0


@dataclass(frozen=True)
class ArenaSpec:
    arena_id: str
    aliases: tuple[str, ...]
    filename: str
    description: str
    floor_size: tuple[float, float, float]
    walls: dict[str, tuple[list[float], list[float]]]
    obstacles: dict[str, tuple[list[float], str, str, list[float], list[float]]]
    maze_walls: dict[str, tuple[list[float], list[float], list[float]]]
    spawn_xy: tuple[float, float]
    spawn_yaw_deg: float
    default_target_xy: tuple[float, float]
    moving_obstacles: dict[str, MovingObstacleSpec] = field(default_factory=dict)
    reference_path_m: float = 0.0


ARENAS: dict[str, ArenaSpec] = {
    "2wheel": ArenaSpec(
        arena_id="2wheel",
        aliases=("default", "basic"),
        filename="arena_2wheel.xml",
        description="Sân mở 4m x 4m cơ bản với 3 vật cản màu (đỏ, xanh dương, vàng).",
        floor_size=(2.0, 2.0, 0.05),
        walls=dict(_BASIC_WALLS),
        obstacles={
            "obs_red": ([0.70, 0.95, 0.12], "obs_red_geom", "box", [0.12, 0.12, 0.12], [0.90, 0.25, 0.20, 1.0]),
            "obs_blue": ([0.85, -0.75, 0.12], "obs_blue_geom", "box", [0.12, 0.12, 0.12], [0.20, 0.48, 0.92, 1.0]),
            "obs_yellow": (
                [-0.85, 0.75, 0.12],
                "obs_yellow_geom",
                "cylinder",
                [0.12, 0.12],
                [0.95, 0.78, 0.15, 1.0],
            ),
        },
        maze_walls={},
        spawn_xy=(0.0, 0.0),
        spawn_yaw_deg=0.0,
        default_target_xy=(1.4, 0.0),
        reference_path_m=0.96,
    ),
    "maze_a": ArenaSpec(
        arena_id="maze_a",
        aliases=("maze",),
        filename="arena_maze_a.xml",
        description="Mê cung 4x4 hành lang rộng 1.0m, 2 nhánh ngõ cụt, lộ trình chữ S xuyên chéo sân.",
        floor_size=(2.0, 2.0, 0.05),
        walls=dict(_MAZE_OUTER_WALLS),
        obstacles={},
        maze_walls={
            "maze_w01": ([-0.5, 1.0, 0.14], [0.52, 0.02, 0.14], list(_STONE_GRAY)),
            "maze_w02": ([1.5, 1.0, 0.14], [0.52, 0.02, 0.14], list(_STONE_GRAY)),
            "maze_w03": ([0.5, 0.0, 0.14], [0.52, 0.02, 0.14], list(_STONE_GRAY)),
            "maze_w04": ([-1.0, 0.0, 0.14], [0.02, 1.02, 0.14], list(_STONE_GRAY)),
            "maze_w05": ([0.0, 1.5, 0.14], [0.02, 0.52, 0.14], list(_STONE_GRAY)),
            "maze_w06": ([0.0, -1.0, 0.14], [0.02, 1.02, 0.14], list(_STONE_GRAY)),
            "maze_w07": ([1.0, -0.5, 0.14], [0.02, 0.52, 0.14], list(_STONE_GRAY)),
        },
        spawn_xy=(-1.5, -1.5),
        spawn_yaw_deg=0.0,
        default_target_xy=(1.5, 1.5),
        reference_path_m=4.22,
    ),
    "maze_b": ArenaSpec(
        arena_id="maze_b",
        aliases=(),
        filename="arena_maze_b.xml",
        description="Mê cung 5x5 hành lang 0.8m, 3 nhánh ngõ cụt và 9 khúc cua vuông góc.",
        floor_size=(2.0, 2.0, 0.05),
        walls=dict(_MAZE_OUTER_WALLS),
        obstacles={},
        maze_walls={
            "maze_w01": ([-0.8, 1.2, 0.14], [0.42, 0.02, 0.14], list(_STONE_GRAY)),
            "maze_w02": ([1.6, 1.2, 0.14], [0.42, 0.02, 0.14], list(_STONE_GRAY)),
            "maze_w03": ([-1.2, 0.4, 0.14], [0.82, 0.02, 0.14], list(_STONE_GRAY)),
            "maze_w04": ([0.0, -0.4, 0.14], [0.42, 0.02, 0.14], list(_STONE_GRAY)),
            "maze_w05": ([1.6, -0.4, 0.14], [0.42, 0.02, 0.14], list(_STONE_GRAY)),
            "maze_w06": ([-0.8, -1.2, 0.14], [0.42, 0.02, 0.14], list(_STONE_GRAY)),
            "maze_w07": ([0.8, -1.2, 0.14], [0.42, 0.02, 0.14], list(_STONE_GRAY)),
            "maze_w08": ([-1.2, -0.8, 0.14], [0.02, 0.42, 0.14], list(_STONE_GRAY)),
            "maze_w09": ([-0.4, 0.8, 0.14], [0.02, 0.42, 0.14], list(_STONE_GRAY)),
            "maze_w10": ([-0.4, -1.6, 0.14], [0.02, 0.42, 0.14], list(_STONE_GRAY)),
            "maze_w11": ([0.4, 0.4, 0.14], [0.02, 1.62, 0.14], list(_STONE_GRAY)),
            "maze_w12": ([1.2, 0.8, 0.14], [0.02, 0.42, 0.14], list(_STONE_GRAY)),
        },
        spawn_xy=(0.0, 0.0),
        spawn_yaw_deg=0.0,
        default_target_xy=(1.6, 1.6),
        reference_path_m=6.50,
    ),
    "maze_c": ArenaSpec(
        arena_id="maze_c",
        aliases=(),
        filename="arena_maze_c.xml",
        description="Mê cung 5x5 hành lang 0.8m với đảo tường chữ U tạo vòng lặp ở tâm và 3 nhánh cụt.",
        floor_size=(2.0, 2.0, 0.05),
        walls=dict(_MAZE_OUTER_WALLS),
        obstacles={},
        maze_walls={
            "maze_w01": ([-0.4, 1.2, 0.14], [0.82, 0.02, 0.14], list(_STONE_GRAY)),
            "maze_w02": ([1.6, 1.2, 0.14], [0.42, 0.02, 0.14], list(_STONE_GRAY)),
            "maze_w03": ([0.0, 0.4, 0.14], [0.42, 0.02, 0.14], list(_ISLAND_GRAY)),
            "maze_w04": ([-0.4, -1.2, 0.14], [0.82, 0.02, 0.14], list(_STONE_GRAY)),
            "maze_w05": ([-1.2, 0.8, 0.14], [0.02, 0.42, 0.14], list(_STONE_GRAY)),
            "maze_w06": ([-1.2, -0.8, 0.14], [0.02, 0.42, 0.14], list(_STONE_GRAY)),
            "maze_w07": ([-0.4, 0.0, 0.14], [0.02, 0.42, 0.14], list(_ISLAND_GRAY)),
            "maze_w08": ([0.4, 1.6, 0.14], [0.02, 0.42, 0.14], list(_STONE_GRAY)),
            "maze_w09": ([0.4, 0.0, 0.14], [0.02, 0.42, 0.14], list(_ISLAND_GRAY)),
            "maze_w10": ([0.4, -1.6, 0.14], [0.02, 0.42, 0.14], list(_STONE_GRAY)),
            "maze_w11": ([1.2, 0.0, 0.14], [0.02, 1.22, 0.14], list(_STONE_GRAY)),
        },
        spawn_xy=(0.0, 0.0),
        spawn_yaw_deg=-90.0,
        default_target_xy=(1.6, 1.6),
        reference_path_m=3.39,
    ),
    "dynamic_a": ArenaSpec(
        arena_id="dynamic_a",
        aliases=("dynamic",),
        filename="arena_dynamic_a.xml",
        description="Sân mở 4m x 4m với 3 làn vật cản màu tuần tra cắt ngang khác tốc độ và pha.",
        floor_size=(2.0, 2.0, 0.05),
        walls=dict(_MAZE_OUTER_WALLS),
        obstacles={},
        maze_walls={},
        spawn_xy=(-1.65, 0.0),
        spawn_yaw_deg=0.0,
        default_target_xy=(1.66, 0.0),
        reference_path_m=2.88,
        moving_obstacles={
            "obs_red": MovingObstacleSpec(
                "obs_red",
                "obs_red_slide",
                "obs_red_geom",
                "cylinder",
                [-0.92, 0.0, 0.12],
                [0.11, 0.12],
                [0.90, 0.25, 0.20, 1.0],
                [0.0, 1.0, 0.0],
                [-1.35, 1.35],
                0.95,
                0.062,
                0.35,
                8.0,
            ),
            "obs_blue": MovingObstacleSpec(
                "obs_blue",
                "obs_blue_slide",
                "obs_blue_geom",
                "box",
                [0.0, 0.0, 0.12],
                [0.11, 0.11, 0.12],
                [0.20, 0.48, 0.92, 1.0],
                [0.0, 1.0, 0.0],
                [-1.35, 1.35],
                1.00,
                0.072,
                2.20,
                8.0,
            ),
            "obs_yellow": MovingObstacleSpec(
                "obs_yellow",
                "obs_yellow_slide",
                "obs_yellow_geom",
                "cylinder",
                [0.88, 0.0, 0.12],
                [0.11, 0.12],
                [0.95, 0.78, 0.15, 1.0],
                [0.0, 1.0, 0.0],
                [-1.35, 1.35],
                0.95,
                0.056,
                -1.10,
                8.0,
            ),
        },
    ),
    "dynamic_b": ArenaSpec(
        arena_id="dynamic_b",
        aliases=(),
        filename="arena_dynamic_b.xml",
        description="Hành lang chữ S rộng 0.9m với 3 cổng nút cổ chai có vật cản tuần tra đóng/mở theo chu kỳ.",
        floor_size=(2.0, 2.0, 0.05),
        walls=dict(_MAZE_OUTER_WALLS),
        obstacles={},
        maze_walls={
            "maze_w01": ([-1.10, 0.45, 0.14], [0.90, 0.02, 0.14], list(_STONE_GRAY)),
            "maze_w02": ([-1.37, -0.45, 0.14], [0.63, 0.02, 0.14], list(_STONE_GRAY)),
            "maze_w03": ([0.17, -0.45, 0.14], [0.53, 0.02, 0.14], list(_STONE_GRAY)),
            "maze_w04": ([-0.55, -0.78, 0.14], [0.24, 0.02, 0.14], list(_STONE_GRAY)),
            "maze_w05": ([0.68, 0.04, 0.14], [0.02, 0.51, 0.14], list(_STONE_GRAY)),
            "maze_w06": ([-0.22, 1.18, 0.14], [0.02, 0.29, 0.14], list(_STONE_GRAY)),
            "maze_w07": ([-0.56, 0.67, 0.14], [0.02, 0.24, 0.14], list(_STONE_GRAY)),
            "maze_w08": ([0.88, 1.45, 0.14], [1.12, 0.02, 0.14], list(_STONE_GRAY)),
            "maze_w09": ([1.53, 0.55, 0.14], [0.47, 0.02, 0.14], list(_STONE_GRAY)),
            "maze_w10": ([0.88, 0.22, 0.14], [0.24, 0.02, 0.14], list(_STONE_GRAY)),
        },
        spawn_xy=(-1.55, 0.0),
        spawn_yaw_deg=0.0,
        default_target_xy=(1.66, 1.00),
        reference_path_m=3.16,
        moving_obstacles={
            "obs_red": MovingObstacleSpec(
                "obs_red",
                "obs_red_slide",
                "obs_red_geom",
                "cylinder",
                [-0.55, -0.20, 0.12],
                [0.12, 0.12],
                [0.90, 0.25, 0.20, 1.0],
                [0.0, 1.0, 0.0],
                [-0.50, 0.50],
                0.40,
                0.085,
                0.80,
                8.0,
            ),
            "obs_blue": MovingObstacleSpec(
                "obs_blue",
                "obs_blue_slide",
                "obs_blue_geom",
                "box",
                [-0.02, 0.67, 0.12],
                [0.11, 0.11, 0.12],
                [0.20, 0.48, 0.92, 1.0],
                [1.0, 0.0, 0.0],
                [-0.45, 0.45],
                0.37,
                0.075,
                1.90,
                8.0,
            ),
            "obs_yellow": MovingObstacleSpec(
                "obs_yellow",
                "obs_yellow_slide",
                "obs_yellow_geom",
                "cylinder",
                [0.88, 0.80, 0.12],
                [0.11, 0.12],
                [0.95, 0.78, 0.15, 1.0],
                [0.0, 1.0, 0.0],
                [-0.50, 0.50],
                0.40,
                0.090,
                -0.80,
                8.0,
            ),
        },
    ),
}

MODEL_NAME_TO_ARENA_ID: dict[str, str] = {
    f"arena_{arena_id}": arena_id for arena_id in ARENAS
}

_ALIAS_MAP: dict[str, ArenaSpec] = {}
for _spec in ARENAS.values():
    _ALIAS_MAP[_spec.arena_id] = _spec
    for _alias in _spec.aliases:
        _ALIAS_MAP[_alias] = _spec


def resolve_arena_model(
    arena_or_path: str | Path | None = None,
    *,
    restrict_to_models_dir: bool = False,
) -> Path:
    """Giải phân giải tên ngắn địa hình hoặc đường dẫn file mô hình XML."""
    models_root = MODELS_DIR.resolve()
    if arena_or_path is None:
        return (models_root / ARENAS["2wheel"].filename).resolve()

    if isinstance(arena_or_path, bool) or not isinstance(arena_or_path, (str, Path)):
        raise ValueError(f"Tham số địa hình/mô hình không hợp lệ: {arena_or_path!r}.")

    if isinstance(arena_or_path, str):
        key = arena_or_path.strip()
        if not key:
            raise ValueError("Tên địa hình hoặc đường dẫn mô hình không được để trống.")
        matched = _ALIAS_MAP.get(key.lower())
        if matched is not None:
            return (models_root / matched.filename).resolve()
        candidate = Path(key)
    else:
        candidate = arena_or_path

    if not candidate.is_absolute() and candidate.parent == Path(".") and (models_root / candidate.name).is_file():
        resolved = (models_root / candidate.name).resolve()
    else:
        resolved = candidate.resolve()

    if restrict_to_models_dir and resolved.parent != models_root:
        raise ValueError(
            f"Chỉ cho phép chọn địa hình chuẩn hoặc file XML nằm trực tiếp trong '{models_root}' "
            f"(nhận được: '{arena_or_path}')."
        )

    if not resolved.is_file():
        valid_names = ", ".join([*ARENAS.keys(), "maze", "dynamic", "default", "basic"])
        raise ValueError(
            f"Không tìm thấy địa hình hoặc file mô hình XML '{arena_or_path}'. "
            f"Các tên địa hình hợp lệ: {valid_names}."
        )

    return resolved


def identify_arena_id(model_path: Path | str) -> str:
    """Nhận diện mã địa hình chuẩn từ tên file XML (hoặc trả về 'custom')."""
    name = Path(model_path).name
    for spec in ARENAS.values():
        if spec.filename == name:
            return spec.arena_id
    return "custom"


def list_arenas(include_debug: bool = False) -> list[dict]:
    """Trả danh bạ các bản đồ địa hình chuẩn."""
    models_root = MODELS_DIR.resolve()
    catalog: list[dict] = []
    for spec in ARENAS.values():
        entry: dict = {
            "id": spec.arena_id,
            "aliases": list(spec.aliases),
            "filename": spec.filename,
            "description": spec.description,
        }
        if include_debug:
            entry["model_path"] = str((models_root / spec.filename).resolve())
            entry["spawn_xy"] = list(spec.spawn_xy)
            entry["default_target_xy"] = list(spec.default_target_xy)
        catalog.append(entry)
    return catalog
