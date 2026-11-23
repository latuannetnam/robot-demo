from __future__ import annotations

import json
import math
from pathlib import Path
import time
from typing import Any
from urllib.parse import urlsplit

from robot_sim.core import PROJECT_ROOT, _require_finite

CARDINALS: tuple[str, ...] = ("E", "N", "W", "S")
_CARDINAL_TO_DEG: dict[str, float] = {"E": 0.0, "N": 90.0, "W": 180.0, "S": -90.0}
_OPPOSITE_CARDINAL: dict[str, str] = {"E": "W", "W": "E", "N": "S", "S": "N"}
_BRANCH_OFFSET_DEG: dict[str, float] = {"left": 90.0, "straight": 0.0, "right": -90.0}

NODE_MATCH_RADIUS_M: float = 0.42
CARDINAL_ALIGN_TOL_DEG: float = 15.0
DEFAULT_TOPO_STATE_PATH: Path = PROJECT_ROOT / "captures" / ".topo_state.json"


def _wrap180(deg: float) -> float:
    return ((deg + 180.0) % 360.0) - 180.0


def normalize_base_url(base_url: str) -> str:
    """Chuẩn hóa URL bridge để làm khóa phiên ổn định giữa CLI và MuJoCo Studio."""
    raw = str(base_url).strip()
    parsed = urlsplit(raw)
    if not parsed.scheme or not parsed.netloc:
        return raw.rstrip("/")
    scheme = parsed.scheme.lower()
    host = (parsed.hostname or "127.0.0.1").lower()
    port = parsed.port or (443 if scheme == "https" else 80)
    path = parsed.path.rstrip("/")
    return f"{scheme}://{host}:{port}{path}"


def yaw_to_cardinal(yaw_deg: float) -> str:
    """Lượng tử hóa góc hướng IMU (hệ quy chiếu lúc khởi hành) sang 4 hướng E/N/W/S."""
    _require_finite(yaw_deg=yaw_deg)
    idx = int(((float(yaw_deg) + 45.0) % 360.0) // 90.0)
    return CARDINALS[idx]


def cardinal_to_deg(cardinal: str) -> float:
    """Đổi tên hướng E/N/W/S sang góc mục tiêu độ trong hệ quy chiếu khởi hành."""
    if not isinstance(cardinal, str):
        raise ValueError(f"Hướng cardinal không hợp lệ: {cardinal!r}. Chỉ chấp nhận E, N, W, S.")
    key = cardinal.strip().upper()
    if key not in _CARDINAL_TO_DEG:
        raise ValueError(f"Hướng cardinal không hợp lệ: {cardinal!r}. Chỉ chấp nhận E, N, W, S.")
    return _CARDINAL_TO_DEG[key]


def opposite_cardinal(cardinal: str) -> str:
    """Trả hướng ngược lại 180° của E/N/W/S."""
    if not isinstance(cardinal, str):
        raise ValueError(f"Hướng cardinal không hợp lệ: {cardinal!r}. Chỉ chấp nhận E, N, W, S.")
    key = cardinal.strip().upper()
    if key not in _OPPOSITE_CARDINAL:
        raise ValueError(f"Hướng cardinal không hợp lệ: {cardinal!r}. Chỉ chấp nhận E, N, W, S.")
    return _OPPOSITE_CARDINAL[key]


def is_cardinal_aligned(yaw_deg: float, tol_deg: float = CARDINAL_ALIGN_TOL_DEG) -> bool:
    """Kiểm tra góc đầu xe có căn thẳng trong phạm vi dung sai so với trục 90° gần nhất hay không."""
    _require_finite(yaw_deg=yaw_deg, tol_deg=tol_deg)
    nearest_90 = round(float(yaw_deg) / 90.0) * 90.0
    return abs(_wrap180(float(yaw_deg) - nearest_90)) <= float(tol_deg)


def branches_to_cardinals(open_branches: list[str], yaw_deg: float) -> list[str]:
    """Quy đổi danh sách nhánh tương đối ['left', 'straight', 'right'] sang danh sách hướng E/N/W/S."""
    _require_finite(yaw_deg=yaw_deg)
    base_deg = round(float(yaw_deg) / 90.0) * 90.0
    out: list[str] = []
    for branch in open_branches:
        if branch in _BRANCH_OFFSET_DEG:
            card = yaw_to_cardinal(base_deg + _BRANCH_OFFSET_DEG[branch])
            if card not in out:
                out.append(card)
    return out


def _open_branches_from_rf(rf: dict[str, float]) -> list[str]:
    from robot_sim.skills import compute_open_branches

    return compute_open_branches(rf)


def _new_port(state: str) -> dict[str, Any]:
    return {
        "state": state,
        "to_node": None,
        "out_visits": 0,
        "in_visits": 0,
        "dist_m": None,
    }


def _load_store(state_path: Path) -> dict[str, Any]:
    if not state_path.is_file():
        return {"sessions": {}}
    try:
        parsed = json.loads(state_path.read_text(encoding="utf-8"))
        if isinstance(parsed, dict) and isinstance(parsed.get("sessions"), dict):
            return parsed
    except (OSError, ValueError, TypeError):
        pass
    return {"sessions": {}}


def _save_store_atomic(state_path: Path, store: dict[str, Any]) -> None:
    state_path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = state_path.with_name(f"{state_path.name}.tmp")
    payload = json.dumps(store, ensure_ascii=False, indent=2)
    tmp_path.write_text(payload, encoding="utf-8")
    last_err: OSError | None = None
    for attempt in range(3):
        try:
            tmp_path.replace(state_path)
            return
        except PermissionError as exc:
            last_err = exc
            if attempt < 2:
                time.sleep(0.015)
    if last_err is not None:
        raise last_err


def read_topo_studio_summary(
    base_url: str = "http://127.0.0.1:8765",
    state_path: Path | str = DEFAULT_TOPO_STATE_PATH,
) -> tuple[int, int, int]:
    """Đọc tóm tắt gọn (node_count, current_node_or_neg1, open_exits_count) an toàn cho MuJoCo Studio."""
    try:
        path = Path(state_path)
        if not path.is_file():
            return 0, -1, 0
        parsed = json.loads(path.read_text(encoding="utf-8"))
        sessions = parsed.get("sessions", {}) if isinstance(parsed, dict) else {}
        sess = sessions.get(normalize_base_url(base_url))
        if not isinstance(sess, dict):
            return 0, -1, 0
        nodes = sess.get("nodes", [])
        if not isinstance(nodes, list) or not nodes:
            return 0, -1, 0
        cur_node = sess.get("current_node")
        cur_id = int(cur_node) if isinstance(cur_node, int) and not isinstance(cur_node, bool) else -1
        open_exits = 0
        for node in nodes:
            exits = node.get("exits", {}) if isinstance(node, dict) else {}
            if isinstance(exits, dict):
                for info in exits.values():
                    if isinstance(info, dict) and info.get("state") == "open":
                        open_exits += 1
        return len(nodes), cur_id, open_exits
    except (OSError, ValueError, KeyError, TypeError):
        return 0, -1, 0


class TopologicalOdometer:
    """Bộ định vị đồ thị Odometry khách quan (100% Sim-to-Real, thuần đo đạc, không lập kế hoạch)."""

    def __init__(
        self,
        base_url: str = "http://127.0.0.1:8765",
        state_path: Path | str = DEFAULT_TOPO_STATE_PATH,
        *,
        auto_persist: bool = True,
    ) -> None:
        self.base_url = normalize_base_url(base_url)
        self.state_path = Path(state_path)
        self.auto_persist = bool(auto_persist)

        self.nodes: list[dict[str, Any]] = []
        self.current_node: int | None = None
        self.pending_leg: dict[str, Any] | None = None
        self.loop_closed: bool = False
        self.last_sim_time: float | None = None
        self.last_odom_dist_m: float | None = None
        self.last_gyro_yaw_deg: float = 0.0
        self.last_odom_xy: list[float] = [0.0, 0.0]

        if self.auto_persist:
            self._load_from_disk()

    @classmethod
    def has_active_session(
        cls,
        base_url: str,
        state_path: Path | str = DEFAULT_TOPO_STATE_PATH,
    ) -> bool:
        path = Path(state_path)
        if not path.is_file():
            return False
        store = _load_store(path)
        sess = store.get("sessions", {}).get(normalize_base_url(base_url))
        return isinstance(sess, dict) and isinstance(sess.get("nodes"), list)

    @classmethod
    def reset_session(
        cls,
        base_url: str,
        state_path: Path | str = DEFAULT_TOPO_STATE_PATH,
    ) -> None:
        path = Path(state_path)
        if not path.is_file():
            return
        store = _load_store(path)
        sessions = store.get("sessions", {})
        key = normalize_base_url(base_url)
        if key in sessions:
            del sessions[key]
            if sessions:
                _save_store_atomic(path, {"sessions": sessions})
            else:
                path.unlink(missing_ok=True)

    def _clear_memory(self) -> None:
        self.nodes = []
        self.current_node = None
        self.pending_leg = None
        self.loop_closed = False
        self.last_sim_time = None
        self.last_odom_dist_m = None
        self.last_gyro_yaw_deg = 0.0
        self.last_odom_xy = [0.0, 0.0]

    def _load_from_disk(self) -> None:
        store = _load_store(self.state_path)
        sess = store.get("sessions", {}).get(self.base_url)
        if not isinstance(sess, dict):
            return
        nodes = sess.get("nodes")
        if isinstance(nodes, list):
            self.nodes = nodes
        cur = sess.get("current_node")
        self.current_node = cur if isinstance(cur, int) and not isinstance(cur, bool) else None
        pleg = sess.get("pending_leg")
        self.pending_leg = pleg if isinstance(pleg, dict) else None
        self.loop_closed = bool(sess.get("loop_closed", False))
        lst = sess.get("last_sim_time")
        self.last_sim_time = float(lst) if isinstance(lst, (int, float)) else None
        lod = sess.get("last_odom_dist_m")
        self.last_odom_dist_m = float(lod) if isinstance(lod, (int, float)) else None
        lyaw = sess.get("last_gyro_yaw_deg")
        self.last_gyro_yaw_deg = float(lyaw) if isinstance(lyaw, (int, float)) else 0.0
        lxy = sess.get("last_odom_xy")
        if isinstance(lxy, list) and len(lxy) == 2:
            self.last_odom_xy = [float(lxy[0]), float(lxy[1])]

    def _persist(self) -> None:
        if not self.auto_persist:
            return
        store = _load_store(self.state_path)
        sessions = store.setdefault("sessions", {})
        sessions[self.base_url] = {
            "nodes": self.nodes,
            "current_node": self.current_node,
            "pending_leg": self.pending_leg,
            "loop_closed": self.loop_closed,
            "last_sim_time": self.last_sim_time,
            "last_odom_dist_m": self.last_odom_dist_m,
            "last_gyro_yaw_deg": self.last_gyro_yaw_deg,
            "last_odom_xy": self.last_odom_xy,
        }
        _save_store_atomic(self.state_path, store)

    @staticmethod
    def _extract_sensors(status: dict) -> tuple[float, float, float, float, float, dict[str, float]]:
        sens = status.get("sensors", {})
        sim_time = float(status.get("sim_time", 0.0))
        x = float(sens.get("odom_x_m", 0.0))
        y = float(sens.get("odom_y_m", 0.0))
        odom_dist = float(sens.get("odom_dist_m", 0.0))
        yaw_deg = float(sens.get("gyro_yaw_deg", 0.0))
        rf = dict(sens.get("rangefinders_m", {}))
        return sim_time, x, y, odom_dist, yaw_deg, rf

    def _check_auto_reset(self, sim_time: float, x: float, y: float, odom_dist: float, yaw_deg: float) -> bool:
        should_reset = False
        if self.last_sim_time is not None and sim_time < self.last_sim_time - 1e-3:
            should_reset = True
        elif (
            self.last_odom_dist_m is not None
            and (self.last_sim_time is None or sim_time <= self.last_sim_time + 1e-3)
            and odom_dist < self.last_odom_dist_m - 1e-3
        ):
            should_reset = True
        elif (
            self.last_odom_dist_m is not None
            and abs(odom_dist) < 0.03
            and math.hypot(x, y) < 0.03
            and self.last_odom_dist_m > 0.10
        ):
            should_reset = True
        elif (
            self.last_odom_dist_m is not None
            and abs(odom_dist) < 1e-3
            and abs(yaw_deg) < 0.5
            and (abs(self.last_odom_dist_m) > 1e-3 or abs(self.last_gyro_yaw_deg) > 2.0)
        ):
            should_reset = True

        if should_reset:
            self._clear_memory()
        return should_reset

    def _record_telemetry_cursor(self, sim_time: float, x: float, y: float, odom_dist: float, yaw_deg: float) -> None:
        self.last_sim_time = round(sim_time, 4)
        self.last_odom_dist_m = round(odom_dist, 4)
        self.last_gyro_yaw_deg = round(yaw_deg, 3)
        self.last_odom_xy = [round(x, 3), round(y, 3)]

    def nearest_existing_node(
        self,
        x_m: float,
        y_m: float,
        radius_m: float = NODE_MATCH_RADIUS_M,
    ) -> int | None:
        best_id: int | None = None
        best_dist = float(radius_m) + 1e-9
        for node in self.nodes:
            nx, ny = float(node["xy"][0]), float(node["xy"][1])
            dist = math.hypot(float(x_m) - nx, float(y_m) - ny)
            if dist <= float(radius_m) and dist < best_dist:
                best_dist = dist
                best_id = int(node["id"])
        return best_id

    def is_open_space_start(self) -> bool:
        return bool(self.nodes and self.nodes[0].get("kind") == "open_space")

    def _update_node_ports(self, node_id: int, rf: dict[str, float], yaw_deg: float) -> None:
        node = self.nodes[node_id]
        if node.get("kind") == "open_space":
            return
        if not is_cardinal_aligned(yaw_deg):
            return
        open_cards = set(branches_to_cardinals(_open_branches_from_rf(rf), yaw_deg))
        base_deg = round(yaw_deg / 90.0) * 90.0
        inspected_cards = {
            yaw_to_cardinal(base_deg + offset)
            for offset in _BRANCH_OFFSET_DEG.values()
        }
        exits: dict[str, dict[str, Any]] = node["exits"]
        front_card = yaw_to_cardinal(yaw_deg)
        rf_center = float(rf.get("center", 3.0))
        for card in inspected_cards:
            existing = exits.get(card)
            if card in open_cards:
                if existing is None:
                    exits[card] = _new_port("open")
                elif existing.get("state") == "uninspected":
                    existing["state"] = "open"
            else:
                if existing is not None and existing.get("state") == "uninspected":
                    del exits[card]
                elif (
                    existing is not None
                    and existing.get("state") == "open"
                    and card == front_card
                    and rf_center < 0.62
                ):
                    del exits[card]

    def _ensure_start_node(self, x: float, y: float, yaw_deg: float, rf: dict[str, float]) -> None:
        if self.nodes:
            return
        rf_l90 = float(rf.get("left90", 3.0))
        rf_r90 = float(rf.get("right90", 3.0))
        rf_c = float(rf.get("center", 3.0))
        is_open = rf_l90 > 0.75 and rf_r90 > 0.75 and rf_c > 0.85
        kind = "open_space" if is_open else "start"
        exits: dict[str, dict[str, Any]] = {}
        if kind == "start":
            rear_card = opposite_cardinal(yaw_to_cardinal(yaw_deg))
            exits[rear_card] = _new_port("uninspected")
        self.nodes.append(
            {
                "id": 0,
                "xy": [round(x, 3), round(y, 3)],
                "kind": kind,
                "exits": exits,
            }
        )
        self.current_node = 0
        self._update_node_ports(0, rf, yaw_deg)

    def _complete_pending_leg(self, dest_id: int, arrival_yaw_deg: float, *, dest_existed: bool) -> None:
        if self.pending_leg is None:
            self.loop_closed = False
            return
        u = int(self.pending_leg["from_node"])
        p_out = str(self.pending_leg["depart_port"])
        d = round(float(self.pending_leg.get("dist_m", 0.0)), 3)
        self.pending_leg = None

        if u == dest_id and d < 0.80:
            self.loop_closed = False
            return

        p_in = opposite_cardinal(yaw_to_cardinal(arrival_yaw_deg))
        exits_u = self.nodes[u]["exits"]
        exits_v = self.nodes[dest_id]["exits"]
        prev_u = exits_u.get(p_out)
        prev_v = exits_v.get(p_in)

        # Nếu hai đầu cổng đã cùng nối qua một nút trung gian đã biết (lướt thẳng qua ngã rẽ trung gian khi đi lùi),
        # giữ nguyên cấu trúc 2 chặng u <-> mid <-> dest_id thay vì ghi đè làm cô lập nút trung gian.
        if (
            isinstance(prev_u, dict)
            and prev_u.get("state") == "explored"
            and isinstance(prev_v, dict)
            and prev_v.get("state") == "explored"
            and prev_u.get("to_node") is not None
            and prev_u.get("to_node") == prev_v.get("to_node")
            and prev_u.get("to_node") not in (u, dest_id)
        ):
            mid = int(prev_u["to_node"])
            prev_u["out_visits"] = int(prev_u.get("out_visits", 0)) + 1
            prev_v["in_visits"] = int(prev_v.get("in_visits", 0)) + 1
            exits_mid = self.nodes[mid]["exits"]
            for info_m in exits_mid.values():
                if info_m.get("state") == "explored" and info_m.get("to_node") == u:
                    info_m["in_visits"] = int(info_m.get("in_visits", 0)) + 1
                    break
            for info_m in exits_mid.values():
                if info_m.get("state") == "explored" and info_m.get("to_node") == dest_id:
                    info_m["out_visits"] = int(info_m.get("out_visits", 0)) + 1
                    break
            self.loop_closed = False
            return

        already_connected = (
            isinstance(prev_u, dict)
            and prev_u.get("state") == "explored"
            and prev_u.get("to_node") == dest_id
            and isinstance(prev_v, dict)
            and prev_v.get("state") == "explored"
            and prev_v.get("to_node") == u
        )
        self.loop_closed = bool(dest_existed and (dest_id != u or d >= 0.80) and not already_connected)

        prev_out_u = int(prev_u.get("out_visits", 0)) if isinstance(prev_u, dict) else 0
        prev_in_u = int(prev_u.get("in_visits", 0)) if isinstance(prev_u, dict) else 0
        exits_u[p_out] = {
            "state": "explored",
            "to_node": dest_id,
            "out_visits": prev_out_u + 1,
            "in_visits": prev_in_u,
            "dist_m": d,
        }

        prev_out_v = int(prev_v.get("out_visits", 0)) if isinstance(prev_v, dict) else 0
        prev_in_v = int(prev_v.get("in_visits", 0)) if isinstance(prev_v, dict) else 0
        exits_v[p_in] = {
            "state": "explored",
            "to_node": u,
            "out_visits": prev_out_v,
            "in_visits": prev_in_v + 1,
            "dist_m": d,
        }

    def observe_status(self, status: dict) -> dict:
        sim_time, x, y, odom_dist, yaw_deg, rf = self._extract_sensors(status)
        self._check_auto_reset(sim_time, x, y, odom_dist, yaw_deg)
        self._record_telemetry_cursor(sim_time, x, y, odom_dist, yaw_deg)
        self.loop_closed = False

        if not self.nodes:
            self._ensure_start_node(x, y, yaw_deg, rf)
        else:
            matched = self.nearest_existing_node(x, y)
            self.current_node = matched
            if matched is not None:
                self._update_node_ports(matched, rf, yaw_deg)

        self._persist()
        return self.export_dict()

    def begin_leg(self, status: dict) -> None:
        sim_time, x, y, odom_dist, yaw_deg, rf = self._extract_sensors(status)
        self._check_auto_reset(sim_time, x, y, odom_dist, yaw_deg)
        self._record_telemetry_cursor(sim_time, x, y, odom_dist, yaw_deg)
        if not self.nodes:
            self._ensure_start_node(x, y, yaw_deg, rf)
        matched = self.nearest_existing_node(x, y)
        if matched is not None:
            self.current_node = matched
            self.pending_leg = {
                "from_node": matched,
                "depart_port": yaw_to_cardinal(yaw_deg),
                "dist_m": 0.0,
            }
        self._persist()

    def record_progress(self, moved_m: float, status: dict) -> dict:
        sim_time, x, y, odom_dist, yaw_deg, rf = self._extract_sensors(status)
        self._check_auto_reset(sim_time, x, y, odom_dist, yaw_deg)
        self._record_telemetry_cursor(sim_time, x, y, odom_dist, yaw_deg)
        if not self.nodes:
            self._ensure_start_node(x, y, yaw_deg, rf)
        step_m = max(0.0, float(moved_m))
        if self.pending_leg is None and self.current_node is not None and step_m >= 0.25:
            self.pending_leg = {
                "from_node": int(self.current_node),
                "depart_port": yaw_to_cardinal(yaw_deg),
                "dist_m": round(step_m, 4),
            }
        elif self.pending_leg is not None:
            self.pending_leg["dist_m"] = round(float(self.pending_leg.get("dist_m", 0.0)) + step_m, 4)

        matched = self.nearest_existing_node(x, y)
        if matched is not None and (
            self.pending_leg is None
            or matched != int(self.pending_leg["from_node"])
            or float(self.pending_leg.get("dist_m", 0.0)) >= 0.80
        ):
            self.current_node = matched
            if self.pending_leg is not None:
                self._complete_pending_leg(matched, yaw_deg, dest_existed=True)
            else:
                self.loop_closed = False
            self._update_node_ports(matched, rf, yaw_deg)
        elif matched is not None:
            self.current_node = matched
            self.loop_closed = False
        else:
            self.current_node = None
            self.loop_closed = False

        self._persist()
        return self.export_dict()

    def cancel_pending_leg(self, status: dict | None = None) -> dict:
        self.pending_leg = None
        self.loop_closed = False
        if status is not None:
            sim_time, x, y, odom_dist, yaw_deg, rf = self._extract_sensors(status)
            self._check_auto_reset(sim_time, x, y, odom_dist, yaw_deg)
            self._record_telemetry_cursor(sim_time, x, y, odom_dist, yaw_deg)
            if not self.nodes:
                self._ensure_start_node(x, y, yaw_deg, rf)
            else:
                matched = self.nearest_existing_node(x, y)
                self.current_node = matched
                if matched is not None:
                    self._update_node_ports(matched, rf, yaw_deg)
        self._persist()
        return self.export_dict()

    def finish_go(
        self,
        event: str,
        moved_m: float,
        status: dict,
        *,
        in_corridor: bool,
    ) -> dict:
        sim_time, x, y, odom_dist, yaw_deg, rf = self._extract_sensors(status)
        self._check_auto_reset(sim_time, x, y, odom_dist, yaw_deg)
        self._record_telemetry_cursor(sim_time, x, y, odom_dist, yaw_deg)
        if not self.nodes:
            self._ensure_start_node(x, y, yaw_deg, rf)

        step_m = max(0.0, float(moved_m))
        if self.pending_leg is not None:
            self.pending_leg["dist_m"] = round(float(self.pending_leg.get("dist_m", 0.0)) + step_m, 4)

        matched = self.nearest_existing_node(x, y)
        if matched is not None:
            self.current_node = matched
            self._complete_pending_leg(matched, yaw_deg, dest_existed=True)
            self._update_node_ports(matched, rf, yaw_deg)
        elif event in ("junction", "dead_end") and in_corridor:
            new_id = len(self.nodes)
            kind = "dead_end" if event == "dead_end" else "junction"
            self.nodes.append(
                {
                    "id": new_id,
                    "xy": [round(x, 3), round(y, 3)],
                    "kind": kind,
                    "exits": {},
                }
            )
            self.current_node = new_id
            self._complete_pending_leg(new_id, yaw_deg, dest_existed=False)
            if event != "dead_end":
                self._update_node_ports(new_id, rf, yaw_deg)
        else:
            self.current_node = None
            self.loop_closed = False

        self._persist()
        return self.export_dict()

    def export_dict(self) -> dict[str, Any]:
        return {
            "current_node": self.current_node,
            "heading_cardinal": yaw_to_cardinal(self.last_gyro_yaw_deg),
            "gyro_yaw_deg": self.last_gyro_yaw_deg,
            "odom_xy": list(self.last_odom_xy),
            "loop_closed": self.loop_closed,
            "pending_from_node": (
                int(self.pending_leg["from_node"]) if isinstance(self.pending_leg, dict) else None
            ),
            "nodes": [
                {
                    "id": int(n["id"]),
                    "xy": list(n["xy"]),
                    "kind": str(n["kind"]),
                    "exits": {port: dict(info) for port, info in n["exits"].items()},
                }
                for n in self.nodes
            ],
        }
