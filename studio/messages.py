"""Shared messages between the robot-demo simulation plugin and Studio UI."""

from __future__ import annotations

import dataclasses
from mujoco.studio import messages


@dataclasses.dataclass(frozen=True)
class ArenaSelectEvent(messages.Event):
  """UI -> sim: switch to one of the 6 robot-demo arenas."""

  arena_id: str = '2wheel'


@dataclasses.dataclass(frozen=True)
class ResetRobotEvent(messages.Event):
  """UI -> sim: reset robot to the arena spawn pose and clear counters."""


@dataclasses.dataclass(frozen=True)
class DriveCommand(messages.Snapshot):
  """UI -> sim: manual wheel commands or reactive Sim-to-Real autopilot."""

  mode: str = 'manual'  # 'manual' | 'auto'
  left_rad_s: float = 0.0
  right_rad_s: float = 0.0
  auto_speed_rad_s: float = 7.0


@dataclasses.dataclass(frozen=True)
class RobotTelemetry(messages.Snapshot):
  """sim -> UI: live robot sensors, odometry, and contact diagnostics."""

  arena_id: str = '2wheel'
  sim_time: float = 0.0
  mode: str = 'manual'
  left_cmd: float = 0.0
  right_cmd: float = 0.0
  left_vel: float = 0.0
  right_vel: float = 0.0
  forward_speed: float = 0.0
  gyro_yaw_deg: float = 0.0
  odom_dist_m: float = 0.0
  rf_left120: float = 3.0
  rf_left90: float = 3.0
  rf_left: float = 3.0
  rf_center: float = 3.0
  rf_right: float = 3.0
  rf_right90: float = 3.0
  rf_right120: float = 3.0
  contact_count: int = 0
  in_contact: bool = False
  topo_node_count: int = 0
  topo_current_node: int = -1
  topo_open_exits: int = 0
