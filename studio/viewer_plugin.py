"""Viewer-side Dear ImGui Studio plugin for robot-demo (hot-reloaded on save)."""

from __future__ import annotations

import messages as user_messages
from mujoco.studio import messages
from mujoco.studio import plugin
from mujoco.studio.app import plugins as app_plugins

imgui = app_plugins.imgui

ARENA_IDS = (
    '2wheel',
    'maze_a',
    'maze_b',
    'maze_c',
    'dynamic_a',
    'dynamic_b',
)


class ViewerPlugin(plugin.ViewerPlugin):
  """Interactive control & Sim-to-Real telemetry panel for the 2-wheel robot."""

  mode: str = 'manual'
  left_rad_s: float = 0.0
  right_rad_s: float = 0.0
  auto_speed_rad_s: float = 7.0
  telemetry: user_messages.RobotTelemetry = user_messages.RobotTelemetry()
  window_open: bool = True

  @messages.handler
  def on_telemetry(self, snapshot: user_messages.RobotTelemetry) -> None:
    self.telemetry = snapshot

  def _send_drive(self) -> None:
    self.viewer.send_to_sim(
        user_messages.DriveCommand(
            mode=self.mode,
            left_rad_s=self.left_rad_s,
            right_rad_s=self.right_rad_s,
            auto_speed_rad_s=self.auto_speed_rad_s,
        )
    )

  @messages.handler
  def on_build_gui(self, event: messages.BuildGuiEvent) -> None:
    del event
    if not self.window_open:
      return
    expanded, self.window_open = imgui.Begin(
        'Robot 2-Wheel Control###Robot2WheelControlWindow', self.window_open
    )
    try:
      if not expanded:
        return
      t = self.telemetry

      # 1. Arena selection
      imgui.TextUnformatted(f'Active Arena: {t.arena_id}   (t = {t.sim_time:.2f} s)')
      for i, aid in enumerate(ARENA_IDS):
        if i % 3 != 0:
          imgui.SameLine()
        label = f'[{aid}]' if aid == t.arena_id else aid
        if imgui.Button(f'{label}##arena_{aid}'):
          self.viewer.send_to_sim(user_messages.ArenaSelectEvent(arena_id=aid))
      if imgui.Button('Reset Robot Spawn'):
        self.left_rad_s = 0.0
        self.right_rad_s = 0.0
        self._send_drive()
        self.viewer.send_to_sim(user_messages.ResetRobotEvent())

      imgui.Separator()

      # 2. Drive mode & wheel controls
      is_auto = self.mode == 'auto'
      auto_changed, new_auto = imgui.Checkbox('Autopilot (Sim-to-Real Rangefinders)', is_auto)
      if auto_changed:
        self.mode = 'auto' if new_auto else 'manual'
        if not new_auto:
          self.left_rad_s = 0.0
          self.right_rad_s = 0.0
        self._send_drive()

      if self.mode == 'auto':
        spd_changed, self.auto_speed_rad_s = imgui.SliderFloat(
            'Auto Speed (rad/s)', self.auto_speed_rad_s, 2.0, 14.0
        )
        if spd_changed:
          self._send_drive()
      else:
        if imgui.Button('Forward'):
          self.left_rad_s = 6.0
          self.right_rad_s = 6.0
          self._send_drive()
        imgui.SameLine()
        if imgui.Button('Turn Left'):
          self.left_rad_s = -4.0
          self.right_rad_s = 4.0
          self._send_drive()
        imgui.SameLine()
        if imgui.Button('Stop'):
          self.left_rad_s = 0.0
          self.right_rad_s = 0.0
          self._send_drive()
        imgui.SameLine()
        if imgui.Button('Turn Right'):
          self.left_rad_s = 4.0
          self.right_rad_s = -4.0
          self._send_drive()
        imgui.SameLine()
        if imgui.Button('Reverse'):
          self.left_rad_s = -5.0
          self.right_rad_s = -5.0
          self._send_drive()

        l_changed, self.left_rad_s = imgui.SliderFloat(
            'Left Wheel (rad/s)', self.left_rad_s, -15.0, 15.0
        )
        r_changed, self.right_rad_s = imgui.SliderFloat(
            'Right Wheel (rad/s)', self.right_rad_s, -15.0, 15.0
        )
        if l_changed or r_changed:
          self._send_drive()

      imgui.Separator()

      # 3. Live Sim-to-Real Telemetry
      imgui.TextUnformatted(
          f'Speed: {t.forward_speed:+.2f} m/s | Odom: {t.odom_dist_m:.2f} m | Gyro: {t.gyro_yaw_deg:+.1f} deg'
      )
      imgui.TextUnformatted(
          f'Wheels (rad/s): L={t.left_vel:+.2f} (cmd {t.left_cmd:+.1f}) | '
          f'R={t.right_vel:+.2f} (cmd {t.right_cmd:+.1f})'
      )
      contact_str = 'COLLISION!' if t.in_contact else 'clear'
      imgui.TextUnformatted(f'Hazard Contacts: {t.contact_count} ({contact_str})')
      imgui.TextUnformatted(
          f'RF Front (m): L={t.rf_left:.2f}  C={t.rf_center:.2f}  R={t.rf_right:.2f}'
      )
      imgui.TextUnformatted(
          f'RF Side  (m): L90={t.rf_left90:.2f} R90={t.rf_right90:.2f} | '
          f'L120={t.rf_left120:.2f} R120={t.rf_right120:.2f}'
      )
    finally:
      imgui.End()
