from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_workspace_skill_and_agents_rules_follow_antigravity_conventions():
    skill_md = ROOT / ".agents" / "skills" / "robot-multi-agent-rd" / "SKILL.md"
    driver_ref = ROOT / ".agents" / "skills" / "robot-multi-agent-rd" / "references" / "driver-prompt.md"
    engineer_ref = ROOT / ".agents" / "skills" / "robot-multi-agent-rd" / "references" / "engineer-prompt.md"
    agents_md = ROOT / "AGENTS.md"

    assert skill_md.is_file()
    text = skill_md.read_text(encoding="utf-8")
    assert text.startswith("---\n")
    frontmatter = text.split("---\n", 2)[1]
    assert "name: robot-multi-agent-rd" in frontmatter
    assert "description:" in frontmatter
    assert "telemetry record" in text and "telemetry stop" in text
    assert "3" in text  # max 3 R&D rounds

    driver_text = driver_ref.read_text(encoding="utf-8")
    engineer_text = engineer_ref.read_text(encoding="utf-8")
    assert driver_ref.is_file() and "--debug" in driver_text
    assert "arenas" in driver_text  # explicitly forbidden for blind driver
    assert engineer_ref.is_file() and "pytest" in engineer_text
    assert "models/robot_2wheel.xml" in engineer_text
    assert "--arena" in engineer_text

    assert "models/robot_2wheel.xml" in text
    assert "--arena" in text

    assert agents_md.is_file()
    agents_text = agents_md.read_text(encoding="utf-8")
    assert not agents_text.startswith("---\n")  # standalone AGENTS.md must NOT use YAML frontmatter
    assert len(agents_text.encode("utf-8")) < 4000
    assert ".venv\\Scripts\\python.exe" in agents_text
    assert "models/robot_2wheel.xml" in agents_text
    assert "--arena" in agents_text


def test_agent_brain_mode_docs_and_driver_prompt():
    skill_md = ROOT / ".agents" / "skills" / "robot-multi-agent-rd" / "SKILL.md"
    driver_brain_ref = ROOT / ".agents" / "skills" / "robot-multi-agent-rd" / "references" / "driver-brain-prompt.md"
    engineer_ref = ROOT / ".agents" / "skills" / "robot-multi-agent-rd" / "references" / "engineer-prompt.md"

    assert driver_brain_ref.is_file(), "driver-brain-prompt.md does not exist"
    brain_text = driver_brain_ref.read_text(encoding="utf-8")
    for kw in (
        ".venv\\Scripts\\python.exe robot_ctl.py",
        "advance",
        "cross-gate",
        "dock",
        "scan --no-recover-occlusion",
        "camera --name front_cam",
        "--debug",
        "arenas",
        "explore",
        "approach",
        "overview_cam",
        "Node",
        "odom_x_m",
        "odom_y_m",
        "gyro_yaw_deg",
        "wrap180(target_cardinal_deg - gyro_yaw_deg)",
        "static_obstacle_ahead",
        "redundant_revisit_count == 0",
        "Parent Stack",
    ):
        assert kw in brain_text

    skill_text = skill_md.read_text(encoding="utf-8")
    engineer_text = engineer_ref.read_text(encoding="utf-8")
    for kw in ("Mode A", "auto-skill", "Mode B", "agent-brain", "driver-brain-prompt.md", "redundant_revisit_count"):
        assert kw in skill_text
    for kw in ("Mode B", "agent-brain", "driver-brain-prompt.md", "models/robot_2wheel.xml", "robot_sim/skills.py"):
        assert kw in engineer_text


def test_hybrid_brain_mode_c_docs_driver_prompt_and_studio_messages():
    skill_md = ROOT / ".agents" / "skills" / "robot-multi-agent-rd" / "SKILL.md"
    hybrid_ref = ROOT / ".agents" / "skills" / "robot-multi-agent-rd" / "references" / "driver-hybrid-prompt.md"
    engineer_ref = ROOT / ".agents" / "skills" / "robot-multi-agent-rd" / "references" / "engineer-prompt.md"
    studio_msgs = (ROOT / "studio" / "messages.py").read_text(encoding="utf-8")
    studio_sim = (ROOT / "studio" / "sim_plugin.py").read_text(encoding="utf-8")
    studio_viewer = (ROOT / "studio" / "viewer_plugin.py").read_text(encoding="utf-8")

    assert hybrid_ref.is_file(), "driver-hybrid-prompt.md does not exist"
    hybrid_text = hybrid_ref.read_text(encoding="utf-8")
    for kw in (
        ".venv\\Scripts\\python.exe robot_ctl.py",
        "topo show",
        "go",
        "cross-gate",
        "dock",
        "scan --no-recover-occlusion",
        "camera --name front_cam",
        "--debug",
        "arenas",
        "topo reset",
        "explore",
        "approach",
        "overview_cam",
        "out_visits == 0",
        "redundant_revisit_count == 0",
        "loop_closed",
        "static_obstacle_ahead",
        "event == \"max_dist\"",
    ):
        assert kw in hybrid_text

    skill_text = skill_md.read_text(encoding="utf-8")
    engineer_text = engineer_ref.read_text(encoding="utf-8")
    for kw in (
        "Mode A",
        "Mode B",
        "Mode C",
        "hybrid-brain",
        "robot-driver-hybrid",
        "driver-hybrid-prompt.md",
        "--max-duration 480",
    ):
        assert kw in skill_text
    for kw in ("Mode C", "hybrid-brain", "driver-hybrid-prompt.md", "topo.py", "go_to_decision"):
        assert kw in engineer_text

    assert "topo_node_count: int = 0" in studio_msgs
    assert "topo_current_node: int = -1" in studio_msgs
    assert "topo_open_exits: int = 0" in studio_msgs
    assert "topo_goal_node: int = -1" in studio_msgs
    assert "keep_topo: bool = False" in studio_msgs
    assert "read_topo_studio_details" in studio_sim
    assert "Topo Graph:" in studio_viewer
    assert "Reset (Keep Topo)" in studio_viewer
    assert "Goal: #" in studio_viewer
    assert ".topo_state.json" not in studio_viewer  # viewer must never perform 60 FPS disk I/O
