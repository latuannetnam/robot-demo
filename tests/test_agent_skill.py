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
