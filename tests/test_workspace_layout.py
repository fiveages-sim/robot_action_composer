"""Workspace layout detection: legacy ``robots/`` vs FaSim USD project (``env/``)."""

from __future__ import annotations

from pathlib import Path

import pytest

from robot_action_composer.discovery.workspace_layout import (
    WorkspaceLayoutError,
    expand_path,
    has_robot_profiles,
    load_workspace_layout,
)


def _write_robot_profile(root: Path, *, rel: str = "Vendor/Bot", key: str = "bot") -> Path:
    robot_dir = root / "robots" / rel
    (robot_dir / "task_configs").mkdir(parents=True)
    (robot_dir / "robot.yaml").write_text(
        f"key: {key}\nlabel: Bot\nbase_link_entity_path: /World/bot/base_link\n",
        encoding="utf-8",
    )
    (robot_dir / "task_configs" / "wave.yaml").write_text(
        "task_key: wave\nlabel: Wave\ntask_queue:\n  - skill: robot.snapshot_state\n",
        encoding="utf-8",
    )
    return robot_dir


def _write_usd_project(root: Path, *, robots_root: Path | None) -> Path:
    (root / "env" / "Anigon").mkdir(parents=True)
    (root / "env" / "Anigon" / "default_v2.usda").write_text("#usda 1.0\n", encoding="utf-8")
    (root / "task_configs").mkdir()
    lines = ["extends: bot", "label: Scene Bot"]
    if robots_root is not None:
        lines.insert(0, f"robots_root: {robots_root}")
    (root / "robot.yaml").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return root


def test_legacy_layout_detected_by_real_robot_profile(tmp_path: Path) -> None:
    _write_robot_profile(tmp_path)
    layout = load_workspace_layout(tmp_path)
    assert layout.legacy is True
    assert layout.robots_root == (tmp_path / "robots").resolve()
    assert layout.scene_project is None


def test_usd_project_with_robots_usd_layers_is_not_legacy(tmp_path: Path) -> None:
    """FaSim projects ship ``robots/`` USD layers; that must not mask the project layout."""
    shared = tmp_path / "shared"
    _write_robot_profile(shared)
    project = tmp_path / "project"
    (project / "robots").mkdir(parents=True)
    (project / "robots" / "robot_v2.usda").write_text("#usda 1.0\n", encoding="utf-8")
    _write_usd_project(project, robots_root=shared / "robots")

    assert has_robot_profiles(project / "robots") is False
    layout = load_workspace_layout(project)
    assert layout.legacy is False
    assert layout.scene_project == project.resolve()
    assert layout.robots_root == (shared / "robots").resolve()
    assert layout.task_configs_root == (project / "task_configs").resolve()


def test_legacy_wins_when_both_layouts_present(tmp_path: Path) -> None:
    _write_robot_profile(tmp_path)
    (tmp_path / "env").mkdir()
    layout = load_workspace_layout(tmp_path)
    assert layout.legacy is True


def test_cli_robots_root_overrides_project_yaml(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    shared_a = tmp_path / "a"
    shared_b = tmp_path / "b"
    _write_robot_profile(shared_a)
    _write_robot_profile(shared_b)
    project = _write_usd_project(tmp_path / "project", robots_root=shared_a / "robots")

    layout = load_workspace_layout(project, robots_root=shared_b / "robots")
    assert layout.robots_root == (shared_b / "robots").resolve()


def test_robots_root_from_env_var(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    shared = tmp_path / "shared"
    _write_robot_profile(shared)
    project = _write_usd_project(tmp_path / "project", robots_root=None)
    monkeypatch.setenv("ROBOT_ACTION_COMPOSER_ROBOTS_ROOT", str(shared / "robots"))

    layout = load_workspace_layout(project)
    assert layout.robots_root == (shared / "robots").resolve()


def test_missing_robots_root_is_actionable(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("ROBOT_ACTION_COMPOSER_ROBOTS_ROOT", raising=False)
    monkeypatch.delenv("LEROBOT_ISAACSIM_ROBOTS", raising=False)
    project = _write_usd_project(tmp_path / "project", robots_root=None)

    with pytest.raises(WorkspaceLayoutError) as err:
        load_workspace_layout(project)
    assert "robots_root" in str(err.value)


def test_neither_layout_raises(tmp_path: Path) -> None:
    with pytest.raises(WorkspaceLayoutError):
        load_workspace_layout(tmp_path)


def test_expand_path_env_and_default(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("LAYOUT_TEST_ROOT", str(tmp_path / "base"))
    assert expand_path("${LAYOUT_TEST_ROOT}/robots") == (tmp_path / "base" / "robots").resolve()
    assert expand_path("${LAYOUT_TEST_MISSING:-fallback}", base=tmp_path) == (tmp_path / "fallback").resolve()

    monkeypatch.delenv("LAYOUT_TEST_MISSING", raising=False)
    with pytest.raises(WorkspaceLayoutError):
        expand_path("${LAYOUT_TEST_MISSING}/x")


def test_lerobot_root_falls_back_to_detected_checkout(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """`${LEROBOT_ROOT}` must not require an export in an editable checkout."""
    from robot_action_composer.discovery import workspace_layout

    monkeypatch.delenv("LEROBOT_ROOT", raising=False)
    monkeypatch.setattr(workspace_layout, "detect_lerobot_root", lambda: tmp_path)

    assert workspace_layout.expand_path("${LEROBOT_ROOT}/examples/IsaacSim/robots") == (
        tmp_path / "examples" / "IsaacSim" / "robots"
    ).resolve()



def test_scene_project_discovery_merges_project_tasks(tmp_path: Path) -> None:
    from robot_action_composer.discovery.registry_loader import load_motion_entries

    shared = tmp_path / "shared"
    _write_robot_profile(shared)
    project = _write_usd_project(tmp_path / "project", robots_root=shared / "robots")
    (project / "task_configs" / "carry.yaml").write_text(
        "task_key: carry\nlabel: Carry\nenv_scene: env/Anigon/default_v2.usda\n"
        "task_queue:\n  - skill: robot.snapshot_state\n",
        encoding="utf-8",
    )

    registry = load_motion_entries(project)
    assert set(registry) == {"bot"}
    entry = registry["bot"]
    assert set(entry["tasks"]) == {"carry"}
    assert entry["scene_project"] == str(project.resolve())
    assert entry["robot_dir"] == str((shared / "robots" / "Vendor" / "Bot").resolve())
    assert entry["task_group_dirs"][""] == str((project / "task_configs").resolve())
    assert entry["tasks"]["carry"]["_config_dir"] == str((project / "task_configs").resolve())
