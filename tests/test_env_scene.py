"""`env_scene` binding between task YAML and a runnable USD under ``<project>/env``."""

from __future__ import annotations

from pathlib import Path

import pytest

from robot_action_composer.task_config_io import iter_env_scenes, resolve_env_scene


def _project(tmp_path: Path) -> Path:
    (tmp_path / "env" / "Anigon").mkdir(parents=True)
    (tmp_path / "env" / "Anigon" / "default_v2.usda").write_text("#usda 1.0\n", encoding="utf-8")
    (tmp_path / "env" / "Galbot").mkdir()
    (tmp_path / "env" / "Galbot" / "S1.usda").write_text("#usda 1.0\n", encoding="utf-8")
    return tmp_path


def test_resolve_existing_scene(tmp_path: Path) -> None:
    project = _project(tmp_path)
    resolved = resolve_env_scene(
        {"env_scene": "env/Anigon/default_v2.usda"}, project_root=project
    )
    assert resolved == (project / "env" / "Anigon" / "default_v2.usda").resolve()


def test_default_env_scene_used_when_task_omits_it(tmp_path: Path) -> None:
    project = _project(tmp_path)
    resolved = resolve_env_scene(
        {},
        project_root=project,
        default_env_scene="env/Anigon/default_v2.usda",
    )
    assert resolved is not None
    assert resolved.name == "default_v2.usda"


def test_no_env_scene_and_no_default_returns_none(tmp_path: Path) -> None:
    project = _project(tmp_path)
    assert resolve_env_scene({}, project_root=project) is None


def test_legacy_workspace_ignores_env_scene(tmp_path: Path) -> None:
    assert resolve_env_scene({"env_scene": "env/x.usda"}, project_root=None) is None


def test_absolute_env_scene_rejected(tmp_path: Path) -> None:
    project = _project(tmp_path)
    absolute = str(project / "env" / "Anigon" / "default_v2.usda")
    with pytest.raises(ValueError, match="relative to the USD project root"):
        resolve_env_scene({"env_scene": absolute}, project_root=project)


def test_missing_env_scene_lists_candidates(tmp_path: Path) -> None:
    project = _project(tmp_path)
    with pytest.raises(FileNotFoundError) as err:
        resolve_env_scene({"env_scene": "env/Anigon/nope.usda"}, project_root=project)
    message = str(err.value)
    assert "env/Anigon/default_v2.usda" in message
    assert "env/Galbot/S1.usda" in message


def test_iter_env_scenes_is_sorted_and_recursive(tmp_path: Path) -> None:
    project = _project(tmp_path)
    names = [p.relative_to(project).as_posix() for p in iter_env_scenes(project)]
    assert names == ["env/Anigon/default_v2.usda", "env/Galbot/S1.usda"]
