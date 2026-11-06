"""`robot.yaml` scene overlays: ``extends`` / ``robots_root`` merge rules."""

from __future__ import annotations

from pathlib import Path

import pytest

from robot_action_composer.config.robot_yaml import (
    motion_profile_from_robot_yaml,
    resolve_profile_dir_for_key,
)


def _base_profile(root: Path, *, key: str = "bot") -> Path:
    robot_dir = root / "robots" / "Vendor" / "Bot"
    robot_dir.mkdir(parents=True)
    (robot_dir / "robot.yaml").write_text(
        "\n".join(
            [
                f"key: {key}",
                "label: Base Bot",
                "base_link_entity_path: /World/base_link",
                "post_reset_wait: 3.0",
                "gripper_action_wait: 2.0",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    return robot_dir


def test_resolve_profile_dir_for_key_scans_nested_dirs(tmp_path: Path) -> None:
    robot_dir = _base_profile(tmp_path)
    assert resolve_profile_dir_for_key(tmp_path / "robots", "bot") == robot_dir

    with pytest.raises(FileNotFoundError):
        resolve_profile_dir_for_key(tmp_path / "robots", "nope")


def test_extends_inherits_key_and_overrides_fields(tmp_path: Path) -> None:
    _base_profile(tmp_path)
    scene = tmp_path / "scene" / "robot.yaml"
    scene.parent.mkdir()
    scene.write_text(
        "\n".join(
            [
                "extends: bot",
                "robots_root: ../robots",
                "base_link_entity_path: /World/robot/Vendor_Bot/base_link",
            ]
        )
        + "\n",
        encoding="utf-8",
    )

    cfg, key, label = motion_profile_from_robot_yaml(scene)
    assert key == "bot"
    assert label == "Base Bot"
    assert cfg.base_link_entity_path == "/World/robot/Vendor_Bot/base_link"
    assert cfg.post_reset_wait == 3.0
    assert cfg.gripper_action_wait == 2.0


def test_extends_accepts_explicit_relative_path(tmp_path: Path) -> None:
    robot_dir = _base_profile(tmp_path)
    scene = tmp_path / "scene" / "robot.yaml"
    scene.parent.mkdir()
    rel = Path("..") / "robots" / "Vendor" / "Bot"
    scene.write_text(f"extends: {rel.as_posix()}\nrobots_root: ../robots\n", encoding="utf-8")

    _, key, _ = motion_profile_from_robot_yaml(scene)
    assert key == "bot"
    assert (robot_dir / "robot.yaml").is_file()


def test_overlay_cannot_change_robot_key(tmp_path: Path) -> None:
    _base_profile(tmp_path)
    scene = tmp_path / "scene" / "robot.yaml"
    scene.parent.mkdir()
    scene.write_text("extends: bot\nrobots_root: ../robots\nkey: other\n", encoding="utf-8")

    with pytest.raises(ValueError, match="must keep the shared robot key"):
        motion_profile_from_robot_yaml(scene)


def test_unknown_root_key_rejected(tmp_path: Path) -> None:
    _base_profile(tmp_path)
    scene = tmp_path / "scene" / "robot.yaml"
    scene.parent.mkdir()
    scene.write_text("extends: bot\nrobots_root: ../robots\nbogus: 1\n", encoding="utf-8")

    with pytest.raises(ValueError, match="unknown root keys"):
        motion_profile_from_robot_yaml(scene)


def test_extends_cycle_is_reported(tmp_path: Path) -> None:
    (tmp_path / "a.yaml").write_text("extends: ./b.yaml\nrobots_root: .\n", encoding="utf-8")
    (tmp_path / "b.yaml").write_text("extends: ./a.yaml\nrobots_root: .\n", encoding="utf-8")

    with pytest.raises(ValueError, match="cycle"):
        motion_profile_from_robot_yaml(tmp_path / "a.yaml")
