"""Scene project navigation args: project-relative map paths expand, bare names do not."""

from __future__ import annotations

from pathlib import Path

from robot_action_composer.ros2_stack.config import load_merged_ros2_stack


def _robot_profile(root: Path, *, map_arg: str) -> Path:
    robot_dir = root / "robots" / "Vendor" / "Bot"
    robot_dir.mkdir(parents=True)
    (robot_dir / "robot.yaml").write_text(
        "\n".join(
            [
                "key: bot",
                "label: Bot",
                "ros2_stack:",
                "  defaults:",
                "    args:",
                "      robot: bot",
                "  navigation:",
                "    required: auto",
                "    profile: map_only",
                "    args:",
                f"      map: {map_arg}",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    return robot_dir


def _scene(project: Path, *, nav_yaml: str, map_file: bool) -> Path:
    (project / "env").mkdir(parents=True)
    (project / "env" / "scene.usda").write_text("#usda 1.0\n", encoding="utf-8")
    if map_file:
        (project / "maps" / "german_poc").mkdir(parents=True)
        (project / "maps" / "german_poc" / "map.yaml").write_text("image: scene.png\n", encoding="utf-8")
    scene_yaml = project / "robot.yaml"
    scene_yaml.write_text(nav_yaml, encoding="utf-8")
    return scene_yaml


def test_project_relative_map_expands_to_project_abs_path(tmp_path: Path) -> None:
    robot_dir = _robot_profile(tmp_path, map_arg="german_poc")
    project = tmp_path / "project"
    scene_yaml = _scene(
        project,
        nav_yaml="\n".join(
            [
                "extends: bot",
                "ros2_stack:",
                "  navigation:",
                "    required: false",
                "    args:",
                "      map: maps/german_poc/map.yaml",
            ]
        )
        + "\n",
        map_file=True,
    )

    merged, resolved, _ = load_merged_ros2_stack(
        robot_dir=robot_dir, robot_key="bot", scene_robot_yaml=scene_yaml
    )
    assert resolved is not None
    assert resolved.navigation is not None
    assert resolved.navigation.required is False
    expected = (project / "maps" / "german_poc" / "map.yaml").resolve()
    assert Path(merged["navigation"]["args"]["map"]) == expected


def test_bare_package_map_name_is_kept(tmp_path: Path) -> None:
    robot_dir = _robot_profile(tmp_path, map_arg="german_poc")
    project = tmp_path / "project"
    scene_yaml = _scene(
        project,
        nav_yaml="extends: bot\nros2_stack:\n  navigation:\n    required: false\n",
        map_file=False,
    )

    merged, _, _ = load_merged_ros2_stack(
        robot_dir=robot_dir, robot_key="bot", scene_robot_yaml=scene_yaml
    )
    assert merged["navigation"]["args"]["map"] == "german_poc"


def test_project_root_placeholder_expands(tmp_path: Path) -> None:
    robot_dir = _robot_profile(tmp_path, map_arg="german_poc")
    project = tmp_path / "project"
    scene_yaml = _scene(
        project,
        nav_yaml="\n".join(
            [
                "extends: bot",
                "ros2_stack:",
                "  navigation:",
                "    args:",
                "      map: ${PROJECT_ROOT}/maps/german_poc/map.yaml",
            ]
        )
        + "\n",
        map_file=True,
    )

    merged, _, _ = load_merged_ros2_stack(
        robot_dir=robot_dir, robot_key="bot", scene_robot_yaml=scene_yaml
    )
    assert Path(merged["navigation"]["args"]["map"]) == (
        project / "maps" / "german_poc" / "map.yaml"
    ).resolve()
