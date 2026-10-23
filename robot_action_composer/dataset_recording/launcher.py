#!/usr/bin/env python3
"""Recording-specific interactive helpers (re-exports shared CLI utilities)."""

from __future__ import annotations

from robot_action_composer.cli.interactive import (
    prompt_positive_int,
    select_option,
    select_task_with_optional_group,
    task_group_menu_label,
)

__all__ = [
    "collect_runtime_options",
    "prompt_positive_int",
    "select_option",
    "select_task_with_optional_group",
    "task_group_menu_label",
]


def collect_runtime_options(
    *,
    pointcloud_supported: bool,
    default_enable_keypoint_pcd: bool = False,
) -> tuple[int, bool, bool]:
    loops = prompt_positive_int("How many episodes to record? ", default=1, min_value=1)

    if pointcloud_supported:
        default_pcd = "Y" if default_enable_keypoint_pcd else "N"
        use_pcd_input = input(f"Capture depth+pointcloud at keypoints? [y/{default_pcd}]: ").strip().lower()
        if use_pcd_input == "":
            enable_keypoint_pcd = default_enable_keypoint_pcd
        else:
            enable_keypoint_pcd = use_pcd_input in {"y", "yes"}
    else:
        enable_keypoint_pcd = False
        print("[info] Pointcloud option hidden: no depth camera configured for this robot.")

    manual_check_input = input("Manually review each episode after return-to-home? [y/N]: ").strip().lower()
    enable_manual_episode_check = manual_check_input in {"y", "yes"}
    return loops, enable_keypoint_pcd, enable_manual_episode_check
