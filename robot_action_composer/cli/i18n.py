"""Lightweight zh/en strings for motion-generation and ros2-stack CLIs."""

from __future__ import annotations

import os
from typing import Literal

Lang = Literal["zh", "en"]

_current: Lang = "en"

_MESSAGES: dict[str, dict[Lang, str]] = {
    "option.back": {
        "en": "  0. « Back (previous menu)",
        "zh": "  0. « 返回上一级",
    },
    "option.default_suffix": {
        "en": " (default)",
        "zh": "（默认）",
    },
    "option.select_with_back": {
        "en": "Select option (0/b/back = previous, Enter = default): ",
        "zh": "请选择（0/b/back 返回上一级，回车=默认）: ",
    },
    "option.select": {
        "en": "Select option (press Enter for default): ",
        "zh": "请选择（回车=默认）: ",
    },
    "option.invalid": {
        "en": "[info] Invalid option '{raw}', using default '{default}'.",
        "zh": "[info] 无效选项 '{raw}'，使用默认 '{default}'。",
    },
    "option.yes": {"en": "yes", "zh": "是"},
    "option.no": {"en": "no", "zh": "否"},
    "group.top_level": {"en": "Top level", "zh": "顶层"},
    "group.robots_root": {"en": "Root", "zh": "根目录"},
    "group.drill_title": {
        "en": "{title} — current: {path}",
        "zh": "{title} — 当前: {path}",
    },
    "prompt.invalid_int": {
        "en": "[info] invalid input, using default {default}",
        "zh": "[info] 输入无效，使用默认值 {default}",
    },
    "motion.title": {"en": "Motion Generation", "zh": "运动生成"},
    "motion.cli_selection": {
        "en": "\n[info] CLI selection: robot={robot} task={task} scene={scene} reset_env={reset}",
        "zh": "\n[info] CLI 选择: 机器人={robot} 任务={task} 场景={scene} 重置环境={reset}",
    },
    "motion.how_to_run": {"en": "\nHow to run?", "zh": "\n如何运行？"},
    "motion.recent_selection_header": {
        "en": "  {index}. Recent selection",
        "zh": "  {index}. 最近选择",
    },
    "motion.menu_divider": {
        "en": "  ──────────────────────────────────────────────────",
        "zh": "  ──────────────────────────────────────────────────",
    },
    "motion.brief.robot": {"en": "       Robot: {value}", "zh": "       机器人: {value}"},
    "motion.brief.task": {"en": "       Task: {value}", "zh": "       任务: {value}"},
    "motion.brief.task_with_label": {
        "en": "{task_key} — {label}",
        "zh": "{task_key} — {label}",
    },
    "motion.brief.scene": {"en": "       Scene: {value}", "zh": "       场景: {value}"},
    "motion.brief.mode": {"en": "       Mode: {value}", "zh": "       模式: {value}"},
    "motion.brief.mode_chain": {"en": "chain", "zh": "链式"},
    "motion.brief.segment_item": {
        "en": "       Segment {index}: {task} @ {scene}",
        "zh": "       段 {index}: {task} @ {scene}",
    },
    "motion.brief.runs": {"en": "       Runs: {value}", "zh": "       次数: {value}"},
    "motion.brief.reset_env": {"en": "       Reset env: {value}", "zh": "       重置环境: {value}"},
    "motion.interactive_single_n": {
        "en": "  {index}. New — single task",
        "zh": "  {index}. 新建 — 单任务",
    },
    "motion.interactive_chain_n": {
        "en": "  {index}. New — multi-segment chain",
        "zh": "  {index}. 新建 — 多段链式",
    },
    "motion.preferences_n": {
        "en": "  {index}. Preferences",
        "zh": "  {index}. 偏好设置",
    },
    "motion.select_mode_n": {
        "en": "Select [1-{max_n}] (Enter = 1): ",
        "zh": "请选择 [1-{max_n}]（回车=1）: ",
    },
    "motion.configure": {"en": "\nConfigure motion generation:", "zh": "\n配置运动生成："},
    "motion.single_task": {"en": "  1. Single task", "zh": "  1. 单任务"},
    "motion.multi_chain": {"en": "  2. Multi-segment chain", "zh": "  2. 多段链式"},
    "motion.language_settings": {"en": "  3. Preferences", "zh": "  3. 偏好设置"},
    "motion.select_config": {"en": "Select [1/2/3] (Enter = 1): ", "zh": "请选择 [1/2/3]（回车=1）: "},
    "motion.using_last": {
        "en": "\n[info] Using recent selection:",
        "zh": "\n[info] 使用最近选择：",
    },
    "motion.select_robot": {"en": "Select robot", "zh": "选择机器人"},
    "motion.chain_runs": {
        "en": "How many full-chain motion runs? (Enter = 1): ",
        "zh": "完整链式运动运行几次？（回车=1）: ",
    },
    "motion.chain_multi_reset": {
        "en": (
            "[info] Multiple chain runs: each run resets on the first segment only "
            "(same pattern as multi-scene motion)."
        ),
        "zh": "[info] 多次链式运行：每次仅在第一段重置环境（与多场景运动相同）。",
    },
    "motion.reset_first_segment": {
        "en": "Reset environment on first segment only?",
        "zh": "是否仅在第一段重置环境？",
    },
    "motion.select_task_folder": {"en": "Select task folder", "zh": "选择任务文件夹"},
    "motion.select_task": {"en": "Select task", "zh": "选择任务"},
    "motion.select_scene_config": {"en": "Select config", "zh": "选择场景配置"},
    "motion.motion_runs": {
        "en": "How many motion runs? (Enter = 1): ",
        "zh": "运动运行几次？（回车=1）: ",
    },
    "motion.multi_run_reset": {
        "en": (
            "[info] Multiple motion runs: each run will reset the environment "
            "& randomize the object (same as record episodes)."
        ),
        "zh": "[info] 多次运行：每次将重置环境并随机化物体（与录制 episode 相同）。",
    },
    "motion.reset_randomize": {
        "en": "Reset environment & randomize object?",
        "zh": "是否重置环境并随机化物体？",
    },
    "motion.record_object_resolution": {
        "en": "Record object-resolution JSON during this sim run?",
        "zh": "是否在本次仿真运行中录制 object-resolution JSON？",
    },
    "motion.object_resolution_manual": {
        "en": "Object resolution JSON path for real robot replay: ",
        "zh": "真机回放 object resolution JSON 路径: ",
    },
    "motion.records_dir_missing": {
        "en": "[info] Records directory not found: {path}",
        "zh": "[info] 未找到 records 目录: {path}",
    },
    "motion.no_object_resolution_json": {
        "en": "[info] No object resolution JSON files found in: {path}",
        "zh": "[info] 目录中未找到 object resolution JSON 文件: {path}",
    },
    "motion.available_object_resolution_json": {
        "en": "\nAvailable object resolution JSON files",
        "zh": "\n可用的 object resolution JSON 文件",
    },
    "motion.json_default_newest": {
        "en": " (default, newest)",
        "zh": "（默认，最新）",
    },
    "motion.select_json": {"en": "Select JSON (Enter = default): ", "zh": "选择 JSON（回车=默认）: "},
    "motion.chain_locked_folder": {
        "en": "[info] Chain locked to folder {folder!r}.",
        "zh": "[info] 链式任务已锁定文件夹 {folder!r}。",
    },
    "motion.select_chain_folder": {
        "en": "Select task folder for entire chain",
        "zh": "为整条链选择任务文件夹",
    },
    "motion.select_scene_for_task": {
        "en": "Select scene for {task!r} (Enter = default; __all__ = all unused presets)",
        "zh": "为 {task!r} 选择场景（回车=默认；__all__ = 所有未用预设）",
    },
    "motion.all_scenes_in_chain": {
        "en": (
            "[info] All scenes for {task!r} are already in the chain; pick a single scene instead."
        ),
        "zh": "[info] {task!r} 的所有场景已在链中，请改选单个场景。",
    },
    "motion.expand_all_scenes": {
        "en": (
            "[info] Segment uses ALL_SCENES — expanding to {count} sub-segment(s): {scenes}"
        ),
        "zh": "[info] 本段使用 ALL_SCENES — 展开为 {count} 个子段: {scenes}",
    },
    "motion.select_task_locked_folder": {
        "en": "Select task (folder {folder!r}, locked)",
        "zh": "选择任务（文件夹 {folder!r}，已锁定）",
    },
    "motion.select_task_segment": {
        "en": "Select task for this segment",
        "zh": "为本段选择任务",
    },
    "motion.task_not_in_folder": {
        "en": "[info] Task {task!r} is not in folder {folder!r}. Pick again.",
        "zh": "[info] 任务 {task!r} 不在文件夹 {folder!r} 中，请重选。",
    },
    "motion.chain_max_segments": {
        "en": "[info] Reached 30 segments; finishing chain.",
        "zh": "[info] 已达 30 段上限，结束链式配置。",
    },
    "motion.add_chain_segment": {
        "en": "Add another segment to the chain? [y/N]: ",
        "zh": "是否再添加一段？[y/N]: ",
    },
    "motion.reset.yes": {"en": "yes", "zh": "是"},
    "motion.reset.no": {"en": "no", "zh": "否"},
    "motion.all_scenes": {"en": "ALL_SCENES", "zh": "全部场景"},
    "motion.real_replay": {
        "en": "[ObjectResolution] Real replay from: {path}",
        "zh": "[ObjectResolution] 真机回放来源: {path}",
    },
    "motion.robot_connected": {
        "en": "[OK] Robot connected ({label})",
        "zh": "[OK] 机器人已连接（{label}）",
    },
    "motion.real_replay_label": {
        "en": "real object-resolution replay",
        "zh": "真机 object-resolution 回放",
    },
    "motion.real_replay_header": {
        "en": "Real replay (scene {index}/{total}: {scene})",
        "zh": "真机回放（场景 {index}/{total}: {scene}）",
    },
    "motion.real_all_no_reset": {
        "en": "[info] REAL __all__ replay: continuing to next scene without env reset.",
        "zh": "[info] 真机 __all__ 回放：继续下一场景，不重置环境。",
    },
    "motion.robot_disconnected": {
        "en": "[OK] Robot disconnected",
        "zh": "[OK] 机器人已断开",
    },
    "motion.chain_connected": {
        "en": "[OK] Robot connected (multi-segment chain)",
        "zh": "[OK] 机器人已连接（多段链式）",
    },
    "motion.sim_recording": {
        "en": "[ObjectResolution] Sim live recording to: {path}",
        "zh": "[ObjectResolution] 仿真实时录制至: {path}",
    },
    "motion.chain_run_title": {
        "en": "Chain run {run}/{total_runs} — segment {seg}/{total_seg}  task={task!r}  scene={scene}",
        "zh": "链式运行 {run}/{total_runs} — 段 {seg}/{total_seg}  任务={task!r}  场景={scene}",
    },
    "motion.chain_skip_reset_detail": {
        "en": (
            "[info] Chain: skipping env reset for segments after the first "
            "(robot state continues across segments)."
        ),
        "zh": "[info] 链式：第一段之后跳过重置环境（机器人状态在段间保持）。",
    },
    "motion.chain_skip_blocks_detail": {
        "en": (
            "[info] Chain: consecutive same task {task!r} (scene {scene!r}) — "
            "skip_when_not_first_scene blocks will be skipped."
        ),
        "zh": (
            "[info] 链式：连续相同任务 {task!r}（场景 {scene!r}）— "
            "将跳过 skip_when_not_first_scene 块。"
        ),
    },
    "motion.chain_skip_reset": {
        "en": "[info] Chain: skip env reset for segments after the first in this run.",
        "zh": "[info] 链式：本次运行中第一段之后跳过重置环境。",
    },
    "motion.all_scenes_connected": {
        "en": "[OK] Robot connected (shared session for ALL_SCENES)",
        "zh": "[OK] 机器人已连接（ALL_SCENES 共享会话）",
    },
    "motion.all_scenes_header": {
        "en": "Motion run {run}/{total} (scene {index}/{total_scenes}: {scene})",
        "zh": "运动运行 {run}/{total}（场景 {index}/{total_scenes}: {scene}）",
    },
    "motion.all_scenes_skip_reset": {
        "en": "[info] ALL_SCENES mode: skip env reset for sub-scenes after the first one.",
        "zh": "[info] ALL_SCENES 模式：第一个子场景之后跳过重置环境。",
    },
    "motion.all_scenes_skip_blocks": {
        "en": (
            "[info] ALL_SCENES: not first scene in batch — "
            "skip_when_not_first_scene blocks will be skipped."
        ),
        "zh": "[info] ALL_SCENES：非批次首场景 — 将跳过 skip_when_not_first_scene 块。",
    },
    "motion.single_scene_header": {
        "en": "Motion run {run}/{total} (scene 1/1: {scene})",
        "zh": "运动运行 {run}/{total}（场景 1/1: {scene}）",
    },
    "motion.select_language": {"en": "Select language", "zh": "选择语言"},
    "motion.lang_option_zh": {"en": "中文 (Chinese)", "zh": "中文 (Chinese)"},
    "motion.lang_option_en": {"en": "English (英文)", "zh": "English (英文)"},
    "motion.language_saved": {
        "en": "[info] Language set to {lang}.",
        "zh": "[info] 语言已设为 {lang}。",
    },
    "motion.language_name_zh": {"en": "Chinese", "zh": "中文"},
    "motion.language_name_en": {"en": "English", "zh": "英文"},
    "motion.current_language": {
        "en": "[info] UI language: {lang}",
        "zh": "[info] 界面语言: {lang}",
    },
    "motion.current_record_object_resolution": {
        "en": "[info] Object-resolution JSON recording: {value}",
        "zh": "[info] Object-resolution JSON 录制: {value}",
    },
    "motion.current_ensure_ros2_stack": {
        "en": "[info] Check ROS2 stack before run: {value}",
        "zh": "[info] 运行前检查 ROS2 栈: {value}",
    },
    "motion.settings_menu": {"en": "\nPreferences:", "zh": "\n偏好设置："},
    "motion.settings_language": {"en": "  1. Language", "zh": "  1. 语言"},
    "motion.settings_object_resolution": {
        "en": "  2. Object-resolution JSON recording (current: {current})",
        "zh": "  2. Object-resolution JSON 录制（当前: {current}）",
    },
    "motion.settings_ensure_ros2_stack": {
        "en": "  3. Check ROS2 stack before run (current: {current})",
        "zh": "  3. 运行前检查 ROS2 栈（当前: {current}）",
    },
    "motion.settings_back": {"en": "  0. « Back", "zh": "  0. « 返回"},
    "motion.settings_select": {
        "en": "Select [0/1/2/3] (Enter = back): ",
        "zh": "请选择 [0/1/2/3]（回车=返回）: ",
    },
    "motion.record_object_resolution_saved": {
        "en": "[info] Object-resolution JSON recording set to {value}.",
        "zh": "[info] Object-resolution JSON 录制已设为 {value}。",
    },
    "motion.ensure_ros2_stack": {
        "en": "Check / ensure ROS2 motion & navigation stack before motion run?",
        "zh": "运动运行前是否检查并确保 ROS2 运控/导航栈已启动？",
    },
    "motion.ensure_ros2_stack_saved": {
        "en": "[info] ROS2 stack check set to {value}.",
        "zh": "[info] ROS2 栈检查已设为 {value}。",
    },
    # ros2-stack (shared UI language with motion-generation)
    "ros2_stack.title": {"en": "ROS2 Stack", "zh": "ROS2 栈启动"},
    "ros2_stack.select_robot": {"en": "Select robot", "zh": "选择机器人"},
    "ros2_stack.load_meta": {
        "en": "Load task-folder .meta/ros2_stack.yaml?",
        "zh": "是否加载任务文件夹下的 .meta/ros2_stack.yaml？",
    },
    "ros2_stack.load_meta_yes": {
        "en": "Yes — select task folder",
        "zh": "是 — 选择任务文件夹",
    },
    "ros2_stack.load_meta_no": {
        "en": "No — robot.yaml only",
        "zh": "否 — 仅使用 robot.yaml",
    },
    "ros2_stack.select_task_folder": {"en": "Select task folder", "zh": "选择任务文件夹"},
    "ros2_stack.motion_header": {
        "en": "\n[ros2-stack] Motion launch configuration",
        "zh": "\n[ros2-stack] 运控启动配置",
    },
    "ros2_stack.nav_header": {
        "en": "\n[ros2-stack] Navigation launch configuration",
        "zh": "\n[ros2-stack] 导航启动配置",
    },
    "ros2_stack.configured_default": {
        "en": "  configured default: {value}",
        "zh": "  配置默认: {value}",
    },
    "ros2_stack.command": {"en": "  command: {value}", "zh": "  命令: {value}"},
    "ros2_stack.select_motion_preset": {
        "en": "Select motion preset",
        "zh": "选择运控启动形态",
    },
    "ros2_stack.select_nav_profile": {
        "en": "Select navigation profile",
        "zh": "选择导航 profile",
    },
    "ros2_stack.preset.ocs2-fullbody": {
        "en": "ocs2-fullbody (full body / WBC)",
        "zh": "ocs2-fullbody（全身 / WBC）",
    },
    "ros2_stack.preset.ocs2-split-body": {
        "en": "ocs2-split-body (split body / arm)",
        "zh": "ocs2-split-body（分体 / 臂控）",
    },
    "ros2_stack.preset.ocs2-demo": {
        "en": "ocs2-demo (demo launch)",
        "zh": "ocs2-demo（Demo）",
    },
    "ros2_stack.profile.default": {"en": "default", "zh": "default（完整导航）"},
    "ros2_stack.profile.map_only": {"en": "map_only", "zh": "map_only（仅地图）"},
    "ros2_stack.nav_map_prompt": {
        "en": "Navigation map [{default}] (Enter to keep): ",
        "zh": "导航地图 [{default}]（回车保持）: ",
    },
    "ros2_stack.start_nav_also": {
        "en": "Start navigation as well?",
        "zh": "是否同时启动导航？",
    },
    "ros2_stack.change_options": {
        "en": "Change launch options (preset / profile / map)?",
        "zh": "是否修改启动选项（preset / profile / 地图）？",
    },
    "ros2_stack.how_to_start": {"en": "\nHow to start?", "zh": "\n如何启动？"},
    "ros2_stack.last_selection_header": {
        "en": "  1. Last selection",
        "zh": "  1. 上次选择",
    },
    "ros2_stack.new_selection": {
        "en": "  2. New selection",
        "zh": "  2. 重新选择",
    },
    "ros2_stack.select_start_mode": {
        "en": "Select [1/2] (Enter = 1): ",
        "zh": "请选择 [1/2]（回车=1）: ",
    },
    "ros2_stack.using_last": {
        "en": "\n[info] Using last selection:",
        "zh": "\n[info] 使用上次选择：",
    },
    "ros2_stack.brief.robot": {"en": "    robot: {value}", "zh": "    机器人: {value}"},
    "ros2_stack.brief.group": {"en": "    group: {value}", "zh": "    任务组: {value}"},
    "ros2_stack.brief.motion": {"en": "    motion: {value}", "zh": "    运控: {value}"},
    "ros2_stack.brief.nav": {"en": "    navigation: {value}", "zh": "    导航: {value}"},
    "ros2_stack.brief.nav_yes": {
        "en": "yes ({detail})",
        "zh": "是（{detail}）",
    },
    "ros2_stack.brief.nav_no": {"en": "no", "zh": "否"},
    "ros2_stack.confirm_start": {
        "en": "Start with this configuration?",
        "zh": "按此配置启动？",
    },
    "ros2_stack.ensure_prompt": {
        "en": "Ensure ROS2 motion/navigation stack is running?",
        "zh": "是否确保运控/导航 ROS2 栈已启动？",
    },
    "ros2_stack.ensure_skipped": {
        "en": "[ros2-stack] ensure skipped",
        "zh": "[ros2-stack] 已跳过自动启动",
    },
    "ros2_stack.ensure_hint_cli": {
        "en": (
            "[ros2-stack] config present; pass --ensure-ros2-stack to auto-start "
            "(or use ros2-stack launch)"
        ),
        "zh": (
            "[ros2-stack] 已有配置；请加 --ensure-ros2-stack 以自动启动"
            "（或使用 ros2-stack launch）"
        ),
    },
    "ros2_stack.no_components": {
        "en": "[ros2-stack] no components required for this task",
        "zh": "[ros2-stack] 本任务无需启动额外组件",
    },
    "ros2_stack.resolved_header": {
        "en": "\n[ros2-stack] Resolved launch configuration:",
        "zh": "\n[ros2-stack] 解析后的启动配置：",
    },
    "ros2_stack.will_ensure": {
        "en": "[ros2-stack] Will ensure:",
        "zh": "[ros2-stack] 将确保启动：",
    },
    "ros2_stack.nothing_to_start": {
        "en": "[ros2-stack] nothing to start",
        "zh": "[ros2-stack] 无需启动",
    },
    "ros2_stack.all_ready": {
        "en": "[ros2-stack] all required components already ready",
        "zh": "[ros2-stack] 所需组件均已就绪",
    },
    "ros2_stack.robot_group": {
        "en": "[ros2-stack] robot={robot} group={group}",
        "zh": "[ros2-stack] 机器人={robot} 任务组={group}",
    },
    "ros2_stack.group_none": {"en": "(none)", "zh": "（无）"},
    "ros2_stack.ready": {"en": "READY", "zh": "就绪"},
    "ros2_stack.not_ready": {"en": "NOT READY", "zh": "未就绪"},
    "ros2_stack.warning": {"en": "[ros2-stack] warning: {msg}", "zh": "[ros2-stack] 警告: {msg}"},
    "ros2_stack.no_config": {
        "en": (
            "No ros2_stack config for robot={robot!r} group={group!r}. "
            "Add ros2_stack to robot.yaml and/or <leaf>/.meta/ros2_stack.yaml"
        ),
        "zh": (
            "机器人 {robot!r} 任务组 {group!r} 无 ros2_stack 配置。"
            "请在 robot.yaml 和/或 <leaf>/.meta/ros2_stack.yaml 中添加。"
        ),
    },
    "ros2_stack.no_robots": {
        "en": "No robots under {path}",
        "zh": "未在 {path} 下找到机器人",
    },
    "ros2_stack.list_nodes_failed": {
        "en": "[ros2-stack] failed to list nodes: {exc}",
        "zh": "[ros2-stack] 列举节点失败: {exc}",
    },
    "ros2_stack.stop_result": {
        "en": "[ros2-stack] stop {name}: {result}",
        "zh": "[ros2-stack] 停止 {name}: {result}",
    },
    "ros2_stack.stop_signalled": {"en": "signalled", "zh": "已发信号"},
    "ros2_stack.stop_no_pid": {"en": "no managed pid", "zh": "无托管 PID"},
    "ros2_stack.stop_auto_robot": {
        "en": "[ros2-stack] Stopping managed stack for robot={robot}",
        "zh": "[ros2-stack] 正在停止机器人 {robot} 的托管栈",
    },
    "ros2_stack.stop_nothing": {
        "en": (
            "[ros2-stack] No managed stack running. "
            "Pass --robot <key> or launch with ros2-stack launch first."
        ),
        "zh": (
            "[ros2-stack] 当前没有由本工具托管的运行中栈。"
            "请先 ros2-stack launch，或加 --robot <key>。"
        ),
    },
    "ros2_stack.unknown_robot": {
        "en": "Unknown robot key: {robot!r}",
        "zh": "未知机器人: {robot!r}",
    },
    "ros2_stack.component_line": {
        "en": "  {name}: {status}",
        "zh": "  {name}: {status}",
    },
    "ros2_stack.logs_header": {
        "en": "[ros2-stack] Log files under {path}:",
        "zh": "[ros2-stack] 日志目录 {path}：",
    },
    "ros2_stack.logs_empty": {
        "en": "[ros2-stack] No log files yet for robot={robot}. Launch the stack first.",
        "zh": "[ros2-stack] 机器人 {robot} 尚无日志。请先 ros2-stack launch。",
    },
    "ros2_stack.logs_auto_robot": {
        "en": "[ros2-stack] Showing logs for robot={robot}",
        "zh": "[ros2-stack] 显示机器人 {robot} 的日志",
    },
    "ros2_stack.logs_nothing": {
        "en": (
            "[ros2-stack] No current stack robot. "
            "Pass --robot <key> or launch with ros2-stack launch first."
        ),
        "zh": (
            "[ros2-stack] 未找到当前栈对应的机器人。"
            "请先 ros2-stack launch，或加 --robot <key>。"
        ),
    },
    "ros2_stack.logs_follow_hint": {
        "en": "[ros2-stack] Following logs (Ctrl+C to stop following, stack keeps running)…",
        "zh": "[ros2-stack] 正在跟踪日志（Ctrl+C 仅退出跟踪，不会停止栈）…",
    },
    "ros2_stack.logs_tail_hint": {
        "en": "[ros2-stack] Tip: ros2-stack logs --robot {robot} -f",
        "zh": "[ros2-stack] 跟踪日志: ros2-stack logs --robot {robot} -f",
    },
    "ros2_stack.stop_hint": {
        "en": "[ros2-stack] Tip: ros2-stack stop --robot {robot}",
        "zh": "[ros2-stack] 停止栈: ros2-stack stop --robot {robot}",
    },
}


def normalize_lang(value: str | None) -> Lang | None:
    if value is None:
        return None
    raw = value.strip().lower().replace("_", "-")
    if raw in {"zh", "zh-cn", "cn", "chinese"}:
        return "zh"
    if raw in {"en", "en-us", "english"}:
        return "en"
    raise ValueError(f"Unsupported language: {value!r} (use zh or en)")


def resolve_language(
    *,
    cli_lang: str | None = None,
    cached_lang: str | Lang | None = None,
) -> Lang:
    for candidate in (cli_lang, os.environ.get("MOTION_GENERATION_LANG"), cached_lang):
        if candidate is None:
            continue
        if isinstance(candidate, str):
            lang = normalize_lang(candidate)
        else:
            lang = candidate
        if lang is not None:
            return lang
    locale = os.environ.get("LANG") or os.environ.get("LC_ALL") or ""
    if locale.lower().startswith("zh"):
        return "zh"
    return "en"


def set_language(lang: Lang) -> None:
    global _current
    _current = lang


def get_language() -> Lang:
    return _current


def t(key: str, **kwargs: object) -> str:
    table = _MESSAGES.get(key)
    if table is None:
        raise KeyError(f"Missing i18n key: {key}")
    text = table[_current]
    if kwargs:
        return text.format(**kwargs)
    return text
