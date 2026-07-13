"""Lightweight zh/en strings for motion-generation CLI."""

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
    "motion.last_selection_header": {"en": "  1. Last selection", "zh": "  1. 上次选择"},
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
    "motion.interactive_single": {
        "en": "  2. New — single task",
        "zh": "  2. 新建 — 单任务",
    },
    "motion.interactive_chain": {
        "en": "  3. New — multi-segment chain",
        "zh": "  3. 新建 — 多段链式",
    },
    "motion.select_mode": {"en": "Select [1/2/3/4] (Enter = 1): ", "zh": "请选择 [1/2/3/4]（回车=1）: "},
    "motion.configure": {"en": "\nConfigure motion generation:", "zh": "\n配置运动生成："},
    "motion.single_task": {"en": "  1. Single task", "zh": "  1. 单任务"},
    "motion.multi_chain": {"en": "  2. Multi-segment chain", "zh": "  2. 多段链式"},
    "motion.language_settings": {"en": "  3. Preferences", "zh": "  3. 偏好设置"},
    "motion.language_settings_how_to_run": {
        "en": "  4. Preferences",
        "zh": "  4. 偏好设置",
    },
    "motion.select_config": {"en": "Select [1/2/3] (Enter = 1): ", "zh": "请选择 [1/2/3]（回车=1）: "},
    "motion.using_last": {
        "en": "\n[info] Using last selection:",
        "zh": "\n[info] 使用上次选择：",
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
    "motion.settings_menu": {"en": "\nPreferences:", "zh": "\n偏好设置："},
    "motion.settings_language": {"en": "  1. Language", "zh": "  1. 语言"},
    "motion.settings_object_resolution": {
        "en": "  2. Object-resolution JSON recording (current: {current})",
        "zh": "  2. Object-resolution JSON 录制（当前: {current}）",
    },
    "motion.settings_back": {"en": "  0. « Back", "zh": "  0. « 返回"},
    "motion.settings_select": {
        "en": "Select [0/1/2] (Enter = back): ",
        "zh": "请选择 [0/1/2]（回车=返回）: ",
    },
    "motion.record_object_resolution_saved": {
        "en": "[info] Object-resolution JSON recording set to {value}.",
        "zh": "[info] Object-resolution JSON 录制已设为 {value}。",
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
