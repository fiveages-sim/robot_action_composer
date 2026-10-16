"""Readiness hints for optional dex-grasp pieces.

Prints which module is missing and which ``./init.sh`` command installs it.
Does not install anything, and does not change grasp or motion control flow:
callers keep their existing success and failure paths.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Optional


def _repo_root(workspace: Path) -> Optional[Path]:
    for parent in (workspace, *workspace.parents):
        if (parent / "init.sh").is_file() and (parent / ".gitmodules").is_file():
            return parent
    return None


def _hug_root_from_python(hug_python: Path) -> Optional[Path]:
    # .../submodules/hug/.venv/bin/python — do not resolve() the symlink.
    parts = hug_python.absolute().parts
    if len(parts) >= 4 and parts[-3] == ".venv" and parts[-2] == "bin":
        return Path(*parts[:-3])
    parent = hug_python.absolute().parent
    if parent.name == "bin" and parent.parent.name == ".venv":
        return parent.parent.parent
    return None


def _using_hug_venv() -> bool:
    exe = Path(sys.executable).absolute()
    root = _hug_root_from_python(exe)
    if root is None:
        return False
    return (root / "pyproject.toml").is_file() and (root / "src").is_dir()


def _check_import(module: str) -> Optional[str]:
    try:
        __import__(module)
    except Exception as exc:  # ImportError and native wheel failures
        return f"{type(exc).__name__}: {exc}"
    return None


def hug_asset_lines(hug_root: Path) -> list[str]:
    """Missing HUG files that git clone does not provide."""
    lines: list[str] = []
    ckpt = hug_root / "checkpoints" / "hug_full.safetensors"
    if not ckpt.is_file():
        lines.append(
            "缺少 HUG 权重 checkpoints/hug_full.safetensors（体积大，已 gitignore）。"
            "处理: cd submodules/hug && source .venv/bin/activate && bash scripts/download_assets.sh"
        )
    mano = hug_root / "assets" / "mano_models" / "models" / "MANO_RIGHT.pkl"
    if not mano.is_file():
        lines.append(
            "缺少 MANO 模型 assets/mano_models/models/MANO_RIGHT.pkl"
            "（许可文件，不能随仓库下载）。"
            "处理: 在 https://mano.is.tue.mpg.de/ 注册并下载，"
            "把解压后的 models/ 与 webuser/ 放到 assets/mano_models/ 。"
            "说明: submodules/hug/docs/SETUP_LOCAL.md"
        )
    return lines


def grasp_readiness_lines(
    workspace: Path,
    cfg: object,
    *,
    command: Optional[str] = None,
) -> list[str]:
    """Lines for items the given command needs. Empty when those items are present."""
    need_hug = command in (None, "serve", "auto")
    need_viser = command in (None, "serve")
    need_retarget = command in (None, "serve", "auto", "retarget-hand", "close-hand")
    lines: list[str] = []

    if _using_hug_venv():
        lines.append(
            "当前解释器是 submodules/hug/.venv（Python 3.10）。"
            "ros2-stack / motion-generation / grasp-generation 请用仓库根目录的环境"
            "（source .venv/bin/activate）。不要把 CLI 装进 HUG 环境。"
        )

    if need_viser:
        err = _check_import("viser")
        if err:
            lines.append(
                f"缺少 Viser（{err}）。抓取调试界面需要它；任务编排不需要。"
                "处理: 在仓库根目录 ./init.sh install-grasp"
            )

    profile = str(getattr(cfg, "hand_profile", "") or "").strip().lower()
    if need_retarget and "wuji" in profile:
        err = _check_import("wuji_sdk")
        if err:
            lines.append(
                f"hand_profile={profile!r} 需要 wuji-sdk，当前导入失败（{err}）。"
                "处理: ./init.sh install-grasp"
                "（装进父仓环境，不是 hug/.venv）。"
                "开源算法树是可选参考，产品路径不依赖嵌套 MuJoCo。"
            )

    if need_hug:
        try:
            hug_python = Path(cfg.resolved_hug_python(workspace))  # type: ignore[attr-defined]
        except Exception as exc:
            lines.append(f"无法解析 HUG 解释器路径: {exc}")
            hug_python = Path()
        if not hug_python.is_file():
            lines.append(
                f"缺少 HUG 独立环境: {hug_python or '(未解析)'} 。"
                "处理: ./init.sh submodules-dex && ./init.sh hug-env"
                "（Python 3.10，与父仓环境分开）。"
            )
        else:
            root = _hug_root_from_python(hug_python)
            if root is not None:
                lines.extend(hug_asset_lines(root))
    return lines


def format_readiness(lines: list[str], *, title: str) -> str:
    if not lines:
        return ""
    body = "\n".join(f"  - {line}" for line in lines)
    return (
        f"{title}\n"
        "  未列出的功能可以继续用。补齐后再跑当前这条命令。\n"
        f"{body}"
    )


def warn_grasp_readiness(
    workspace: Path,
    cfg: object,
    *,
    command: Optional[str] = None,
) -> None:
    try:
        text = format_readiness(
            grasp_readiness_lines(workspace, cfg, command=command),
            title="[grasp] 当前命令还缺这些（不改变已有抓取流程，只提示）:",
        )
    except Exception as exc:
        print(f"[grasp] 就绪检查跳过: {exc}", file=sys.stderr)
        return
    if text:
        print(text, file=sys.stderr)


def warn_hug_python_missing(hug_python: Path) -> str:
    root = _hug_root_from_python(Path(hug_python))
    extra = ""
    if root is not None:
        asset_lines = hug_asset_lines(root)
        if asset_lines:
            extra = "\n" + "\n".join(f"  - {line}" for line in asset_lines)
    return (
        f"HUG python not found: {hug_python}. "
        "Create with: ./init.sh hug-env "
        "(or: cd submodules/hug && bash scripts/setup_uv.sh). "
        "Weights and MANO are not in git."
        f"{extra}"
    )


def warn_motion_environment() -> None:
    """Hints for the orchestration CLI. Dex-grasp gaps are reported only by that skill."""
    lines: list[str] = []
    try:
        if _using_hug_venv():
            lines.append(
                "当前解释器是 submodules/hug/.venv。"
                "motion-generation / ros2-stack 请 source 仓库根目录 .venv/bin/activate。"
            )
        err = _check_import("rclpy")
        if err:
            lines.append(
                f"未导入 rclpy（{err}）。"
                "请 source ROS 2，或 ./init.sh ros2-workspace 把工作空间写进环境激活脚本。"
                "这一项与灵巧手无关。"
            )
        err = _check_import("ros2_robot_interface")
        if err:
            if "No module named 'ros2_robot_interface'" in err:
                lines.append(
                    "未安装 ros2_robot_interface。处理: ./init.sh submodules-core && ./init.sh install"
                )
            else:
                lines.append(
                    f"ros2_robot_interface 已装上，但导入失败（{err}）。"
                    "常见原因是当前 shell 没有运控工作空间里的自定义消息。"
                    "处理: source 该工作空间，或 ./init.sh ros2-workspace。"
                )
    except Exception as exc:
        print(f"[motion] 环境检查跳过: {exc}", file=sys.stderr)
        return
    text = format_readiness(
        lines,
        title="[motion] 任务编排环境还缺这些（灵巧手未装不影响 pick/place 等技能）:",
    )
    if text:
        print(text, file=sys.stderr)
