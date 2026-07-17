"""Bash (and friends) tab-completion helpers for ``ros2-stack``."""

from __future__ import annotations

from pathlib import Path

from robot_action_composer.cli.motion_main import resolve_workspace_dir
from robot_action_composer.ros2_stack.presets import NAV_PROFILES, list_motion_preset_keys

COMMANDS = ("status", "start", "stop", "logs", "completion")
GLOBAL_FLAGS = ("--workspace", "--lang", "-h", "--help")
COMMON_FLAGS = (
    "--robot",
    "--group",
    "--set",
    "--motion-preset",
    "--nav-profile",
    "--skip-group",
    "-h",
    "--help",
)
START_EXTRA = ("--new-terminal", "--timeout", "--no-interactive-override", "--force-nav")
STOP_FLAGS = ("--robot", "--component", "-h", "--help")
LOGS_FLAGS = ("--robot", "--component", "-f", "--follow", "-n", "--lines", "-h", "--help")
COMPLETION_SHELLS = ("bash",)
LANGS = ("zh", "en")
COMPONENTS = ("motion", "navigation", "all")


def _iter_robot_dirs(robots_root: Path) -> list[Path]:
    """Mirror registry discovery without importing ROS / loading MotionRobotConfig."""
    found: list[Path] = []
    if not robots_root.is_dir():
        return found
    for child in sorted(p for p in robots_root.iterdir() if p.is_dir() and not p.name.startswith("__")):
        if (child / "task_configs").is_dir() and (child / "robot.yaml").is_file():
            found.append(child)
            continue
        for nested in sorted(p for p in child.iterdir() if p.is_dir() and not p.name.startswith("__")):
            if (nested / "task_configs").is_dir() and (nested / "robot.yaml").is_file():
                found.append(nested)
    return found


def _robot_key_from_yaml(path: Path) -> str | None:
    try:
        import yaml  # type: ignore[import-untyped]
    except ImportError:
        return None
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError):
        return None
    if not isinstance(data, dict):
        return None
    key = data.get("key")
    return str(key) if isinstance(key, str) and key else None


def complete_robots(*, workspace: str | None = None) -> list[str]:
    """Lightweight robot-key listing for tab completion (no rclpy required)."""
    workspace_dir = resolve_workspace_dir(Path(workspace) if workspace else None)
    keys: list[str] = []
    for robot_dir in _iter_robot_dirs(workspace_dir / "robots"):
        key = _robot_key_from_yaml(robot_dir / "robot.yaml")
        if key:
            keys.append(key)
    return sorted(set(keys))


def complete_groups(*, workspace: str | None = None, robot: str | None = None) -> list[str]:
    """List task-group ids for ``--group`` without loading full motion registry."""
    if not robot:
        return []
    workspace_dir = resolve_workspace_dir(Path(workspace) if workspace else None)
    robot_dir: Path | None = None
    for d in _iter_robot_dirs(workspace_dir / "robots"):
        if _robot_key_from_yaml(d / "robot.yaml") == robot:
            robot_dir = d
            break
    if robot_dir is None:
        return []
    try:
        from robot_action_composer.task_config_io import discover_task_configs
    except Exception:
        return []
    try:
        discovery = discover_task_configs(
            robot_dir / "task_configs",
            robot_dir_name=robot_dir.name,
        )
    except Exception:
        return []
    return sorted(g for g, keys in discovery.task_groups.items() if keys)


def bash_completion_script() -> str:
    """Return a bash completion script for ``ros2-stack`` (eval or install under completions/)."""
    presets = " ".join(list_motion_preset_keys())
    profiles = " ".join(sorted(NAV_PROFILES))
    cmds = " ".join(COMMANDS)
    # NOTE: Never set IFS=$'\\n' around ``compgen -W`` — that prevents -W from
    # splitting on spaces, so the whole wordlist becomes ONE completion and bash
    # inserts e.g. ``status start stop logs completion`` in one go.
    return f"""# ros2-stack bash completion — eval "$(ros2-stack completion bash)"
_ros2_stack_completion() {{
  local cur prev words cword
  if declare -F _init_completion >/dev/null 2>&1; then
    _init_completion -n =: || return
  else
    COMPREPLY=()
    cur="${{COMP_WORDS[COMP_CWORD]}}"
    prev="${{COMP_WORDS[COMP_CWORD-1]}}"
    words=("${{COMP_WORDS[@]}}")
    cword=$COMP_CWORD
  fi

  local i cmd="" workspace="" robot=""
  for ((i = 1; i < cword; i++)); do
    case "${{words[i]}}" in
      status|start|stop|logs|completion) cmd="${{words[i]}}" ;;
      --workspace)
        ((i++))
        workspace="${{words[i]}}"
        ;;
      --robot)
        ((i++))
        robot="${{words[i]}}"
        ;;
    esac
  done

  # Populate COMPREPLY from a space/tab/newline-separated wordlist.
  # Keep default IFS so compgen -W splits correctly; only ``read`` clears IFS.
  _ros2_stack_reply() {{
    local wordlist="$1" item
    COMPREPLY=()
    while IFS= read -r item; do
      [[ -n "$item" ]] && COMPREPLY+=("$item")
    done < <(compgen -W "$wordlist" -- "$cur")
  }}

  # Global flag values before subcommand
  if [[ -z "$cmd" ]]; then
    case "$prev" in
      --lang) _ros2_stack_reply "{' '.join(LANGS)}"; return ;;
      --workspace)
        COMPREPLY=()
        while IFS= read -r item; do
          [[ -n "$item" ]] && COMPREPLY+=("$item")
        done < <(compgen -d -- "$cur")
        return
        ;;
    esac
    if [[ "$cur" == -* ]]; then
      _ros2_stack_reply "{' '.join(GLOBAL_FLAGS)}"
      return
    fi
    _ros2_stack_reply "{cmds}"
    return
  fi

  case "$prev" in
    --lang) _ros2_stack_reply "{' '.join(LANGS)}"; return ;;
    --workspace)
      COMPREPLY=()
      while IFS= read -r item; do
        [[ -n "$item" ]] && COMPREPLY+=("$item")
      done < <(compgen -d -- "$cur")
      return
      ;;
    --robot)
      local robots
      robots=$(ros2-stack _complete robots ${{workspace:+--workspace "$workspace"}} 2>/dev/null)
      _ros2_stack_reply "$robots"
      return
      ;;
    --group)
      local groups
      # newline-separated from _complete; default IFS still splits -W on newlines
      groups=$(ros2-stack _complete groups ${{workspace:+--workspace "$workspace"}} ${{robot:+--robot "$robot"}} 2>/dev/null)
      _ros2_stack_reply "$groups"
      return
      ;;
    --motion-preset) _ros2_stack_reply "{presets}"; return ;;
    --nav-profile) _ros2_stack_reply "{profiles}"; return ;;
    --component) _ros2_stack_reply "{' '.join(COMPONENTS)}"; return ;;
    --timeout|-n|--lines) return ;;
    completion) _ros2_stack_reply "{' '.join(COMPLETION_SHELLS)}"; return ;;
  esac

  if [[ "$cur" == -* ]]; then
    case "$cmd" in
      status) _ros2_stack_reply "{' '.join(COMMON_FLAGS)}" ;;
      start) _ros2_stack_reply "{' '.join(COMMON_FLAGS + START_EXTRA)}" ;;
      stop) _ros2_stack_reply "{' '.join(STOP_FLAGS)}" ;;
      logs) _ros2_stack_reply "{' '.join(LOGS_FLAGS)}" ;;
      completion) _ros2_stack_reply "-h --help" ;;
    esac
    return
  fi

  case "$cmd" in
    completion) _ros2_stack_reply "{' '.join(COMPLETION_SHELLS)}" ;;
  esac
}}

complete -F _ros2_stack_completion ros2-stack
"""


__all__ = [
    "bash_completion_script",
    "complete_groups",
    "complete_robots",
]
