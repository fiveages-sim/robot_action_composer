"""Spawn / stop ros2_stack launch processes."""

from __future__ import annotations

import json
import os
import shutil
import signal
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

from robot_action_composer.ros2_stack.config import ResolvedComponent
from robot_action_composer.ros2_stack.status import component_is_ready, list_ros_node_names


@dataclass
class LaunchHandle:
    component: str
    pid: int
    log_path: Path
    command: list[str]


def stack_state_dir(workspace_dir: Path, robot_key: str) -> Path:
    return workspace_dir / ".ros2_stack" / robot_key


def _pid_path(state_dir: Path, component: str) -> Path:
    return state_dir / f"{component}.pid"


def _meta_path(state_dir: Path, component: str) -> Path:
    return state_dir / f"{component}.json"


def resolve_setup_bash() -> Path | None:
    """Find a ROS setup.bash to source when spawning launches."""
    # Prefer already-active workspace prefix
    ament = os.environ.get("AMENT_PREFIX_PATH", "")
    if ament:
        first = ament.split(os.pathsep)[0]
        # install prefix -> parent/setup or install/setup
        candidate = Path(first) / "setup.bash"
        if candidate.is_file():
            return candidate
        parent_setup = Path(first).parent / "setup.bash"
        if parent_setup.is_file():
            return parent_setup

    # .fa-env.toml near repo / cwd
    for base in (Path.cwd(), *Path.cwd().parents):
        toml = base / ".fa-env.toml"
        if not toml.is_file():
            continue
        ws = _read_ros2_workspace_from_toml(toml)
        if ws is None:
            continue
        setup = ws.expanduser().resolve() / "install" / "setup.bash"
        if setup.is_file():
            return setup

    distro = os.environ.get("ROS_DISTRO", "jazzy")
    opt = Path(f"/opt/ros/{distro}/setup.bash")
    if opt.is_file():
        return opt
    return None


def _read_ros2_workspace_from_toml(path: Path) -> Path | None:
    text = path.read_text(encoding="utf-8")
    in_ros2 = False
    for line in text.splitlines():
        stripped = line.strip()
        if stripped.startswith("[") and stripped.endswith("]"):
            in_ros2 = stripped == "[ros2]"
            continue
        if in_ros2 and stripped.startswith("workspace"):
            _, _, value = stripped.partition("=")
            value = value.strip().strip('"').strip("'")
            if value:
                return Path(value)
    return None


def _shell_launch_command(argv: Sequence[str], setup_bash: Path | None) -> str:
    launch = " ".join(_shell_quote(a) for a in argv)
    if setup_bash is None:
        return launch
    return f"source {_shell_quote(str(setup_bash))} && {launch}"


def _shell_quote(value: str) -> str:
    if not value:
        return "''"
    if all(c.isalnum() or c in "._-:=/" for c in value):
        return value
    return "'" + value.replace("'", "'\"'\"'") + "'"


def spawn_component(
    component: ResolvedComponent,
    *,
    workspace_dir: Path,
    robot_key: str,
    new_terminal: bool = False,
) -> LaunchHandle:
    state_dir = stack_state_dir(workspace_dir, robot_key)
    state_dir.mkdir(parents=True, exist_ok=True)
    log_path = state_dir / f"{component.name}.log"
    argv = component.build_launch_argv()
    setup = resolve_setup_bash()
    shell_cmd = _shell_launch_command(argv, setup)

    if new_terminal:
        pid = _spawn_in_terminal(shell_cmd, log_path=log_path)
    else:
        pid = _spawn_background(shell_cmd, log_path=log_path)

    handle = LaunchHandle(
        component=component.name,
        pid=pid,
        log_path=log_path,
        command=list(argv),
    )
    _pid_path(state_dir, component.name).write_text(f"{pid}\n", encoding="utf-8")
    _meta_path(state_dir, component.name).write_text(
        json.dumps(
            {
                "pid": pid,
                "component": component.name,
                "command": argv,
                "log": str(log_path),
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    return handle


def _spawn_background(shell_cmd: str, *, log_path: Path) -> int:
    log_f = open(log_path, "a", encoding="utf-8")  # noqa: SIM115 — kept open by child inheritance
    log_f.write(f"\n--- spawn {time.strftime('%Y-%m-%d %H:%M:%S')} ---\n{shell_cmd}\n")
    log_f.flush()
    # Use ``bash -c`` (not ``-lc``) and detach stdin so login-shell side effects /
    # terminal SIGHUP are less likely to tear down long-running Nav2 launches.
    proc = subprocess.Popen(
        ["bash", "-c", shell_cmd],
        stdin=subprocess.DEVNULL,
        stdout=log_f,
        stderr=subprocess.STDOUT,
        start_new_session=True,
        cwd=str(Path.cwd()),
    )
    return int(proc.pid)


def _spawn_in_terminal(shell_cmd: str, *, log_path: Path) -> int:
    """Best-effort GUI terminal; falls back to background spawn."""
    title = "ros2_stack"
    holders = [
        ["gnome-terminal", "--", "bash", "-lc", f"{shell_cmd}; exec bash"],
        ["konsole", "-e", "bash", "-lc", f"{shell_cmd}; exec bash"],
        ["xterm", "-T", title, "-e", "bash", "-lc", f"{shell_cmd}; exec bash"],
    ]
    for cmd in holders:
        if shutil.which(cmd[0]) is None:
            continue
        try:
            proc = subprocess.Popen(cmd, start_new_session=True)
            log_path.parent.mkdir(parents=True, exist_ok=True)
            with open(log_path, "a", encoding="utf-8") as log_f:
                log_f.write(
                    f"\n--- terminal spawn {time.strftime('%Y-%m-%d %H:%M:%S')} ---\n"
                    f"{' '.join(cmd)}\n"
                )
            return int(proc.pid)
        except OSError:
            continue
    return _spawn_background(shell_cmd, log_path=log_path)


def wait_until_ready(
    component: ResolvedComponent,
    *,
    timeout_sec: float = 60.0,
    poll_sec: float = 1.0,
) -> bool:
    deadline = time.monotonic() + max(0.0, timeout_sec)
    while time.monotonic() < deadline:
        try:
            names = list_ros_node_names()
        except Exception:
            names = []
        if component_is_ready(component, names):
            return True
        time.sleep(max(0.1, poll_sec))
    return component_is_ready(component)


def stop_component(*, workspace_dir: Path, robot_key: str, component: str) -> bool:
    """Send SIGINT to a previously spawned component. Returns True if signalled."""
    state_dir = stack_state_dir(workspace_dir, robot_key)
    pid_file = _pid_path(state_dir, component)
    if not pid_file.is_file():
        return False
    try:
        pid = int(pid_file.read_text(encoding="utf-8").strip())
    except ValueError:
        return False
    try:
        os.killpg(pid, signal.SIGINT)
    except ProcessLookupError:
        pass
    except PermissionError:
        try:
            os.kill(pid, signal.SIGINT)
        except ProcessLookupError:
            pass
    # Also try the process itself
    try:
        os.kill(pid, signal.SIGINT)
    except ProcessLookupError:
        pass
    pid_file.unlink(missing_ok=True)
    meta = _meta_path(state_dir, component)
    meta.unlink(missing_ok=True)
    return True


def read_managed_pids(workspace_dir: Path, robot_key: str) -> dict[str, int]:
    state_dir = stack_state_dir(workspace_dir, robot_key)
    out: dict[str, int] = {}
    if not state_dir.is_dir():
        return out
    for path in state_dir.glob("*.pid"):
        try:
            out[path.stem] = int(path.read_text(encoding="utf-8").strip())
        except ValueError:
            continue
    return out


def _pid_is_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def read_live_managed_pids(workspace_dir: Path, robot_key: str) -> dict[str, int]:
    """Like ``read_managed_pids`` but only PIDs that still exist."""
    return {
        name: pid
        for name, pid in read_managed_pids(workspace_dir, robot_key).items()
        if _pid_is_alive(pid)
    }


def list_robots_with_live_managed_pids(workspace_dir: Path) -> list[str]:
    """Robot keys under ``.ros2_stack/`` that still have a live managed PID."""
    root = workspace_dir / ".ros2_stack"
    if not root.is_dir():
        return []
    found: list[str] = []
    for path in sorted(root.iterdir()):
        if not path.is_dir():
            continue
        if read_live_managed_pids(workspace_dir, path.name):
            found.append(path.name)
    return found


def component_log_path(workspace_dir: Path, robot_key: str, component: str) -> Path:
    return stack_state_dir(workspace_dir, robot_key) / f"{component}.log"


def list_component_logs(workspace_dir: Path, robot_key: str) -> dict[str, Path]:
    """Return existing log files for a robot (motion/navigation/…)."""
    state_dir = stack_state_dir(workspace_dir, robot_key)
    out: dict[str, Path] = {}
    if not state_dir.is_dir():
        return out
    for path in sorted(state_dir.glob("*.log")):
        out[path.stem] = path
    return out


def ensure_components(
    components: Sequence[ResolvedComponent],
    *,
    workspace_dir: Path,
    robot_key: str,
    new_terminal: bool = False,
    ready_timeout_sec: float = 60.0,
) -> list[LaunchHandle]:
    """Start missing components and wait until ready. Skip already-ready ones."""
    handles: list[LaunchHandle] = []
    try:
        names = list_ros_node_names()
    except Exception as exc:
        print(f"[ros2-stack] warning: could not list ROS nodes ({exc}); will try to start")
        names = []

    for component in components:
        if component_is_ready(component, names):
            print(f"[ros2-stack] {component.name}: already ready — skip launch")
            continue
        print(f"[ros2-stack] {component.name}: starting — {component.launch_command_str()}")
        handle = spawn_component(
            component,
            workspace_dir=workspace_dir,
            robot_key=robot_key,
            new_terminal=new_terminal,
        )
        handles.append(handle)
        print(f"[ros2-stack] {component.name}: pid={handle.pid} log={handle.log_path}")
        ok = wait_until_ready(component, timeout_sec=ready_timeout_sec)
        if not ok:
            raise TimeoutError(
                f"{component.name} did not become ready within {ready_timeout_sec:.0f}s "
                f"(log: {handle.log_path})"
            )
        print(f"[ros2-stack] {component.name}: ready")
        try:
            names = list_ros_node_names()
        except Exception:
            pass
    if handles:
        print(
            f"[ros2-stack] logs: ros2-stack logs --robot {robot_key} -f\n"
            f"[ros2-stack] stop: ros2-stack stop --robot {robot_key}"
        )
    return handles


__all__ = [
    "LaunchHandle",
    "component_log_path",
    "ensure_components",
    "list_component_logs",
    "list_robots_with_live_managed_pids",
    "read_live_managed_pids",
    "read_managed_pids",
    "resolve_setup_bash",
    "spawn_component",
    "stack_state_dir",
    "stop_component",
    "wait_until_ready",
]
