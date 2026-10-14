"""RGB-D helpers for HUG: depth unit conversion, click mapping, ROS/file IO."""

from __future__ import annotations

import json
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import cv2
import numpy as np


@dataclass(frozen=True)
class RgbdFrame:
    """One synchronized head-camera frame (full resolution)."""

    rgb: np.ndarray  # (H, W, 3) uint8 RGB
    depth_m: np.ndarray  # (H, W) float32 meters, 0 = invalid
    K: np.ndarray  # (3, 3) at full rgb resolution
    stamp_sec: float = 0.0


def depth_to_uint16_mm(depth_m: np.ndarray, max_m: float = 10.0) -> np.ndarray:
    """Convert metric depth (32FC1) to HUG uint16 millimetres."""
    d = np.asarray(depth_m, dtype=np.float32)
    d = np.where(np.isfinite(d) & (d > 0) & (d < max_m), d, 0.0)
    mm = np.clip(np.rint(d * 1000.0), 0, 65535).astype(np.uint16)
    return mm


def full_pixel_to_224(
    u_px: float,
    v_px: float,
    width: int,
    height: int,
) -> tuple[float, float]:
    """Map a full-res pixel into HUG's center-crop → 224 space (matches prepare_pkl)."""
    size = min(height, width)
    x_off = (width - size) // 2
    y_off = (height - size) // 2
    u_c = float(np.clip(u_px - x_off, 0, size - 1))
    v_c = float(np.clip(v_px - y_off, 0, size - 1))
    return u_c * 224.0 / size, v_c * 224.0 / size


def full_norm_to_224(
    u_norm: float,
    v_norm: float,
    width: int,
    height: int,
) -> tuple[float, float]:
    """Normalized (0–1) coords on the **full** image → 224 crop space."""
    return full_pixel_to_224(u_norm * (width - 1), v_norm * (height - 1), width, height)


def uv224_to_full_pixel(
    u224: float,
    v224: float,
    width: int,
    height: int,
) -> tuple[float, float]:
    """Inverse of :func:`full_pixel_to_224` (224 crop space → full-res pixel)."""
    size = min(height, width)
    x_off = (width - size) // 2
    y_off = (height - size) // 2
    u_c = float(u224) * float(size) / 224.0
    v_c = float(v224) * float(size) / 224.0
    return u_c + x_off, v_c + y_off


def K_from_fx_fy_cx_cy(fx: float, fy: float, cx: float, cy: float) -> np.ndarray:
    return np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)


def load_frame_from_dir(path: Path) -> RgbdFrame:
    """Load ``*rgb*``, ``*depth*``, ``*intrinsics*`` from a folder (HUG prepare_inputs layout)."""
    path = Path(path)
    rgb_path = _find_one(path, ("rgb",), (".png", ".jpg", ".jpeg"))
    depth_path = _find_one(path, ("depth",), (".png", ".npy", ".exr"))
    K_path = _find_one(path, ("intrinsics", "camera_info", "K"), (".txt", ".csv", ".npy", ".json"))

    bgr = cv2.imread(str(rgb_path), cv2.IMREAD_COLOR)
    if bgr is None:
        raise FileNotFoundError(f"failed to read rgb: {rgb_path}")
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

    depth_m = _load_depth_any(depth_path)
    if depth_m.shape[:2] != rgb.shape[:2]:
        raise ValueError(f"rgb {rgb.shape[:2]} != depth {depth_m.shape[:2]}")
    K = _load_K(K_path)
    return RgbdFrame(rgb=rgb, depth_m=depth_m, K=K)


def save_frame_to_dir(frame: RgbdFrame, out_dir: Path, stem: str = "capture") -> Path:
    """Write rgb.png / depth_mm.png / intrinsics.txt for offline HUG prepare_inputs."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out_dir / f"{stem}_rgb.png"), cv2.cvtColor(frame.rgb, cv2.COLOR_RGB2BGR))
    cv2.imwrite(str(out_dir / f"{stem}_depth.png"), depth_to_uint16_mm(frame.depth_m))
    fx, fy, cx, cy = frame.K[0, 0], frame.K[1, 1], frame.K[0, 2], frame.K[1, 2]
    (out_dir / f"{stem}_intrinsics.txt").write_text(f"{fx} {fy} {cx} {cy}\n")
    meta = {
        "stamp_sec": frame.stamp_sec,
        "shape": list(frame.rgb.shape),
        "K": frame.K.tolist(),
    }
    (out_dir / f"{stem}_meta.json").write_text(json.dumps(meta, indent=2))
    return out_dir


def ros_python_usable() -> bool:
    """True if this interpreter can import Jazzy ``rclpy`` C extensions."""
    try:
        import rclpy  # noqa: F401
        from rclpy.impl.implementation_singleton import (  # noqa: F401
            rclpy_implementation,
        )

        return True
    except Exception:
        return False


def resolve_ros_python() -> str:
    """Prefer system Python 3.12 (Jazzy); never the HUG 3.10 venv interpreter."""
    import shutil
    import sys

    for cand in ("/usr/bin/python3.12", "/usr/bin/python3"):
        if Path(cand).is_file():
            return cand
    found = shutil.which("python3.12") or shutil.which("python3")
    if found and Path(found).resolve() != Path(sys.executable).resolve():
        return found
    raise RuntimeError(
        "No system Python 3.12 found for ROS capture. Install ROS Jazzy python3-rclpy."
    )


def capture_head_camera_ros_subprocess(
    out_dir: Path,
    *,
    stem: str = "w2_head",
    rgb_topic: str = "/head_camera/rgb",
    depth_topic: str = "/head_camera/depth",
    info_topic: str = "/head_camera/camera_info",
    timeout_sec: float = 30.0,
) -> RgbdFrame:
    """Capture via ``/usr/bin/python3.12`` then load the saved folder.

    HUG's uv venv is Python 3.10; Jazzy ``rclpy`` only works on 3.12.
    """
    import subprocess
    import sys

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    helper = Path(__file__).resolve().parent / "capture_head_rgbd_ros_main.py"
    py = resolve_ros_python()
    cmd = [
        py,
        str(helper),
        "--out-dir",
        str(out_dir),
        "--stem",
        stem,
        "--rgb-topic",
        rgb_topic,
        "--depth-topic",
        depth_topic,
        "--info-topic",
        info_topic,
        "--timeout",
        str(timeout_sec),
    ]
    print(f"[rgbd_io] ROS capture via {py} (HUG py={sys.version.split()[0]})")
    print(" ", " ".join(cmd))
    import os

    env = os.environ.copy()
    venv = env.pop("VIRTUAL_ENV", None)
    if venv:
        env["PATH"] = ":".join(
            p for p in env.get("PATH", "").split(":") if p and not p.startswith(venv)
        )
    proc = subprocess.run(cmd, check=False, env=env)
    if proc.returncode != 0:
        raise RuntimeError(
            f"ROS capture helper failed (exit {proc.returncode}). "
            "Confirm Isaac is Playing and: ros2 topic hz /head_camera/rgb"
        )
    return load_frame_from_dir(out_dir)


def capture_head_camera_ros(
    *,
    rgb_topic: str = "/head_camera/rgb",
    depth_topic: str = "/head_camera/depth",
    info_topic: str = "/head_camera/camera_info",
    timeout_sec: float = 30.0,
    out_dir: Optional[Path] = None,
) -> RgbdFrame:
    """Block until one RGB + depth + CameraInfo arrive (Isaac ROS2 bridge).

    If the current interpreter cannot load ``rclpy`` (typical: HUG Python 3.10
    vs Jazzy 3.12), automatically spawn the system-python helper and load files.
    """
    if not ros_python_usable():
        import tempfile

        dump = Path(out_dir) if out_dir is not None else Path(
            tempfile.mkdtemp(prefix="w2_head_capture_")
        )
        return capture_head_camera_ros_subprocess(
            dump,
            rgb_topic=rgb_topic,
            depth_topic=depth_topic,
            info_topic=info_topic,
            timeout_sec=timeout_sec,
        )

    import rclpy
    from cv_bridge import CvBridge
    from rclpy.executors import SingleThreadedExecutor
    from sensor_msgs.msg import CameraInfo, Image

    if not rclpy.ok():
        rclpy.init()

    bridge = CvBridge()
    lock = threading.Lock()
    state: dict = {"rgb": None, "depth": None, "K": None, "stamp": 0.0}
    ready = threading.Event()

    node = rclpy.create_node("smoke_head_rgbd_capture")

    def _maybe_ready() -> None:
        if state["rgb"] is not None and state["depth"] is not None and state["K"] is not None:
            ready.set()

    def on_rgb(msg: Image) -> None:
        try:
            bgr = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        except Exception:
            arr = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            if arr.ndim == 2:
                rgb = cv2.cvtColor(arr, cv2.COLOR_GRAY2RGB)
            elif arr.shape[2] == 4:
                rgb = cv2.cvtColor(arr, cv2.COLOR_BGRA2RGB)
            else:
                rgb = arr[:, :, :3]
        with lock:
            state["rgb"] = np.asarray(rgb, dtype=np.uint8)
            state["stamp"] = float(msg.header.stamp.sec) + 1e-9 * float(msg.header.stamp.nanosec)
            _maybe_ready()

    def on_depth(msg: Image) -> None:
        depth = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        depth = np.asarray(depth)
        if depth.dtype == np.uint16:
            depth_m = depth.astype(np.float32) / 1000.0
        else:
            depth_m = depth.astype(np.float32)
        with lock:
            state["depth"] = depth_m
            _maybe_ready()

    def on_info(msg: CameraInfo) -> None:
        K = K_from_fx_fy_cx_cy(msg.k[0], msg.k[4], msg.k[2], msg.k[5])
        with lock:
            state["K"] = K
            _maybe_ready()

    node.create_subscription(Image, rgb_topic, on_rgb, 10)
    node.create_subscription(Image, depth_topic, on_depth, 10)
    node.create_subscription(CameraInfo, info_topic, on_info, 1)

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        if not ready.wait(timeout_sec):
            raise TimeoutError(
                f"timeout waiting for {rgb_topic}, {depth_topic}, {info_topic} "
                f"(got rgb={state['rgb'] is not None}, depth={state['depth'] is not None}, "
                f"info={state['K'] is not None})"
            )
        with lock:
            rgb = state["rgb"].copy()
            depth_m = state["depth"].copy()
            K = state["K"].copy()
            stamp = state["stamp"]
        if depth_m.shape[:2] != rgb.shape[:2]:
            raise ValueError(f"rgb {rgb.shape[:2]} != depth {depth_m.shape[:2]}")
        return RgbdFrame(rgb=rgb, depth_m=depth_m, K=K, stamp_sec=stamp)
    finally:
        executor.shutdown()
        node.destroy_node()


def _find_one(root: Path, needles: tuple[str, ...], exts: tuple[str, ...]) -> Path:
    cands: list[Path] = []
    for p in root.iterdir():
        if not p.is_file():
            continue
        name = p.name.lower()
        if p.suffix.lower() not in exts:
            continue
        if any(n in name for n in needles):
            cands.append(p)
    if not cands:
        raise FileNotFoundError(f"no file matching {needles}+{exts} under {root}")
    return sorted(cands)[0]


def _load_depth_any(path: Path) -> np.ndarray:
    if path.suffix.lower() == ".npy":
        d = np.load(path)
        if d.dtype == np.uint16:
            return d.astype(np.float32) / 1000.0
        return d.astype(np.float32)
    img = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if img is None:
        raise FileNotFoundError(path)
    if img.dtype == np.uint16:
        return img.astype(np.float32) / 1000.0
    return img.astype(np.float32)


def _load_K(path: Path) -> np.ndarray:
    if path.suffix == ".npy":
        vals = np.asarray(np.load(path), dtype=np.float64).ravel()
    elif path.suffix == ".json":
        data = json.loads(path.read_text())
        vals = np.asarray(
            data["K"] if isinstance(data, dict) and "K" in data else data, dtype=np.float64
        ).ravel()
    else:
        vals = np.loadtxt(path).astype(np.float64).ravel()
    if vals.size == 4:
        return K_from_fx_fy_cx_cy(*vals.tolist())
    if vals.size == 9:
        return vals.reshape(3, 3)
    raise ValueError(f"{path}: expected 4 or 9 numbers, got {vals.size}")
