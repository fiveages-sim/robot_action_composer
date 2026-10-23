#!/usr/bin/env python3
"""Capture one W2 head RGB-D frame via ROS2 (system Python 3.12 / Jazzy).

HUG runs in a Python 3.10 uv venv; ROS Jazzy ``rclpy`` is built for 3.12 only.
This script must be launched with the system interpreter, e.g.:

    /usr/bin/python3.12 capture_head_rgbd_ros_main.py --out-dir /tmp/w2_head

It is intentionally stdlib + ROS + OpenCV (no hug / composer imports).
"""

from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from pathlib import Path


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--stem", type=str, default="w2_head")
    parser.add_argument("--rgb-topic", default="/head_camera/rgb")
    parser.add_argument("--depth-topic", default="/head_camera/depth")
    parser.add_argument("--info-topic", default="/head_camera/camera_info")
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args(argv)

    try:
        import cv2
        import numpy as np
        import rclpy
        from cv_bridge import CvBridge
        from rclpy.executors import SingleThreadedExecutor
        from sensor_msgs.msg import CameraInfo, Image
    except Exception as e:
        print(
            "FATAL: need system Python with ROS Jazzy rclpy + cv_bridge + opencv.\n"
            f"  interpreter={sys.executable} version={sys.version.split()[0]}\n"
            f"  error={e}",
            file=sys.stderr,
        )
        return 2

    if not rclpy.ok():
        rclpy.init()

    bridge = CvBridge()
    lock = threading.Lock()
    state: dict = {"rgb": None, "depth": None, "K": None, "stamp": 0.0}
    ready = threading.Event()

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
            state["stamp"] = float(msg.header.stamp.sec) + 1e-9 * float(
                msg.header.stamp.nanosec
            )
            _maybe_ready()

    def on_depth(msg: Image) -> None:
        depth = np.asarray(bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough"))
        if depth.dtype == np.uint16:
            depth_m = depth.astype(np.float32) / 1000.0
        else:
            depth_m = depth.astype(np.float32)
        with lock:
            state["depth"] = depth_m
            _maybe_ready()

    def on_info(msg: CameraInfo) -> None:
        fx, fy, cx, cy = msg.k[0], msg.k[4], msg.k[2], msg.k[5]
        K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)
        with lock:
            state["K"] = K
            _maybe_ready()

    node = rclpy.create_node("capture_head_rgbd_ros")
    node.create_subscription(Image, args.rgb_topic, on_rgb, 10)
    node.create_subscription(Image, args.depth_topic, on_depth, 10)
    node.create_subscription(CameraInfo, args.info_topic, on_info, 1)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    print(
        f"Waiting for {args.rgb_topic}, {args.depth_topic}, {args.info_topic} "
        f"(timeout={args.timeout}s) ..."
    )
    try:
        if not ready.wait(args.timeout):
            print(
                "timeout: "
                f"rgb={state['rgb'] is not None} depth={state['depth'] is not None} "
                f"info={state['K'] is not None}",
                file=sys.stderr,
            )
            return 1
        with lock:
            rgb = state["rgb"].copy()
            depth_m = state["depth"].copy()
            K = state["K"].copy()
            stamp = state["stamp"]
    finally:
        executor.shutdown()
        node.destroy_node()

    if depth_m.shape[:2] != rgb.shape[:2]:
        print(f"rgb {rgb.shape[:2]} != depth {depth_m.shape[:2]}", file=sys.stderr)
        return 1

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = args.stem

    # depth as uint16 mm for HUG prepare_inputs
    d = np.where(np.isfinite(depth_m) & (depth_m > 0) & (depth_m < 10.0), depth_m, 0.0)
    depth_mm = np.clip(np.rint(d * 1000.0), 0, 65535).astype(np.uint16)

    cv2.imwrite(str(out_dir / f"{stem}_rgb.png"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    cv2.imwrite(str(out_dir / f"{stem}_depth.png"), depth_mm)
    fx, fy, cx, cy = float(K[0, 0]), float(K[1, 1]), float(K[0, 2]), float(K[1, 2])
    (out_dir / f"{stem}_intrinsics.txt").write_text(f"{fx} {fy} {cx} {cy}\n")
    (out_dir / f"{stem}_meta.json").write_text(
        json.dumps(
            {
                "stamp_sec": stamp,
                "shape": list(rgb.shape),
                "K": K.tolist(),
                "python": sys.version.split()[0],
                "captured_at": time.time(),
            },
            indent=2,
        )
    )
    print(f"OK wrote {out_dir}/{stem}_{{rgb,depth,intrinsics}}")
    print(
        f"frame rgb={rgb.shape} depth_valid_min="
        f"{float(depth_m[depth_m > 0].min()) if (depth_m > 0).any() else 0:.3f} "
        f"fx={fx:.1f}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
