#!/usr/bin/env python3
"""Generic dataset record runner for IsaacSim tasks."""

from __future__ import annotations

import pickle
import queue
import signal
import sys
import threading
import time
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, Optional

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import CameraInfo, Image

from ros2_robot_interface import FSM_HOLD, FSM_OCS2  # pyright: ignore[reportMissingImports]

from robot_action_composer.motion_generation.sequence.cartesian_stages import StageTarget  # pyright: ignore[reportMissingImports]

from lerobot.datasets.lerobot_dataset import LeRobotDataset  # pyright: ignore[reportMissingImports]
from lerobot_robot_ros2 import (  # pyright: ignore[reportMissingImports]
    ROS2Robot,
    ROS2RobotConfig,
)
from robot_action_composer.dataset_recording.recorder import DatasetRecorder  # pyright: ignore[reportMissingImports]
import robot_action_composer.task_runtime.skills  # noqa: F401 - register skills

from robot_action_composer.motion_generation.tasks.movej_return import (  # pyright: ignore[reportMissingImports]
    capture_initial_arm_joint_positions,
    capture_initial_body_joint_positions,
    movej_return_to_initial_state,
)
from robot_action_composer.isaac_sim import SimTimeHelper  # pyright: ignore[reportMissingImports]
from robot_action_composer.task_runtime.runner import (  # pyright: ignore[reportMissingImports]
    _execute_block,
    _execute_parallel,
    build_queue_runtime_context,
    reset_queue_task_environment,
)
from robot_action_composer.task_runtime.config.merged import MergedQueueConfig  # pyright: ignore[reportMissingImports]
from robot_action_composer.task_runtime.types import BlockSpec, ParallelSpec, block_spec_from_mapping  # pyright: ignore[reportMissingImports]


@dataclass(frozen=True)
class RecordConfig:
    fps: int = 30
    camera_info_timeout: float = 10.0
    task_name: str = "pick_place"
    enable_keypoint_pcd: bool = False
    include_depth_feature: bool = False
    image_writer_processes: int = 0
    image_writer_threads: int = 8
    video_encoding_batch_size: int = 5
    switch_to_hold_after_episode: bool = False
    async_episode_save: bool = False
    episode_save_queue_size: int = 0


DEFAULT_RECORD_CFG = RecordConfig()


class RawDepthListener:
    def __init__(self, depth_topic: str) -> None:
        self._latest_depth: Optional[np.ndarray] = None
        self._lock = threading.Lock()
        self._bridge = CvBridge()
        if not rclpy.ok():
            rclpy.init()
        self._node = rclpy.create_node("record_depth_listener")
        self._sub = self._node.create_subscription(Image, depth_topic, self._depth_callback, 10)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()

    def _depth_callback(self, msg: Image) -> None:
        try:
            depth_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            with self._lock:
                self._latest_depth = depth_image.astype(np.float32)
        except Exception:
            return

    def get_latest_depth(self) -> Optional[np.ndarray]:
        with self._lock:
            return self._latest_depth.copy() if self._latest_depth is not None else None

    def shutdown(self) -> None:
        self._executor.shutdown()
        self._node.destroy_node()


class DepthCameraInfoListener:
    def __init__(self, info_topic: str) -> None:
        self._intrinsics: Optional[tuple[float, float, float, float]] = None
        self._lock = threading.Lock()
        self._ready = threading.Event()
        if not rclpy.ok():
            rclpy.init()
        self._node = rclpy.create_node("record_camera_info_listener")
        self._sub = self._node.create_subscription(CameraInfo, info_topic, self._info_callback, 1)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()

    def _info_callback(self, msg: CameraInfo) -> None:
        with self._lock:
            self._intrinsics = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])
            self._ready.set()

    def wait_for_intrinsics(self, timeout: float) -> Optional[tuple[float, float, float, float]]:
        if not self._ready.wait(timeout):
            return None
        with self._lock:
            return self._intrinsics

    def shutdown(self) -> None:
        self._executor.shutdown()
        self._node.destroy_node()


def _build_robot_config(*, robot_cfg: Any, record_cfg: Any) -> ROS2RobotConfig:
    if not getattr(robot_cfg, "cameras", None):
        raise ValueError("robot_cfg.cameras must contain at least one camera definition")
    camera_cfg = {name: replace(cam, fps=record_cfg.fps) for name, cam in robot_cfg.cameras.items()}
    robot_id = getattr(robot_cfg, "robot_id", None)
    if not robot_id:
        robot_id = getattr(robot_cfg, "ROBOT_KEY", None) or robot_cfg.__class__.__name__.lower()
    return ROS2RobotConfig(
        id=str(robot_id),
        cameras=camera_cfg,
        ros2_interface=robot_cfg.ros2_interface,
        gripper_control_mode=robot_cfg.gripper_control_mode,
    )


def _save_pointcloud_frame(
    dataset_dir: Path,
    episode_index: int,
    frame_index: int,
    depth_image: Optional[np.ndarray],
    intrinsics: Optional[tuple[float, float, float, float]],
    rgb_image: Optional[np.ndarray] = None,
    pointcloud_key: str = "observation.points.zed_pcd",
) -> None:
    pcd_dir = dataset_dir / "points" / pointcloud_key / "chunk-000" / f"episode_{episode_index:06d}"
    pcd_dir.mkdir(parents=True, exist_ok=True)
    if depth_image is None:
        depth_image = np.zeros((1, 1), dtype=np.float32)
    if intrinsics is None:
        h, w = depth_image.shape[:2]
        intrinsics = (max(w, 1), max(h, 1), w / 2.0, h / 2.0)
    fx, fy, cx, cy = intrinsics
    if depth_image.ndim == 3:
        depth_image = depth_image[..., 0]
    h, w = depth_image.shape
    uu, vv = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))
    z = depth_image.astype(np.float32)
    x = (uu - cx) * z / (fx if fx != 0 else 1.0)
    y = (vv - cy) * z / (fy if fy != 0 else 1.0)
    pcd_xyz = np.stack((x, y, z), axis=-1).astype(np.float32)
    pcd = pcd_xyz
    if rgb_image is not None and rgb_image.shape[:2] == pcd_xyz.shape[:2]:
        rgb = rgb_image.astype(np.float32)
        if rgb.max() > 1.0:
            rgb = rgb / 255.0
        pcd = np.concatenate([pcd_xyz, np.clip(rgb, 0.0, 1.0)], axis=-1).astype(np.float32)
    with open(pcd_dir / f"frame_{frame_index:06d}.pkl", "wb") as f:
        pickle.dump(pcd, f)


def _flush_pending_video_batches(dataset: Any) -> None:
    """Encode any remaining episodes when batch video encoding is enabled."""
    try:
        batch_size = int(getattr(dataset, "batch_encoding_size", 1))
        pending = int(getattr(dataset, "episodes_since_last_encoding", 0))
        total_eps = int(getattr(dataset, "num_episodes", 0))
    except Exception:
        return
    if batch_size <= 1 or pending <= 0 or total_eps <= 0:
        return
    start_ep = max(0, total_eps - pending)
    end_ep = total_eps
    if hasattr(dataset, "_batch_save_episode_video"):
        dataset._batch_save_episode_video(start_ep, end_ep)
        dataset.episodes_since_last_encoding = 0


def _make_recorder_stage_start_bridge(
    recorder: DatasetRecorder,
) -> Any:
    """Bridge StageTarget callback to recorder's ActionDict callback."""
    def on_stage_start(stage_name: str, target: StageTarget) -> None:
        action_dict = target.to_action_dict()
        recorder.on_stage_start(stage_name, action_dict)
    return on_stage_start


def run_recording(
    *,
    robot_cfg: Any,
    task_cfg: Any,
    record_cfg: Any,
    loops: int,
    enable_keypoint_pcd: bool,
    enable_manual_episode_check: bool,
    task_name: str | None = None,
    use_stamped: bool = True,
    merged_task_queue: list[Any] | None = None,
) -> None:
    resolved_task_name = str(task_name if task_name else getattr(record_cfg, "task_name", "queue_task"))
    frame_id = robot_cfg.base_link_entity_path.rsplit("/", 1)[-1] if use_stamped else "arm_base"
    robot = ROS2Robot(_build_robot_config(robot_cfg=robot_cfg, record_cfg=record_cfg))
    depth_required = bool(enable_keypoint_pcd)
    default_cam_name = next(iter(robot_cfg.cameras.keys()), "")
    depth_cam_name = getattr(robot_cfg, "depth_camera_name", default_cam_name)
    depth_cam = robot_cfg.cameras.get(depth_cam_name) if getattr(robot_cfg, "cameras", None) else None
    depth_topic = getattr(depth_cam, "depth_topic_name", None) if depth_cam is not None else None
    depth_info_topic = getattr(robot_cfg, "depth_info_topic", None)
    depth_listener: RawDepthListener | None = None
    cam_info_listener: DepthCameraInfoListener | None = None
    if depth_required:
        if depth_topic is None:
            raise ValueError("Depth recording requested, but no camera depth topic is configured")
        if depth_info_topic is None:
            raise ValueError("Depth recording requested, but depth_info_topic is not configured")
        depth_listener = RawDepthListener(depth_topic)
        cam_info_listener = DepthCameraInfoListener(depth_info_topic)
    sim_time = SimTimeHelper()

    def shutdown_handler(sig, frame) -> None:  # type: ignore[override]
        try:
            if depth_listener is not None:
                depth_listener.shutdown()
            if cam_info_listener is not None:
                cam_info_listener.shutdown()
            if robot.is_connected:
                robot.disconnect()
        finally:
            sim_time.shutdown()
            sys.exit(0)

    signal.signal(signal.SIGINT, shutdown_handler)

    try:
        robot.connect()
        robot.ros2_interface.send_fsm_command(FSM_HOLD)
        sim_time.sleep(robot_cfg.fsm_switch_delay)
        robot.ros2_interface.send_fsm_command(FSM_OCS2)
        sim_time.sleep(robot_cfg.fsm_switch_delay)
        in_ocs2 = True

        intrinsics = (
            cam_info_listener.wait_for_intrinsics(timeout=record_cfg.camera_info_timeout)
            if cam_info_listener is not None
            else None
        )
        features, action_names, include_gripper = DatasetRecorder.build_dataset_features(
            robot, include_depth_feature=record_cfg.include_depth_feature
        )
        out_root = (Path.cwd() / "lerobot_dataset" / f"grasp_record_{int(time.time())}").resolve()
        out_root.parent.mkdir(parents=True, exist_ok=True)
        dataset = LeRobotDataset.create(
            repo_id=str(out_root),
            fps=record_cfg.fps,
            features=features,
            robot_type=robot.name,
            use_videos=True,
            image_writer_processes=record_cfg.image_writer_processes,
            image_writer_threads=record_cfg.image_writer_threads,
            batch_encoding_size=record_cfg.video_encoding_batch_size,
        )
        write_queue: queue.Queue[tuple[list[dict[str, object]], str] | None] | None = None
        writer_thread: threading.Thread | None = None
        writer_error: list[BaseException] = []
        writer_lock = threading.Lock()

        def _raise_writer_error_if_any() -> None:
            with writer_lock:
                if writer_error:
                    raise RuntimeError("Background episode save failed") from writer_error[0]

        def _append_episode_records(records_to_save: list[dict[str, object]], task_to_save: str) -> None:
            DatasetRecorder.append_episode_to_dataset(
                dataset=dataset,
                records=records_to_save,
                action_names=action_names,
                include_gripper=include_gripper,
                task_name=task_to_save,
            )

        def _writer_loop() -> None:
            assert write_queue is not None
            while True:
                item = write_queue.get()
                try:
                    if item is None:
                        return
                    records_to_save, task_to_save = item
                    _append_episode_records(records_to_save, task_to_save)
                except BaseException as exc:  # noqa: BLE001
                    with writer_lock:
                        if not writer_error:
                            writer_error.append(exc)
                    return
                finally:
                    write_queue.task_done()

        if bool(getattr(record_cfg, "async_episode_save", False)):
            queue_size = int(getattr(record_cfg, "episode_save_queue_size", 0))
            write_queue = queue.Queue(maxsize=max(0, queue_size))
            writer_thread = threading.Thread(target=_writer_loop, daemon=True)
            writer_thread.start()

        if not merged_task_queue:
            raise ValueError(
                "Recording requires merged_task_queue (task_queue merged with skill_defaults / scene)"
            )
        if not isinstance(task_cfg, MergedQueueConfig):
            raise TypeError(
                "Recording expects task_cfg: MergedQueueConfig "
                f"(from build_merged_queue_from_flat), got {type(task_cfg).__name__}"
            )
        task_runtime: MergedQueueConfig = task_cfg
        pick_specs: list[BlockSpec | ParallelSpec] = [
            block_spec_from_mapping(b) if isinstance(b, dict) else b for b in merged_task_queue
        ]

        left_initial_joint_positions, right_initial_joint_positions = capture_initial_arm_joint_positions(
            robot.ros2_interface
        )
        body_initial_joint_positions = capture_initial_body_joint_positions(robot.ros2_interface)

        kept_episode = 0
        while kept_episode < loops:
            _raise_writer_error_if_any()
            if not in_ocs2:
                print("[FSM] Re-enter OCS2 before next episode")
                robot.ros2_interface.send_fsm_command(FSM_OCS2)
                sim_time.sleep(robot_cfg.fsm_switch_delay)
                in_ocs2 = True
            episode_index = kept_episode

            reset_queue_task_environment(
                specs=pick_specs,
                runtime=task_runtime,
                robot_cfg=robot_cfg,
                sim_time=sim_time,
            )
            ctx = build_queue_runtime_context(
                interface=robot.ros2_interface,
                robot_cfg=robot_cfg,
                sim_time=sim_time,
                runtime=task_runtime,
                use_stamped=use_stamped,
                left_initial_joint_positions=left_initial_joint_positions,
                right_initial_joint_positions=right_initial_joint_positions,
                body_initial_joint_positions=body_initial_joint_positions,
            )
            recorder = DatasetRecorder(
                robot=robot,
                sim_time=sim_time,
                task_cfg=task_runtime.single_arm,
                dataset_features=features,
                dataset_root=out_root,
                episode_index=episode_index,
                intrinsics=intrinsics,
                depth_listener=depth_listener,
                record_cfg=record_cfg,
                enable_keypoint_pcd=enable_keypoint_pcd,
                camera_name=depth_cam_name,
                save_pointcloud_frame_fn=_save_pointcloud_frame,
            )
            recorder.start_sequence()
            stage_start_bridge = _make_recorder_stage_start_bridge(recorder)
            exec_extras = {
                "on_stage_start": stage_start_bridge,
                "on_stage_poll": recorder.on_stage_poll,
            }
            for idx, spec in enumerate(pick_specs):
                lbl = f"block {idx + 1}/{len(pick_specs)}"
                if isinstance(spec, ParallelSpec):
                    _execute_parallel(
                        ctx,
                        spec,
                        runner_prefix="RecordQ",
                        idx_label=lbl,
                        execute_stage_kwargs=exec_extras,
                    )
                else:
                    _execute_block(
                        ctx,
                        spec,
                        runner_prefix="RecordQ",
                        idx_label=lbl,
                        execute_stage_kwargs=exec_extras,
                    )
            episode_records = recorder.finish_sequence()
            if not episode_records:
                raise RuntimeError("No frames captured during sequence.")
            try:
                moved_by_movej = False
                for retry_idx in range(2):
                    moved_by_movej = movej_return_to_initial_state(
                        interface=robot.ros2_interface,
                        left_initial_positions=left_initial_joint_positions,
                        right_initial_positions=right_initial_joint_positions,
                        body_initial_positions=body_initial_joint_positions,
                        arrival_timeout=robot_cfg.arrival_timeout,
                        arrival_poll=robot_cfg.arrival_poll,
                        sim_time=sim_time,
                    )
                    if moved_by_movej:
                        break
                    if retry_idx == 0:
                        print("[MoveJ] Retry once: return-to-initial was not successful.")
                if moved_by_movej:
                    # MoveJ changes FSM state; force OCS2 re-enter before next episode.
                    in_ocs2 = False
            except Exception as err:
                print(f"[WARN] MoveJ return-to-initial failed: {err}")
            if record_cfg.switch_to_hold_after_episode and in_ocs2:
                robot.ros2_interface.send_fsm_command(FSM_HOLD)
                in_ocs2 = False

            keep_episode = True
            if enable_manual_episode_check:
                delete_input = input("Delete this episode data? [y/N]: ").strip().lower()
                keep_episode = delete_input not in {"y", "yes"}
                if not keep_episode:
                    points_root = out_root / "points"
                    if points_root.exists():
                        target_suffix = f"episode_{episode_index:06d}"
                        for path in points_root.rglob(target_suffix):
                            if path.is_dir():
                                import shutil
                                shutil.rmtree(path, ignore_errors=True)
            if keep_episode:
                if write_queue is not None:
                    write_queue.put((episode_records, resolved_task_name))
                else:
                    _append_episode_records(episode_records, resolved_task_name)
                kept_episode += 1
        if write_queue is not None:
            write_queue.put(None)
            write_queue.join()
            if writer_thread is not None:
                writer_thread.join(timeout=5.0)
            _raise_writer_error_if_any()
        _flush_pending_video_batches(dataset)
    finally:
        try:
            if robot.is_connected:
                robot.ros2_interface.send_fsm_command(FSM_HOLD)
        except Exception:
            pass
        if depth_listener is not None:
            depth_listener.shutdown()
        if cam_info_listener is not None:
            cam_info_listener.shutdown()
        sim_time.shutdown()
        if robot.is_connected:
            robot.disconnect()
