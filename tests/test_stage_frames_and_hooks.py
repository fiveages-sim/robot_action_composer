"""Unit tests for stage frame conversion, hold-current ordering and skill imports."""

from __future__ import annotations

from types import SimpleNamespace
from typing import Any

from geometry_msgs.msg import Pose

from robot_action_composer.motion_generation.sequence.cartesian_stages import (
    ArmSide,
    ArmTarget,
    SendMode,
    StageTarget,
    _apply_hold_current_pose,
    execute_stage_sequence,
)
from robot_action_composer.task_runtime.placement import stages_in_command_ee_frame


def _pose(x: float, y: float, z: float, *, w: float = 1.0) -> Pose:
    p = Pose()
    p.position.x = x
    p.position.y = y
    p.position.z = z
    p.orientation.w = w
    return p


def _xyz(p: Pose) -> tuple[float, float, float]:
    return (p.position.x, p.position.y, p.position.z)


class _FakeArmHandler:
    def __init__(self, pose: Pose, frame_id: str = "world") -> None:
        self._pose = pose
        self._frame_id = frame_id
        self.sent: list[tuple[str, tuple[float, float, float]]] = []

    def get_pose(self) -> Pose:
        return self._pose

    def get_frame_id(self) -> str:
        return self._frame_id

    def send_target_stamped(self, frame_id: str, pose: Pose) -> None:
        self.sent.append((frame_id, _xyz(pose)))

    def send_target(self, pose: Pose) -> None:
        self.sent.append(("", _xyz(pose)))


class _FakeInterface:
    def __init__(self, left: _FakeArmHandler | None = None, right: _FakeArmHandler | None = None) -> None:
        self.left_arm_handler = left
        self.right_arm_handler = right
        self.left_gripper_handler = None
        self.right_gripper_handler = None

    def wait_until_arrive(self, **_kwargs: Any) -> None:  # pragma: no cover - not used
        raise AssertionError("skip_arrival_check should avoid this")


def test_apply_hold_current_pose_uses_feedback_pose_and_frame() -> None:
    handler = _FakeArmHandler(_pose(1.0, 2.0, 3.0), frame_id="world")
    iface = _FakeInterface(left=handler)
    stage = StageTarget(
        "Grasp",
        left=ArmTarget(pose=_pose(9.0, 9.0, 9.0), gripper=0.0),
        hold_current_pose=True,
    )

    _apply_hold_current_pose(iface, stage)

    assert stage.left is not None
    assert _xyz(stage.left.pose) == (1.0, 2.0, 3.0)
    assert stage.frame_id == "world"


def test_execute_records_pose_after_hold_override() -> None:
    handler = _FakeArmHandler(_pose(1.0, 2.0, 3.0), frame_id="world")
    iface = _FakeInterface(left=handler)
    stage = StageTarget(
        "Grasp",
        left=ArmTarget(pose=_pose(9.0, 9.0, 9.0), gripper=0.0),
        hold_current_pose=True,
        skip_arrival_check=True,
        skip_gripper_command=True,
    )
    recorded: list[tuple[float, float, float]] = []

    execute_stage_sequence(
        interface=iface,
        sequence=[stage],
        send_mode=SendMode.STAMPED,
        frame_id="world",
        arrival_timeout=1.0,
        arrival_poll=0.01,
        time_now_fn=lambda: 0.0,
        sleep_fn=lambda _s: None,
        gripper_action_wait=0.0,
        on_stage_start=lambda _name, st: recorded.append(_xyz(st.left.pose)),
    )

    assert recorded == [(1.0, 2.0, 3.0)]
    assert handler.sent == [("world", (1.0, 2.0, 3.0))]


def test_stages_in_command_ee_frame_converts_selected_arm_only() -> None:
    class _Convert:
        def convert_ee_frame_pose(self, pose: Pose, ee_frame: str, command_frame: str, timeout: Any = None) -> Pose:
            out = _pose(pose.position.x + 0.1, pose.position.y, pose.position.z)
            return out

    ctx = SimpleNamespace(
        interface=_Convert(),
        robot_cfg=SimpleNamespace(ee_target_frame="{arm}_eef", ee_command_frame="{arm}_tcp"),
    )
    stage = StageTarget(
        "S",
        left=ArmTarget(pose=_pose(1.0, 0.0, 0.0), gripper=0.0),
        right=ArmTarget(pose=_pose(2.0, 0.0, 0.0), gripper=0.0),
    )

    out = stages_in_command_ee_frame(
        ctx,  # type: ignore[arg-type]
        [stage],
        arm_side=ArmSide.RIGHT,
        ee_target_frame=None,
        ee_command_frame=None,
        tf_lookup_timeout=None,
        label="test",
    )

    assert out[0].left is not None and _xyz(out[0].left.pose) == (1.0, 0.0, 0.0)
    assert out[0].right is not None and _xyz(out[0].right.pose) == (2.1, 0.0, 0.0)


def test_stages_in_command_ee_frame_empty_target_disables_conversion() -> None:
    class _Convert:
        def convert_ee_frame_pose(self, pose: Pose, ee_frame: str, command_frame: str, timeout: Any = None) -> Pose:
            raise AssertionError("conversion should be disabled")

    ctx = SimpleNamespace(
        interface=_Convert(),
        robot_cfg=SimpleNamespace(ee_target_frame="{arm}_eef", ee_command_frame="{arm}_tcp"),
    )
    stage = StageTarget("S", left=ArmTarget(pose=_pose(1.0, 0.0, 0.0), gripper=0.0))

    out = stages_in_command_ee_frame(
        ctx,  # type: ignore[arg-type]
        [stage],
        arm_side=ArmSide.LEFT,
        ee_target_frame="",
        ee_command_frame=None,
        tf_lookup_timeout=None,
        label="test",
    )

    assert out[0].left is not None and _xyz(out[0].left.pose) == (1.0, 0.0, 0.0)


def test_builtin_skills_import_and_register() -> None:
    # Import side effect populates the registry; catches missing placement exports.
    import robot_action_composer.task_runtime.skills  # noqa: F401
    from robot_action_composer.task_runtime.registry import get_skill

    assert get_skill("single_arm.insert") is not None
    assert get_skill("single_arm.place") is not None


def test_skill_defaults_reach_insert_params() -> None:
    from robot_action_composer.task_runtime.merge.block_params import (
        _skill_defaults_for_block,
    )

    defaults = {
        "single_arm.insert": {"insert_depth": 0.06, "insert_axis": "-z"},
        "insert_tube": {"insert_depth": 0.08},
    }
    merged = _skill_defaults_for_block("single_arm.insert", "insert_tube", defaults)
    assert merged == {"insert_depth": 0.08, "insert_axis": "-z"}
