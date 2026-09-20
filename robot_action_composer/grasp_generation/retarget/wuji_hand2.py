"""Wuji Hand2 retarget via ``wuji_sdk.RetargetSession`` (right-hand smoke first)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal, Sequence

import numpy as np

# Matches fa_w2_ws wuji_hand2_protocol.hpp kJointNameSuffixes (SDK index 0..19).
WUJI_HAND2_JOINT_SUFFIXES: tuple[str, ...] = (
    "thumb_cmc_flex",
    "thumb_cmc_abd",
    "thumb_mcp",
    "thumb_ip",
    "index_finger_mcp_flex",
    "index_finger_mcp_abd",
    "index_finger_pip",
    "index_finger_dip",
    "middle_finger_mcp_flex",
    "middle_finger_mcp_abd",
    "middle_finger_pip",
    "middle_finger_dip",
    "ring_mcp_flex",
    "ring_mcp_abd",
    "ring_pip",
    "ring_dip",
    "pinky_mcp_flex",
    "pinky_mcp_abd",
    "pinky_pip",
    "pinky_dip",
)

Side = Literal["left", "right"]


@dataclass
class RetargetResult:
    q: np.ndarray  # (20,) float32
    side: Side
    joint_suffixes: tuple[str, ...] = WUJI_HAND2_JOINT_SUFFIXES
    backend: str = "wuji_sdk"

    def as_list(self) -> list[float]:
        return [float(x) for x in np.asarray(self.q, dtype=np.float64).reshape(-1)]

    def named_pairs(self) -> list[tuple[str, float]]:
        q = self.as_list()
        return list(zip(self.joint_suffixes, q, strict=True))


class WujiHand2Retarget:
    """Thin wrapper around ``wuji_sdk.RetargetSession`` for Hand2."""

    def __init__(self, side: Side = "right") -> None:
        if side not in ("left", "right"):
            raise ValueError(f"side must be left|right, got {side!r}")
        self.side: Side = side
        self._session = None

    def _ensure_session(self):
        if self._session is not None:
            return self._session
        try:
            from wuji_sdk import Handedness, HandModel, RetargetSession
        except ImportError as e:  # pragma: no cover
            raise ImportError(
                "wuji-sdk is required for hand_profile=wuji_hand2. "
                "Install: pip install 'wuji-sdk>=0.10.0' (Linux x86_64/aarch64). "
                "See docs/WUJI_RETARGETING.md"
            ) from e
        handed = Handedness.Right if self.side == "right" else Handedness.Left
        self._session = RetargetSession.for_hand(HandModel.WujiHand2, side=handed)
        return self._session

    def reset(self) -> None:
        if self._session is not None:
            self._session.reset()

    def step(self, keypoints_21x3: np.ndarray, *, reset: bool = False) -> RetargetResult:
        kp = np.asarray(keypoints_21x3, dtype=np.float32)
        if kp.shape != (21, 3):
            raise ValueError(f"keypoints expected (21, 3), got {kp.shape}")
        session = self._ensure_session()
        if reset:
            session.reset()
        q = np.asarray(session.step(kp), dtype=np.float32).reshape(-1)
        if q.shape != (20,):
            raise RuntimeError(f"RetargetSession returned shape {q.shape}, expected (20,)")
        if not np.isfinite(q).all():
            raise RuntimeError("RetargetSession returned non-finite q")
        return RetargetResult(q=q, side=self.side)


def retarget_landmarks(
    landmarks_3d: np.ndarray,
    *,
    side: Side = "right",
    keypoint_rpy_deg: Sequence[float] | None = None,
    reset: bool = True,
    retargeter: WujiHand2Retarget | None = None,
) -> RetargetResult:
    """Convenience: landmarks → preprocess → session.step."""
    from .keypoints import preprocess_keypoints

    kp = preprocess_keypoints(landmarks_3d, keypoint_rpy_deg=keypoint_rpy_deg)
    eng = retargeter or WujiHand2Retarget(side=side)
    return eng.step(kp, reset=reset)
