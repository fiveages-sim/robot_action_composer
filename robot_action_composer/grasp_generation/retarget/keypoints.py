"""MANO / HUG landmarks → MediaPipe-shaped (21, 3) for Wuji retarget.

``wuji-sdk`` ``RetargetSession.step`` already wrist-origins and canonicalizes
internally (same q whether raw camera landmarks or wrist-subtracted). We still
expose an optional fixed ``keypoint_rpy_deg`` for later TF tuning without
touching HUG.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

from ..se3 import R_from_rpy_deg


def load_landmarks_from_proposal(path: Path | str) -> np.ndarray:
    """Load ``landmarks_3d`` from HUG ``proposal.json`` → (21, 3) float64."""
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    return landmarks_from_mapping(data)


def landmarks_from_mapping(data: Mapping[str, Any]) -> np.ndarray:
    if "landmarks_3d" not in data:
        raise KeyError("proposal missing landmarks_3d")
    lm = np.asarray(data["landmarks_3d"], dtype=np.float64)
    if lm.shape != (21, 3):
        raise ValueError(f"landmarks_3d expected (21, 3), got {lm.shape}")
    if not np.isfinite(lm).all():
        raise ValueError("landmarks_3d contains NaN/Inf")
    return lm


def preprocess_keypoints(
    landmarks_3d: np.ndarray,
    *,
    keypoint_rpy_deg: Sequence[float] | None = None,
    wrist_origin: bool = True,
) -> np.ndarray:
    """Prepare keypoints for ``RetargetSession.step``.

    Returns float32 (21, 3). Default: optional wrist subtraction + optional
    fixed rotation about wrist (for axis tuning). SDK still re-normalizes.
    """
    kp = np.asarray(landmarks_3d, dtype=np.float64).reshape(21, 3).copy()
    if wrist_origin:
        kp = kp - kp[0:1, :]
    rpy = list(keypoint_rpy_deg or []) + [0.0, 0.0, 0.0]
    if any(abs(float(v)) > 1e-9 for v in rpy[:3]):
        R = R_from_rpy_deg(float(rpy[0]), float(rpy[1]), float(rpy[2]))
        kp = (kp @ R.T).astype(np.float64)
        kp[0] = 0.0
    out = kp.astype(np.float32)
    if not np.isfinite(out).all():
        raise ValueError("preprocessed keypoints contain NaN/Inf")
    return out
