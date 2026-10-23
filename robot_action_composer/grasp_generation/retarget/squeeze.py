"""Post-retarget finger squeeze / open (wrist unchanged).

Squeeze keeps the retarget shape and **amplifies** each joint's existing
offset from the open pose (inward trend), instead of slamming toward
joint limits. That avoids:
- thumb flipping "open" when absolute oppose/flex targets fight the MANO shape
- over-closing into the table on small objects
"""

from __future__ import annotations

from typing import Sequence

import numpy as np

from .wuji_hand2 import WUJI_HAND2_JOINT_SUFFIXES

THUMB_CMC_FLEX = 0
THUMB_CMC_ABD = 1

# Amplify these: thumb flex/abd + all finger flex. Skip finger mcp_abd (splay).
AMPLIFY_JOINT_INDICES: tuple[int, ...] = (
    0,  # thumb_cmc_flex
    1,  # thumb_cmc_abd — keep / deepen opposition from retarget
    2,  # thumb_mcp
    3,  # thumb_ip
    4,  # index mcp_flex
    6,
    7,
    8,  # middle
    10,
    11,
    12,  # ring
    14,
    15,
    16,  # pinky
    18,
    19,
)

# Near-open rest pose (rad) — also the reference for amplify deltas.
OPEN_HAND_Q = np.zeros(20, dtype=np.float32)

_DEFAULT_UPPER = np.array(
    [
        1.291,
        0.698,
        1.570,
        1.570,
        1.570,
        0.698,
        2.094,
        1.570,
        1.570,
        0.698,
        2.094,
        1.570,
        1.570,
        0.698,
        2.094,
        1.570,
        1.570,
        0.698,
        2.094,
        1.570,
    ],
    dtype=np.float64,
)
_DEFAULT_LOWER = np.array(
    [
        -1.187,
        -1.484,
        -1.047,
        -1.047,
        -1.047,
        -0.698,
        -1.047,
        -1.047,
        -1.047,
        -0.698,
        -1.047,
        -1.047,
        -1.047,
        -0.698,
        -1.047,
        -1.047,
        -1.047,
        -0.698,
        -1.047,
        -1.047,
    ],
    dtype=np.float64,
)

# Tiny |delta| ignored so noise near open is not amplified.
_DELTA_EPS = 1e-3


def _limits(
    upper: Sequence[float] | None,
    lower: Sequence[float] | None,
) -> tuple[np.ndarray, np.ndarray]:
    hi = np.asarray(upper if upper is not None else _DEFAULT_UPPER, dtype=np.float64)
    lo = np.asarray(lower if lower is not None else _DEFAULT_LOWER, dtype=np.float64)
    if hi.shape != (20,) or lo.shape != (20,):
        raise ValueError("joint limits must be length 20")
    return hi, lo


def apply_squeeze(
    q: np.ndarray,
    *,
    amount: float = 0.45,
    open_ref: Sequence[float] | None = None,
    thumb_abd_boost: float = 1.25,
    upper: Sequence[float] | None = None,
    lower: Sequence[float] | None = None,
    indices: Sequence[int] = AMPLIFY_JOINT_INDICES,
) -> np.ndarray:
    """Amplify retarget curl: ``q' = q + amount * (q - q_open)`` on selected DOFs.

    ``amount`` in [0, 1+] (UI usually 0..1): 0 = unchanged, 0.45 ≈ +45% of the
    existing open→retarget delta. Thumb abd gets ``thumb_abd_boost`` extra gain
    so opposition deepens without a separate absolute target.
    """
    q0 = np.asarray(q, dtype=np.float64).reshape(20).copy()
    a = float(max(0.0, amount))
    if a <= 1e-9:
        return q0.astype(np.float32)

    q_open = (
        np.asarray(open_ref, dtype=np.float64).reshape(20)
        if open_ref is not None
        else OPEN_HAND_Q.astype(np.float64)
    )
    hi, lo = _limits(upper, lower)
    out = q0.copy()
    boost = float(max(0.0, thumb_abd_boost))

    for i in indices:
        delta = float(q0[i] - q_open[i])
        if abs(delta) < _DELTA_EPS:
            continue
        gain = a * boost if i == THUMB_CMC_ABD else a
        out[i] = q0[i] + gain * delta
        out[i] = float(np.clip(out[i], lo[i], hi[i]))
    return out.astype(np.float32)


def open_hand_q(base: np.ndarray | None = None) -> np.ndarray:
    """Rest / open pose for re-grasp."""
    if base is None:
        return OPEN_HAND_Q.copy()
    return np.asarray(base, dtype=np.float32).reshape(20).copy()


def squeeze_delta_named(q_before: np.ndarray, q_after: np.ndarray) -> list[tuple[str, float]]:
    b = np.asarray(q_before, dtype=np.float64).reshape(20)
    a = np.asarray(q_after, dtype=np.float64).reshape(20)
    out: list[tuple[str, float]] = []
    for i in range(20):
        d = float(a[i] - b[i])
        if abs(d) > 1e-6:
            out.append((WUJI_HAND2_JOINT_SUFFIXES[i], d))
    return out
