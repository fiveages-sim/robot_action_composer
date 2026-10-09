"""Hand execution profiles: finger→joint groups + contact link names.

New dex hands register here; close executors stay hand-agnostic.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Mapping, Optional, Sequence

import numpy as np

from ..types import FINGER_IDS


@dataclass(frozen=True)
class HandExecutionProfile:
    """Decouples executor logic from a concrete hand kinematics / ROS topics."""

    name: str
    n_dof: int
    # finger_id → joint indices in the commanded q vector
    finger_joint_indices: Mapping[str, tuple[int, ...]]
    # finger_id → ROS link names under ``/<prefix>/in_contact/<link>``
    contact_links: Mapping[str, tuple[str, ...]]
    # arm side → Isaac topicPrefix (Wuji: right→wuji_rh, left→wuji_lh)
    topic_prefix_by_arm: Mapping[str, str]
    joint_suffixes: tuple[str, ...] = ()
    # Distal joints kept free under ``freeze_mode=keep_tip`` (wrap small objects)
    tip_joint_indices: Mapping[str, tuple[int, ...]] = field(default_factory=dict)
    # Optional squeeze / open helpers (Wuji uses retarget.squeeze)
    apply_squeeze_fn: Optional[
        Callable[..., np.ndarray]
    ] = None
    open_hand_fn: Optional[Callable[..., np.ndarray]] = None

    def all_contact_links(self) -> tuple[str, ...]:
        seen: list[str] = []
        for links in self.contact_links.values():
            for link in links:
                if link not in seen:
                    seen.append(link)
        return tuple(seen)

    def finger_contact_from_links(
        self, link_in_contact: Mapping[str, bool]
    ) -> dict[str, bool]:
        out: dict[str, bool] = {}
        for finger in FINGER_IDS:
            links = self.contact_links.get(finger, ())
            out[finger] = any(bool(link_in_contact.get(L, False)) for L in links)
        return out

    def apply_squeeze(
        self,
        q: Sequence[float],
        *,
        amount: float,
        thumb_abd_boost: float = 1.25,
    ) -> np.ndarray:
        if self.apply_squeeze_fn is None:
            return np.asarray(q, dtype=np.float32).reshape(self.n_dof)
        return np.asarray(
            self.apply_squeeze_fn(
                np.asarray(q, dtype=np.float32),
                amount=amount,
                thumb_abd_boost=thumb_abd_boost,
            ),
            dtype=np.float32,
        ).reshape(self.n_dof)

    def open_hand_q(self) -> np.ndarray:
        if self.open_hand_fn is None:
            return np.zeros(self.n_dof, dtype=np.float32)
        return np.asarray(self.open_hand_fn(None), dtype=np.float32).reshape(
            self.n_dof
        )


def get_hand_profile(name: str) -> HandExecutionProfile:
    key = str(name).strip().lower().replace("-", "_")
    if key in ("wuji_hand2", "wuji_hand_2"):
        from .wuji_hand2 import WUJI_HAND2_EXECUTION_PROFILE

        return WUJI_HAND2_EXECUTION_PROFILE
    raise KeyError(
        f"unknown hand execution profile {name!r}; "
        "register under grasp_generation.execution.profiles"
    )
