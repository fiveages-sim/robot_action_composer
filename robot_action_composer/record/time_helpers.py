"""Time helpers for real-robot record/playback (wall clock, no ROS /clock)."""

from __future__ import annotations

import time


class WallTimeHelper:
    """Monotonic wall-clock sleep/time for manifest playback on real hardware."""

    def __init__(self) -> None:
        print("[Clock] wall_time=True")

    def now_seconds(self) -> float:
        return time.monotonic()

    def sleep(self, duration: float) -> None:
        if duration > 0.0:
            time.sleep(duration)

    def shutdown(self) -> None:
        pass
