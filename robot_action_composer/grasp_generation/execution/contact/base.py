"""Contact sensor abstractions for close executors."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Optional

from ..profiles.base import HandExecutionProfile
from ..types import ContactSnapshot


class ContactSensor(ABC):
    """Binary (and optional magnitude) contact for one hand side."""

    @abstractmethod
    def start(self) -> None:
        """Begin spinning / updating."""

    @abstractmethod
    def stop(self) -> None:
        """Release ROS resources."""

    @abstractmethod
    def snapshot(self) -> ContactSnapshot:
        """Latest contact state (finger fields filled via profile)."""

    def __enter__(self) -> "ContactSensor":
        self.start()
        return self

    def __exit__(self, *args) -> None:
        self.stop()


class NullContactSensor(ContactSensor):
    """Always no-contact (open-loop modes / dry runs without Isaac)."""

    def __init__(self, profile: Optional[HandExecutionProfile] = None) -> None:
        self._profile = profile

    def start(self) -> None:
        return

    def stop(self) -> None:
        return

    def snapshot(self) -> ContactSnapshot:
        fingers = {}
        if self._profile is not None:
            fingers = {f: False for f in self._profile.finger_joint_indices}
        return ContactSnapshot(finger_in_contact=fingers)
