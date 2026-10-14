"""Subscribe Wuji Hand2 ``Sensor=contact_force`` ROS2 topics.

Topics (Side=right → ``wuji_rh``, left → ``wuji_lh``)::

    /{prefix}/in_contact/<link>     std_msgs/Bool
    /{prefix}/contact_wrench/<link> geometry_msgs/WrenchStamped
    /{prefix}/any_contact           std_msgs/Bool
    /{prefix}/contact_partners      std_msgs/String  (``link=path;...``)

Binary Bool is the gate; table/ground partners are filtered out so fingertips
resting on the desk do not freeze the hand during close.
"""

from __future__ import annotations

import threading
import time
from typing import Optional, Sequence

from ..profiles.base import HandExecutionProfile
from ..types import ContactSnapshot
from .base import ContactSensor

# Substrings matched against partner USD paths (case-insensitive).
# NOTE: Do NOT put ``default_environment`` or bare ``table`` here — grasp props
# live under ``/World/default_environment/EastRural_Table/sm_apple_...`` and
# those tokens would discard *real* object contacts (B3/B4 never freeze).
_DEFAULT_IGNORE_PARTNERS: tuple[str, ...] = (
    "ground",
    "groundplane",
    "defaultgroundplane",
    "tabletop",  # EastRural TableTop collider only (not EastRural_Table/sm_*)
    "floor",
    # Self / other-finger collisions must not freeze the close gate.
    "wuji_",
    "_hand_",
    "finger_",
    "thumb_",
    "pinky_",
    "ring_",
    "index_finger",
    "middle_finger",
)

# If any of these appear in the partner path, never treat as desk/ground.
_OBJECT_KEEP_HINTS: tuple[str, ...] = (
    "sm_",
    "apple",
    "bottle",
    "banana",
    "lime",
    "hammer",
    "mug",
    "ycb",
    "model_normalized",
    "_obj_",
    "/geometry/",
)


def _is_ignored_partner(path: str, ignore: Sequence[str]) -> bool:
    low = str(path).lower()
    if any(h in low for h in _OBJECT_KEEP_HINTS):
        return False
    return any(tok.lower() in low for tok in ignore if tok)


class WujiRosContactSensor(ContactSensor):
    """Background-spun subscriptions for one hand side."""

    def __init__(
        self,
        profile: HandExecutionProfile,
        *,
        arm: str = "right",
        topic_prefix: Optional[str] = None,
        force_mag_threshold_n: float = 0.0,
        use_bool: bool = True,
        ignore_partners: Optional[Sequence[str]] = None,
        node_name: str = "grasp_wuji_contact",
    ) -> None:
        self._profile = profile
        self._arm = "left" if str(arm).lower().startswith("l") else "right"
        if topic_prefix:
            self._prefix = topic_prefix.strip().strip("/")
        else:
            self._prefix = profile.topic_prefix_by_arm.get(
                self._arm, "wuji_rh" if self._arm == "right" else "wuji_lh"
            )
        self._force_thr = float(force_mag_threshold_n)
        self._use_bool = bool(use_bool)
        # YAML may only list desk/ground; always keep self-collision filters.
        _self = (
            "wuji_",
            "_hand_",
            "finger_",
            "thumb_",
            "pinky_",
            "ring_",
            "index_finger",
            "middle_finger",
        )
        if ignore_partners is not None:
            merged = list(ignore_partners)
            for tok in _self:
                if not any(str(t).lower() == tok for t in merged):
                    merged.append(tok)
            self._ignore = tuple(merged)
        else:
            self._ignore = _DEFAULT_IGNORE_PARTNERS
        self._node_name = node_name

        self._lock = threading.Lock()
        self._link_bool: dict[str, bool] = {
            L: False for L in profile.all_contact_links()
        }
        self._link_mag: dict[str, float] = {
            L: 0.0 for L in profile.all_contact_links()
        }
        self._link_partners: dict[str, list[str]] = {
            L: [] for L in profile.all_contact_links()
        }
        self._any = False
        self._stamp = 0.0

        self._node = None
        self._executor = None
        self._thread: Optional[threading.Thread] = None
        self._started = False

    @property
    def topic_prefix(self) -> str:
        return self._prefix

    def start(self) -> None:
        if self._started:
            return
        import rclpy
        from geometry_msgs.msg import WrenchStamped
        from rclpy.executors import SingleThreadedExecutor
        from std_msgs.msg import Bool, String

        if not rclpy.ok():
            rclpy.init()

        self._node = rclpy.create_node(self._node_name)
        links = self._profile.all_contact_links()

        def _on_bool(link: str):
            def _cb(msg: Bool) -> None:
                with self._lock:
                    self._link_bool[link] = bool(msg.data)
                    self._stamp = time.time()

            return _cb

        def _on_wrench(link: str):
            def _cb(msg: WrenchStamped) -> None:
                f = msg.wrench.force
                mag = float((f.x * f.x + f.y * f.y + f.z * f.z) ** 0.5)
                with self._lock:
                    self._link_mag[link] = mag
                    self._stamp = time.time()

            return _cb

        def _on_any(msg: Bool) -> None:
            with self._lock:
                self._any = bool(msg.data)
                self._stamp = time.time()

        def _on_partners(msg: String) -> None:
            # Format: "thumb_distal=/World/...;index_finger_distal=/World/..."
            parsed: dict[str, list[str]] = {L: [] for L in links}
            raw = (msg.data or "").strip()
            if raw:
                for bit in raw.split(";"):
                    bit = bit.strip()
                    if not bit or "=" not in bit:
                        continue
                    link, path = bit.split("=", 1)
                    link = link.strip()
                    if link in parsed:
                        parsed[link].append(path.strip())
            with self._lock:
                self._link_partners = parsed
                self._stamp = time.time()

        for link in links:
            self._node.create_subscription(
                Bool,
                f"/{self._prefix}/in_contact/{link}",
                _on_bool(link),
                10,
            )
            self._node.create_subscription(
                WrenchStamped,
                f"/{self._prefix}/contact_wrench/{link}",
                _on_wrench(link),
                10,
            )
        self._node.create_subscription(
            Bool, f"/{self._prefix}/any_contact", _on_any, 10
        )
        self._node.create_subscription(
            String, f"/{self._prefix}/contact_partners", _on_partners, 10
        )

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._thread = threading.Thread(
            target=self._executor.spin, daemon=True, name="wuji-contact-spin"
        )
        self._thread.start()
        self._started = True
        time.sleep(0.15)

    def stop(self) -> None:
        if not self._started:
            return
        try:
            if self._executor is not None:
                self._executor.shutdown()
        except Exception:
            pass
        try:
            if self._node is not None:
                self._node.destroy_node()
        except Exception:
            pass
        self._executor = None
        self._node = None
        self._thread = None
        self._started = False

    def snapshot(self) -> ContactSnapshot:
        with self._lock:
            link_bool = dict(self._link_bool)
            link_mag = dict(self._link_mag)
            link_partners = {k: list(v) for k, v in self._link_partners.items()}
            any_c = bool(self._any)
            stamp = float(self._stamp)

        if not self._use_bool and self._force_thr > 0.0:
            for link, mag in link_mag.items():
                if mag >= self._force_thr:
                    link_bool[link] = True

        # Gate on *real* object contact. Past bugs:
        #   - Bool True + force=0 on self (ring↔pinky) → false freeze
        #   - ignore list matching ``default_environment`` / ``table`` → apple
        #     under EastRural_Table discarded → B3/B4 never freeze, squeeze
        #     keeps pushing, PhysX yields fingers open (cmd holds closed)
        #
        # Rules (force-first when partners name an object):
        #   1) object partner → mag >= 0.15 (bottle thumb often 0.2–0.4 N)
        #   2) no partners → Bool + mag >= 0.5 (avoid wrench noise / false Bool)
        #   3) partners only self/desk → never freeze
        #   4) object + force enough even when Bool lags False
        force_gate = max(float(self._force_thr), 0.5)
        object_gate = max(float(self._force_thr), 0.15)
        filtered: dict[str, bool] = {}
        for link in link_bool:
            mag = float(link_mag.get(link, 0.0) or 0.0)
            partners = link_partners.get(link) or []
            non_ignore = [
                p for p in partners if not _is_ignored_partner(p, self._ignore)
            ]
            if partners:
                filtered[link] = bool(non_ignore) and mag >= object_gate
            else:
                filtered[link] = bool(link_bool.get(link, False)) and mag >= force_gate

        fingers = self._profile.finger_contact_from_links(filtered)
        any_c = any(fingers.values()) or any(filtered.values())

        return ContactSnapshot(
            stamp_sec=stamp,
            any_contact=any_c,
            link_in_contact=filtered,
            link_force_mag=link_mag,
            finger_in_contact=fingers,
            link_partners=link_partners,
        )


def make_contact_sensor(
    profile: HandExecutionProfile,
    *,
    arm: str,
    topic_prefix: str = "",
    enabled: bool = True,
    force_mag_threshold_n: float = 0.0,
    use_bool: bool = True,
    ignore_partners: Optional[Sequence[str]] = None,
) -> ContactSensor:
    """Factory: Null when disabled / import fails."""
    if not enabled:
        from .base import NullContactSensor

        return NullContactSensor(profile)
    try:
        return WujiRosContactSensor(
            profile,
            arm=arm,
            topic_prefix=topic_prefix or None,
            force_mag_threshold_n=force_mag_threshold_n,
            use_bool=use_bool,
            ignore_partners=ignore_partners,
        )
    except Exception:
        from .base import NullContactSensor

        return NullContactSensor(profile)
