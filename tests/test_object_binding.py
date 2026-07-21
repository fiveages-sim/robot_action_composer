"""Unit tests for object binding helpers that do not require ROS/Isaac."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

_ROOT = Path(__file__).resolve().parents[1]
_OB_PATH = _ROOT / "robot_action_composer" / "task_runtime" / "object_binding.py"


def _load_object_binding():
    """Load object_binding.py without importing task_runtime package (avoids rclpy)."""
    name = "rac_object_binding_under_test"
    if name in sys.modules:
        return sys.modules[name]
    spec = importlib.util.spec_from_file_location(name, _OB_PATH)
    assert spec is not None and spec.loader is not None
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


ob = _load_object_binding()


def test_normalize_prim_path() -> None:
    assert ob.normalize_prim_path("/World/a/") == "/World/a"
    assert ob.normalize_prim_path("World/a") == "/World/a"


def test_deep_merge_dicts() -> None:
    base = {"wind": {"grasps": {"left": "a"}, "object_prim_path": "/A"}}
    over = {"wind": {"grasps": {"right": "b"}, "object_prim_path": "/B"}}
    merged = ob.deep_merge_dicts(base, over)
    assert merged["wind"]["object_prim_path"] == "/B"
    assert merged["wind"]["grasps"]["left"] == "a"
    assert merged["wind"]["grasps"]["right"] == "b"


def test_merge_objects_registry_layers() -> None:
    meta = {"blade": {"grasps": {"left": "small_side/point_01"}}}
    task = {"blade": {"object_prim_path": "/World/blade"}}
    scene = {"blade": {"object_prim_path": "/World/other"}}
    reg = ob.merge_objects_registry(meta_objects=meta, task_objects=task, scene_objects=scene)
    assert reg["blade"]["grasps"]["left"] == "small_side/point_01"
    assert reg["blade"]["object_prim_path"] == "/World/other"


def test_load_leaf_meta_objects(tmp_path: Path) -> None:
    objects_dir = tmp_path / ".meta" / "objects"
    objects_dir.mkdir(parents=True)
    (objects_dir / "wind_turbo_blade.yaml").write_text(
        "object_key: wind_turbo_blade\ngrasps:\n  left: small_side/point_01\n",
        encoding="utf-8",
    )
    reg = ob.load_leaf_meta_objects(tmp_path)
    assert "wind_turbo_blade" in reg
    assert reg["wind_turbo_blade"]["grasps"]["left"] == "small_side/point_01"


def test_resolve_pick_legacy_explicit_offset() -> None:
    prim, off = ob.resolve_pick_object_binding(
        {
            "object_prim_path": "/World/obj",
            "object_position_offset": (0.1, 0.2, 0.3),
        }
    )
    assert prim == "/World/obj"
    assert off == (0.1, 0.2, 0.3)


def test_resolve_pick_grasp_id_from_registry(monkeypatch: pytest.MonkeyPatch) -> None:
    def _fake_compose(obj: str, grasp: str, *, cache=None):
        assert obj == "/World/blade"
        assert grasp == "/World/blade/small_side/point_01"
        return (0.0, -0.072, -0.005)

    monkeypatch.setattr(ob, "_compose_local_chain_translation", _fake_compose)
    objects = {
        "blade": {
            "object_prim_path": "/World/blade",
            "grasps": {"left": "small_side/point_01"},
        }
    }
    prim, off = ob.resolve_pick_object_binding(
        {"grasp_id": "left"},
        objects=objects,
        active_object="blade",
    )
    assert prim == "/World/blade"
    assert off == (0.0, -0.072, -0.005)


def test_resolve_pick_explicit_offset_wins_over_grasp(monkeypatch: pytest.MonkeyPatch) -> None:
    called = {"n": 0}

    def _fake_compose(*_a, **_k):
        called["n"] += 1
        return (9.0, 9.0, 9.0)

    monkeypatch.setattr(ob, "_compose_local_chain_translation", _fake_compose)
    prim, off = ob.resolve_pick_object_binding(
        {
            "object_prim_path": "/World/blade",
            "grasp_id": "left",
            "object_position_offset": (1.0, 2.0, 3.0),
        },
        objects={"blade": {"grasps": {"left": "a/b"}}},
        active_object="blade",
    )
    assert prim == "/World/blade"
    assert off == (1.0, 2.0, 3.0)
    assert called["n"] == 0


def test_resolve_active_object_scene_wins() -> None:
    assert ob.resolve_active_object(task_active="a", scene_active="b") == "b"
    assert ob.resolve_active_object(task_active="a", scene_active=None) == "a"


def test_resolve_object_prim_path_from_key() -> None:
    objects = {"bg_cube": {"object_prim_path": "/World/scene/bg_1/collision/Cube"}}
    prim = ob.resolve_object_prim_path(
        {"object_key": "bg_cube"},
        objects=objects,
        required=True,
        label="nav",
    )
    assert prim == "/World/scene/bg_1/collision/Cube"


def test_resolve_object_prim_path_explicit_wins() -> None:
    objects = {"bg_cube": {"object_prim_path": "/World/from_registry"}}
    prim = ob.resolve_object_prim_path(
        {
            "reference_object_prim_path": "/World/explicit",
            "reference_object_key": "bg_cube",
        },
        objects=objects,
        prim_param="reference_object_prim_path",
        key_params=("reference_object_key", "object_key"),
    )
    assert prim == "/World/explicit"


def test_resolve_object_prim_path_no_active_fallback() -> None:
    objects = {"blade": {"object_prim_path": "/World/blade"}}
    prim = ob.resolve_object_prim_path(
        {},
        objects=objects,
        active_object="blade",
        use_active_object=False,
    )
    assert prim is None
