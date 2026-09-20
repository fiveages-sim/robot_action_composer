"""Interactive grasp-generation session: capture → click → HUG → wrist move.

Runs in the **composer / system Python** (ROS Jazzy 3.12). HUG inference is a
subprocess into ``submodules/hug/.venv``. Does **not** start ros2-stack.

Viser UX mirrors upstream HUG ``app.py``:
- left canvas: RGB-D point cloud (drag to orbit) + dry-run MANO skeleton
- right sidebar: clickable RGB image to set UV / propose
"""

from __future__ import annotations

import base64
import json
import threading
import time
from pathlib import Path
from typing import Any, Optional

import cv2
import numpy as np

from .bridge import (
    dump_plan,
    load_proposal_T_camera_wrist,
    plan_wrist_from_proposal,
    send_wrist_plan,
)
from .config import GraspGenerationConfig
from .hug_subprocess import run_hug_propose
from .rgbd_io import capture_head_camera_ros, save_frame_to_dir
from .scene_vis import GraspScene3D, backproject_rgbd, pixel_to_cam_xyz


def _image_to_data_url(image_rgb: np.ndarray) -> str:
    _, buf = cv2.imencode(".jpg", cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR))
    return "data:image/jpeg;base64," + base64.b64encode(buf).decode()


def _make_clickable_image_html(image_rgb: np.ndarray, bridge_label: str) -> str:
    """Same pattern as hug.utils.viser_utils.make_clickable_image_html."""
    data_url = _image_to_data_url(image_rgb)
    bridge_label_js = bridge_label.replace("\\", "\\\\").replace("'", "\\'")
    return (
        f'<img src="{data_url}" '
        f'style="width:100%;display:block;cursor:crosshair;" '
        f'onload="(function(){{'
        f"var label=Array.from(document.querySelectorAll('label')).find(function(el){{"
        f"return el.textContent==='{bridge_label_js}';"
        f"}});"
        f"if(!label) return;"
        f"var row=label.closest('.mantine-Flex-root');"
        f"if(row) row.style.display='none';"
        f'}})()" '
        f'onclick="(function(e){{'
        f"var label=Array.from(document.querySelectorAll('label')).find(function(el){{"
        f"return el.textContent==='{bridge_label_js}';"
        f"}});"
        f"if(!label) return;"
        f"var row=label.closest('.mantine-Flex-root');"
        f"if(row) row.style.display='none';"
        f"var input=row?row.querySelector('input,textarea'):null;"
        f"if(!input) return;"
        f"var r=e.target.getBoundingClientRect();"
        f"var u=(e.clientX-r.left)/r.width;"
        f"var v=(e.clientY-r.top)/r.height;"
        f"var setter=Object.getOwnPropertyDescriptor("
        f"window.HTMLInputElement.prototype,'value').set;"
        f"setter.call(input, u.toFixed(6)+','+v.toFixed(6));"
        f"input.dispatchEvent(new Event('input',{{bubbles:true}}));"
        f'}})(event)">'
    )


def _make_hidden_bridge_bootstrap_html(bridge_label: str) -> str:
    bridge_label_js = bridge_label.replace("\\", "\\\\").replace("'", "\\'")
    return (
        '<img src="x" style="display:none" '
        'onerror="(function(){'
        "var label=Array.from(document.querySelectorAll('label')).find(function(el){"
        f"return el.textContent==='{bridge_label_js}';"
        "});"
        "if(!label) return;"
        "var row=label.closest('.mantine-Flex-root');"
        "if(row) row.style.display='none';"
        '})()">'
    )


def _draw_click_marker(rgb: np.ndarray, u_norm: float, v_norm: float) -> np.ndarray:
    out = rgb.copy()
    h, w = out.shape[:2]
    x = int(np.clip(u_norm * (w - 1), 0, w - 1))
    y = int(np.clip(v_norm * (h - 1), 0, h - 1))
    cv2.drawMarker(
        out,
        (x, y),
        (0, 255, 0),
        markerType=cv2.MARKER_CROSS,
        markerSize=24,
        thickness=2,
    )
    cv2.circle(out, (x, y), 8, (255, 0, 0), 2)
    return out


def run_serve(
    *,
    cfg: GraspGenerationConfig,
    workspace: Path,
    connect_interface: bool = True,
) -> None:
    """Blocking Viser UI on ``cfg.viser_port``."""
    try:
        import viser
    except ImportError as exc:
        raise SystemExit(
            "viser is required for grasp-generation serve.\n"
            "  uv pip install 'viser>=0.2'\n"
            f"  ({exc})"
        ) from exc

    capture_dir = cfg.resolved_capture_dir(workspace)
    capture_dir.mkdir(parents=True, exist_ok=True)
    hug_py = cfg.resolved_hug_python(workspace)
    composer_root = Path(__file__).resolve().parents[2]

    interface = None
    if connect_interface:
        from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig

        interface = ROS2RobotInterface(ROS2RobotInterfaceConfig())
        print("[grasp] connecting ROS2RobotInterface ...")
        interface.connect()
        time.sleep(0.5)

    state: dict[str, Any] = {
        "rgb": None,
        "depth_m": None,
        "K": None,
        "proposal_path": capture_dir / "proposal.json",
        "plan_path": capture_dir / "wrist_plan.json",
        "last_plan": None,
        "last_q_retarget": None,
        "last_q_sent": None,
        "lock": threading.Lock(),
        "auto_propose_on_click": True,
    }

    server = viser.ViserServer(host="0.0.0.0", port=int(cfg.viser_port))
    server.gui.configure_theme(titlebar_content=None, control_layout="collapsible")
    scene = GraspScene3D(server)

    click_bridge_label = "__click_uv_bridge__"

    with server.gui.add_folder("Grasp generation"):
        gui_status = server.gui.add_markdown(
            "**Ready.** ① Capture → ② click → ③ Plan wrist → ④ Retarget → "
            "⑤ Close (amplify curl from retarget) → ⑥ Open. "
            "HUG MANO is **right-hand**; left arm reuses that wrist pose."
        )
        gui_arm = server.gui.add_dropdown(
            "arm",
            options=["right", "left"],
            initial_value=cfg.arm if cfg.arm in ("left", "right") else "right",
        )
        gui_u = server.gui.add_slider("u_norm", min=0.0, max=1.0, step=0.01, initial_value=0.48)
        gui_v = server.gui.add_slider("v_norm", min=0.0, max=1.0, step=0.01, initial_value=0.42)
        gui_auto = server.gui.add_checkbox(
            "auto_propose_on_click", initial_value=True
        )
        gui_pos_only = server.gui.add_checkbox("position_only", initial_value=cfg.position_only)
        gui_execute = server.gui.add_checkbox(
            "execute_move", initial_value=bool(cfg.execute_default)
        )
        gui_squeeze = server.gui.add_slider(
            "squeeze_gain",
            min=0.0,
            max=1.2,
            step=0.05,
            initial_value=float(cfg.squeeze_amount),
        )
        gui_max_depth = server.gui.add_slider(
            "max_depth_m", min=0.5, max=8.0, step=0.1, initial_value=3.0
        )
        gui_pc_size = server.gui.add_slider(
            "pc_point_size", min=0.001, max=0.012, step=0.0005, initial_value=0.004
        )
        btn_capture = server.gui.add_button("1) Capture head RGB-D")
        btn_propose = server.gui.add_button("2) HUG propose @ UV (manual)")
        btn_move = server.gui.add_button("3) Plan / move wrist")
        btn_retarget = server.gui.add_button("4) Retarget fingers (shape)")
        btn_close = server.gui.add_button("5) Close / squeeze hand")
        btn_open = server.gui.add_button("6) Open hand (reset fingers)")
        gui_click_bridge = server.gui.add_text(click_bridge_label, initial_value="")
        server.gui.add_html(_make_hidden_bridge_bootstrap_html(click_bridge_label))
        placeholder = np.zeros((240, 320, 3), dtype=np.uint8)
        gui_image_html = server.gui.add_html(
            _make_clickable_image_html(placeholder, click_bridge_label)
        )

    def _sync_arm_from_ui() -> str:
        arm = str(gui_arm.value).lower()
        if arm not in ("left", "right"):
            arm = "right"
        cfg.arm = arm
        return arm

    def _set_status(msg: str) -> None:
        gui_status.content = msg
        print("[grasp]", msg.replace("**", ""))

    def _refresh_image(u: Optional[float] = None, v: Optional[float] = None) -> None:
        with state["lock"]:
            rgb = state["rgb"]
        if rgb is None:
            return
        show = rgb
        if u is not None and v is not None:
            show = _draw_click_marker(rgb, u, v)
        gui_image_html.content = _make_clickable_image_html(show, click_bridge_label)

    def _update_point_cloud() -> None:
        with state["lock"]:
            rgb = state["rgb"]
            depth = state["depth_m"]
            K = state["K"]
        if rgb is None or depth is None or K is None:
            return
        result = backproject_rgbd(
            depth, rgb, K, max_depth=float(gui_max_depth.value)
        )
        if result is None:
            _set_status("**Warning:** no valid depth for point cloud")
            return
        pts, cols = result
        scene.set_point_cloud(pts, cols, point_size=float(gui_pc_size.value))

    def _show_hand_from_proposal(prop_path: Path, *, also_click: bool = True) -> None:
        data = json.loads(Path(prop_path).read_text(encoding="utf-8"))
        lm = np.asarray(data["landmarks_3d"], dtype=np.float32)
        T = np.asarray(data["T_camera_wrist"], dtype=np.float64)
        scene.set_hand_from_landmarks(lm, T, show_wrist=True)
        if also_click and data.get("depth_at_click_m") and data.get("click_uv_224"):
            K224 = np.asarray(data.get("camera_K_224"), dtype=np.float64)
            if K224.shape == (3, 3):
                u224, v224 = data["click_uv_224"]
                xyz = pixel_to_cam_xyz(
                    float(u224), float(v224), float(data["depth_at_click_m"]), K224
                )
                scene.set_click_xyz(xyz)

    def do_capture(_: Any = None) -> None:
        try:
            _set_status("Capturing `/head_camera/*` ...")
            frame = capture_head_camera_ros(
                rgb_topic=cfg.rgb_topic,
                depth_topic=cfg.depth_topic,
                info_topic=cfg.info_topic,
                timeout_sec=30.0,
                out_dir=capture_dir,
            )
            save_frame_to_dir(frame, capture_dir, stem="w2_head")
            with state["lock"]:
                state["rgb"] = frame.rgb
                state["depth_m"] = frame.depth_m
                state["K"] = frame.K
            scene.clear_hand()
            _refresh_image()
            _update_point_cloud()
            _set_status(
                f"Captured `{frame.rgb.shape[1]}x{frame.rgb.shape[0]}` — "
                "**left = RGB-D cloud** (drag to orbit). "
                "**Click the image** on the object."
            )
        except Exception as exc:
            _set_status(f"**Capture failed:** `{exc}`")

    def do_propose(_: Any = None) -> None:
        try:
            if not (capture_dir / "w2_head_rgb.png").is_file():
                raise FileNotFoundError("No capture yet — press Capture first")
            u, v = float(gui_u.value), float(gui_v.value)
            _refresh_image(u, v)
            out = state["proposal_path"]
            _set_status(f"HUG propose at UV=({u:.3f},{v:.3f}) ...")
            run_hug_propose(
                hug_python=hug_py,
                capture_dir=capture_dir,
                out_json=out,
                u_norm=u,
                v_norm=v,
                sampling_steps=cfg.sampling_steps,
                composer_root=composer_root,
            )
            data = json.loads(out.read_text(encoding="utf-8"))
            t = data["T_camera_wrist"]
            tw = [t[0][3], t[1][3], t[2][3]]
            _show_hand_from_proposal(out)
            _set_status(
                f"HUG OK depth={data.get('depth_at_click_m'):.3f}m "
                f"wrist_cam={tw}. **Green hand on left** = dry MANO. "
                f"Next: **3) Plan** (`execute_move` off first)."
            )
        except Exception as exc:
            _set_status(f"**HUG propose failed:** `{exc}`")

    def do_move(_: Any = None) -> None:
        try:
            if interface is None:
                raise RuntimeError("ROS interface not connected")
            arm = _sync_arm_from_ui()
            prop_path = state["proposal_path"]
            if not Path(prop_path).is_file():
                raise FileNotFoundError("No proposal.json — click image / propose first")
            T, meta = load_proposal_T_camera_wrist(prop_path)
            cfg.position_only = bool(gui_pos_only.value)
            plan = plan_wrist_from_proposal(
                T,
                interface=interface,
                cfg=cfg,
                click_uv_224=tuple(meta.get("click_uv_224") or ()),
                depth_at_click_m=meta.get("depth_at_click_m"),
            )
            dump_plan(plan, state["plan_path"])
            state["last_plan"] = plan
            # Always refresh dry hand from proposal (camera frame, like HUG).
            _show_hand_from_proposal(Path(prop_path))
            execute = bool(gui_execute.value)
            info = send_wrist_plan(
                interface,
                plan,
                execute=execute,
                arrival_timeout=cfg.arrival_timeout,
            )
            xyz = info["target_xyz"]
            hug = info.get("hug_wrist_xyz") or xyz
            mode = "EXECUTE" if execute else "DRY-RUN"
            tip = f"{plan.hug_tip_frame}→{plan.control_tip_frame}"
            _set_status(
                f"**{mode}** arm={arm} frame=`{plan.motion_frame_id}` "
                f"map=`{tip}` "
                f"hug_xyz=({hug[0]:.3f},{hug[1]:.3f},{hug[2]:.3f}) "
                f"cmd_xyz=({xyz[0]:.3f},{xyz[1]:.3f},{xyz[2]:.3f}) "
                f"cam_via={plan.cam_resolution.method}/{plan.cam_resolution.camera_frame}"
            )
        except Exception as exc:
            _set_status(f"**Move/plan failed:** `{exc}`")

    def do_retarget(_: Any = None) -> None:
        """Match MANO finger shape (no extra squeeze)."""
        try:
            if cfg.hand_profile not in ("wuji_hand2", "wuji-hand2"):
                raise RuntimeError(f"hand_profile={cfg.hand_profile} unsupported")
            arm = _sync_arm_from_ui()
            prop_path = Path(state["proposal_path"])
            if not prop_path.is_file():
                raise FileNotFoundError("No proposal.json — propose first")
            from .retarget.pipeline import run_retarget_pipeline

            execute = bool(gui_execute.value)
            if execute and interface is None:
                raise RuntimeError("ROS interface not connected")
            code, out = run_retarget_pipeline(
                cfg=cfg,
                workspace=workspace,
                proposal_path=prop_path,
                arm=arm,
                execute=execute,
                reset=True,
                squeeze_amount=0.0,
                interface=interface,
                owns_interface=False,
            )
            if code != 0:
                raise RuntimeError(out.get("error", str(out)))
            state["last_q_retarget"] = list(out["q_retarget"])
            state["last_q_sent"] = list(out["q"])
            mode = "SENT" if execute else "DRY"
            _set_status(
                f"**Retarget {mode}** arm={arm} absmax={out['q_absmax']:.3f}rad "
                f"(shape only, squeeze=0). Next: **5) Close/squeeze** "
                f"or raise squeeze_amount."
            )
        except Exception as exc:
            _set_status(f"**Retarget failed:** `{exc}`")

    def do_close(_: Any = None) -> None:
        """Retarget then amplify inward deltas (single send, wrist unchanged)."""
        try:
            if cfg.hand_profile not in ("wuji_hand2", "wuji-hand2"):
                raise RuntimeError(f"hand_profile={cfg.hand_profile} unsupported")
            arm = _sync_arm_from_ui()
            prop_path = Path(state["proposal_path"])
            if not prop_path.is_file():
                raise FileNotFoundError("No proposal.json — propose first")
            from .retarget.pipeline import run_retarget_pipeline

            if interface is None:
                raise RuntimeError("ROS interface not connected (need -- no-interface off)")
            sq = float(gui_squeeze.value)
            code, out = run_retarget_pipeline(
                cfg=cfg,
                workspace=workspace,
                proposal_path=prop_path,
                arm=arm,
                execute=True,
                reset=True,
                squeeze_amount=sq,
                interface=interface,
                owns_interface=False,
            )
            if code != 0:
                raise RuntimeError(out.get("error", str(out)))
            state["last_q_retarget"] = list(out["q_retarget"])
            state["last_q_sent"] = list(out["q"])
            n_d = len(out.get("squeeze_deltas") or {})
            _set_status(
                f"**Close SENT** arm={arm} amplify_gain={sq:.2f} "
                f"({n_d} DOF Δ) absmax={out['q_absmax']:.3f}rad — "
                "shape kept, curl deepened. **6) Open** to re-grasp."
            )
        except Exception as exc:
            _set_status(f"**Close/squeeze failed:** `{exc}`")

    def do_open(_: Any = None) -> None:
        """Reset fingers to open/rest so the next grasp can approach."""
        try:
            if interface is None:
                raise RuntimeError("ROS interface not connected")
            arm = _sync_arm_from_ui()
            from .retarget.pipeline import run_open_hand

            q_list = run_open_hand(arm=arm, interface=interface)
            state["last_q_sent"] = list(q_list)
            _set_status(
                f"**Open SENT** arm={arm} — fingers at rest. "
                "You can Plan / Retarget / Close again."
            )
        except Exception as exc:
            _set_status(f"**Open failed:** `{exc}`")

    btn_capture.on_click(do_capture)
    btn_propose.on_click(do_propose)
    btn_move.on_click(do_move)
    btn_retarget.on_click(do_retarget)
    btn_close.on_click(do_close)
    btn_open.on_click(do_open)

    @gui_click_bridge.on_update
    def _on_click(_: Any) -> None:
        raw = gui_click_bridge.value.strip()
        if not raw or "," not in raw:
            return
        try:
            u_s, v_s = raw.split(",", 1)
            u, v = float(u_s), float(v_s)
        except ValueError:
            return
        gui_u.value = u
        gui_v.value = v
        _refresh_image(u, v)
        _set_status(f"Clicked UV=({u:.3f}, {v:.3f})")
        if bool(gui_auto.value):
            do_propose()

    @gui_max_depth.on_update
    def _on_max_depth(_: Any) -> None:
        _update_point_cloud()

    @gui_pc_size.on_update
    def _on_pc_size(_: Any) -> None:
        scene.set_point_size(float(gui_pc_size.value))

    _set_status(
        f"Serving **http://127.0.0.1:{cfg.viser_port}** — "
        "arm → Capture → click → Plan → Retarget → Close → **Open** to re-grasp."
    )
    print(f"[grasp] Viser http://127.0.0.1:{cfg.viser_port}")
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n[grasp] shutting down")
    finally:
        if interface is not None:
            try:
                interface.disconnect()
            except Exception:
                pass


def run_follow_wrist_cli(
    *,
    cfg: GraspGenerationConfig,
    workspace: Path,
    proposal: Path,
    execute: bool,
) -> int:
    """One-shot: proposal.json → plan → optional move (no Viser)."""
    from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig

    T, meta = load_proposal_T_camera_wrist(proposal)
    interface = ROS2RobotInterface(ROS2RobotInterfaceConfig())
    interface.connect()
    time.sleep(0.3)
    try:
        plan = plan_wrist_from_proposal(
            T,
            interface=interface,
            cfg=cfg,
            click_uv_224=tuple(meta["click_uv_224"]) if meta.get("click_uv_224") else None,
            depth_at_click_m=meta.get("depth_at_click_m"),
        )
        out = cfg.resolved_capture_dir(workspace) / "wrist_plan.json"
        dump_plan(plan, out)
        print(json.dumps(plan.to_dict(), indent=2))
        info = send_wrist_plan(
            interface, plan, execute=execute, arrival_timeout=cfg.arrival_timeout
        )
        print(json.dumps(info, indent=2, default=str))
        return 0
    finally:
        interface.disconnect()
