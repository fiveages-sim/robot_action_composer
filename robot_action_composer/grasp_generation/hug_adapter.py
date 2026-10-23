"""Thin optional adapter around the HUG submodule.

Import-safe when HUG is not installed. Heavy imports happen inside methods.
"""

from __future__ import annotations

import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

import numpy as np

from .rgbd_io import (
    RgbdFrame,
    crop_frame_to_legacy_vfov,
    depth_to_uint16_mm,
    full_pixel_to_224,
    suppress_depth_spikes,
)


@dataclass
class HugPaths:
    """Filesystem layout for the local HUG submodule."""

    root: Path
    checkpoint: Path
    default_dataset: Path

    @classmethod
    def from_repo_layout(cls, composer_file: Optional[Path] = None) -> "HugPaths":
        here = Path(composer_file or __file__).resolve()
        for parent in here.parents:
            candidate = parent / "submodules" / "hug"
            if candidate.is_dir():
                root = candidate
                break
            if parent.name == "robot_action_composer" and (parent.parent / "hug").is_dir():
                root = parent.parent / "hug"
                break
        else:
            root = Path("submodules/hug").resolve()
        return cls(
            root=root,
            checkpoint=root / "checkpoints" / "hug_full.safetensors",
            default_dataset=root / "data" / "hug_bench",
        )


@dataclass
class HugGraspProposal:
    """Camera-frame MANO grasp from HUG."""

    T_camera_wrist: np.ndarray  # (4, 4)
    landmarks_3d: np.ndarray  # (21, 3)
    landmarks_2d: np.ndarray  # (21, 2) in 224 space
    pose: np.ndarray
    shape: np.ndarray
    t: np.ndarray
    click_uv_224: tuple[float, float]
    depth_at_click_m: float
    camera_K_224: np.ndarray
    raw: dict[str, Any]


class HugAdapter:
    """Lazy wrapper: load model once, propose grasps from RGB-D + click."""

    def __init__(
        self,
        paths: Optional[HugPaths] = None,
        *,
        device: Optional[str] = None,
        use_ema: bool = True,
        sampling_steps: int = 1,
    ) -> None:
        self.paths = paths or HugPaths.from_repo_layout()
        self.device = device
        self.use_ema = use_ema
        self.sampling_steps = sampling_steps
        self._model = None
        self._torch = None

    @property
    def available(self) -> bool:
        if not self._can_import():
            return False
        return self.paths.checkpoint.is_file()

    def describe(self) -> dict[str, Any]:
        return {
            "hug_importable": self._can_import(),
            "root": str(self.paths.root),
            "checkpoint_exists": self.paths.checkpoint.is_file(),
            "checkpoint": str(self.paths.checkpoint),
            "default_dataset": str(self.paths.default_dataset),
            "mano_dir": str(self.paths.root / "assets" / "mano_models"),
            "mano_present": (self.paths.root / "assets" / "mano_models" / "models").is_dir()
            and any(
                (self.paths.root / "assets" / "mano_models" / "models").glob("MANO_*.pkl")
            ),
            "model_loaded": self._model is not None,
        }

    def load(self) -> None:
        """Load checkpoint onto device (idempotent)."""
        if self._model is not None:
            return
        if not self.available:
            raise RuntimeError(
                "HUG not ready. Install/run under submodules/hug .venv and place checkpoint. "
                f"describe={self.describe()}"
            )
        import torch
        from hug.inference import load_model

        self._torch = torch
        device = self.device or ("cuda" if torch.cuda.is_available() else "cpu")
        self.device = device
        self._model = load_model(self.paths.checkpoint, self.use_ema, device)

    def propose(
        self,
        frame: RgbdFrame,
        *,
        u_norm: Optional[float] = None,
        v_norm: Optional[float] = None,
        u_px: Optional[float] = None,
        v_px: Optional[float] = None,
        work_dir: Optional[Path] = None,
    ) -> HugGraspProposal:
        """Run HUG on one RGB-D frame at a click (full-image coords).

        Provide either normalized ``(u_norm, v_norm)`` or pixel ``(u_px, v_px)``
        on the **full-resolution** image. A wide head camera is center-cropped
        back to the legacy ~47° vertical FOV before the 224 mapping, and K moves
        with that crop.
        """
        self.load()
        assert self._model is not None and self._torch is not None

        h, w = frame.rgb.shape[:2]
        if u_px is None or v_px is None:
            un = 0.5 if u_norm is None else float(u_norm)
            vn = 0.5 if v_norm is None else float(v_norm)
            u_px = un * max(w - 1, 1)
            v_px = vn * max(h - 1, 1)

        depth = suppress_depth_spikes(frame.depth_m)
        rgb_c, depth_c, K_c, x_off, y_off = crop_frame_to_legacy_vfov(
            frame.rgb, depth, frame.K
        )
        u_c = float(np.clip(u_px - x_off, 0, rgb_c.shape[1] - 1))
        v_c = float(np.clip(v_px - y_off, 0, rgb_c.shape[0] - 1))
        ch, cw = rgb_c.shape[:2]
        u_224, v_224 = full_pixel_to_224(u_c, v_c, cw, ch)
        frame = RgbdFrame(
            rgb=rgb_c,
            depth_m=depth_c,
            K=K_c,
            stamp_sec=frame.stamp_sec,
        )

        depth_u16 = depth_to_uint16_mm(frame.depth_m)
        from hug.dataloader.grasp_dataset import GraspDataset
        from hug.models.mano import mano_params_to_grasp_dict
        from hug.prepare_inputs import prepare_pkl
        from hug.utils.pcl_utils import depth_to_pcl_tensors, pixel_to_xyz

        torch = self._torch
        model = self._model
        device = self.device

        if work_dir is None:
            tmp = tempfile.TemporaryDirectory(prefix="hug_propose_")
            out_dir = Path(tmp.name)
            cleanup = tmp
        else:
            out_dir = Path(work_dir)
            out_dir.mkdir(parents=True, exist_ok=True)
            cleanup = None

        try:
            prepare_pkl(
                frame.rgb,
                depth_u16,
                np.asarray(frame.K, dtype=np.float64),
                stem="frame",
                out_dir=out_dir,
                object_name="w2_head",
            )
            # Avoid writing samples.txt that locks the folder for other globs.
            samples = out_dir / "samples.txt"
            if samples.exists():
                samples.unlink()

            use_rgb = getattr(model, "use_rgb", True)
            use_depth = getattr(model, "use_depth", False)
            pcl_use_rgb = getattr(model, "pcl_use_rgb", False)
            crop_r = getattr(model, "pcl_crop_radius", None)

            dataset = GraspDataset(
                str(out_dir),
                split="val",
                use_rgb=use_rgb,
                use_depth=use_depth,
                pcl_crop_radius=crop_r,
            )
            sample = dataset.get_inference_data("frame")
            K_224 = np.asarray(sample["camera_K"], dtype=np.float64)
            depth_image = sample.get("depth_image")
            depth_m = 0.0
            if depth_image is not None:
                dh, dw = depth_image.shape[:2]
                du = int(np.clip(u_224 * dw / 224.0, 0, dw - 1))
                dv = int(np.clip(v_224 * dh / 224.0, 0, dh - 1))
                depth_m = float(depth_image[dv, du]) / 1000.0

            point_uv = torch.tensor(
                [[u_224, v_224, depth_m]], dtype=torch.float32, device=device
            )
            camera_K = torch.from_numpy(K_224).float().unsqueeze(0).to(device)
            betas = model.fixed_betas.squeeze(0)
            rgb = sample["rgb"].unsqueeze(0).to(device) if use_rgb else None

            pcl_xyz = None
            pcl_rgb = None
            if use_depth:
                if crop_r is not None and depth_image is not None:
                    depth_m_arr = depth_image.astype(np.float32) / 1000.0
                    point_xyz = pixel_to_xyz(u_224, v_224, depth_m, K_224)
                    xyz, rgb_pcl = depth_to_pcl_tensors(
                        depth_m_arr,
                        sample["rgb_original"],
                        K_224,
                        center=point_xyz,
                        crop_radius=crop_r,
                    )
                    pcl_xyz = xyz.unsqueeze(0).to(device)
                    pcl_rgb = rgb_pcl.unsqueeze(0).to(device) if pcl_use_rgb else None
                else:
                    pcl_xyz = sample["pcl_xyz"].unsqueeze(0).to(device)
                    pcl_rgb = (
                        sample["pcl_rgb"].unsqueeze(0).to(device) if pcl_use_rgb else None
                    )

            with torch.no_grad():
                with torch.autocast(
                    "cuda", dtype=torch.bfloat16, enabled=(device == "cuda")
                ):
                    preds = model.sample(
                        point_uv,
                        camera_K,
                        steps=self.sampling_steps,
                        rgb=rgb,
                        pcl_xyz=pcl_xyz,
                        pcl_rgb=pcl_rgb,
                    )

            grasp = mano_params_to_grasp_dict(
                preds[0],
                betas,
                model.mano,
                K_224,
                model.mesh_faces,
            )
            return HugGraspProposal(
                T_camera_wrist=np.asarray(grasp["T_camera_wrist"], dtype=np.float32),
                landmarks_3d=np.asarray(grasp["landmarks_3d"], dtype=np.float32),
                landmarks_2d=np.asarray(grasp["landmarks_2d"], dtype=np.float32),
                pose=np.asarray(grasp["pose"], dtype=np.float32),
                shape=np.asarray(grasp["shape"], dtype=np.float32),
                t=np.asarray(grasp["t"], dtype=np.float32),
                click_uv_224=(float(u_224), float(v_224)),
                depth_at_click_m=float(depth_m),
                camera_K_224=K_224.astype(np.float32),
                raw=grasp,
            )
        finally:
            if cleanup is not None:
                cleanup.cleanup()

    @staticmethod
    def _can_import() -> bool:
        try:
            import hug  # noqa: F401

            return True
        except ImportError:
            return False
