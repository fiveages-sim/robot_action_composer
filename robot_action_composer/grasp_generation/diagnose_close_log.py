#!/usr/bin/env python3
"""Diagnose close-executor JSONL logs (commanded vs measured, contact, open pulses).

Usage (from examples/IsaacSim or repo root)::

    python -m robot_action_composer.grasp_generation.diagnose_close_log \\
        submodules/hug/data/w2_wuji_head_capture/close_logs/B4_right_auto_apple.jsonl

    # Compare latest of each executor:
    python -m robot_action_composer.grasp_generation.diagnose_close_log --latest-dir \\
        submodules/hug/data/w2_wuji_head_capture/close_logs
"""

from __future__ import annotations

import argparse
import json
import sys
from collections import Counter
from pathlib import Path
from typing import Any

import numpy as np

_NAMES = [
    "t_flex",
    "t_abd",
    "t_mcp",
    "t_ip",
    "i_flex",
    "i_abd",
    "i_pip",
    "i_dip",
    "m_flex",
    "m_abd",
    "m_pip",
    "m_dip",
    "r_flex",
    "r_abd",
    "r_pip",
    "r_dip",
    "p_flex",
    "p_abd",
    "p_pip",
    "p_dip",
]
_FLEX = [i for i in range(20) if i % 4 != 1]

# Same keep/ignore heuristics as wuji_ros (for offline "would ignore" audit).
_OBJECT_KEEP = (
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
_BAD_IGNORE_TOKENS = ("default_environment", "table", "desk", "plane")


def _load(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with path.open(encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def _flex(q: np.ndarray) -> float:
    return float(np.mean(np.abs(q[_FLEX])))


def _object_partners(partners: dict[str, Any]) -> list[tuple[str, str]]:
    out: list[tuple[str, str]] = []
    for link, paths in (partners or {}).items():
        for p in paths or []:
            low = str(p).lower()
            if any(h in low for h in _OBJECT_KEEP):
                out.append((link, str(p)))
    return out


def diagnose(path: Path) -> None:
    rows = _load(path)
    if not rows:
        print(f"{path}: empty")
        return
    print(f"\n==== {path.name}  n={len(rows)} ====")
    print("phases:", Counter(r.get("phase") for r in rows))

    qs = np.array([r["q"] for r in rows], dtype=float)
    flex_cmd = np.array([_flex(q) for q in qs])
    has_meas = any(r.get("q_meas") for r in rows)
    if has_meas:
        qm = np.array(
            [
                r["q_meas"] if r.get("q_meas") else r["q"]
                for r in rows
            ],
            dtype=float,
        )
        flex_meas = np.array([_flex(q) for q in qm])
    else:
        qm = None
        flex_meas = None
        print("NOTE: no q_meas in log (re-run after update to compare bounce)")

    prev = None
    for i, r in enumerate(rows):
        if r.get("phase") != prev:
            fr = r.get("frozen_fingers")
            c = r.get("contact")
            force = {k: v for k, v in (r.get("force_mag") or {}).items() if v}
            print(
                f"  @{i:3d} {r.get('phase'):10s} flex_cmd={flex_cmd[i]:.3f}"
                + (f" flex_meas={flex_meas[i]:.3f}" if flex_meas is not None else "")
                + f" frozen={fr} contact={c}"
            )
            if force:
                print(f"       force>0: {force}")
            objs = _object_partners(r.get("contact_partners") or {})
            if objs:
                print(f"       object_partners: {objs[:4]}")
            prev = r.get("phase")

    # Commanded toward-open pulses (true open commands)
    n_cmd_open = 0
    for i in range(1, len(rows)):
        d = qs[i] - qs[i - 1]
        hits = []
        for j, name in enumerate(_NAMES):
            if "abd" in name:
                if abs(qs[i, j]) < abs(qs[i - 1, j]) - 0.04:
                    hits.append(name)
            elif qs[i, j] < qs[i - 1, j] - 0.04 and qs[i - 1, j] > 0.05:
                hits.append(name)
        if hits:
            n_cmd_open += 1
            if n_cmd_open <= 5:
                print(
                    f"  CMD open-ish @{i} {rows[i].get('phase')}: {hits}"
                )
    print(f"commanded toward-open steps: {n_cmd_open}")

    # Measured opening while command holds/closes (PhysX yield)
    n_meas_open = 0
    if qm is not None:
        for i in range(1, len(rows)):
            dcmd = flex_cmd[i] - flex_cmd[i - 1]
            dmeas = flex_meas[i] - flex_meas[i - 1]
            if dmeas < -0.02 and dcmd > -0.005:
                n_meas_open += 1
                if n_meas_open <= 5:
                    print(
                        f"  MEAS open @{i} {rows[i].get('phase')} "
                        f"Δmeas={dmeas:.3f} Δcmd={dcmd:.3f} "
                        f"cmd={flex_cmd[i]:.3f} meas={flex_meas[i]:.3f} "
                        f"frozen={rows[i].get('frozen_fingers')}"
                    )
        print(f"meas opening while cmd hold/close: {n_meas_open} steps")

        n_bounce = 0
        for i in range(len(rows)):
            gap = flex_cmd[i] - flex_meas[i]
            if gap > 0.08:
                force = {
                    k: v for k, v in (rows[i].get("force_mag") or {}).items() if v
                }
                objs = _object_partners(rows[i].get("contact_partners") or {})
                if force or objs or rows[i].get("any_contact"):
                    n_bounce += 1
                    if n_bounce <= 5:
                        print(
                            f"  YIELD @{i} {rows[i].get('phase')} "
                            f"cmd_flex={flex_cmd[i]:.3f} meas_flex={flex_meas[i]:.3f} "
                            f"gap={gap:.3f} force={force}"
                        )
        print(f"cmd>>meas with object/force (PhysX yield): {n_bounce} steps")

    # Object partner seen but finger contact never True → ignore-list bug
    n_obj = 0
    n_obj_no_contact = 0
    n_frozen_any = 0
    bad_ignore_hits: list[str] = []
    for r in rows:
        objs = _object_partners(r.get("contact_partners") or {})
        if objs:
            n_obj += 1
            if not any((r.get("contact") or {}).values()):
                n_obj_no_contact += 1
                for _, p in objs[:1]:
                    low = p.lower()
                    for tok in _BAD_IGNORE_TOKENS:
                        if tok in low and tok not in bad_ignore_hits:
                            bad_ignore_hits.append(tok)
        if r.get("frozen_fingers"):
            n_frozen_any += 1
    print(
        f"steps with object partner: {n_obj}/{len(rows)}; "
        f"of those contact=all-False: {n_obj_no_contact}"
    )
    if bad_ignore_hits:
        print(
            f"  HINT: object paths contain {bad_ignore_hits} — "
            f"if ignore_contact_partners lists these, gate will never freeze"
        )
    print(f"steps with any frozen finger: {n_frozen_any}/{len(rows)}")
    n_soft = sum(1 for r in rows if r.get("phase") == "soft_stop")
    n_pinch = sum(1 for r in rows if r.get("phase") == "tip_pinch")
    print(f"soft_stop steps: {n_soft}; tip_pinch steps: {n_pinch}")

    # Peak force trajectory (over-squeeze after grip)
    peaks: list[tuple[int, str, float]] = []
    for i, r in enumerate(rows):
        force = r.get("force_mag") or {}
        if force:
            pk = float(max(force.values()))
            if pk >= 1.0:
                peaks.append((i, str(r.get("phase")), pk))
    if peaks:
        i_max, ph_max, pk_max = max(peaks, key=lambda t: t[2])
        print(f"peak force: {pk_max:.1f} N @{i_max} ({ph_max})")
        # force collapse after peak while still commanding close
        after = [t for t in peaks if t[0] > i_max]
        if after and pk_max >= 8.0:
            i_lo, ph_lo, pk_lo = min(after, key=lambda t: t[2])
            if pk_lo < 0.5 * pk_max:
                print(
                    f"  force collapse after peak: {pk_max:.1f}→{pk_lo:.1f} N "
                    f"by @{i_lo} ({ph_lo}) — post-grip over-curl?"
                )

    n_false = 0
    for r in rows:
        if not any((r.get("contact") or {}).values()):
            continue
        if not any(v > 0.5 for v in (r.get("force_mag") or {}).values()):
            n_false += 1
    print(f"contact=True but force<0.5 (false Bool): {n_false}/{len(rows)} steps")


def latest_per_executor(log_dir: Path) -> list[Path]:
    best: dict[str, Path] = {}
    for p in log_dir.glob("*_right_*.jsonl"):
        if p.name.endswith("_summary.jsonl"):
            continue
        eid = p.name.split("_", 1)[0]
        if eid not in best or p.stat().st_mtime > best[eid].stat().st_mtime:
            best[eid] = p
    return [best[k] for k in sorted(best)]


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("paths", nargs="*", type=Path, help="JSONL close logs")
    ap.add_argument(
        "--latest-dir",
        type=Path,
        default=None,
        help="Diagnose latest JSONL per executor in this directory",
    )
    args = ap.parse_args(argv)
    paths: list[Path] = list(args.paths)
    if args.latest_dir:
        paths.extend(latest_per_executor(args.latest_dir))
    if not paths:
        ap.print_help()
        return 2
    for p in paths:
        if not p.is_file():
            print(f"missing: {p}", file=sys.stderr)
            continue
        diagnose(p)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
