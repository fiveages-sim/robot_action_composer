"""JSONL / summary writers for ablation tables."""

from __future__ import annotations

import json
import time
from dataclasses import asdict
from pathlib import Path
from typing import Any, Optional

from .types import CloseRunResult


def write_close_log(
    result: CloseRunResult,
    out_dir: Path,
    *,
    trial_id: str = "",
) -> Path:
    """Write one trial JSONL (steps) + summary JSON beside it."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    tid = trial_id.strip() or stamp
    stem = f"{result.executor_id}_{result.arm}_{tid}"

    jsonl = out_dir / f"{stem}.jsonl"
    with jsonl.open("w", encoding="utf-8") as f:
        for step in result.steps:
            f.write(json.dumps(asdict(step), ensure_ascii=False) + "\n")

    summary = {
        **result.summary_row(),
        "q_start": result.q_start,
        "q_goal": result.q_goal,
        "q_final": result.q_final,
        "extras": result.extras,
        "trial_id": tid,
    }
    summary_path = out_dir / f"{stem}_summary.json"
    summary_path.write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    result.log_path = str(jsonl)
    return jsonl


def append_table_row(
    csv_path: Path,
    result: CloseRunResult,
    *,
    extra: Optional[dict[str, Any]] = None,
) -> None:
    """Append one flat row to a CSV used for paper tables."""
    import csv

    row = result.summary_row()
    if extra:
        row.update(extra)
    csv_path = Path(csv_path)
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    write_header = not csv_path.is_file()
    with csv_path.open("a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(row.keys()))
        if write_header:
            writer.writeheader()
        writer.writerow(row)
