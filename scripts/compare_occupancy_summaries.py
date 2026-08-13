#!/usr/bin/env python3

"""Compare two captured OccupancyGrid summaries and emit parity evidence."""

import argparse
import json
import math
from pathlib import Path
from typing import Any, Dict, Iterable, List


def load(path: Path) -> Dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path}: expected a JSON object")
    return value


def nearly_equal(left: Any, right: Any, tolerance: float) -> bool:
    try:
        return math.isclose(float(left), float(right), rel_tol=0.0,
                            abs_tol=tolerance)
    except (TypeError, ValueError):
        return left == right


def origin_values(summary: Dict[str, Any]) -> List[float]:
    origin = summary.get("origin", {})
    position = origin.get("position", {}) if isinstance(origin, dict) else {}
    return [
        float(position.get("x", float("nan"))),
        float(position.get("y", float("nan"))),
        float(position.get("z", float("nan"))),
    ]


def semantic_counts(summary: Dict[str, Any]) -> Dict[str, int]:
    semantics = summary.get("cell_semantics", {})
    if not isinstance(semantics, dict):
        return {}
    return {
        "occupied_positive": int(semantics.get("occupied_positive", 0)),
        "free_zero": int(semantics.get("free_zero", 0)),
        "unknown_negative": int(semantics.get("unknown_negative", 0)),
    }


def fractions(summary: Dict[str, Any]) -> Dict[str, float]:
    counts = semantic_counts(summary)
    total = sum(counts.values())
    if total <= 0:
        return {key: 0.0 for key in counts}
    return {key: value / total for key, value in counts.items()}


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Compare two record_occupancy_summary.py outputs."
    )
    parser.add_argument("left", type=Path)
    parser.add_argument("right", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--origin-tolerance", type=float, default=1.0e-6)
    parser.add_argument("--resolution-tolerance", type=float, default=1.0e-9)
    parser.add_argument(
        "--fail-on-metadata-mismatch",
        action="store_true",
        help="return failure when topic/frame/resolution/shape/origin differ",
    )
    args = parser.parse_args()

    left = load(args.left)
    right = load(args.right)
    metadata_keys = ["topic", "frame_id", "width", "height"]
    metadata_equal = {
        key: left.get(key) == right.get(key) for key in metadata_keys
    }
    metadata_equal["resolution"] = nearly_equal(
        left.get("resolution"), right.get("resolution"),
        args.resolution_tolerance,
    )
    left_origin = origin_values(left)
    right_origin = origin_values(right)
    metadata_equal["origin"] = (
        len(left_origin) == len(right_origin)
        and all(nearly_equal(a, b, args.origin_tolerance)
                for a, b in zip(left_origin, right_origin))
    )

    left_counts = semantic_counts(left)
    right_counts = semantic_counts(right)
    evidence: Dict[str, Any] = {
        "schema_version": 1,
        "left": str(args.left.resolve()),
        "right": str(args.right.resolve()),
        "left_status": left.get("status"),
        "right_status": right.get("status"),
        "metadata_equal": metadata_equal,
        "metadata_parity": all(metadata_equal.values()),
        "cell_semantics": {
            "left_counts": left_counts,
            "right_counts": right_counts,
            "left_fractions": fractions(left),
            "right_fractions": fractions(right),
            "counts_equal": left_counts == right_counts,
        },
        "data_hash_equal": left.get("data_sha256") == right.get("data_sha256"),
        "classification_hash_equal": (
            left.get("classification_sha256") ==
            right.get("classification_sha256")
        ),
    }

    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.output.with_suffix(args.output.suffix + ".tmp")
    temporary.write_text(
        json.dumps(evidence, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(args.output)

    if args.fail_on_metadata_mismatch:
        return 0 if evidence["metadata_parity"] else 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
