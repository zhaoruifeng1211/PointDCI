#!/usr/bin/env python3

from __future__ import annotations

import argparse
import math
from pathlib import Path
from statistics import mean, pstdev


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Read C2M_signed_distances from text files, take absolute values, "
            "drop values greater than the threshold, and save summary stats."
        )
    )
    parser.add_argument(
        "inputs",
        nargs="+",
        type=Path,
        help="Input .txt files that contain C2M_signed_distances in the 4th column.",
    )
    parser.add_argument(
        "--threshold",
        type=float,
        default=2.0,
        help="Drop absolute values greater than this threshold. Default: 2.0",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Optional directory for output files. Default: same directory as each input.",
    )
    return parser.parse_args()


def load_filtered_distances(path: Path, threshold: float) -> tuple[list[float], int]:
    filtered: list[float] = []
    total_count = 0

    with path.open("r", encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, start=1):
            stripped = line.strip()
            if not stripped or stripped.startswith("//"):
                continue

            parts = stripped.split()

            # The second non-comment line is the point count header, not data.
            if len(parts) == 1 and line_number <= 3:
                continue

            if len(parts) < 4:
                raise ValueError(f"{path}: invalid data line {line_number}: {stripped}")

            value = abs(float(parts[3]))
            total_count += 1
            if value <= threshold:
                filtered.append(value)

    return filtered, total_count


def build_output_path(input_path: Path, output_dir: Path | None) -> Path:
    target_dir = output_dir if output_dir is not None else input_path.parent
    return target_dir / f"{input_path.stem}_stats.txt"


def compute_stats(values: list[float]) -> tuple[float, float, float]:
    avg = mean(values)
    std = pstdev(values)
    rms = math.sqrt(sum(value * value for value in values) / len(values))
    return avg, std, rms


def save_stats(
    output_path: Path,
    input_path: Path,
    threshold: float,
    kept_values: list[float],
    total_count: int,
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    avg, std, rms = compute_stats(kept_values)

    lines = [
        f"input_file: {input_path}",
        f"metric: C2M_signed_distances",
        "transform: abs(value)",
        f"filter: abs(value) <= {threshold}",
        f"total_count: {total_count}",
        f"kept_count: {len(kept_values)}",
        f"removed_count: {total_count - len(kept_values)}",
        f"mean: {avg:.10f}",
        f"std: {std:.10f}",
        f"RMS: {rms:.10f}",
    ]

    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    args = parse_args()

    for input_path in args.inputs:
        if not input_path.is_file():
            raise FileNotFoundError(f"Input file not found: {input_path}")

        kept_values, total_count = load_filtered_distances(input_path, args.threshold)
        if not kept_values:
            raise ValueError(
                f"{input_path}: no values remain after filtering with threshold {args.threshold}"
            )

        output_path = build_output_path(input_path, args.output_dir)
        save_stats(output_path, input_path, args.threshold, kept_values, total_count)
        print(f"Saved stats to {output_path}")


if __name__ == "__main__":
    main()
