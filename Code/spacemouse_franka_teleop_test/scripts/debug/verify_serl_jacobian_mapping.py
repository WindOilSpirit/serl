#!/usr/bin/env python3
"""Verify SERL debug CSV Jacobian mapping against logged tau_task."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


def as_float(row: dict[str, str], key: str) -> float:
    try:
        return float(row.get(key, "nan"))
    except ValueError:
        return math.nan


def vector(row: dict[str, str], prefix: str, count: int, start: int = 1) -> list[float]:
    return [as_float(row, f"{prefix}_{index}") for index in range(start, start + count)]


def mat_vec_mul_transpose(matrix: list[list[float]], wrench: list[float]) -> list[float]:
    return [sum(matrix[row][col] * wrench[row] for row in range(6)) for col in range(7)]


def matrix_column_major(raw: list[float]) -> list[list[float]]:
    return [[raw[row + 6 * col] for col in range(7)] for row in range(6)]


def matrix_row_major(raw: list[float]) -> list[list[float]]:
    return [[raw[7 * row + col] for col in range(7)] for row in range(6)]


def rms_error(a: list[float], b: list[float]) -> float:
    pairs = [(x, y) for x, y in zip(a, b) if math.isfinite(x) and math.isfinite(y)]
    if not pairs:
        return math.nan
    return math.sqrt(sum((x - y) ** 2 for x, y in pairs) / len(pairs))


def max_abs_error(a: list[float], b: list[float]) -> float:
    pairs = [(x, y) for x, y in zip(a, b) if math.isfinite(x) and math.isfinite(y)]
    if not pairs:
        return math.nan
    return max(abs(x - y) for x, y in pairs)


def format_vector(values: list[float]) -> str:
    return " ".join(f"{value:+.6f}" for value in values)


def select_rows(rows: list[dict[str, str]], phase: str, limit: int) -> list[dict[str, str]]:
    filtered = [row for row in rows if row.get("phase") == phase]
    if not filtered:
        filtered = rows
    if len(filtered) <= limit:
        return filtered
    step = max(1, len(filtered) // limit)
    selected = filtered[::step][:limit]
    if filtered[-1] not in selected:
        selected.append(filtered[-1])
    return selected


def verify_row(row: dict[str, str]) -> dict[str, tuple[float, float, list[float]]]:
    raw_jacobian = vector(row, "zero_jacobian", 42, start=0)
    logged_tau_task = vector(row, "tau_task", 7)
    force = [
        as_float(row, "cartesian_force_x"),
        as_float(row, "cartesian_force_y"),
        as_float(row, "cartesian_force_z"),
    ]
    torque = [
        as_float(row, "cartesian_torque_x"),
        as_float(row, "cartesian_torque_y"),
        as_float(row, "cartesian_torque_z"),
    ]
    wrench = force + torque
    swapped_wrench = torque + force

    candidates = {
        "column_major_FT": mat_vec_mul_transpose(matrix_column_major(raw_jacobian), wrench),
        "column_major_TF": mat_vec_mul_transpose(matrix_column_major(raw_jacobian), swapped_wrench),
        "row_major_FT": mat_vec_mul_transpose(matrix_row_major(raw_jacobian), wrench),
        "row_major_TF": mat_vec_mul_transpose(matrix_row_major(raw_jacobian), swapped_wrench),
    }
    return {
        name: (rms_error(values, logged_tau_task), max_abs_error(values, logged_tau_task), values)
        for name, values in candidates.items()
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Compare Jacobian Eigen mapping candidates against logged tau_task."
    )
    parser.add_argument("csv_path", type=Path)
    parser.add_argument("--phase", default="offset_plus_x")
    parser.add_argument("--limit", type=int, default=20)
    args = parser.parse_args()

    rows = list(csv.DictReader(args.csv_path.open()))
    selected = select_rows(rows, args.phase, args.limit)
    if not selected:
        raise SystemExit("CSV has no data rows.")

    aggregate: dict[str, list[float]] = {}
    last_results: dict[str, tuple[float, float, list[float]]] | None = None
    for row in selected:
        results = verify_row(row)
        last_results = results
        for name, (rms, _max_abs, _values) in results.items():
            aggregate.setdefault(name, []).append(rms)

    print(f"CSV: {args.csv_path}")
    print(f"Rows checked: {len(selected)} phase={args.phase}")
    print("Mean RMS error against logged tau_task:")
    for name, errors in sorted(aggregate.items(), key=lambda item: sum(item[1]) / len(item[1])):
        mean_error = sum(errors) / len(errors)
        print(f"  {name}: {mean_error:.12g}")

    assert last_results is not None
    best_name = min(
        last_results,
        key=lambda name: last_results[name][0],
    )
    print(f"Best mapping on final checked row: {best_name}")
    print(f"  RMS error: {last_results[best_name][0]:.12g}")
    print(f"  max abs error: {last_results[best_name][1]:.12g}")
    print("  calculated tau_task:")
    print(f"    {format_vector(last_results[best_name][2])}")
    print("  logged tau_task:")
    print(f"    {format_vector(vector(selected[-1], 'tau_task', 7))}")


if __name__ == "__main__":
    main()
