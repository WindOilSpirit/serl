#!/usr/bin/env python3
"""Offline check: compare zeroJacobian*dq velocity with O_T_EE pose-diff velocity."""

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


def finite_vec(values: tuple[float, float, float]) -> bool:
    return all(math.isfinite(value) for value in values)


def norm(values: tuple[float, float, float]) -> float:
    if not finite_vec(values):
        return math.nan
    return math.sqrt(sum(value * value for value in values))


def dot(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    if not finite_vec(a) or not finite_vec(b):
        return math.nan
    return sum(a[index] * b[index] for index in range(3))


def cosine(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    an = norm(a)
    bn = norm(b)
    if not math.isfinite(an) or not math.isfinite(bn) or an <= 1.0e-12 or bn <= 1.0e-12:
        return math.nan
    return dot(a, b) / (an * bn)


def jacobian_velocity(row: dict[str, str]) -> tuple[float, float, float]:
    jacobian = [as_float(row, f"zero_jacobian_{index}") for index in range(42)]
    dq = [as_float(row, f"dq_{index}") for index in range(1, 8)]
    if not all(math.isfinite(value) for value in jacobian + dq):
        return (math.nan, math.nan, math.nan)
    return tuple(
        sum(jacobian[row_index + 6 * col] * dq[col] for col in range(7))
        for row_index in range(3)
    )


def o_t_ee_position(row: dict[str, str]) -> tuple[float, float, float]:
    return (
        as_float(row, "O_T_EE_12"),
        as_float(row, "O_T_EE_13"),
        as_float(row, "O_T_EE_14"),
    )


def row_stamp(row: dict[str, str]) -> float:
    for key in ("stamp_sec", "controller_stamp_sec", "time_since_start_s"):
        value = as_float(row, key)
        if math.isfinite(value):
            return value
    return math.nan


def pose_diff_velocity(
    previous_row: dict[str, str] | None,
    row: dict[str, str],
) -> tuple[float, float, float]:
    if previous_row is None:
        return (math.nan, math.nan, math.nan)
    previous_position = o_t_ee_position(previous_row)
    position = o_t_ee_position(row)
    previous_stamp = row_stamp(previous_row)
    stamp = row_stamp(row)
    if not all(math.isfinite(value) for value in (*previous_position, *position, previous_stamp, stamp)):
        return (math.nan, math.nan, math.nan)
    dt = stamp - previous_stamp
    if dt <= 0.0:
        return (math.nan, math.nan, math.nan)
    return tuple((position[index] - previous_position[index]) / dt for index in range(3))


def diff_norm(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    if not finite_vec(a) or not finite_vec(b):
        return math.nan
    return math.sqrt(sum((a[index] - b[index]) ** 2 for index in range(3)))


def mean(values: list[float]) -> float:
    finite = [value for value in values if math.isfinite(value)]
    if not finite:
        return math.nan
    return sum(finite) / len(finite)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Compare J*dq translational velocity with O_T_EE pose-diff velocity."
    )
    parser.add_argument("csv_path", type=Path)
    parser.add_argument("--phase", default="offset_plus_x")
    args = parser.parse_args()

    rows = list(csv.DictReader(args.csv_path.open()))
    phase_rows = [row for row in rows if row.get("phase") == args.phase]
    if not phase_rows:
        raise SystemExit(f"No rows found for phase={args.phase!r}")

    cosines: list[float] = []
    ratios: list[float] = []
    diffs: list[float] = []
    valid = 0
    previous: dict[str, str] | None = None
    for row in phase_rows:
        jv = jacobian_velocity(row)
        pv = pose_diff_velocity(previous, row)
        previous = row
        if not finite_vec(jv) or not finite_vec(pv):
            continue
        jn = norm(jv)
        pn = norm(pv)
        if jn > 1.0e-12 and pn > 1.0e-12:
            cosines.append(cosine(jv, pv))
            ratios.append(jn / pn)
        diffs.append(diff_norm(jv, pv))
        valid += 1

    print(f"CSV: {args.csv_path}")
    print(f"phase: {args.phase}")
    print(f"rows: {len(phase_rows)} valid_velocity_pairs: {valid}")
    print(f"mean direction cosine: {mean(cosines):.12g}")
    print(f"mean |J*dq| / |pose_diff|: {mean(ratios):.12g}")
    print(f"mean velocity_diff_norm: {mean(diffs):.12g} m/s")
    if cosines:
        print(f"min direction cosine: {min(cosines):.12g}")
    if diffs:
        print(f"max velocity_diff_norm: {max(diffs):.12g} m/s")


if __name__ == "__main__":
    main()
