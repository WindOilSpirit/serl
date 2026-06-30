#!/usr/bin/env python3
"""Analyze SERL debug CSV Jacobian authority for Cartesian force directions.

The active controller maps Cartesian wrench to joint effort with

    tau_task = J.T @ wrench

This script is intentionally offline: it reads a debug CSV that already
contains zero_jacobian_* and controller status fields, then reports whether the
current posture makes x/y/z force directions expensive, correlated, or
ill-conditioned. It does not command the robot.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from statistics import median

import numpy as np


AXES = ("x", "y", "z")


def as_float(row: dict[str, str], key: str) -> float:
    try:
        return float(row.get(key, "nan"))
    except (TypeError, ValueError):
        return math.nan


def vector(row: dict[str, str], prefix: str, count: int, start: int = 1) -> np.ndarray:
    return np.array([as_float(row, f"{prefix}_{index}") for index in range(start, start + count)])


def jacobian_from_row(row: dict[str, str]) -> np.ndarray:
    raw = vector(row, "zero_jacobian", 42, start=0)
    if not np.all(np.isfinite(raw)):
        raise ValueError("row is missing finite zero_jacobian_0..41 fields")
    return raw.reshape((6, 7), order="F")


def norm(values: np.ndarray) -> float:
    return float(np.linalg.norm(values))


def safe_cosine(a: np.ndarray, b: np.ndarray) -> float:
    denom = norm(a) * norm(b)
    if denom <= 1.0e-12:
        return math.nan
    return float(np.dot(a, b) / denom)


def fmt(value: float, digits: int = 6) -> str:
    if not math.isfinite(value):
        return "nan"
    return f"{value:.{digits}g}"


def fmt_vec(values: np.ndarray, digits: int = 5) -> str:
    return "[" + ", ".join(fmt(float(value), digits) for value in values) + "]"


def finite_rows(rows: list[dict[str, str]]) -> list[dict[str, str]]:
    good_rows = []
    for row in rows:
        try:
            jacobian_from_row(row)
        except ValueError:
            continue
        good_rows.append(row)
    return good_rows


def phase_rows(rows: list[dict[str, str]], phase: str | None) -> list[dict[str, str]]:
    if phase is None:
        return rows
    return [row for row in rows if row.get("phase") == phase]


def offset_phases(rows: list[dict[str, str]]) -> list[str]:
    phases = []
    for row in rows:
        phase = row.get("phase", "")
        if phase.startswith("offset_") and phase not in phases:
            phases.append(phase)
    return phases


def select_row(rows: list[dict[str, str]], which: str) -> dict[str, str]:
    if not rows:
        raise ValueError("no rows selected")
    if which == "first":
        return rows[0]
    if which == "mid":
        return rows[len(rows) // 2]
    return rows[-1]


def force_torque_vectors(jacobian: np.ndarray) -> dict[str, np.ndarray]:
    vectors = {}
    for index, axis in enumerate(AXES):
        wrench = np.zeros(6)
        wrench[index] = 1.0
        vectors[axis] = jacobian.T @ wrench
    return vectors


def print_phase_response(rows: list[dict[str, str]]) -> None:
    phases = offset_phases(rows)
    if not phases:
        return
    print("Measured response by offset phase:")
    for phase in phases:
        selected = [row for row in rows if row.get("phase") == phase]
        if len(selected) < 2:
            continue
        first = selected[0]
        last = selected[-1]
        measured_delta_mm = np.array(
            [
                (as_float(last, f"measured_{axis}") - as_float(first, f"measured_{axis}")) * 1000.0
                for axis in AXES
            ]
        )
        target_from_initial_mm = np.array(
            [as_float(last, f"target_delta_from_initial_{axis}") * 1000.0 for axis in AXES]
        )
        measured_from_initial_mm = np.array(
            [as_float(last, f"measured_delta_from_initial_{axis}") * 1000.0 for axis in AXES]
        )
        force = np.array([as_float(last, f"cartesian_force_{axis}") for axis in AXES])
        tau_task = vector(last, "tau_task", 7)
        print(
            f"  {phase}: command_m={row_command(selected)} "
            f"target_from_initial_mm={fmt_vec(target_from_initial_mm, 4)} "
            f"measured_from_initial_mm={fmt_vec(measured_from_initial_mm, 4)} "
            f"phase_motion_mm={fmt_vec(measured_delta_mm, 4)} "
            f"force_N={fmt_vec(force, 4)} tau_task_norm={fmt(norm(tau_task), 4)}"
        )
    print()


def row_command(rows: list[dict[str, str]]) -> str:
    row = rows[-1]
    axis = row.get("command_axis", "")
    offset = as_float(row, "command_offset_m")
    if axis and math.isfinite(offset):
        return f"{axis}:{fmt(offset, 4)}"
    return "unknown"


def analyze_row(row: dict[str, str], label: str, lambdas: list[float]) -> None:
    jacobian = jacobian_from_row(row)
    jj_t = jacobian @ jacobian.T
    j_linear = jacobian[:3, :]
    linear_gram = j_linear @ j_linear.T
    singular_values = np.linalg.svd(jacobian, compute_uv=False)
    linear_singular_values = np.linalg.svd(j_linear, compute_uv=False)
    unit_tau = force_torque_vectors(jacobian)

    print(f"Jacobian authority at {label}:")
    print(
        "  pose_xyz_m="
        + fmt_vec(np.array([as_float(row, f"measured_{axis}") for axis in AXES]), 6)
        + " q="
        + fmt_vec(vector(row, "q", 7), 5)
    )
    print(f"  J singular values: {fmt_vec(singular_values, 5)}")
    print(f"  J_linear singular values: {fmt_vec(linear_singular_values, 5)}")
    print(f"  cond(J): {fmt(singular_values[0] / singular_values[-1], 5)}")
    print()

    print("  Unit force -> tau_task = J.T @ F:")
    for axis in AXES:
        tau = unit_tau[axis]
        print(
            f"    F{axis}=1N: tau_norm={fmt(norm(tau), 5)} "
            f"tau_max_abs={fmt(float(np.max(np.abs(tau))), 5)} tau={fmt_vec(tau, 5)}"
        )
    print()

    print("  Unit-force torque-vector cosines:")
    for left, right in (("x", "y"), ("x", "z"), ("y", "z")):
        cosine = safe_cosine(unit_tau[left], unit_tau[right])
        print(f"    cos(F{left}, F{right})={fmt(cosine, 6)}")
    print(f"  J_linear @ J_linear.T:")
    for row_values in linear_gram:
        print(f"    {fmt_vec(row_values, 6)}")
    print()

    print("  Damped inverse-preconditioned mapping tau = J.T @ inv(JJ.T + lambda I) @ F:")
    identity = np.eye(6)
    for lambda_value in lambdas:
        damped_inv = np.linalg.inv(jj_t + lambda_value * identity)
        norms = []
        for axis_index in range(3):
            wrench = np.zeros(6)
            wrench[axis_index] = 1.0
            tau = jacobian.T @ (damped_inv @ wrench)
            norms.append(norm(tau))
        print(f"    lambda={fmt(lambda_value, 4)} tau_norm_per_1N_xyz={fmt_vec(np.array(norms), 5)}")
    print()


def aggregate_phase_authority(rows: list[dict[str, str]]) -> None:
    phases = offset_phases(rows)
    if not phases:
        return
    print("Median unit-force tau_norm by phase:")
    for phase in phases:
        selected = finite_rows([row for row in rows if row.get("phase") == phase])
        if not selected:
            continue
        values: dict[str, list[float]] = {axis: [] for axis in AXES}
        cos_xz: list[float] = []
        for row in selected:
            unit_tau = force_torque_vectors(jacobian_from_row(row))
            for axis in AXES:
                values[axis].append(norm(unit_tau[axis]))
            cos_xz.append(safe_cosine(unit_tau["x"], unit_tau["z"]))
        print(
            f"  {phase}: "
            f"Fx={fmt(median(values['x']), 5)} "
            f"Fy={fmt(median(values['y']), 5)} "
            f"Fz={fmt(median(values['z']), 5)} "
            f"cos(Fx,Fz)={fmt(median(cos_xz), 5)}"
        )
    print()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Offline Jacobian authority analysis for SERL controller debug CSVs."
    )
    parser.add_argument("csv_path", type=Path)
    parser.add_argument(
        "--phase",
        default=None,
        help="Analyze a specific phase. Default uses all rows for aggregate and the final row.",
    )
    parser.add_argument("--row", choices=("first", "mid", "last"), default="last")
    parser.add_argument(
        "--lambda",
        dest="lambdas",
        type=float,
        nargs="*",
        default=[0.001, 0.01, 0.05, 0.1, 0.5, 1.0],
        help="Damping lambda values for inverse-preconditioned torque estimates.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    with args.csv_path.open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise SystemExit("CSV has no data rows.")

    print(f"CSV: {args.csv_path}")
    print(f"Rows: {len(rows)}")
    print()

    print_phase_response(rows)
    aggregate_phase_authority(rows)

    selected = finite_rows(phase_rows(rows, args.phase))
    if not selected:
        raise SystemExit("No selected rows contain complete zero_jacobian fields.")
    selected_row = select_row(selected, args.row)
    label = f"phase={args.phase or selected_row.get('phase', '')} row={args.row}"
    analyze_row(selected_row, label, args.lambdas)


if __name__ == "__main__":
    main()
