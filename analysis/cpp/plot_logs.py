#!/usr/bin/env python3
import argparse
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt

REQUIRED_COLUMNS = [
    "t",
    "ee_x", "ee_y", "ee_z",
    "des_x", "des_y", "des_z",
    "err_x", "err_y", "err_z",
    "following_error",
]


def read_csv(path: Path):
    with path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    if not rows:
        raise SystemExit(f"no rows in {path}")
    if reader.fieldnames is None:
        raise SystemExit(f"invalid csv: {path}")
    missing = [c for c in REQUIRED_COLUMNS if c not in reader.fieldnames]
    if missing:
        msg = (
            f"{path} does not have required header columns.\n"
            f"missing: {', '.join(missing)}\n"
            "This usually means you selected an old log file generated before header/target/error logging was added.\n"
            "Use the latest file that starts with header: 't,ee_x,ee_y,...'."
        )
        raise SystemExit(msg)
    return rows


def to_float(rows, key):
    out = []
    for r in rows:
        v = r.get(key, "")
        try:
            out.append(float(v))
        except ValueError:
            out.append(float("nan"))
    return out


def has_columns(rows, columns):
    if not rows:
        return False
    return all(c in rows[0] for c in columns)


def has_any_finite(values):
    return any(v == v for v in values)


def set_wide_ylim(ax, values, factor=1.8):
    finite_vals = [v for v in values if isinstance(v, (int, float)) and math.isfinite(v)]
    if not finite_vals:
        return
    vmin = min(finite_vals)
    vmax = max(finite_vals)
    center = 0.5 * (vmin + vmax)
    half = 0.5 * (vmax - vmin)
    if half <= 1e-12:
        half = max(abs(center) * 0.1, 1e-3)
    half *= factor
    ax.set_ylim(center - half, center + half)


def main():
    ap = argparse.ArgumentParser(description="Plot desired vs EE position and following error from cpp_data_*.csv")
    ap.add_argument("csv_path", type=Path, help="Path to cpp_data_*.csv (with header)")
    ap.add_argument("--save", type=Path, default=None, help="If set, save figure to this path")
    args = ap.parse_args()

    rows = read_csv(args.csv_path)

    t = to_float(rows, "t")
    ee_x = to_float(rows, "ee_x")
    ee_y = to_float(rows, "ee_y")
    ee_z = to_float(rows, "ee_z")
    des_x = to_float(rows, "des_x")
    des_y = to_float(rows, "des_y")
    des_z = to_float(rows, "des_z")
    err_x = to_float(rows, "err_x")
    err_y = to_float(rows, "err_y")
    err_z = to_float(rows, "err_z")
    fe = to_float(rows, "following_error")

    # If everything is NaN, fail fast with a useful hint instead of blank axes.
    if not any(v == v for v in t):
        raise SystemExit(
            f"all time values are NaN from {args.csv_path}. "
            "Please check that the file is a new-format CSV with header."
        )

    fig, axes = plt.subplots(2, 3, figsize=(14, 8), sharex=True)
    ax = axes[0, 0]
    ax.plot(t, des_x, label="des_x")
    ax.plot(t, ee_x, "--", label="cur_x")
    ax.set_ylabel("x (m)")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8, loc="upper right")
    ax.set_title("ee_x")

    ax = axes[0, 1]
    ax.plot(t, des_y, label="des_y")
    ax.plot(t, ee_y, "--", label="cur_y")
    ax.set_ylabel("y (m)")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8, loc="upper right")
    ax.set_title("ee_y")

    ax = axes[0, 2]
    ax.plot(t, des_z, label="des_z")
    ax.plot(t, ee_z, "--", label="cur_z")
    ax.set_ylabel("z (m)")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8, loc="upper right")
    ax.set_title("ee_z")
    set_wide_ylim(ax, des_z + ee_z, factor=2.2)

    fig.tight_layout()

    pos_err_series = [
        ("err_x", err_x, "x error (m)"),
        ("err_y", err_y, "y error (m)"),
        ("err_z", err_z, "z error (m)"),
        ("error_pos", fe, "pos norm (m)"),
    ]
    ori_err_series = []

    ori_cols = [
        "ee_roll", "ee_pitch",
        "des_roll", "des_pitch",
        "err_roll", "err_pitch",
        "following_error_ori",
    ]
    if has_columns(rows, ori_cols):
        ee_roll = to_float(rows, "ee_roll")
        ee_pitch = to_float(rows, "ee_pitch")
        des_roll = to_float(rows, "des_roll")
        des_pitch = to_float(rows, "des_pitch")
        err_roll = to_float(rows, "err_roll")
        err_pitch = to_float(rows, "err_pitch")
        fe_ori = to_float(rows, "following_error_ori")

        ax = axes[1, 0]
        ax.plot(t, des_roll, label="des_roll")
        ax.plot(t, ee_roll, "--", label="cur_roll")
        ax.set_xlabel("time (s)")
        ax.set_ylabel("roll (rad)")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8, loc="upper right")
        ax.set_title("roll")

        ax = axes[1, 1]
        ax.plot(t, des_pitch, label="des_pitch")
        ax.plot(t, ee_pitch, "--", label="cur_pitch")
        ax.set_xlabel("time (s)")
        ax.set_ylabel("pitch (rad)")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8, loc="upper right")
        ax.set_title("pitch")
        set_wide_ylim(ax, des_pitch + ee_pitch, factor=2.2)

        ori_err_series.append(("err_roll", err_roll, "roll error (rad)"))
        ori_err_series.append(("err_pitch", err_pitch, "pitch error (rad)"))
        if has_any_finite(fe_ori):
            ori_err_series.append(("error_ori", fe_ori, "ori norm (rad)"))

        axes[1, 2].axis("off")
    else:
        # Fallback when orientation columns are missing.
        axes[1, 0].axis("off")
        axes[1, 1].axis("off")
        axes[1, 2].axis("off")

    fig.tight_layout()

    # Position error figure: 2x2 layout.
    fig_err_pos, axes_err_pos = plt.subplots(2, 2, figsize=(12, 8), sharex=True)
    flat_axes_err_pos = axes_err_pos.flatten()
    for i, (name, values, ylabel) in enumerate(pos_err_series):
        ax = flat_axes_err_pos[i]
        ax.plot(t, values, label=name)
        ax.set_xlabel("time (s)")
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8, loc="upper right")
        ax.set_title(name)
        if name == "err_z":
            set_wide_ylim(ax, values, factor=2.2)
    fig_err_pos.tight_layout()

    # Orientation error figure: 3x1 layout.
    fig_err_ori = None
    if ori_err_series:
        fig_err_ori, axes_err_ori = plt.subplots(3, 1, figsize=(8, 10), sharex=True)
        for i, (name, values, ylabel) in enumerate(ori_err_series):
            ax = axes_err_ori[i]
            ax.plot(t, values, label=name)
            ax.set_xlabel("time (s)")
            ax.set_ylabel(ylabel)
            ax.grid(True, alpha=0.3)
            ax.legend(fontsize=8, loc="upper right")
            ax.set_title(name)
            if name == "err_pitch":
                set_wide_ylim(ax, values, factor=2.2)
        for j in range(len(ori_err_series), 3):
            axes_err_ori[j].axis("off")
        fig_err_ori.tight_layout()

    q_cols = [f"des_q_{i}" for i in range(1, 5)]
    if has_columns(rows, q_cols):
        des_q = [to_float(rows, f"des_q_{i}") for i in range(1, 5)]
        q = [to_float(rows, f"q_{i}") for i in range(1, 5)]
        fig_q, axes_q = plt.subplots(2, 2, figsize=(12, 8), sharex=True)
        for i in range(4):
            r, c = divmod(i, 2)
            axq = axes_q[r, c]
            axq.plot(t, des_q[i], label=f"des_q{i + 1}")
            axq.plot(t, q[i], "--", label=f"cur_q{i + 1}")
            axq.set_ylabel(f"q{i + 1} (rad)")
            axq.set_xlabel("time (s)")
            axq.grid(True, alpha=0.3)
            axq.legend(fontsize=8, loc="upper right")
            axq.set_title(f"q{i + 1}")
        fig_q.tight_layout()

    q_err_cols = [f"err_q_{i}" for i in range(1, 5)]
    if has_columns(rows, q_err_cols):
        q_err = [to_float(rows, f"err_q_{i}") for i in range(1, 5)]
        fig_q_err, axes_q_err = plt.subplots(2, 2, figsize=(12, 8), sharex=True)
        for i in range(4):
            r, c = divmod(i, 2)
            axe = axes_q_err[r, c]
            axe.plot(t, q_err[i], label=f"err_q{i + 1}")
            axe.set_ylabel(f"err_q{i + 1} (rad)")
            axe.set_xlabel("time (s)")
            axe.grid(True, alpha=0.3)
            axe.legend(fontsize=8, loc="upper right")
            axe.set_title(f"err_q{i + 1}")
        fig_q_err.tight_layout()

    if args.save:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(args.save, dpi=150)
        if "fig_q" in locals():
            q_path = args.save.with_name(f"{args.save.stem}_q{args.save.suffix}")
            fig_q.savefig(q_path, dpi=150)
        if "fig_q_err" in locals():
            q_err_path = args.save.with_name(f"{args.save.stem}_q_err{args.save.suffix}")
            fig_q_err.savefig(q_err_path, dpi=150)
        err_pos_path = args.save.with_name(f"{args.save.stem}_err_pos{args.save.suffix}")
        fig_err_pos.savefig(err_pos_path, dpi=150)
        if fig_err_ori is not None:
            err_ori_path = args.save.with_name(f"{args.save.stem}_err_ori{args.save.suffix}")
            fig_err_ori.savefig(err_ori_path, dpi=150)
    else:
        plt.show()


if __name__ == "__main__":
    main()

