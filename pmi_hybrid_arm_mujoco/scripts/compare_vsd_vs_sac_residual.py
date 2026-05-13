#!/usr/bin/env python3
"""VSD + zero residual vs VSD + SAC task-space force — comparison plots."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any, Callable

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))
from envs.pmi_cable_residual_env import PMICableResidualEnv


def _zero(_o: np.ndarray) -> np.ndarray:
    return np.zeros(3, dtype=np.float32)


def collect(
    env: PMICableResidualEnv,
    pol: Callable[[np.ndarray], np.ndarray],
    *,
    seed: int,
    opt: dict[str, Any],
) -> dict[str, np.ndarray]:
    obs, _ = env.reset(seed=seed, options=opt)
    T: list[float] = []
    ed = np.zeros((0, 3))
    ea = np.zeros((0, 3))
    en: list[float] = []
    Fxyz = np.zeros((0, 3))
    trn: list[float] = []
    tv = np.zeros((0, 4))
    tt = np.zeros((0, 4))
    ti = np.zeros((0, 4))
    to = np.zeros((0, 4))
    hz = np.zeros((0, 3))
    while True:
        a = pol(obs)
        obs, _, term, trunc, inf = env.step(a)
        T.append(float(inf["time"]))
        des = np.asarray(inf["ee_des_xyz"], dtype=float).reshape(1, 3)
        act = np.asarray(inf["ee_act_xyz"], dtype=float).reshape(1, 3)
        ed = np.vstack([ed, des])
        ea = np.vstack([ea, act])
        en.append(float(inf["ee_error_norm"]))
        F = np.asarray(inf["F_residual_xyz"], dtype=float).reshape(1, 3)
        Fxyz = np.vstack([Fxyz, F])
        trn.append(float(np.linalg.norm(inf["tau_residual_jnt"])))
        tv = np.vstack([tv, np.asarray(inf["tau_vsd"], dtype=float).reshape(1, 4)])
        tt = np.vstack([tt, np.asarray(inf["tau_jnt_cmd"], dtype=float).reshape(1, 4)])
        ti = np.vstack([ti, np.asarray(inf["tau_act_ideal"], dtype=float).reshape(1, 4)])
        to = np.vstack([to, np.asarray(inf["tau_act_out"], dtype=float).reshape(1, 4)])
        hz = np.vstack([hz, np.asarray(inf["hys_z_q2q4"], dtype=float).reshape(1, 3)])
        if term or trunc:
            break
    t = np.asarray(T)
    return {
        "t": t,
        "ed": ed,
        "ea": ea,
        "en": np.asarray(en),
        "F": Fxyz,
        "tr": np.asarray(trn),
        "tau_vsd": tv,
        "tau_total": tt,
        "tau_ideal": ti,
        "tau_out": to,
        "hz": hz,
    }


def save_plots(a: dict[str, np.ndarray], b: dict[str, np.ndarray], out: Path, la: str, lb: str) -> None:
    out.mkdir(parents=True, exist_ok=True)
    ta, tb = a["t"], b["t"]

    fig, axs = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
    for i, w in enumerate(["x", "y", "z"]):
        axs[i].plot(ta, a["ed"][:, i], label=f"{la} des")
        axs[i].plot(ta, a["ea"][:, i], "--", label=f"{la} ee")
        axs[i].plot(tb, b["ed"][:, i], label=f"{lb} des")
        axs[i].plot(tb, b["ea"][:, i], "--", label=f"{lb} ee")
        axs[i].set_ylabel(w)
        axs[i].legend(fontsize=6, ncol=2)
    axs[-1].set_xlabel("time [s]")
    fig.tight_layout()
    fig.savefig(out / "01_ee_xyz.png", dpi=150)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(9, 4))
    ax.plot(ta, a["en"], label=la)
    ax.plot(tb, b["en"], label=lb)
    ax.set_ylabel("‖e_ee‖")
    ax.set_xlabel("time [s]")
    ax.legend()
    fig.tight_layout()
    fig.savefig(out / "02_ee_err_norm.png", dpi=150)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(9, 4))
    for j, c in enumerate(["Fx", "Fy", "Fz"]):
        ax.plot(ta, a["F"][:, j], label=f"{la} {c}")
        ax.plot(tb, b["F"][:, j], "--", label=f"{lb} {c}")
    ax.set_xlabel("time [s]")
    ax.legend(fontsize=7, ncol=3)
    fig.tight_layout()
    fig.savefig(out / "03_residual_force.png", dpi=150)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(9, 4))
    ax.plot(ta, a["tr"], label=la)
    ax.plot(tb, b["tr"], label=lb)
    ax.set_ylabel("‖τ_res‖")
    ax.set_xlabel("time [s]")
    ax.legend()
    fig.tight_layout()
    fig.savefig(out / "04_tau_residual_norm.png", dpi=150)
    plt.close(fig)

    r0 = np.linalg.norm(a["tau_total"] - a["tau_vsd"], axis=1)
    r1 = np.linalg.norm(b["tau_total"] - b["tau_vsd"], axis=1)
    fig, ax = plt.subplots(figsize=(9, 4))
    ax.plot(ta, r0, label=la)
    ax.plot(tb, r1, label=lb)
    ax.set_ylabel("‖τ_tot−τ_vsd‖")
    ax.set_xlabel("time [s]")
    ax.legend()
    fig.tight_layout()
    fig.savefig(out / "05_tau_total_minus_vsd.png", dpi=150)
    plt.close(fig)

    da = np.mean(np.abs(a["tau_ideal"][:, 1:4] - a["tau_out"][:, 1:4]), axis=1)
    db = np.mean(np.abs(b["tau_ideal"][:, 1:4] - b["tau_out"][:, 1:4]), axis=1)
    fig, ax = plt.subplots(figsize=(9, 4))
    ax.plot(ta, da, label=la)
    ax.plot(tb, db, label=lb)
    ax.set_ylabel("mean |Δτ_act| q2–q4")
    ax.set_xlabel("time [s]")
    ax.legend()
    fig.tight_layout()
    fig.savefig(out / "06_actuator_ideal_vs_out.png", dpi=150)
    plt.close(fig)

    h0 = np.linalg.norm(a["hz"], axis=1)
    h1 = np.linalg.norm(b["hz"], axis=1)
    fig, ax = plt.subplots(figsize=(9, 4))
    ax.plot(ta, h0, label=la)
    ax.plot(tb, h1, label=lb)
    ax.set_ylabel("‖hys_z q2–q4‖")
    ax.set_xlabel("time [s]")
    ax.legend()
    fig.tight_layout()
    fig.savefig(out / "07_hysteresis_states.png", dpi=150)
    plt.close(fig)

    fig, (axe, axe2) = plt.subplots(1, 2, figsize=(10, 4))
    axe.plot(a["ed"][:, 0], a["ed"][:, 1], label=f"{la} des")
    axe.plot(a["ea"][:, 0], a["ea"][:, 1], "--", label=f"{la} ee")
    axe.plot(b["ed"][:, 0], b["ed"][:, 1], label=f"{lb} des")
    axe.plot(b["ea"][:, 0], b["ea"][:, 1], "--", label=f"{lb} ee")
    axe.set_xlabel("x [m]")
    axe.set_ylabel("y [m]")
    axe.legend(fontsize=6)
    axe.set_title("XY projection")
    axe2.plot(a["ed"][:, 0], a["ed"][:, 2], label=f"{la} des")
    axe2.plot(a["ea"][:, 0], a["ea"][:, 2], "--", label=f"{la} ee")
    axe2.plot(b["ed"][:, 0], b["ed"][:, 2], label=f"{lb} des")
    axe2.plot(b["ea"][:, 0], b["ea"][:, 2], "--", label=f"{lb} ee")
    axe2.set_xlabel("x [m]")
    axe2.set_ylabel("z [m]")
    axe2.legend(fontsize=6)
    axe2.set_title("XZ projection")
    plt.tight_layout()
    plt.savefig(out / "08_path_projections.png", dpi=150)
    plt.close(fig)

    names = ["RMS EE", "mean ‖F‖", "mean ‖τ_res‖"]
    v0 = [
        float(np.sqrt(np.mean(a["en"] ** 2))),
        float(np.mean(np.linalg.norm(a["F"], axis=1))),
        float(np.mean(a["tr"])),
    ]
    v1 = [
        float(np.sqrt(np.mean(b["en"] ** 2))),
        float(np.mean(np.linalg.norm(b["F"], axis=1))),
        float(np.mean(b["tr"])),
    ]
    xv = np.arange(len(names))
    w = 0.35
    fig, ax = plt.subplots(figsize=(7, 4))
    ax.bar(xv - w / 2, v0, w, label=la)
    ax.bar(xv + w / 2, v1, w, label=lb)
    ax.set_xticks(xv)
    ax.set_xticklabels(names, rotation=15)
    ax.legend()
    plt.tight_layout()
    plt.savefig(out / "09_metric_bars.png", dpi=150)
    plt.close(fig)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--model-path", type=Path, default=None)
    ap.add_argument("--profile", type=str, default="medium_train")
    ap.add_argument("--randomize-cable", action="store_true")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--config", type=Path, default=_ROOT / "configs" / "rl_sac.yaml")
    ap.add_argument(
        "--out-dir",
        type=Path,
        default=_ROOT / "debug_outputs" / "sac_residual_task_force" / "compare_plots",
    )
    args = ap.parse_args()

    cfg = Path(args.config)

    rnd = bool(args.randomize_cable)


    overrides = {"env": {"randomization_profile": str(args.profile), "randomize_cable": rnd}}


    opts = {"randomization_profile": str(args.profile), "randomize_cable": rnd}




    seed0 = int(args.seed)






    env_a = PMICableResidualEnv(config_path=cfg, overrides=overrides)
    lab_a = "VSD_zero_residual"
    A = collect(env_a, _zero, seed=seed0, opt=opts)
    env_a.close()






    mp = args.model_path
    lab_b = lab_a




    sac = SAC.load(mp) if mp is not None and mp.is_file() else None




    env_b = PMICableResidualEnv(config_path=cfg, overrides=overrides)




    

    lab_b = "VSD_plus_SAC" if sac is not None else lab_a

    if sac is None:
        B = collect(env_b, _zero, seed=seed0 + 1, opt=opts)
    else:
        sac_m = sac

        def _sac(o: np.ndarray) -> np.ndarray:
            a, _ = sac_m.predict(o, deterministic=True)
            return np.asarray(a, dtype=np.float32).reshape(3)

        B = collect(env_b, _sac, seed=seed0 + 1, opt=opts)
    env_b.close()






    Path(args.out_dir).mkdir(parents=True, exist_ok=True)




    

    save_plots(A, B, Path(args.out_dir), lab_a, lab_b)
    md = Path(args.out_dir) / "compare_README.md"
    md.write_text(
        f"# compare_vsd_vs_sac_residual\n\n- profile: `{args.profile}`\n- randomize_cable: {rnd}\n"
        f"- model: `{mp}`\n- outputs in this folder (.png).\n",

        encoding="utf-8",
    )


    print(f"[compare] Saved plots → {args.out_dir}")



if __name__ == '__main__':
    main()
