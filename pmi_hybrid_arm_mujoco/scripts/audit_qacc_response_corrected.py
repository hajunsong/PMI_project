#!/usr/bin/env python3
"""관절별 qfrc_applied 단위 테스트 후 mj_forward 로 qacc 부호 일관성 확인."""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import mujoco as mj

from utils.mujoco_helpers import PKG_ROOT

MODEL_DEFAULT_A = PKG_ROOT / "models" / "pmi_arm_only_torque_debug.xml"
MODEL_DEFAULT_B = PKG_ROOT / "models" / "pmi_arm_only_no_collision.xml"
JOINT_NAMES = ["jnt1", "jnt2", "jnt3", "jnt4"]
OUT_DIR = PKG_ROOT / "debug_outputs" / "arm_only_physics"


def dof_addrs(model: mj.MjModel) -> list[int]:
    return [int(model.jnt_dofadr[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, n)]) for n in JOINT_NAMES]


def qpos_addrs(model: mj.MjModel) -> list[int]:
    return [int(model.jnt_qposadr[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, n)]) for n in JOINT_NAMES]


def audit_one_model(mp: Path, tau_mag: float, q_init: list[float]) -> list[dict]:
    model = mj.MjModel.from_xml_path(str(mp))
    data = mj.MjData(model)
    dadr = dof_addrs(model)
    qadr = qpos_addrs(model)

    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    for i in range(4):
        data.qpos[qadr[i]] = float(q_init[i])
    data.qfrc_applied[:] = 0.0

    stem = mp.stem
    rows: list[dict] = []
    for ji in range(4):
        dof = int(dadr[ji])
        for sign in (+1.0, -1.0):
            data.qfrc_applied[:] = 0.0
            data.qfrc_applied[dof] = float(sign * tau_mag)
            mj.mj_forward(model, data)
            qacc = float(data.qacc[dof])
            ok_same = ((qacc > 0 and sign > 0) or (qacc < 0 and sign < 0)) if abs(qacc) > 1e-12 else False
            rows.append(
                {
                    "mjcf": stem,
                    "joint": JOINT_NAMES[ji],
                    "dof_index": dof,
                    "tau_applied": float(sign * tau_mag),
                    "qacc": qacc,
                    "sign_consistent_nonzero": ok_same,
                }
            )
            print(f"[{stem}] {JOINT_NAMES[ji]}  tau={sign * tau_mag:g}  qacc={qacc:.8g}  consistent={ok_same}")
    return rows


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--models",
        type=Path,
        nargs="*",
        default=None,
        help="MJCF 목록 (기본: torque_debug + no_collision)",
    )
    ap.add_argument("--tau-test", type=float, default=1.0, help="|applied torque| per joint test")
    ap.add_argument("--q", type=float, nargs=4, default=[0.0, 0.0, 0.0, 0.0])
    args = ap.parse_args()

    paths = list(args.models) if args.models else [MODEL_DEFAULT_A, MODEL_DEFAULT_B]
    resolved = []
    for p in paths:
        pp = Path(p)
        resolved.append(pp if pp.is_absolute() else PKG_ROOT / pp)

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    tau_mag = float(abs(args.tau_test))
    q_init = list(args.q)

    all_rows: list[dict] = []
    for mp in resolved:
        if not mp.exists():
            raise FileNotFoundError(mp)
        all_rows.extend(audit_one_model(mp, tau_mag, q_init))

    csv_path = OUT_DIR / "qacc_response_audit.csv"
    if all_rows:
        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(all_rows[0].keys()))
            w.writeheader()
            w.writerows(all_rows)

    note_path = OUT_DIR / "qacc_response_audit_report.md"
    note_path.write_text(
        "# qacc response audit\n\n"
        "단일 관절에 `qfrc_applied` 토크를 걸고 `mj_forward` 후 해당 dof 의 `qacc` 부호가 토크 부호와 일치하는지 확인합니다.\n\n"
        "**주의:** 메시 접촉이 있는 경우 접촉·제약이 가속도를 지배하면 부호 검증이 무의미할 수 있습니다. "
        "이 경우 `pmi_arm_only_no_collision.xml` 결과를 참고합니다.\n\n"
        f"- CSV: `{csv_path.resolve()}`\n",
        encoding="utf-8",
    )
    print(f"[saved] {csv_path.resolve()}")


if __name__ == "__main__":
    main()
