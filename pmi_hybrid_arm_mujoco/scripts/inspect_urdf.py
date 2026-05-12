#!/usr/bin/env python3
"""PHASE 0: URDF(pmi_description3) 검사 출력 — 링크, 조인트, 관성, mimic, 메시 등."""

from __future__ import annotations

import argparse
from pathlib import Path
import xml.etree.ElementTree as ET


def fmt_vec_xyz(org: ET.Element | None, key: str) -> str:
    if org is None:
        return "—"
    return org.get(key, "—") or "—"


def main() -> None:
    pkg = Path(__file__).resolve().parents[2]
    default_urdf = pkg / "ros_ws" / "pmi_description3" / "urdf" / "pmi_description3.urdf"
    ap = argparse.ArgumentParser(description="Inspect pmi_description3 URDF")
    ap.add_argument("--urdf", type=str, default=str(default_urdf))
    path = Path(ap.parse_args().urdf)

    r = ET.parse(path).getroot()
    if r.tag != "robot":
        raise SystemExit("<robot>가 루트가 아님")
    robot_name = r.get("name", "?")
    print("=" * 72)
    print(f"ROBOT NAME: {robot_name}")
    print(f"파일       : {path.resolve()}")

    links = list(r.findall("link"))
    joints = list(r.findall("joint"))
    print(f"\n링크 수: {len(links)}")
    print(f"조인트 수: {len(joints)}")

    print("\n—— 링크 목록 및 관성/visual ——")
    for lk in sorted(links, key=lambda x: x.get("name", "")):
        name = lk.get("name") or "?"
        print(f"\n[{name}]")
        ii = lk.find("inertial")
        if ii is not None:
            org = ii.find("origin")
            print(f"  inertial xyz rpy : {fmt_vec_xyz(org, 'xyz')} | {fmt_vec_xyz(org, 'rpy')}")
            me = ii.find("mass")
            if me is not None:
                print(f"  mass value       : {me.get('value')}")
            inertia = ii.find("inertia")
            if inertia is not None:
                print(
                    "  inertia ixx,iyy,izz: "
                    f"{inertia.get('ixx')} {inertia.get('iyy')} {inertia.get('izz')}"
                )
        viz = lk.find("visual")
        if viz is not None:
            gm = viz.find("geometry/mesh") if viz.find("geometry") is not None else None
            if gm is not None and gm.get("filename"):
                print(f"  visual mesh      : {gm.get('filename')}")

    print("\n—— 조인트(타입 · 부모 자식 · origin · 축 · limit · mimic) ——")

    children_by_parent: dict[str, list[str]] = {}
    for joint in sorted(joints, key=lambda x: x.get("name", "")):
        jn = joint.get("name")
        jtype = joint.get("type") or "?"
        par_el = joint.find("parent")
        ch_el = joint.find("child")
        parent = par_el.get("link") if par_el is not None else "?"
        child = ch_el.get("link") if ch_el is not None else "?"
        children_by_parent.setdefault(parent, []).append(child)
        org = joint.find("origin")
        ax = joint.find("axis")
        lim = joint.find("limit")
        print(f"\n[{jn}] type={jtype}")
        print(f"  parent -> child  : {parent} -> {child}")
        print(f"  origin xyz rpy : {fmt_vec_xyz(org, 'xyz')} | {fmt_vec_xyz(org, 'rpy')}")
        if ax is not None:
            print(f"  axis xyz       : {ax.get('xyz')}")
        if lim is not None:
            print(
                "  limit L U eff vel: "
                f"{lim.get('lower')} {lim.get('upper')} "
                f"{lim.get('effort')} {lim.get('velocity')}"
            )
        mimic_el = joint.find("mimic")
        if mimic_el is not None:
            print(
                "  mimic            : joint="
                f"{mimic_el.get('joint')} mult={mimic_el.get('multiplier')} "
                f"offset={mimic_el.get('offset')}"
            )

    print("\n—— 부모 트리 (base_link 기준) ——")

    def show_tree(node: str, indent: str = "") -> None:
        print(f"{indent}{node}")
        for c in sorted(set(children_by_parent.get(node, []))):
            show_tree(c, indent + "  ")

    if "base_link" in {lk.get("name") for lk in links}:
        show_tree("base_link")

    print("\n—— package:// 메시 참조 목록 ——")
    pkgs_found = False
    for lk in links:
        for gm in lk.findall("./visual/geometry/mesh"):
            fn = gm.get("filename") or ""
            if "package://" in fn:
                print(f"  {lk.get('name')}: {fn}")
                pkgs_found = True
        col = lk.find("collision")
        if col is not None:
            gm = col.find("geometry/mesh")
            fn = gm.get("filename") if gm is not None else ""
            if fn and "package://" in fn:
                print(f"  {lk.get('name')} collision: {fn}")
                pkgs_found = True
    if not pkgs_found:
        print("  (없음)")

    print("\n—— 중요 검사 포인트 (MuJoCo) ——")
    print("- package:// 경로 대신 meshes 상대 또는 절대 경로 권장")
    print("- mimic은 MJCF equality joint 또는 매 스텝 Python 동기화로 구현")
    print("- STL collision 과다 패싯 수는 성능 문제 가능성")

    ee = next((lk for lk in links if lk.get("name") == "end_effector"), None)
    if ee is not None:
        print("\n[end_effector] 링크 존재")
        if ee.find("inertial") is None:
            print("  (주의: inertial 블록 없음 — 무질량 placeholder로 흔함)")

    ee_j = next((j for j in joints if j.get("name") == "end_effector_joint"), None)
    if ee_j is not None:
        pa = ee_j.find("parent")
        ci = ee_j.find("child")
        ox = ee_j.find("origin")
        print("\n[end_effector_joint]")
        print("  type        : fixed")
        print(f"  parent-child: {(pa.get('link') if pa is not None else '?')} "
              f"-> {(ci.get('link') if ci is not None else '?')}")
        print(f"  origin xyz rpy: {fmt_vec_xyz(ox, 'xyz')} | {fmt_vec_xyz(ox, 'rpy')}")

    q4 = next((lk for lk in links if lk.get("name") == "q4_motor"), None)
    print("\nq4_motor inertial 검사:")
    if q4 is None or q4.find("inertial") is None:
        print("  (링크 또는 inertial 없음)")
    else:
        me = q4.find("inertial/mass")
        inertia = q4.find("inertial/inertia")
        if me is None:
            print("  mass 태그 없음")
        else:
            mz = float(me.get("value", "nan"))
            if mz <= 1e-9:
                print(f"  mass={mz}: MuJoCo용 소량 관성 필요")
            else:
                print(f"  mass OK: {mz}")
        if inertia is not None:
            zs = ["ixx", "iyy", "izz"]
            vals = [float(inertia.get(a, "nan")) for a in zs]
            if all(v <= 1e-12 for v in vals):
                print("  (주의) 대각관성 거의 0")

    print("=" * 72)


if __name__ == "__main__":
    main()
