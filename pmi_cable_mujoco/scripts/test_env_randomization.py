#!/usr/bin/env python3
"""Hybrid belt + cable parameter randomization sanity check."""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import yaml

from transmission.randomization import HybridTransmissionSampler


def main() -> None:
    belt_path = ROOT / "configs" / "belt_params.yaml"
    cable_path = ROOT / "configs" / "cable_params.yaml"
    with open(belt_path, "r", encoding="utf-8") as f:
        belt = yaml.safe_load(f)
    with open(cable_path, "r", encoding="utf-8") as f:
        cable = yaml.safe_load(f)
    belt.setdefault("randomization", {})["enabled"] = True
    cable.setdefault("randomization", {})["enabled"] = True

    sampler = HybridTransmissionSampler(belt, cable, seed=42)

    for i in range(6):
        b, c = sampler.sample()
        print(f"--- sample {i} ---")
        print(
            "  belt:  gear_ratio=%.4f efficiency=%.4f damping=%.4f"
            % (b["gear_ratio"], b["efficiency"], b["belt_damping"])
        )
        print(
            "  cable: pulley[0]=%.4f stiffness[min,max]=(%.2f, %.2f)"
            % (c["pulley_radius"][0], float(c["stiffness"].min()), float(c["stiffness"].max()))
        )

    print("OK.")


if __name__ == "__main__":
    main()
