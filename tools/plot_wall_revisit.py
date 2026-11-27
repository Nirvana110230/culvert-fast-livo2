"""Plot the two supported opposite walls from a frozen region's corner_audit export."""
import argparse
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "backend"))
from heldout import find_planes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("points", type=Path, help="corner_points.npz from backend/corner_audit.py")
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    plt.rcParams.update({"font.sans-serif": ["Microsoft YaHei", "Noto Sans CJK SC", "DejaVu Sans"],
                         "axes.unicode_minus": False, "font.size": 11})
    with np.load(args.points) as clouds:
        walls = {name: anchor for name, _, _, _, anchor in find_planes(clouds["target"])
                 if name in ("wall_a_1", "wall_a_2")}
        old = np.vstack([walls["wall_a_1"], walls["wall_a_2"]])
        fig, axes = plt.subplots(1, 2, figsize=(9.0, 5.5), sharex=True, sharey=True, layout="constrained")
        for axis, stage, title in zip(axes, ("before", "after"), ("前端里程计拼接", "联合位姿图重建")):
            new = np.vstack([clouds[stage + "_" + name] for name in walls])
            axis.scatter(*old[:, :2].T, s=5, color="#29486e", alpha=.70, label="旧访次")
            axis.scatter(*new[:, :2].T, s=4, color="#c35b33", alpha=.55, label="新访次")
            axis.set(title=title, xlabel="局部 X（m）", aspect="equal")
            axis.grid(alpha=.18)
        axes[0].set_ylabel("局部 Y（m）")
        axes[1].legend(loc="upper center", frameon=False)
        fig.savefig(args.output, dpi=165)
        plt.close(fig)


if __name__ == "__main__":
    main()
