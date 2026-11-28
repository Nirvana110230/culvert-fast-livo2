"""Assemble report metrics from the exported JSON and the signed 1 m grid residuals."""
import argparse
import json
from pathlib import Path
import numpy as np
import pandas as pd


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("root",type=Path);p.add_argument("--out",type=Path)
    a=p.parse_args(); dest=a.out or a.root
    dest.mkdir(exist_ok=True,parents=True)
    def read(name):
        return json.loads((a.root/name).read_text(encoding="utf8"))
    cells=pd.read_csv(a.root/"geometry/floor_cells.csv").dropna(subset=["residual_z_m"])
    data=[]
    for (cohort,stage),rows in cells.groupby(["cohort","stage"]):
        z,r=rows.z,rows.residual_z_m
        data.append(dict(cohort=cohort,stage=stage,cells=len(rows),z_p05_m=z.quantile(.05),z_median_m=z.median(),
            z_p95_m=z.quantile(.95),z_residual_min_m=r.min(),z_residual_max_m=r.max(),
            z_residual_range_m=r.max()-r.min(),z_residual_abs_p95_m=r.abs().quantile(.95),
            z_residual_rms_m=np.sqrt(np.mean(r**2))))
    pd.DataFrame(data).to_csv(dest/"floor_distribution.csv",index=False,float_format="%.8f")
    mapping={"acquisition_and_frontend":"frontend/metrics.json","clock_cpu":"frontend/historical_cpu_source.json",
        "vision_and_covariance":"frontend/visual_uncertainty.json","static_bias_evidence":"frontend/extra_static/summary.json",
        "loop_and_revisit":"loops/metrics.json","floor_and_profiles":"geometry/geometry_metrics.json",
        "planes_and_inertia":"structure_eval/metrics.json","map_products":"maps/metrics.json",
        "rigid_reconstruction":"maps/readback.json","movement_thresholds":"maps/motion_metrics.json",
        "start_end_orientation":"maps/static_orientation.json","three_rounds":"history/metrics.json"}
    report=dict(schema=1,date="2026-11-28",timezone="Asia/Shanghai",source_revision="f6d0792e9aeb5e3825b916451918ac56cba9f1cb",
        algorithm_revision="59c47ee53aa931bd7766ae0c9045ae8e48083969",
        field_elf_sha256="9e9fc891dcef46c6859e215467fb1ad9620a7f215a6991e60e9b9eef22fe0bcd",
        stages={"A":"pgo/map_before.pcd","B":"pgo/map_after.pcd","C":"structure/map_after.pcd"},
        field_relative_dirs={"run":"20261128T072648Z_c05694da93","out":"full_z_5R7iGB","bag":"raw_bag"},
        decision="Adopt the corridor mapping baseline for prior-map and localization development",
        operator_move_to_stop_s=1994.767387286,floor_distribution=data,sections={k:read(v) for k,v in mapping.items()})
    (dest/"metrics.json").write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+"\n",encoding="utf8")


if __name__=="__main__":
    main()
