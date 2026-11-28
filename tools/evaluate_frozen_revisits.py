"""Summarize archived fixed ground pairs and certified wall surfaces."""
import argparse
import json
import zipfile
from pathlib import Path

import numpy as np
import pandas as pd


def summarize(pairs, reference, old_holdout, new_holdout):
    results={}
    with zipfile.ZipFile(reference) as archive:
        for file in ("six_final_paired","five_final_no_causal_paired"):
            fixed=pd.read_csv(archive.open("analysis/common_ground/"+file+".csv"))
            cases={}
            for name,trial in pairs.groupby("case"):
                joined=fixed.merge(trial,on=["i","j"],how="left",validate="one_to_one")
                valid=joined.dropna(subset=["overlap_height_difference_m"])
                cases[name]=dict(fixed=len(fixed),supported=len(valid),
                    previous_median_p95_m=np.quantile(abs(valid.watermark_joint),[.5,.95]).tolist(),
                    candidate_median_p95_m=np.quantile(abs(valid.overlap_height_difference_m),[.5,.95]).tolist(),
                    missing=joined.loc[joined.overlap_height_difference_m.isna(),["i","j"]].values.tolist())
            results[file]=cases
    def walls(row):
        return {k:v["after"]["published"]["two_visit_thickness_p90_p10_m"]
                for k,v in row["quality"]["surfaces"].items() if k.startswith("wall") and v["after"]["valid"]}
    rows=[]
    for a,b in zip(old_holdout,new_holdout):
        if (a["target_id"],a["source_id"])!=(b["target_id"],b["source_id"]):
            raise ValueError("frozen holdout identities differ")
        old,new=walls(a),walls(b)
        rows.append(dict(indices=[a["target_id"],a["source_id"]],
            distance_median_raw_old_new_m=[r["point_to_point"][side]["source_median_m"] for r,side in
                                          ((a,"before"),(a,"after"),(b,"after"))],
            wall_width_old_new_m={k:[old.get(k),new.get(k)] for k in sorted(old.keys()|new.keys())}))
    results["regions"]=rows
    return results


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for name in ("ground_cases","reference_zip","previous_validated","candidate_holdout","output"):
        p.add_argument(name,type=Path)
    a=p.parse_args()
    result=summarize(pd.read_csv(a.ground_cases),a.reference_zip,json.loads(a.previous_validated.read_text())["regions"],
                     json.loads(a.candidate_holdout.read_text()))
    a.output.write_text(json.dumps(result,indent=2)+"\n",encoding="utf8")
