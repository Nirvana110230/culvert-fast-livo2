"""Common eligibility rules for thickness, layer offset, spacing, corner and opening claims."""
import argparse
import json
from pathlib import Path
import numpy as np


RULES={"max_normal_difference_deg":15.,"min_reference_points":42,"min_source_points":28,
       "min_reference_footprint_coverage":.35,"min_peak_fraction":.30,
       "min_second_tangential_span_m":.20,"min_first_tangential_span_m":.50,
       "parallel_max_deg":15.,"corner_min_angle_deg":75.,"corner_max_angle_change_deg":15.}


def span_ok(points,basis):
    if len(points)<3:
        return False
    span=np.sort(np.ptp(np.quantile(points@basis,[.05,.95],axis=0),axis=0))
    return bool(span[0]>=RULES["min_second_tangential_span_m"] and span[1]>=RULES["min_first_tangential_span_m"])


def certify(row,planes,scans):
    from heldout import normal_at,plane_measure
    result={"rules":RULES,"surfaces":{},"spacing":[],"corners":[],"openings":[]}
    normals={k:normal_at(v) for k,v in scans.items()}
    originals={p["type"]:p for p in row["surfaces"]}
    for label,center,n,basis,anchor in planes:
        orig=originals[label]
        item={}
        result["surfaces"][label]=item
        for stage,cloud in scans.items():
            raw=orig[stage]
            ns,planar=normals[stage]
            filtered=cloud[planar & (np.abs(ns@n)>=np.cos(np.deg2rad(RULES["max_normal_difference_deg"])))]
            obs=plane_measure(filtered,center,n,basis,anchor)
            angle=RULES["max_normal_difference_deg"]
            # The original fit and the normal-compatible remeasurement must both
            # have physical support. Retain rejected fits for diagnosis.
            reasons=[]
            if len(anchor)<RULES["min_reference_points"] or not span_ok(anchor,basis):
                reasons.append("insufficient_reference_patch")
            if raw.get("angle_deg",180)>angle:
                reasons.append("raw_normal_conflict_or_missing")
            if obs.get("angle_deg",180)>angle:
                reasons.append("supported_normal_conflict_or_missing")
            for name,fit in (("raw",raw),("normal_filtered",obs)):
                if fit.get("n",0)<RULES["min_source_points"]:
                    reasons.append(name+"_too_few_points")
                if fit.get("target_footprint_coverage",0)<RULES["min_reference_footprint_coverage"]:
                    reasons.append(name+"_insufficient_footprint")
                if fit.get("n",0)/max(1,fit.get("candidate_n",0))<RULES["min_peak_fraction"]:
                    reasons.append(name+"_weak_peak")
            if "source_plane_centroid_m" in obs:
                from scipy.spatial import cKDTree
                offset=obs["offset_m"]+np.median((anchor-center)@n)
                mask=np.abs((filtered-center)@n-offset)<.2
                f=filtered[mask]
                near=cKDTree((anchor-center)@basis).query((f-center)@basis)[0]<.28 if len(f) else []
                if not span_ok(f[near],basis):
                    reasons.append("insufficient_source_tangential_span")
            item[stage]={"valid":not reasons,"status":"valid" if not reasons else "excluded","reasons":reasons,
                         "raw_fit":raw,"normal_filtered_fit":obs,"published":None if reasons else obs}
    for pair in row["parallel_wall_spacing"]:
        labels=[x for x in result["surfaces"] if x.startswith(pair["family"])]
        entry={"family":pair["family"],"reference_m":pair["target_spacing_m"]}
        for stage in scans:
            gates=[result["surfaces"][label][stage] for label in labels]
            reasons=[]
            value=None
            if len(gates)!=2 or not all(v["valid"] for v in gates):
                reasons.append("invalid_constituent_surface")
            else:
                a,b=[originals[k] for k in labels]
                va,vb=[x["published"] for x in gates]
                n=np.array(a["normal"])
                if abs(np.dot(va["source_normal"],vb["source_normal"]))<np.cos(np.deg2rad(RULES["parallel_max_deg"])):
                    reasons.append("walls_not_parallel")
                else:
                    span=float((np.array(b["centroid_m"])-a["centroid_m"])@n)
                    value=abs(span+vb["offset_m"]*np.dot(n,b["normal"])-va["offset_m"])
            entry[stage]={"valid":not reasons,"reasons":reasons,"spacing_m":value}
        result["spacing"].append(entry)
    for c in row["corner_candidates"]:
        entry={"walls":c["walls"],"reference_deg":c["target_angle_deg"]}
        for stage in scans:
            gates=[result["surfaces"][x][stage] for x in c["walls"]]
            reasons=[]
            ang=None
            if not all(g["valid"] for g in gates):
                reasons.append("invalid_constituent_surface")
            else:
                v=[g["published"]["source_normal"] for g in gates]
                angle=float(np.rad2deg(np.arccos(np.clip(abs(np.dot(*v)),0,1))))
                if c["target_angle_deg"]<RULES["corner_min_angle_deg"] or abs(angle-c["target_angle_deg"])>RULES["corner_max_angle_change_deg"]:
                    reasons.append("corner_angle_inconsistent")
                else:
                    ang=angle
            entry[stage]={"valid":not reasons,"reasons":reasons,"angle_deg":ang}
        result["corners"].append(entry)
    for opening in row["opening_candidates"]:
        entry={"wall":opening["wall"],"reference":opening["target"],"identity":"unlabelled geometric opening"}
        for stage in scans:
            good=result["surfaces"][opening["wall"]][stage]["valid"] and stage in opening
            entry[stage]={"valid":good,"width_m":opening[stage]["width_m"] if good else None,
                          "reasons":[] if good else ["invalid_surface_or_missing_matching_gap"]}
        result["openings"].append(entry)
    return result


def main():
    from frame_io import frames_and_gravity,save_json
    from heldout import find_planes,region_clouds
    from loop_edges import EdgeBuilder
    from verify import load_tum
    p=argparse.ArgumentParser()
    for name in ("index","gravity","result","out"):
        p.add_argument("--"+name,type=Path,required=True)
    a=p.parse_args()
    fr,G,_=frames_and_gravity(a.index,a.gravity)
    original=json.loads((a.result/"holdout.json").read_text())
    before=np.array([G@f.odom for f in fr]);_,after=load_tum(a.result/"trajectory_after.tum")
    builder=EdgeBuilder(fr,before,[],set())
    for row in original:
        old,scans=region_clouds(builder,before,after,row["target_index"],row["source_index"])
        row["quality"]=certify(row,find_planes(old),scans) if len(old)>=35 and min(map(len,scans.values()))>=35 else {"status":"insufficient_region_support","surfaces":{}}
    save_json(a.out,{"input_original":str(a.result/"holdout.json"),"same_poses_and_loops":True,"rules":RULES,"regions":original})


if __name__=="__main__":
    main()
