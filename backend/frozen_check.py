"""Measure another frontend at the same preselected B1 sensor times."""
import argparse
import json
from pathlib import Path
import numpy as np

from frame_io import frames_and_gravity, save_json
from heldout import evaluate
from loop_edges import EdgeBuilder


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--frozen",type=Path,required=True)
    p.add_argument("--index",type=Path,required=True)
    p.add_argument("--gravity",type=Path,required=True)
    p.add_argument("--out",type=Path,required=True)
    a=p.parse_args()
    if a.out.exists():
        raise FileExistsError(a.out)
    table=json.loads(a.frozen.read_text())
    frames,G,record=frames_and_gravity(a.index,a.gravity)
    stamps=np.array([f.stamp for f in frames],dtype=np.int64)
    raw=np.array([G@f.odom for f in frames])
    builder=EdgeBuilder(frames,raw,[],set())
    results=[]
    for pair in table["pairs"]:
        match={"frozen":pair}
        ids=[]
        for side in ("target","source"):
            stamp=pair[side+"_stamp_ns"]
            k=int(np.argmin(np.abs(stamps-stamp)))
            ids.append(k)
            match[side]={"id":frames[k].id,"stamp_ns":frames[k].stamp,"offset_s":(frames[k].stamp-stamp)*1e-9}
        if max(abs(match[x]["offset_s"]) for x in ("target","source"))<=table["maximum_time_difference_s"] and ids[0]<ids[1]:
            v=evaluate([tuple(ids)],builder,raw,raw)[0]
            # Both raw pose sets above are identical. Preserve just one set of
            # measurements and retain the original point/support counts.
            match["metrics"]={"point_to_point":v["point_to_point"].get("before"),"surfaces":[],
                              "surface_quality":v["quality"],
                              "corner_candidates":v["corner_candidates"],"opening_candidates":v["opening_candidates"],
                              "parallel_wall_spacing":v["parallel_wall_spacing"]}
            for s in v["surfaces"]:
                match["metrics"]["surfaces"].append({"type":s["type"],"centroid_m":s["centroid_m"],"normal":s["normal"],**s["before"]})
        else:
            match["unavailable"]="nearest keyframe outside frozen temporal tolerance"
        results.append(match)
    save_json(a.out,{"reference_plan_sha256":table["plan_sha256"],"index":str(a.index),"gravity":record,
                     "method":"same measured sensor times; cylinders and surfaces estimated from each frontend raw trajectory",
                     "regions":results})


if __name__=="__main__":
    main()
