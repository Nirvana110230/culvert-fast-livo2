"""Reproduce all A/B statistics from the bounded extraction, run journals and the previous-run evidence."""
import argparse
from pathlib import Path
from clock_analysis import load_clocks,analyze as clocks,write_json
from estimator_analysis import analyze as estimator, read_json

def main():
    p=argparse.ArgumentParser()
    p.add_argument("--raw",type=Path,required=True,help="raw run subset plus clocks.csv.gz, metadata.yaml, events.txt and session_subset.json")
    p.add_argument("--tools",type=Path,required=True,help="livo-pgo-dev/tools")
    p.add_argument("--previous",type=Path,required=True,help="z_geometry/front_static diagnostic directory")
    p.add_argument("--imu",type=Path,required=True,help="third run full_z/imu.json")
    p.add_argument("--output",type=Path,required=True)
    args=p.parse_args(); raw=args.raw; out=args.output; out.mkdir(parents=True,exist_ok=True)
    b=load_clocks(raw); cm=clocks(raw,out,b,args.tools.resolve())
    prev={k:read_json(args.previous/fn) for k,fn in [("offline","offline.json"),("su","static_updates.json"),("cases","frontend_comparison.json"),("imu","imu.json")]}
    em=estimator(raw,out,b,args.tools.resolve(),read_json(args.imu),prev)
    from attitude_analysis import analyze as attitude
    em["attitude_elevation"]=attitude(raw,out)
    write_json(out/"metrics.json",{"bag":cm,"frontend":em})
    from make_plots import draw
    draw(out,raw,cm,em)

if __name__=="__main__": main()
