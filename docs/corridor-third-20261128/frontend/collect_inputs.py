"""Retrieve only diagnostic text and bounded CDR fields over strict, preverified SSH."""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import shlex
import subprocess
import tarfile

RUN_INPUTS = [
    "diagnostics/deskew_timing.csv","diagnostics/imu_init_samples.csv","diagnostics/lio_information_6d.csv",
    "diagnostics/measurement_weight_sources.csv","diagnostics/plane_constraints.csv","diagnostics/preprocess_timing.csv",
    "diagnostics/timing_windows.csv","Log/visual_lru.csv","Log/processing.csv","Log/input_counters.json",
    "keyframes/states.csv","keyframes/index.csv","keyframes/metadata.txt","resources.jsonl","resources.json",
    "algorithm.log","algorithm.log.1","active_parameters.json","resolved.yaml","algorithm_identity.json",
    "sensors/camera/frames.csv","sensors/camera/frames.csv.1","sensors/camera/events.csv","sensors/camera/events.csv.1",
    "sensors/camera/clock_latch.csv","sensors/camera/settings.csv","sensors/camera/driver_summary.json","sensors/camera/usb_stats.json",
    "sensors/camera/parameters.active.json","sensors/camera/timeshare_reads.csv","sensors/camera/timeshare_reads.csv.1",
    "sensors/timeshare_writes.csv","sensors/timeshare_writes.csv.1","sensors/transport_cadence_ready.json","gravity_transform.json"]
FIELD_INPUTS = ["events.txt","raw_bag/metadata.yaml","recorder.log"]

def digest(p):
    with p.open("rb") as f: return hashlib.file_digest(f,"sha256").hexdigest()

def collect(alias,field,run,raw,verify_only):
    raw.mkdir(parents=True,exist_ok=True)
    if alias.startswith("-"): raise ValueError("SSH alias")
    ssh=["ssh","-o","BatchMode=yes","-o","StrictHostKeyChecking=yes",alias]
    if not PurePosixPath(field).is_absolute() or "/" in run or run in (".",".."): raise ValueError("run arguments")
    rdir=field.rstrip("/")+"/"+run
    shell=shlex.quote
    for source,files in ((rdir,RUN_INPUTS),(field,FIELD_INPUTS)):
        args=" ".join(map(shell,files))
        if not verify_only:
            with subprocess.Popen([*ssh,"cd "+shell(source)+" && tar -czf - "+args],stdout=subprocess.PIPE) as proc:
                with tarfile.open(fileobj=proc.stdout,mode="r|gz") as archive:
                    for item in archive:
                        if not item.isfile() or item.name not in files: raise ValueError(item.name)
                        path=raw/(item.name if source==rdir else Path(item.name).name)
                        path.parent.mkdir(parents=True,exist_ok=True)
                        path.write_bytes(archive.extractfile(item).read())
                if proc.wait(): raise RuntimeError("read failed")
        sums=subprocess.check_output([*ssh,"cd "+shell(source)+" && sha256sum "+args],text=True).splitlines()
        if len(sums)!=len(files): raise ValueError("incomplete sums")
        for line in sums:
            sha,name=line.split(None,1)
            assert name in files
            local=raw/(name if source==rdir else Path(name).name)
            if digest(local)!=sha: raise RuntimeError("digest mismatch "+name)
    size_output=subprocess.check_output([*ssh, "cd "+shell(field+"/raw_bag")+" && stat -c '%s' *.db3"],text=True)
    total=sum(map(int,size_output.splitlines()))
    if not verify_only:
        keep=("mode","state","started_host_ns","strict_image_windows","stop_reason","elapsed_s","ended_host_ns","errors")
        code='import json,sys; a=json.load(open(sys.argv[1])); print(json.dumps({k:a[k] for k in '+repr(keep)+'}))'
        (raw/"session_subset.json").write_bytes(subprocess.check_output([*ssh,"python3 -c "+shell(code)+" "+shell(rdir+"/session.json")]))
        with Path(__file__).with_name("bag_extract.py").open("rb") as script,(raw/"clocks.csv.gz").open("wb") as out:
            subprocess.run([*ssh,"python3 - "+shell(field+"/raw_bag")],stdin=script,stdout=out,check=True)
    manifest={"verified_remote_sha256":{n:digest(raw/n) for n in RUN_INPUTS+[Path(i).name for i in FIELD_INPUTS]},
              "sqlite_bytes_from_stat":total,"sqlite_shards":len(size_output.splitlines()),
              "local_clocks_gz_sha256":digest(raw/"clocks.csv.gz")}
    return manifest

if __name__=="__main__":
    p=argparse.ArgumentParser()
    for k in ["alias","field","run","raw","manifest"]: p.add_argument("--"+k,required=True,type=Path if k in ("raw","manifest") else str)
    p.add_argument("--verify-only",action="store_true")
    a=p.parse_args(); a.manifest.write_text(json.dumps(collect(a.alias,a.field,a.run,a.raw,a.verify_only),indent=2)+"\n")
