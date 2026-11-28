"""Read per-CPU historical sysstat for this test; emits only date/time and numeric fields."""
import argparse
import csv
import json
from pathlib import Path
import shlex
import subprocess

def collect(alias,archive,out):
    ssh=["ssh","-o","BatchMode=yes","-o","StrictHostKeyChecking=yes",alias]
    if alias.startswith("-"): raise ValueError("bad alias")
    def cmd(remote):
        return subprocess.run([*ssh,remote],capture_output=True,text=True,encoding="utf8")
    path=shlex.quote(archive)
    args=" -s 15:15:00 -e 16:15:00 "
    snapshot=cmd("LC_ALL=C sadf -d -T"+args+path+" -- -u ALL -P ALL")
    snapshot.check_returncode()
    d=list(csv.DictReader(snapshot.stdout.splitlines(),delimiter=";"))
    # Preserve UTC+8 timestamps only for the experiment day. Drop hostname.
    keys=[k for k in d[0] if k!="# hostname" and "hostname" not in k]
    result=[{k:v for k,v in r.items() if k in keys} for r in d if r["timestamp"].startswith("2026-11-28")]
    assert len(result)==45 and len({r["CPU"] for r in result})==9
    with (out/"historical_cpu.csv").open("w",encoding="utf8",newline="") as f:
        writer=csv.DictWriter(f,fieldnames=keys); writer.writeheader(); writer.writerows(result)
    tmp=cmd("LC_ALL=C sar -m TEMP"+args+"-f "+path+" 2>&1")
    assert "Requested activities not available" in tmp.stdout
    candidates=["/var/log/atop","/var/log/collectl","/var/log/pcp/pmlogger","/var/log/armbian-hardware-monitor.log","/var/log/armbianmonitor.log","/var/log/armbian-hardware-monitor"]
    paths={c:cmd("test -e "+shlex.quote(c)).returncode==0 for c in candidates}
    summary={"tested_date":"2026-11-28","timezone":"+08:00","cpu_cores":8,"table_rows":len(result),"sysstat_file":"sa28",
        "thermal_activity_available":False,"thermal_sar_response":"Requested activities not available in file","other_archives_present":paths}
    (out/"historical_cpu_source.json").write_text(json.dumps(summary,indent=2)+"\n")
if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("--alias",required=True);p.add_argument("--archive",default="/var/log/sysstat/sa28");p.add_argument("--out",required=True,type=Path)
    a=p.parse_args();collect(a.alias,a.archive,a.out)
