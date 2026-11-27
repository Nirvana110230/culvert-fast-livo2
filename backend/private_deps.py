"""Install the pinned ARM64 wheels into an empty private prefix using Python 3.10."""
import argparse
import hashlib
import json
import platform
from pathlib import Path
import subprocess
import sys
import urllib.request


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--wheels",type=Path,required=True)
    p.add_argument("--prefix",type=Path,required=True)
    p.add_argument("--download",action="store_true",help="fetch a missing wheel from the URL in the lock")
    a=p.parse_args()
    if platform.machine() not in ("aarch64","arm64") or sys.version_info[:2]!=(3,10):
        p.error("the supplied binary lock targets Linux ARM64 on Python 3.10")
    lock=json.loads(Path(__file__).with_name("arm64-wheel-lock.json").read_text())
    if a.prefix.exists() and any(a.prefix.iterdir()):
        p.error("choose an empty private installation directory")
    a.wheels.mkdir(parents=True,exist_ok=True)
    paths=[]
    for entry in lock:
        path=a.wheels/entry["filename"]
        if not path.exists() and a.download:
            pending=path.with_suffix(".part")
            with urllib.request.urlopen(entry["url"],timeout=60) as src, pending.open("wb") as dst:
                while chunk:=src.read(512*1024):
                    dst.write(chunk)
            if hashlib.sha256(pending.read_bytes()).hexdigest()!=entry["sha256"]:
                raise ValueError(f"download hash mismatch: {entry['filename']}")
            pending.replace(path)
        if hashlib.sha256(path.read_bytes()).hexdigest()!=entry["sha256"]:
            raise ValueError(f"wheel hash mismatch: {entry['filename']}")
        paths.append(str(path))
    subprocess.run([sys.executable,"-m","pip","install","--no-deps","--no-index",
                    "--disable-pip-version-check","--target",str(a.prefix),*paths],check=True)


if __name__=="__main__":
    main()
