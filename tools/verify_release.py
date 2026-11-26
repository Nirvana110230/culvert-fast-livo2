"""Verify a maintenance checkout and its archived AArch64 executable without ROS."""
import argparse
import hashlib
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]


def read(path):
    return json.loads((ROOT / path).read_bytes())


def digest(path):
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--with-mvs", action="store_true", help="also check the locally supplied MVS SDK")
    args = parser.parse_args()
    release = read("deploy/release.json")
    expected = read("records/maintained_source_sha256.json")
    expected["docs/board-operation.md"] = expected.pop("README.md")
    expected.update(release["maintenance_overrides_sha256"])
    vendor = read("records/vendor_sha256.json")
    expected.update({name: value for name, value in vendor["files"].items()
                     if args.with_mvs or not name.startswith("deps/mvs/")})
    binary = "deploy/aarch64/fastlivo_mapping"
    expected[binary] = release["install_elf_sha256"]
    errors = []
    for name, checksum in sorted(expected.items()):
        path = ROOT / name
        if not path.is_file() or digest(path) != checksum:
            errors.append(name)
    if args.with_mvs:
        for name, target in vendor["links"].items():
            path = ROOT / name
            if not path.is_symlink() or str(path.readlink()) != target:
                errors.append(name)
    if (ROOT / binary).is_file():
        with (ROOT / binary).open("rb") as stream:
            header = stream.read(20)
        if header[:6] != b"\x7fELF\x02\x01" or int.from_bytes(header[18:20], "little") != 183:
            errors.append(f"{binary} ELF header")
    if errors:
        print("\n".join(errors), file=sys.stderr)
        return 1
    print(f"{release['stage']}: {len(expected)} SHA256 checks passed, AArch64 ELF verified")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
