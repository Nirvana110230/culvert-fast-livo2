"""Build only the standalone callback test against the selected production libraries."""
import json
import shlex
import subprocess
import sys
from pathlib import Path

build = Path(sys.argv[1]).resolve()
e = Path(__file__).resolve().parent
tag = sys.argv[2]
commands = json.loads((build/"compile_commands.json").read_text())
source = next(c for c in commands if c["file"].endswith("/contract_regression.cpp"))
cmd = shlex.split(source["command"])
obj = e/f"fixture-{tag}.o"
cmd[cmd.index("-o")+1] = str(obj)
cmd[cmd.index(source["file"])] = str(e/"startup_regression.cpp")
subprocess.run(cmd, cwd=build, check=True)
link = shlex.split((build/"CMakeFiles/contract_regression.dir/link.txt").read_text())
link[link.index("-o")+1] = str(e/f"fixture-{tag}")
link[next(i for i,x in enumerate(link) if x.endswith("contract_regression.cpp.o"))] = str(obj)
subprocess.run(link, cwd=build, check=True)
