"""Read-only inspection on the verified Orange Pi. Prints aggregate evidence only."""
from pathlib import Path
import hashlib
import json
import subprocess
import xml.etree.ElementTree as ET

p=Path('/data/robot/maps/fast_livo2/runs/corridor03_20261128T072245Z_tzunG9/20261128T072648Z_c05694da93')
proc=json.loads((p/'algorithm_process.json').read_text())
session=json.loads((p/'session.json').read_text())
boot=proc['boot_id'].replace('-','')


def journal(filters):
    command=['journalctl','-q','--no-pager', '_BOOT_ID='+boot,
             '--since=2026-11-28 15:26:00','--until=2026-11-28 16:02:30','-o','cat']+filters
    result=subprocess.run(command,capture_output=True)
    return dict(return_code=result.returncode,line_count=len(result.stdout.splitlines()),stderr_empty=not result.stderr)


root=ET.parse(p/'sensors/camera/genicam.xml').getroot()
xml_tags={}
for obj in root.iter():
    name=obj.attrib.get('Name','')
    if name in ['DeviceTimestamp','DeviceTimestamp_Reg','DeviceTimestampLatch','DeviceTimestampIncrement','DeviceTimestampLatchValue']:
        xml_tags[name]={node.tag.rsplit('}',1)[-1]:node.text for node in obj if node.tag.rsplit('}',1)[-1] in
                        ['Description','Unit','pValue','Length','Address','CommandValue']}
tag_source=p/'sensors/camera/genicam.xml'
print(json.dumps(dict(
    roslog_bytes={f.name:f.stat().st_size for f in (p/'roslog').iterdir() if f.is_file()},
    diagnostics_columns={f.name:f.open().readline().strip().split(',') for f in (p/'diagnostics').glob('*.csv')},
    input_counters=json.loads((p/'Log/input_counters.json').read_text()),
    session_mapper_counter_names=list(session['mapper_counters']),
    keyframe_completion=json.loads((p/'keyframes/complete.json').read_text()),
    disabled_external_lib_logs='--disable-external-lib-logs' in proc['argv'],
    historical_boot_in_journal_list=boot in subprocess.check_output(['journalctl','--list-boots','--no-pager'],text=True),
    historical_journal_filtered=journal(['--grep','static gyro update']),historical_journal_process=journal(['_PID='+str(proc['pid'])]),
    historical_journal_any=journal([]),camera_genicam_sha256=hashlib.sha256(tag_source.read_bytes()).hexdigest(),
    camera_xml_nodes=xml_tags), indent=2,ensure_ascii=False))
