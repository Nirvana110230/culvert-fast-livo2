"""Repeat actual RViz attach/detach during a recorded session and query its TF independently."""
import json
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
import rclpy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException

root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'runtime'))
from rviz_window import owned_window
out=Path(sys.argv[1])
run=Path((root/'state/latest_run.txt').read_text().strip())
expected=json.loads((run/'gravity_tf_check.json').read_text())['quaternion_xyzw']
result=[]
old_gids=set()
rclpy.init()
try:
    for index in (1,2):
        node=rclpy.create_node('reopen_view_verification_'+str(index))
        buffer=Buffer();listener=TransformListener(buffer,node)
        with (out/('rviz_'+str(index)+'.log')).open('w') as log:
            proc=subprocess.Popen(['bash',root/'view.sh'],stdout=log,stderr=subprocess.STDOUT)
            try:
                started=time.monotonic(); subscriptions=[]; got=None; visible=None; pid=None
                while time.monotonic()-started < 25:
                    if proc.poll() is not None:raise RuntimeError('RViz exited before subscribing')
                    rclpy.spin_once(node,timeout_sec=.1)
                    subscriptions=[bytes(x.endpoint_gid).hex() for x in node.get_subscriptions_info_by_topic('/livo_view/accumulated_rgb') if 'rviz' in x.node_name.lower() and bytes(x.endpoint_gid).hex() not in old_gids]
                    children=Path(f'/proc/{proc.pid}/task/{proc.pid}/children').read_text().split()
                    if children:
                        pid=int(children[0])
                        visible=owned_window(pid, dict(os.environ, DISPLAY=os.environ.get('DISPLAY',':1')))
                    try:
                        q=buffer.lookup_transform('map_gravity','camera_init',Time()).transform.rotation
                        got=[q.x,q.y,q.z,q.w]
                    except TransformException:pass
                    if subscriptions and visible and got is not None:break
                assert got is not None and subscriptions and visible,(got,subscriptions,visible)
                old_gids.update(subscriptions)
                assert abs(abs(sum(a*b for a,b in zip(expected,got)))-1)<1e-9
                result.append({'cycle':index,'subscription_gids':subscriptions,'pid':pid,'window':hex(visible),'lookup_xyzw':got})
            finally:
                if proc.poll() is None:proc.send_signal(signal.SIGINT)
                assert proc.wait(timeout=28)==0
        assert not Path(f'/proc/{pid}').exists()
        node.destroy_node()
finally:
    rclpy.shutdown()
(out/'viewer_cycles.json').write_text(json.dumps({'ok':True,'session':run.name,'cycles':result},indent=2))
