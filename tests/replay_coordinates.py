"""Independently compare native ROS outputs to the PCD/TUM a headless replay saved."""
import json
import struct
import sys
import time
from pathlib import Path

import numpy as np


def observe(out):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import Odometry
    rclpy.init()
    node=rclpy.create_node('fast_livo2_coordinate_probe')
    rows={'rgb':[],'geometry':[],'poses':[]}

    def cloud(msg,kind):
        if len(rows[kind])>=3 or not msg.width or not msg.height or not msg.fields:return
        assert msg.header.frame_id=='camera_init'
        fields={f.name:f for f in msg.fields}
        assert all(fields[k].datatype==7 for k in 'xyz')
        buf=memoryview(msg.data)
        for i in range(msg.width):
            p=[struct.unpack_from(('>' if msg.is_bigendian else '<')+'f',buf,i*msg.point_step+fields[k].offset)[0] for k in 'xyz']
            if all(np.isfinite(p)):
                rows[kind].append(p);return

    def pose(msg):
        if len(rows['poses'])>=4:return
        assert msg.header.frame_id=='camera_init'
        t=msg.header.stamp;p=msg.pose.pose.position;q=msg.pose.pose.orientation
        rows['poses'].append([f'{t.sec}.{t.nanosec:09d}',[p.x,p.y,p.z],[q.x,q.y,q.z,q.w]])
    subscriptions=[
        node.create_subscription(PointCloud2,'/cloud_registered',lambda m:cloud(m,'rgb'),qos_profile_sensor_data),
        node.create_subscription(PointCloud2,'/cloud_effected',lambda m:cloud(m,'geometry'),qos_profile_sensor_data),
        node.create_subscription(Odometry,'/aft_mapped_to_init',pose,qos_profile_sensor_data)]
    out.with_suffix('.ready').touch()
    try:
        deadline=time.monotonic()+165
        while min(len(v) for v in rows.values())<3 and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.15)
        if min(len(v) for v in rows.values())<3:raise RuntimeError('missing native samples')
        out.write_text(json.dumps(rows))
    finally:
        node.destroy_node();rclpy.shutdown()


def pcd(path):
    with path.open('rb') as f:
        header={}
        for _ in range(32):
            line=f.readline().decode('ascii').strip()
            if line=='# frame_id map_gravity':header['FRAME']=['map_gravity']
            if not line or line.startswith('#'):continue
            k,*v=line.split();header[k]=v
            if k=='DATA':break
        assert header['DATA']==['binary'] and header['FRAME']==['map_gravity']
        ns,ts,ss,cs=(header[k] for k in ('FIELDS','TYPE','SIZE','COUNT'))
        fmts={'F':'f','I':'i','U':'u'}
        dtype=np.dtype([(n,'<'+fmts[t]+s) if c=='1' else (n,'<'+fmts[t]+s,int(c))
                        for n,t,s,c in zip(ns,ts,ss,cs)])
        a=np.fromfile(f,dtype=dtype)
        assert len(a)==int(header['POINTS'][0])
        return np.column_stack([a[k] for k in 'xyz']).astype(float)


def verify(source,run):
    rows=json.loads(source.read_text())
    main=run/'map'; manifest=json.loads((main/'manifest.json').read_text())
    g=np.asarray(manifest['transform']['gravity_camera_init_m_s2'])
    # Rodrigues' formula uses the gravity vectors directly, independent of the stored quaternion.
    a=g/np.linalg.norm(g); b=np.array([0.,0.,-1.]); v=np.cross(a,b); c=np.dot(a,b)
    K=np.array([[0.,-v[2],v[1]],[v[2],0.,-v[0]],[-v[1],v[0],0.]])
    if c>-1+1e-12: R=np.eye(3)+K+(K@K)/(1+c)
    else:
        axis=np.cross(a,np.eye(3)[np.argmin(np.abs(a))]);axis/=np.linalg.norm(axis)
        R=2*np.outer(axis,axis)-np.eye(3)
    def matrix(q):
        x,y,z,w=np.array(q)/np.linalg.norm(q)
        return np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],
                         [2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],
                         [2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])
    proof={'input_frame':'camera_init','output_frame':'map_gravity','samples':{}}
    for kind in ['rgb','geometry']:
        xyz=pcd(main/f'map_{kind}.pcd')
        finite=xyz[np.isfinite(xyz).all(axis=1)]
        errors=[float(np.linalg.norm(finite-R@p,axis=1).min()) for p in rows[kind]]
        assert max(errors)<2e-5,(kind,errors)
        proof['samples'][kind]={'native':rows[kind],'nearest_saved_errors_m':errors,'file_points':len(xyz)}
    poses={l.split()[0]:list(map(float,l.split()[1:])) for l in (main/'trajectory.tum').read_text().splitlines() if l and not l.startswith('#')}
    pe=[];qe=[]
    for stamp,p,q in rows['poses']:
        saved=poses[stamp]
        pe.append(float(np.linalg.norm(R@p-saved[:3])))
        qe.append(float(2*np.arcsin(min(1.,np.linalg.norm(R@matrix(q)-matrix(saved[3:]))/(2*np.sqrt(2))))))
    assert max(pe)<1e-8 and max(qe)<1e-8,(pe,qe)
    proof['samples']['poses']={'stamps':[v[0] for v in rows['poses']],'position_error_m':pe,'orientation_error_rad':qe,'file_poses':len(poses)}
    proof['gravity_residual']=float(np.linalg.norm(R@g-[0,0,-np.linalg.norm(g)]))
    proof['ok']=True
    return proof

if __name__=='__main__':
    if sys.argv[1]=='observe':observe(Path(sys.argv[2]))
    else:
        result=verify(Path(sys.argv[2]),Path(sys.argv[3]))
        Path(sys.argv[4]).write_text(json.dumps(result,indent=2))
        print('independent native PointCloud2 and full odometry poses match committed PCD/TUM')
