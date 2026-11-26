"""Bounded passive wire check; runs with read-only raw-socket privilege."""
import argparse,json,socket,struct,time
p=argparse.ArgumentParser();p.add_argument('--seconds',type=float,default=2);p.add_argument('--expect',choices=['idle','sampling']);a=p.parse_args();assert 0<a.seconds<=10
s=socket.socket(socket.AF_PACKET,socket.SOCK_RAW,socket.htons(3));s.bind(('enP3p49s0',0));s.settimeout(.1);counts={'point':0,'imu':0};end=time.monotonic()+a.seconds
while time.monotonic()<end:
 try:b=s.recv(65536)
 except socket.timeout:continue
 if b[12:14]!=b'\x08\x00':continue
 ip=b[14:];h=(ip[0]&15)*4
 if ip[9]!=17 or socket.inet_ntoa(ip[12:16])!='192.168.2.163':continue
 port=struct.unpack_from('!H',ip,h)[0]
 if port in [56300,56400]:counts['point' if port==56300 else 'imu']+=1
stats=struct.unpack('II',s.getsockopt(263,6,8));s.close();r={'host_s':time.time(),'seconds':a.seconds,'counts':counts,'capture_socket_drops':stats[1],'expected':a.expect}
ok=not any(counts.values()) if a.expect=='idle' else all(counts.values()) if a.expect=='sampling' else True
r['passed']=ok;print(json.dumps(r));raise SystemExit(0 if ok and stats[1]==0 else 1)
