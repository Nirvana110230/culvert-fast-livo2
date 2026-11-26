"""Mid-360 series protocol v1.4.12. Status performs UDP queries only; no SDK init."""
import argparse,binascii,json,os,pathlib,socket,struct,sys,time,zlib
IP='192.168.2.163';HOST='192.168.2.100';SERIAL=os.environ.get('FAST_LIVO2_RADAR_SERIAL', '')
KEYS={0x001a:'target_mode',0x8006:'current_mode',0x8007:'core_temp_raw',0x8002:'firmware',0x8008:'powerup_count',0x001c:'imu_enabled',0x8000:'serial'}
class Device:
 def __init__(self):
  if not SERIAL.strip():raise RuntimeError('FAST_LIVO2_RADAR_SERIAL is required')
  self.sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);self.sock.bind((HOST,0));self.seq=time.monotonic_ns()&0xffff
 def request(self,cmd,data):
  self.seq=(self.seq+1)&0xffff;header=struct.pack('<BBHIHBB6s',0xaa,0,24+len(data),self.seq,cmd,0,0,b'\0'*6);packet=header+struct.pack('<HI',binascii.crc_hqx(header,0xffff),zlib.crc32(data) if data else 0)+data
  for attempt in range(3):
   self.sock.sendto(packet,(IP,56100));deadline=time.monotonic()+1.5
   while time.monotonic()<deadline:
    self.sock.settimeout(max(.01,deadline-time.monotonic()))
    try:r,addr=self.sock.recvfrom(65535)
    except socket.timeout:break
    if addr[0]!=IP or len(r)<24:continue
    sof,ver,length,seq,r_cmd,kind,sender=struct.unpack_from('<BBHIHBB',r)
    if (sof,seq,r_cmd,kind)!=(0xaa,self.seq,cmd,1):continue
    if length!=len(r) or binascii.crc_hqx(r[:18],0xffff)!=struct.unpack_from('<H',r,18)[0] or (zlib.crc32(r[24:]) if r[24:] else 0)!=struct.unpack_from('<I',r,20)[0]:raise RuntimeError('Reply CRC/length invalid')
    return r[24:]
  raise RuntimeError('Device query/control timeout; check radar power and cable')
 def query(self,keys=None):
  keys=list(KEYS) if keys is None else keys;r=self.request(0x0101,struct.pack('<HH',len(keys),0)+struct.pack('<'+'H'*len(keys),*keys));ret,num=struct.unpack_from('<BH',r);values={};pos=3
  if ret:raise RuntimeError('Query rejected ret_code='+str(ret))
  for _ in range(num):
   key,n=struct.unpack_from('<HH',r,pos);pos+=4;v=r[pos:pos+n];pos+=n;name=KEYS.get(key,hex(key))
   values[name]=v.split(b'\0')[0].decode() if key==0x8000 else list(v) if key==0x8002 else int.from_bytes(v,'little',signed=key==0x8007)
  if 'serial' in values:
   if values['serial']!=SERIAL:raise RuntimeError('Wrong radar serial; refusing control')
   values['serial']='verified'
  if 'core_temp_raw' in values:values['core_temp_c']=values['core_temp_raw']/100
  if 'current_mode' in values:values['state_name']={1:'SAMPLING',2:'IDLE',9:'READY'}.get(values['current_mode'],'TRANSITION_OR_OTHER')
  return {'host_s':time.time(),'ret_code':ret,'values':values}
 def set_mode(self,mode):
  r=self.request(0x0100,struct.pack('<HHHHB',1,0,0x001a,1,mode));ret,key=struct.unpack_from('<BH',r);ack={'ret_code':ret,'error_key':key,'host_s':time.time()}
  if ret or key:raise RuntimeError('Mode rejected '+str(ack))
  deadline=time.monotonic()+15
  while True:
   state=self.query()
   if state['values'].get('current_mode')==mode:return {'ack':ack,'confirmed':state}
   if time.monotonic()>deadline:raise RuntimeError('Mode ACK received but state not reached '+str(state))
   time.sleep(.2)
 def close(self):self.sock.close()
def main():
 p=argparse.ArgumentParser();p.add_argument('action',choices=['status','standby','start']);p.add_argument('--output');p.add_argument('--watch',type=int,default=0);a=p.parse_args();d=Device();result={'action':a.action,'host_s':time.time()};rc=0
 try:
  result['before']=d.query()
  if a.action!='status':result.update(d.set_mode(2 if a.action=='standby' else 1))
  if a.watch:
   result['watch']=[];end=time.monotonic()+min(300,a.watch)
   while time.monotonic()<end:result['watch'].append(d.query());time.sleep(min(10,max(0,end-time.monotonic())))
 except Exception as e:result['error']=repr(e);rc=1
 finally:d.close()
 if a.output:pathlib.Path(a.output).write_text(json.dumps(result,indent=2))
 print(json.dumps(result,indent=2));return rc
if __name__=='__main__':sys.exit(main())
