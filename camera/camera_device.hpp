#pragma once
#include <MvCameraControl.h>
#include <chrono>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
inline int64_t host_ns(){return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();}
inline void require_ok(int rc,const std::string& what){if(rc){std::ostringstream s;s<<what<<" rc=0x"<<std::hex<<static_cast<unsigned>(rc);throw std::runtime_error(s.str());}}
class CameraDevice {
public:
 void* handle=nullptr;bool initialized=false,opened=false,grabbing=false;
 std::string model,serial;std::vector<std::pair<std::string,int>> cleanup;
 explicit CameraDevice(const std::string& target){
  try{
   require_ok(MV_CC_Initialize(),"SDK initialize");initialized=true;
   MV_CC_DEVICE_INFO_LIST list{};require_ok(MV_CC_EnumDevices(MV_USB_DEVICE,&list),"USB enumeration");
   MV_CC_DEVICE_INFO* selected=nullptr;
   for(unsigned i=0;i<list.nDeviceNum;i++){
    auto* d=list.pDeviceInfo[i];if(d->nTLayerType!=MV_USB_DEVICE)continue;
    std::string sn(reinterpret_cast<char*>(d->SpecialInfo.stUsb3VInfo.chSerialNumber));
    if(sn==target){if(selected)throw std::runtime_error("Duplicate serial");selected=d;}
   }
   if(!selected)throw std::runtime_error("Requested camera serial not present");
   serial=target;model=reinterpret_cast<char*>(selected->SpecialInfo.stUsb3VInfo.chModelName);
   require_ok(MV_CC_CreateHandle(&handle,selected),"create handle");
   require_ok(MV_CC_OpenDevice(handle,MV_ACCESS_Exclusive,0),"exclusive open");opened=true;
  }catch(...){close();throw;}
 }
 void close(){
  if(grabbing){cleanup.emplace_back("StopGrabbing",MV_CC_StopGrabbing(handle));grabbing=false;}
  if(opened){cleanup.emplace_back("CloseDevice",MV_CC_CloseDevice(handle));opened=false;}
  if(handle){cleanup.emplace_back("DestroyHandle",MV_CC_DestroyHandle(handle));handle=nullptr;}
  if(initialized){cleanup.emplace_back("Finalize",MV_CC_Finalize());initialized=false;}
 }
 ~CameraDevice(){close();}
 void start(){require_ok(MV_CC_SetImageNodeNum(handle,4),"SetImageNodeNum(4)");require_ok(MV_CC_StartGrabbing(handle),"StartGrabbing");grabbing=true;}
 void snapshot(const std::string& path){
  std::ofstream o(path);o<<std::setprecision(12)<<"{\"host_ns\":"<<host_ns()<<",\"sdk_version\":"<<MV_CC_GetSDKVersion()<<",\"model\":\""<<model<<"\",\"serial_masked\":\"verified\",\"parameters\":{";bool first=true;
  auto key=[&](const char* s,int rc){if(!first)o<<',';first=false;o<<'\"'<<s<<"\":{\"rc\":"<<static_cast<unsigned>(rc);};
  for(auto s:{"Width","Height","WidthMax","HeightMax","OffsetX","OffsetY","PayloadSize","AutoExposureTimeUpperLimit","AutoExposureTimeLowerLimit","TimestampTickFrequency","GevTimestampTickFrequency"}){MVCC_INTVALUE_EX v{};int rc=MV_CC_GetIntValueEx(handle,s,&v);key(s,rc);if(!rc)o<<",\"value\":"<<v.nCurValue;o<<'}';}
  for(auto s:{"AcquisitionMode","TriggerSelector","TriggerMode","TriggerSource","ExposureAuto","UserSetSelector","UserSetDefault","UserSetDefaultSelector","PixelFormat","GainAuto","BalanceWhiteAuto"}){MVCC_ENUMVALUE v{};int rc=MV_CC_GetEnumValue(handle,s,&v);key(s,rc);if(!rc){o<<",\"value\":"<<v.nCurValue<<",\"supported\":[";for(unsigned j=0;j<v.nSupportedNum;j++){if(j)o<<',';o<<v.nSupportValue[j];}o<<']';}o<<'}';}
  for(auto s:{"AcquisitionFrameRate","ResultingFrameRate","ExposureTime","Gain"}){MVCC_FLOATVALUE v{};int rc=MV_CC_GetFloatValue(handle,s,&v);key(s,rc);if(!rc)o<<",\"value\":"<<v.fCurValue;o<<'}';}
  for(auto s:{"AcquisitionFrameRateEnable","ReverseX","ReverseY"}){bool v=false;int rc=MV_CC_GetBoolValue(handle,s,&v);key(s,rc);if(!rc)o<<",\"value\":"<<(v?"true":"false");o<<'}';}
  o<<"},\"trigger_selector_checks\":[";
  MVCC_ENUMVALUE selected{};int rc=MV_CC_GetEnumValue(handle,"TriggerSelector",&selected);
  if(!rc){for(unsigned i=0;i<selected.nSupportedNum;i++){if(i)o<<',';auto v=selected.nSupportValue[i];int set=MV_CC_SetEnumValue(handle,"TriggerSelector",v);MVCC_ENUMVALUE mode{},source{};int m=MV_CC_GetEnumValue(handle,"TriggerMode",&mode);int s=MV_CC_GetEnumValue(handle,"TriggerSource",&source);o<<"{\"selector\":"<<v<<",\"select_rc\":"<<static_cast<unsigned>(set)<<",\"mode_rc\":"<<static_cast<unsigned>(m)<<",\"mode\":"<<mode.nCurValue<<",\"source_rc\":"<<static_cast<unsigned>(s)<<",\"source\":"<<source.nCurValue<<'}';}require_ok(MV_CC_SetEnumValue(handle,"TriggerSelector",selected.nCurValue),"restore TriggerSelector");}
  o<<"],\"nonpersistent_writes\":\"TriggerSelector read sweep then restored; no UserSetSave/Load/default change\"}";
 }
 void xml(const std::string& path){std::vector<unsigned char>b(8*1024*1024);unsigned len=0;int rc=MV_XML_GetGenICamXML(handle,b.data(),b.size(),&len);if(!rc){std::ofstream f(path,std::ios::binary);f.write(reinterpret_cast<char*>(b.data()),len);} }
 void usb_stats(const std::string& path){MV_MATCH_INFO_USB_DETECT u{};MV_ALL_MATCH_INFO m{};m.nType=MV_MATCH_TYPE_USB_DETECT;m.pInfo=&u;m.nInfoSize=sizeof(u);int rc=MV_CC_GetAllMatchInfo(handle,&m);std::ofstream o(path);o<<"{\"rc\":"<<static_cast<unsigned>(rc)<<",\"received_bytes\":"<<u.nReceiveDataSize<<",\"received_frames\":"<<u.nReceivedFrameCount<<",\"error_frames\":"<<u.nErrorFrameCount<<'}';}
 void assert_baseline(){
  MVCC_INTVALUE_EX w{},h{};require_ok(MV_CC_GetIntValueEx(handle,"Width",&w),"Width");require_ok(MV_CC_GetIntValueEx(handle,"Height",&h),"Height");
  if(w.nCurValue!=1280 || h.nCurValue!=1024)throw std::runtime_error("Unexpected image dimensions; inspect saved settings");
  MVCC_ENUMVALUE trigger{};require_ok(MV_CC_GetEnumValue(handle,"TriggerMode",&trigger),"TriggerMode");if(trigger.nCurValue!=0)throw std::runtime_error("TriggerMode is on; inspect before any setting changes");
 }
};
struct FrameGuard{void* h;MV_FRAME_OUT frame{};bool held=false;explicit FrameGuard(void* h):h(h){}~FrameGuard(){if(held)MV_CC_FreeImageBuffer(h,&frame);}void release(){if(held){require_ok(MV_CC_FreeImageBuffer(h,&frame),"FreeImageBuffer");held=false;}}};
