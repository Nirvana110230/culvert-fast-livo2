#include <cstdlib>
// Official reproduction: image header is the shared LiDAR scan base.
#include "camera_device.hpp"
#include "shared_timestamp.hpp"
#include "../include/rolling_csv.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <iostream>
#include <mutex>
#include <map>
#include <memory>
#include <algorithm>
#include <csignal>
#include <deque>

static volatile std::sig_atomic_t stop_requested=0;
static void request_stop(int){stop_requested=1;}

static int64_t mono_ns(){return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
struct Settings {
 void *h;std::ofstream log;std::vector<std::string> order;
 struct Value {char type;int64_t value;};std::map<std::string,Value> old;
 bool restored=false;
 Settings(void *handle,const std::filesystem::path& out):h(handle),log(out/"settings.csv") {log<<"operation,node,requested,rc,readback\n";}
 void save(const std::string& name,char type,int64_t value){if(!old.count(name)){old[name]={type,value};order.push_back(name);}}
 void set_enum(const std::string& name,unsigned value){
  MVCC_ENUMVALUE before{};require_ok(MV_CC_GetEnumValue(h,name.c_str(),&before),"read "+name);
  if(std::find(before.nSupportValue,before.nSupportValue+before.nSupportedNum,value)==before.nSupportValue+before.nSupportedNum)throw std::runtime_error("Requested enum unsupported: "+name);
  save(name,'e',before.nCurValue);int rc=MV_CC_SetEnumValue(h,name.c_str(),value);MVCC_ENUMVALUE after{};int rd=MV_CC_GetEnumValue(h,name.c_str(),&after);log<<"set,"<<name<<','<<value<<','<<static_cast<unsigned>(rc)<<','<<(rd?-1:int64_t(after.nCurValue))<<'\n';log.flush();require_ok(rc,"set "+name);require_ok(rd,"readback "+name);if(after.nCurValue!=value)throw std::runtime_error("Enum readback mismatch: "+name);
 }
 void set_int(const std::string& name,int64_t value){MVCC_INTVALUE_EX before{};require_ok(MV_CC_GetIntValueEx(h,name.c_str(),&before),"read "+name);save(name,'i',before.nCurValue);int rc=MV_CC_SetIntValueEx(h,name.c_str(),value);MVCC_INTVALUE_EX after{};int rd=MV_CC_GetIntValueEx(h,name.c_str(),&after);log<<"set,"<<name<<','<<value<<','<<static_cast<unsigned>(rc)<<','<<(rd?-1:after.nCurValue)<<'\n';log.flush();require_ok(rc,"set "+name);require_ok(rd,"readback "+name);if(after.nCurValue!=value)throw std::runtime_error("Integer readback mismatch: "+name);}
 void set_bool(const std::string& name,bool value,bool optional=false){bool before=false;int rd=MV_CC_GetBoolValue(h,name.c_str(),&before);if(optional&&rd){log<<"unavailable,"<<name<<','<<value<<','<<static_cast<unsigned>(rd)<<",-1\n";return;}require_ok(rd,"read "+name);save(name,'b',before);int rc=MV_CC_SetBoolValue(h,name.c_str(),value);bool after=false;rd=MV_CC_GetBoolValue(h,name.c_str(),&after);log<<"set,"<<name<<','<<value<<','<<static_cast<unsigned>(rc)<<','<<(rd?-1:int(after))<<'\n';log.flush();require_ok(rc,"set "+name);require_ok(rd,"readback "+name);if(after!=value)throw std::runtime_error("Bool readback mismatch: "+name);}
 int apply(const std::string& name,Value v){if(v.type=='e')return MV_CC_SetEnumValue(h,name.c_str(),v.value);if(v.type=='b')return MV_CC_SetBoolValue(h,name.c_str(),v.value);return MV_CC_SetIntValueEx(h,name.c_str(),v.value);}
 bool matches(const std::string& name,Value v){if(v.type=='e'){MVCC_ENUMVALUE x{};return !MV_CC_GetEnumValue(h,name.c_str(),&x)&&x.nCurValue==v.value;}if(v.type=='b'){bool x=false;return !MV_CC_GetBoolValue(h,name.c_str(),&x)&&x==bool(v.value);}MVCC_INTVALUE_EX x{};return !MV_CC_GetIntValueEx(h,name.c_str(),&x)&&x.nCurValue==v.value;}
 bool restore(){if(restored)return true;bool ok=true;
  if(old.count("TriggerMode")){int rc=MV_CC_SetEnumValue(h,"TriggerMode",0);log<<"restore_off,TriggerMode,0,"<<static_cast<unsigned>(rc)<<",-1\n";ok=ok&&!rc;}
  for(auto it=order.rbegin();it!=order.rend();++it){if(*it=="TriggerMode")continue;auto v=old[*it];int rc=apply(*it,v);bool match=!rc&&matches(*it,v);log<<"restore,"<<*it<<','<<v.value<<','<<static_cast<unsigned>(rc)<<','<<match<<'\n';ok=ok&&match;}
  if(old.count("TriggerMode")){auto v=old["TriggerMode"];int rc=apply("TriggerMode",v);bool match=!rc&&matches("TriggerMode",v);log<<"restore,TriggerMode,"<<v.value<<','<<static_cast<unsigned>(rc)<<','<<match<<'\n';ok=ok&&match;}
  log.flush();restored=true;return ok;
 }
 ~Settings(){if(!restored)restore();}
};

struct Events {
 void *h;RollingCsv log;std::ofstream setup;std::mutex mutex;std::vector<std::pair<unsigned,unsigned>> previous;
 struct Record {std::string name;unsigned id;uint64_t tick,block;int64_t host,mono;};
 std::deque<Record> pending;uint64_t written=0,dropped=0;int64_t last_exposure_mono=0;
 unsigned old_selector=0;bool have_selector=false,restored=false;int register_rc=-1;
 rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub;
 Events(void *handle,const std::filesystem::path& out,rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher):h(handle),log((out/"events.csv").string(),"event_name,event_id,event_device_timestamp_raw,block_id_raw,host_realtime_ns,host_monotonic_ns"),setup(out/"event_setup.csv"),pub(publisher){
  setup<<"selector,select_rc,notification_read_rc,notification_set_rc\n";
  MVCC_ENUMVALUE selectors{};int rc=MV_CC_GetEnumValue(h,"EventSelector",&selectors);if(rc){setup<<"unavailable,"<<static_cast<unsigned>(rc)<<",-1,-1\n";return;}
  old_selector=selectors.nCurValue;have_selector=true;register_rc=MV_CC_RegisterAllEventCallBack(h,callback,this);if(register_rc){setup<<"register,"<<static_cast<unsigned>(register_rc)<<",-1,-1\n";return;}
  // Use only values advertised by this device. XML presence is insufficient.
  for(unsigned wanted:{0x9002u,0x3u,0x4u,0x9006u,0x9007u,0x900fu,0x9011u,0x900bu,0x800du,0x9010u}) {
   if(std::find(selectors.nSupportValue,selectors.nSupportValue+selectors.nSupportedNum,wanted)==selectors.nSupportValue+selectors.nSupportedNum)continue;
   int sel=MV_CC_SetEnumValue(h,"EventSelector",wanted);MVCC_ENUMVALUE notification{};int rd=sel?sel:MV_CC_GetEnumValue(h,"EventNotification",&notification);int wr=-1;
   if(!rd){
    wr=MV_CC_SetEnumValue(h,"EventNotification",1);
    MVCC_ENUMVALUE after{};int ar=MV_CC_GetEnumValue(h,"EventNotification",&after);
    // Some advertised events are read-only on this firmware. Restore only
    // successfully changed state; a rejected write with unchanged readback
    // must not create a fictitious restore failure.
    if(!wr||ar||after.nCurValue!=notification.nCurValue)previous.push_back({wanted,notification.nCurValue});
   }setup<<wanted<<','<<static_cast<unsigned>(sel)<<','<<static_cast<unsigned>(rd)<<','<<static_cast<unsigned>(wr)<<'\n';
  }
  MV_CC_SetEnumValue(h,"EventSelector",old_selector);setup.flush();
 }
 static void __stdcall callback(MV_EVENT_OUT_INFO *info,void *user){
  auto &e=*static_cast<Events*>(user);auto host=host_ns(),mono=mono_ns();uint64_t tick=(uint64_t(info->nTimestampHigh)<<32)|info->nTimestampLow;uint64_t block=(uint64_t(info->nBlockIdHigh)<<32)|info->nBlockIdLow;
  std::string name(info->EventName,strnlen(info->EventName,MAX_EVENT_NAME_SIZE));
  // Never call DDS, disk I/O or a blocking SDK operation from its callback.
  // Keep callback work bounded. Intermittent upstream event loss is separately
  // detected; moving DDS out of this callback alone did not eliminate it.
  std::lock_guard<std::mutex>guard(e.mutex);
  if(e.pending.size()>=50000){e.dropped++;return;}
  e.pending.push_back({std::move(name),info->nEventID,tick,block,host,mono});
 }
 void drain(){std::deque<Record> batch;{std::lock_guard<std::mutex>guard(mutex);batch.swap(pending);}
  for(const auto&r:batch){log.row()<<r.name<<','<<r.id<<','<<r.tick<<','<<r.block<<','<<r.host<<','<<r.mono<<'\n';written++;if(r.name=="ExposureStart")last_exposure_mono=r.mono;
   std_msgs::msg::String m;std::ostringstream s;s<<"{\"event_name\":\""<<r.name<<"\",\"event_id\":"<<r.id<<",\"event_device_timestamp_raw\":"<<r.tick<<",\"block_id_raw\":"<<r.block<<",\"host_realtime_ns\":"<<r.host<<",\"host_monotonic_ns\":"<<r.mono<<",\"block_id_validated\":false}";m.data=s.str();pub->publish(m);
  }log.flush();
 }
 bool restore(){if(restored)return true;bool ok=true;for(auto it=previous.rbegin();it!=previous.rend();++it){int a=MV_CC_SetEnumValue(h,"EventSelector",it->first);int b=a?a:MV_CC_SetEnumValue(h,"EventNotification",it->second);MVCC_ENUMVALUE v{};int rd=b?b:MV_CC_GetEnumValue(h,"EventNotification",&v);setup<<"restore_"<<it->first<<','<<static_cast<unsigned>(a)<<','<<static_cast<unsigned>(b)<<','<<(rd?-1:int64_t(v.nCurValue))<<'\n';ok=ok&&!a&&!b&&!rd&&v.nCurValue==it->second;}if(have_selector)ok=(!MV_CC_SetEnumValue(h,"EventSelector",old_selector))&&ok;setup.flush();restored=true;return ok;}
 ~Events(){restore();}
};

int main(int argc,char**argv){
 const char* configured_serial=std::getenv("FAST_LIVO2_CAMERA_SERIAL");
 std::string serial=configured_serial ? configured_serial : "",mode,line,edge,out_string;double seconds=0;
 try{for(int i=1;i<argc;i++){std::string k=argv[i];if(i+1>=argc)throw std::runtime_error("Option requires value: "+k);std::string v=argv[++i];if(k=="--serial")serial=v;else if(k=="--mode")mode=v;else if(k=="--line")line=v;else if(k=="--edge")edge=v;else if(k=="--output")out_string=v;else if(k=="--seconds")seconds=std::stod(v);else throw std::runtime_error("Unknown option "+k);}
 if(serial.empty())throw std::runtime_error("FAST_LIVO2_CAMERA_SERIAL or --serial is required");
 if((mode!="free"&&mode!="trigger")||out_string.empty()||!(seconds>=0&&std::isfinite(seconds)))throw std::runtime_error("Require --mode free|trigger --output DIR --seconds >=0 (0 until stopped)");
 if(mode=="trigger"&&((line!="Line0"&&line!="Line2")||(edge!="RisingEdge"&&edge!="FallingEdge")))throw std::runtime_error("Trigger requires explicit --line Line0|Line2 --edge RisingEdge|FallingEdge");
 if(mode=="free"&&(!line.empty()||!edge.empty()))throw std::runtime_error("Line/edge only apply to trigger mode");
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 2;}
 std::filesystem::path out(out_string);if(std::filesystem::exists(out/"frames.csv")){std::cerr<<"Existing capture output\n";return 2;}std::filesystem::create_directories(out);
 // Keep ROS alive until the camera/event workers are stopped and settings are
 // restored. The default ROS handler shuts publishers down during an SDK read,
 // losing the final metadata/image pair before our cleanup can finish.
 rclcpp::init(0,nullptr,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);
 std::signal(SIGINT,request_stop);std::signal(SIGTERM,request_stop);
 auto node=std::make_shared<rclcpp::Node>("hik_sync_capture");
 auto image_pub=node->create_publisher<sensor_msgs::msg::Image>("/hik_camera/image_timeshare",rclcpp::QoS(8));auto info_pub=node->create_publisher<sensor_msgs::msg::CameraInfo>("/hik_camera/camera_info_timeshare",rclcpp::SensorDataQoS().keep_last(5));auto meta_pub=node->create_publisher<std_msgs::msg::String>("/hardware_sync/camera_metadata",100);auto event_pub=node->create_publisher<std_msgs::msg::String>("/hardware_sync/camera_events",100);
 auto clock_pub=node->create_publisher<std_msgs::msg::String>("/hardware_sync/camera_clock",100);
 RollingCsv clocks((out/"clock_latch.csv").string(),"host_before_ns,monotonic_before_ns,latch_rc,read_rc,device_timestamp_value,host_after_ns,monotonic_after_ns");
 RollingCsv csv((out/"frames.csv").string(),"frame_num,device_timestamp_raw,sdk_host_timestamp_raw,host_after_get_ns,host_monotonic_after_get_ns,publish_done_ns,width,height,pixel_type,source_bytes,rgb_bytes,exposure_frame_us,gain_frame,trigger_index,frame_counter,input_bits,output_bits,lost_packets,conversion_ms");
 unsigned count=0,timeouts=0;uint64_t events_written=0,events_dropped=0;int result=0;bool restored=false;std::string error;cv::Mat last;
 try{
  SharedTimestamp shared(timestamp_environment("LIVO_TIMESHARE_PATH"), false);
  RollingCsv share_trace((out/"timeshare_reads.csv").string(),
              "frame_num,device_timestamp_raw,host_after_get_ns,host_monotonic_after_get_ns,"
              "read_before_mono_ns,read_after_mono_ns,shared_stamp_ns,image_header_ns,"
              "offset_ns,fusion_candidate_ns");
  CameraDevice c(serial);c.xml((out/"genicam.xml").string());c.snapshot((out/"parameters.before.json").string());
  {Settings settings(c.handle,out);Events events(c.handle,out,event_pub);
   struct CleanupGuard { CameraDevice &c;Settings &s;Events &e;~CleanupGuard(){if(c.grabbing){MV_CC_StopGrabbing(c.handle);c.grabbing=false;}e.restore();s.restore();c.close();} } cleanup_guard{c,settings,events};
   try{
    MVCC_ENUMVALUE acquisition{};require_ok(MV_CC_GetEnumValue(c.handle,"AcquisitionMode",&acquisition),"AcquisitionMode");if(acquisition.nCurValue!=2)throw std::runtime_error("Require existing Continuous acquisition mode");
    if(mode=="trigger"){
     settings.set_enum("TriggerMode",0);settings.set_enum("TriggerSelector",6);settings.set_enum("TriggerSource",line=="Line0"?0:2);settings.set_enum("TriggerActivation",edge=="RisingEdge"?0:1);settings.set_int("AcquisitionBurstFrameCount",1);settings.set_bool("AcquisitionFrameRateEnable",false);settings.set_bool("TriggerCacheEnable",false,true);settings.set_enum("TriggerMode",1);
    }else c.assert_baseline();
    c.snapshot((out/"parameters.active.json").string());c.start();auto started=std::chrono::steady_clock::now();int64_t next_latch=0;
    while(!stop_requested&&rclcpp::ok()&&(seconds==0||std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count()<seconds)){
     events.drain();
     if(mode=="trigger"&&count>=20&&(!events.last_exposure_mono||mono_ns()-events.last_exposure_mono>1500000000))
      throw std::runtime_error("Exposure event stream stopped while image frames continue; rejecting synchronization evidence");
     if(mono_ns()>=next_latch){auto hb=host_ns(),mb=mono_ns();int lr=MV_CC_SetCommandValue(c.handle,"DeviceTimestampLatch");MVCC_INTVALUE_EX v{};int rr=MV_CC_GetIntValueEx(c.handle,"DeviceTimestamp",&v);auto ma=mono_ns(),ha=host_ns();clocks.row()<<hb<<','<<mb<<','<<unsigned(lr)<<','<<unsigned(rr)<<','<<v.nCurValue<<','<<ha<<','<<ma<<'\n';clocks.flush();next_latch=ma+1000000000;
      std_msgs::msg::String msg;std::ostringstream s;s<<"{\"monotonic_before_ns\":"<<mb<<",\"monotonic_after_ns\":"<<ma<<",\"device_timestamp_value\":"<<v.nCurValue<<",\"latch_rc\":"<<unsigned(lr)<<",\"read_rc\":"<<unsigned(rr)<<'}';msg.data=s.str();clock_pub->publish(msg);
     }
     FrameGuard frame(c.handle);int rc=MV_CC_GetImageBuffer(c.handle,&frame.frame,500);if(rc==MV_E_NODATA){timeouts++;continue;}require_ok(rc,"GetImageBuffer");frame.held=true;auto host=host_ns(),mono=mono_ns();auto i=frame.frame.stFrameInfo;
     // Official grab_trigger reads the shared value immediately after successful acquisition.
     const auto shared_read_before = mono_ns();
     const uint64_t shared_stamp = shared.read_valid();
     const auto shared_read_after = mono_ns();
     if(i.nWidth!=1280||i.nHeight!=1024||!i.nFrameLen)throw std::runtime_error("Unexpected source frame shape");std::vector<unsigned char>rgb(size_t(i.nWidth)*i.nHeight*3);MV_CC_PIXEL_CONVERT_PARAM p{};p.nWidth=i.nWidth;p.nHeight=i.nHeight;p.pSrcData=frame.frame.pBufAddr;p.nSrcDataLen=i.nFrameLen;p.enSrcPixelType=i.enPixelType;p.enDstPixelType=PixelType_Gvsp_RGB8_Packed;p.pDstBuffer=rgb.data();p.nDstBufferSize=rgb.size();require_ok(MV_CC_ConvertPixelType(c.handle,&p),"RGB conversion");if(p.nDstLen!=rgb.size())throw std::runtime_error("RGB byte count mismatch");auto converted=host_ns();uint64_t dev=(uint64_t(i.nDevTimeStampHigh)<<32)|i.nDevTimeStampLow;
     share_trace.row() << i.nFrameNum << ',' << dev << ',' << host << ',' << mono << ','
                 << shared_read_before << ',' << shared_read_after << ',' << shared_stamp << ','
                 << shared_stamp << ',' << 100000000 << ',' << shared_stamp + 100000000ULL << '\n';
     share_trace.flush();
     std_msgs::msg::Header header;header.frame_id="hik_camera_optical_frame";header.stamp=rclcpp::Time(static_cast<int64_t>(shared_stamp),RCL_SYSTEM_TIME);cv::Mat mat(i.nHeight,i.nWidth,CV_8UC3,rgb.data());auto image=cv_bridge::CvImage(header,"rgb8",mat).toImageMsg();sensor_msgs::msg::CameraInfo info;info.header=header;info.width=i.nWidth;info.height=i.nHeight;image_pub->publish(*image);info_pub->publish(info);auto published=host_ns();
     std::ostringstream s;s<<std::setprecision(12)<<"{\"frame_num\":"<<i.nFrameNum<<",\"device_timestamp_raw\":"<<dev<<",\"sdk_host_timestamp_raw\":"<<i.nHostTimeStamp<<",\"host_after_get_ns\":"<<host<<",\"host_monotonic_after_get_ns\":"<<mono<<",\"publish_done_ns\":"<<published<<",\"exposure_frame_us\":"<<i.fExposureTime<<",\"gain_frame\":"<<i.fGain<<",\"trigger_index\":"<<i.nTriggerIndex<<",\"frame_counter\":"<<i.nFrameCounter<<",\"input_bits\":"<<i.nInput<<",\"output_bits\":"<<i.nOutput<<",\"lost_packets\":"<<i.nLostPacket<<",\"trigger_fields_validated\":false,\"header_source\":\"timeshare LiDAR scan base; offset applied only inside FAST-LIVO2\"}";std_msgs::msg::String metadata;metadata.data=s.str();meta_pub->publish(metadata);
     csv.row()<<i.nFrameNum<<','<<dev<<','<<host<<','<<mono<<','<<published<<','<<i.nWidth<<','<<i.nHeight<<','<<unsigned(i.enPixelType)<<','<<i.nFrameLen<<','<<p.nDstLen<<','<<i.fExposureTime<<','<<i.fGain<<','<<i.nTriggerIndex<<','<<i.nFrameCounter<<','<<i.nInput<<','<<i.nOutput<<','<<i.nLostPacket<<','<<(converted-host)/1e6<<'\n';
     if(count==0||count%100==0){cv::cvtColor(mat,last,cv::COLOR_RGB2BGR);if(count==0)cv::imwrite((out/"first.png").string(),last);}frame.release();count++;if(count%10==0)csv.flush();
    }
   }catch(const std::exception&e){result=1;error=e.what();}
   c.usb_stats((out/"usb_stats.json").string());if(c.grabbing){int rc=MV_CC_StopGrabbing(c.handle);c.grabbing=false;if(rc){result=1;error+="; StopGrabbing failed";}}
   events.drain();bool events_ok=events.restore();bool settings_ok=settings.restore();restored=events_ok&&settings_ok;if(!restored){result=1;error+="; settings/event restoration failed";}c.snapshot((out/"parameters.after.json").string());
   // Keep callback state alive until SDK closes its event worker threads.
   c.close();events.drain();events_written=events.written;events_dropped=events.dropped;for(auto &entry:c.cleanup)if(entry.second){result=1;error+="; cleanup "+entry.first+" failed";}
  }
 }catch(const std::exception&e){result=1;error=e.what();}
 if(!last.empty())cv::imwrite((out/"last_sample.png").string(),last);csv.close();std::ofstream summary(out/"driver_summary.json");summary<<"{\"frames\":"<<count<<",\"timeouts\":"<<timeouts<<",\"events_written\":"<<events_written<<",\"events_queue_dropped\":"<<events_dropped<<",\"exit_code\":"<<result<<",\"settings_restored\":"<<(restored?"true":"false")<<",\"mode\":\""<<mode<<"\",\"calibrated\":false,\"header_time_source\":\"timeshare LiDAR scan base; FAST-LIVO2 img_time_offset=0.1\",\"error_present\":"<<(!error.empty()?"true":"false")<<"}";if(!error.empty())std::cerr<<error<<'\n';if(rclcpp::ok())rclcpp::shutdown();return result;
}
