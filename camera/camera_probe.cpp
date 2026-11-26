// Non-streaming SDK capability probe. It never saves/loads a User Set.
#include "camera_device.hpp"
#include <thread>
#include <filesystem>
#include <iostream>

static int64_t monotonic_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}
int main(int argc, char **argv) {
  if (argc != 3) { std::cerr << "usage: camera_probe SERIAL NEW_OUTPUT_DIR\n"; return 2; }
  std::filesystem::path out(argv[2]);
  if (std::filesystem::exists(out)) { std::cerr << "Output exists; choose a new probe directory\n"; return 2; }
  std::filesystem::create_directories(out);
  try {
    CameraDevice c(argv[1]);
    c.xml((out / "genicam.xml").string());
    std::ofstream f(out / "capabilities.json");
    f << "{\"host_realtime_ns\":" << host_ns() << ",\"nodes\":{";
    bool first=true;
    auto key=[&](const char *name,int rc) { if(!first)f<<',';first=false;f<<'"'<<name<<"\":{\"rc\":"<<static_cast<unsigned>(rc); };
    for (auto name : {"Width","Height","PayloadSize","DeviceTimestamp","DeviceTimestampIncrement","TimestampTickFrequency","GevTimestampTickFrequency","AcquisitionBurstFrameCount","AcquisitionFrameCount","LineDebouncerTime","LineStatusAll","CounterValue","CounterCurrentValue","CounterValueAtReset","EventExposureStartTimestamp","EventFrameStartTimestamp","EventLine0RisingEdgeTimestamp","EventFrameStartOverTriggerTimestamp"}) {
      MVCC_INTVALUE_EX v{};int rc=MV_CC_GetIntValueEx(c.handle,name,&v);key(name,rc);
      if(!rc)f<<",\"value\":"<<v.nCurValue<<",\"min\":"<<v.nMin<<",\"max\":"<<v.nMax<<",\"inc\":"<<v.nInc;f<<'}';
    }
    for (auto name : {"AcquisitionMode","TriggerSelector","TriggerMode","TriggerSource","TriggerActivation","ExposureMode","ExposureAuto","LineSelector","LineMode","EventSelector","EventNotification","CounterSelector","CounterEventSource","CounterResetSource","FrameSpecInfoSelector","ChunkSelector","UserSetSelector","UserSetDefault"}) {
      MVCC_ENUMVALUE v{};int rc=MV_CC_GetEnumValue(c.handle,name,&v);key(name,rc);
      if(!rc){f<<",\"value\":"<<v.nCurValue<<",\"supported\":[";for(unsigned j=0;j<v.nSupportedNum;j++){if(j)f<<',';f<<v.nSupportValue[j];}f<<']';}f<<'}';
    }
    for (auto name : {"AcquisitionFrameRateEnable","TriggerCacheEnable","LineInverter","LineStatus","FrameSpecInfo","ChunkEnable","ChunkModeActive"}) {
      bool v=false;int rc=MV_CC_GetBoolValue(c.handle,name,&v);key(name,rc);if(!rc)f<<",\"value\":"<<(v?"true":"false");f<<'}';
    }
    for(auto name : {"AcquisitionFrameRate","ResultingFrameRate","ExposureTime","Gain","TriggerDelay"}) {
      MVCC_FLOATVALUE v{};int rc=MV_CC_GetFloatValue(c.handle,name,&v);key(name,rc);if(!rc)f<<",\"value\":"<<v.fCurValue;f<<'}';
    }
    f<<"},\"writes\":\"DeviceTimestampLatch command only; no acquisition, timestamp reset/increment, trigger or UserSet writes\"}";f.close();
    std::ofstream times(out/"clock_latch.csv");times<<"host_before_ns,monotonic_before_ns,latch_rc,read_rc,device_timestamp_value,host_after_ns,monotonic_after_ns\n";
    for(int i=0;i<20;i++) {
      auto before=host_ns(),mono=monotonic_ns();int latch=MV_CC_SetCommandValue(c.handle,"DeviceTimestampLatch");
      MVCC_INTVALUE_EX v{};int rc=MV_CC_GetIntValueEx(c.handle,"DeviceTimestamp",&v);
      times<<before<<','<<mono<<','<<static_cast<unsigned>(latch)<<','<<static_cast<unsigned>(rc)<<','<<v.nCurValue<<','<<host_ns()<<','<<monotonic_ns()<<'\n';
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    c.close();std::ofstream cleanup(out/"cleanup.json");cleanup<<"{";first=true;
    for(auto &entry:c.cleanup){if(!first)cleanup<<',';first=false;cleanup<<'"'<<entry.first<<"\":"<<entry.second;}cleanup<<"}";
    for(auto &entry:c.cleanup)require_ok(entry.second,entry.first);
    return 0;
  } catch(const std::exception &e) { std::cerr<<e.what()<<'\n'; return 1; }
}
