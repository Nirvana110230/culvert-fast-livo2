#include "IMU_Processing.h"
#include "lidar_image_windows.hpp"
#include <map>
#include <set>
#include <stdexcept>

namespace {
void require(bool ok, const char *why) { if (!ok) throw std::runtime_error(why); }
sensor_msgs::msg::Imu::SharedPtr imuAt(int ns) {
  auto p=std::make_shared<sensor_msgs::msg::Imu>(); p->header.stamp.sec=100;
  p->header.stamp.nanosec=ns; p->linear_acceleration.z=G_m_s2;
  return p;
}

void watermarkChecks()
{
  std::deque<PointCloudXYZI::Ptr> clouds;
  std::deque<double> headers;
  auto deliver = [&](double header, float first, float last) {
    auto cloud = std::make_shared<PointCloudXYZI>();
    for (float offset : {first, last}) {
      PointType p{}; p.curvature = offset;
      cloud->push_back(p);
    }
    clouds.push_back(cloud);
    headers.push_back(header);
  };
  const double end = 100.03125;
  deliver(100, 0.0f, 125.0f);
  require(!lidar_image_windows::covered(clouds, headers, end, false), "first scan tail is not a watermark");
  deliver(100.015625, 0.0f, 1.0f);
  require(!lidar_image_windows::covered(clouds, headers, end, false), "overlapping scan also precedes cut");
  deliver(end, 0.0f, 0.5f);
  require(lidar_image_windows::covered(clouds, headers, end, false), "equal next header certifies half-open interval");
  PointCloudXYZI out;
  auto nop = [](const void *, double, float, float) {};
  lidar_image_windows::consume(clouds, headers, out, 100, end, nop);
  require(out.size() == 3 && out.points[1].curvature == 15.625f, "late-arriving overlap not assigned");
  require(!lidar_image_windows::covered(clouds, headers, 100.0625, false), "online cannot infer EOF");
  require(lidar_image_windows::covered(clouds, headers, 100.0625, true), "known EOF uses longest remaining scan");
  require(!lidar_image_windows::covered(clouds, headers, 100.25, true), "EOF cannot invent coverage");
  bool rejected = false;
  out.points[0].curvature = -1;
  try { lidar_image_windows::validateOffsets(out.points); } catch (const std::runtime_error &) { rejected = true; }
  require(rejected, "negative offsets invalidate header watermark");
}
}

int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  watermarkChecks();
  ImuProcess imu;
  imu.set_imu_init_frame_num(1);
  StatesGroup state; state.vel_end=V3D::UnitX();
  LidarMeasureGroup group; group.lio_vio_flg=LIO;
  group.measures.emplace_back(); auto &slice=group.measures.back();
  slice.lio_time=100; slice.imu.push_back(imuAt(0));
  auto output=std::make_shared<PointCloudXYZI>(); imu.Process2(group,state,output);

  std::deque<PointCloudXYZI::Ptr> clouds;
  std::deque<double> headers;
  std::map<int,double> source_time;
  int id=0;
  auto addScan=[&](double header, const std::vector<float>& offsets) {
    auto c=std::make_shared<PointCloudXYZI>();
    for (float offset:offsets) {
      PointType p{}; p.x=2; p.z=1; p.intensity=++id; p.curvature=offset;
      c->push_back(p); source_time[id]=header+offset*0.001;
    }
    clouds.push_back(c); headers.push_back(header);
  };
  // One scan spans three image boundaries, two scans overlap and two IDs
  // share an exact acquisition stamp. The callback sorts each scan.
  addScan(100,{0,4,30,31.25f,40,62.5f,72,93.75f,99});
  addScan(100.0625,{0,8,20,31.25f,48});
  int legacy_future=0;
  for (const auto &p:clouds.front()->points)
    if (p.curvature>=31.25f && p.curvature>62.501f) ++legacy_future;
  require(legacy_future==3,"fixture cannot expose premature carry consumption");
  std::set<int> seen;
  for (int window=0; window<5; ++window) {
    double start=100+window/32.0, end=100+(window+1)/32.0;
    lidar_image_windows::consume(clouds,headers,*group.pcl_proc_cur,start,end,
                                [](const void*,double,float,float){});
    slice.imu.clear(); slice.lio_time=end;
    for (int k=1;k<=5;++k) slice.imu.push_back(imuAt(window*31250000+k*6250000));
    imu.Process2(group,state,output);
    require(output->size()==group.pcl_proc_cur->size(),"deskew point count");
    require(std::abs(state.pos_end.x()-(end-100))<1e-6,"propagation endpoint");
    for (std::size_t i=0;i<output->size();++i) {
      const auto &p=output->points[i]; const int key=static_cast<int>(p.intensity);
      const double t=source_time.at(key);
      require(t>=start && t<end,"incorrect half-open assignment");
      require(!i || output->points[i-1].curvature<=p.curvature,"backward deskew requires sorted times");
      require(std::abs(start+p.curvature*0.001-t)<1e-7,"incorrect rebasing");
      require(seen.insert(key).second,"duplicate identity");
      require(std::abs(p.x-(2.0+t-end))<1e-5,"deskew at known 1 m/s, wrong transform or interval");
    }
  }
  require(clouds.empty() && headers.empty() && seen.size()==source_time.size(),"tail or identity loss");
  std::cout<<"PASS stream watermark, late-arriving overlap, known EOF, three-cut scan, exact cutoff, empty window, unique identities and production ImuProcess deskew\n";
  rclcpp::shutdown();
}
