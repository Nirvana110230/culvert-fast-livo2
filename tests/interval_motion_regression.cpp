#include "IMU_Processing.h"
#include <array>
#include <iostream>

namespace {
const std::array<double, 4> rates{0., 1., 3., 7.};

sensor_msgs::msg::Imu::SharedPtr measurement(int k) {
  auto msg = std::make_shared<sensor_msgs::msg::Imu>();
  msg->header.stamp.sec = 100;
  msg->header.stamp.nanosec = k * 10000000;
  msg->linear_acceleration.z = G_m_s2 + rates[k];
  msg->angular_velocity.z = rates[k];
  return msg;
}

// The fixture uses the forward model's piecewise constant interval averages.
// Axial acceleration and yaw commute, so position and rotation have closed forms.
std::pair<double,double> yawHeight(double seconds) {
  double yaw = 0, z = 0, vz = 0;
  for (int k = 1; k < 4; ++k) {
    const double dt = std::clamp(seconds - (k-1)*.01, 0., .01);
    const double rate = .5 * (rates[k-1] + rates[k]);
    yaw += rate*dt;
    z += vz*dt + .5*rate*dt*dt;
    vz += rate*dt;
  }
  return {yaw,z};
}

double error(bool corrected) {
  ImuProcess imu;
  imu.set_interval_deskew(corrected);
  imu.set_imu_init_frame_num(1);
  StatesGroup state;
  LidarMeasureGroup group;
  group.lio_vio_flg = LIO;
  group.measures.emplace_back();
  auto& slice = group.measures.back();
  auto out = std::make_shared<PointCloudXYZI>();
  slice.lio_time = 100;
  slice.imu.push_back(measurement(0));
  imu.Process2(group,state,out);
  slice.imu.clear();
  for (int i=1; i<4; ++i) slice.imu.push_back(measurement(i));
  slice.lio_time = 100.03;
  for (float ms : {0.f, 4.f, 10.f, 16.f, 20.f, 25.f, 30.f}) {
    PointType pt{};
    pt.x = 5;
    pt.curvature = ms;
    group.pcl_proc_cur->push_back(pt);
  }
  imu.Process2(group,state,out);
  if (out->size() != group.pcl_proc_cur->size()) throw std::runtime_error("missing output");
  const auto [yaw_end,z_end] = yawHeight(.03);
  double largest = 0;
  for (const auto& pt : *out) {
    const auto [yaw,z] = yawHeight(pt.curvature * .001);
    V3D expected(5*std::cos(yaw-yaw_end),5*std::sin(yaw-yaw_end),z-z_end);
    largest = std::max(largest,(expected - V3D(pt.x,pt.y,pt.z)).norm());
  }
  return largest;
}
}

int main(int argc, char** argv) {
  rclcpp::init(argc,argv);
  const double previous = error(false), interval = error(true);
  std::cout << "point_max_error_m legacy=" << previous << " interval=" << interval << std::endl;
  rclcpp::shutdown();
  return previous > .01 && interval < 2e-6 ? 0 : 1;
}
