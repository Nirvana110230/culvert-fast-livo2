#include "LIVMapper.h"
#include <stdexcept>

static void check(bool condition, const char* message)
{
  if (!condition) throw std::runtime_error(message);
}

static auto imu(double time)
{
  auto m = std::make_shared<sensor_msgs::msg::Imu>();
  m->header.stamp = sec2Stamp(time);
  m->linear_acceleration.z = 9.81;
  return m;
}

static auto lidar()
{
  auto scan = std::make_shared<livox_ros_driver2::msg::CustomMsg>();
  scan->header.stamp = sec2Stamp(10.0);
  for (unsigned i = 0; i < 300; ++i) {
    livox_ros_driver2::msg::CustomPoint pt;
    pt.x = 2.0 + i * 0.01;
    pt.z = 1.0;
    pt.tag = 0x10;
    pt.line = 0;
    pt.offset_time = i * 500000U;
    scan->points.push_back(pt);
  }
  scan->point_num = scan->points.size();
  return scan;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<rclcpp::Node>("b0_fixture");
    for (int before : {0, 2, 4}) {
      LIVMapper mapper(node, "laserMapping");
      mapper.imu_prop_enable = false;
      mapper.imu_time_offset = 0.03125;
      const double input[] = {10.015625, 10.0625, 10.09375, 10.15625};
      for (int i = 0; i < before; ++i) mapper.imu_cbk(imu(input[i]));
      LidarMeasureGroup g;
      check(!mapper.sync_packages(g), "must wait for first LiDAR");
      mapper.livox_pcl_cbk(lidar());
      for (int i = before; i < 4; ++i) mapper.imu_cbk(imu(input[i]));
      mapper.img_buffer.emplace_back(8,8,CV_8UC1);
      mapper.img_time_buffer.push_back(10.125);
      check(mapper.sync_packages(g), "first complete sensor cut missing");
      const auto& group = g.measures.back();
      check(group.imu.size() == 3, "callback order lost eligible IMU samples");
      for (int i = 0; i < 3; ++i)
        check(stamp2Sec(group.imu[i]->header.stamp) == input[i+1]-0.03125,
              "wrong IMU timestamp or order");
    }
    LIVMapper legacy(node, "laserMapping");
    legacy.ros_driver_fix_en = true;
    legacy.imu_cbk(imu(10.125));
    check(legacy.imu_buffer.empty(), "legacy rounding requires first LiDAR");
    {
      LIVMapper bounded(node, "laserMapping");
      bounded.imu_time_offset = 0.0;
      for (int i=1; i<=10001; ++i) {
        auto sample = imu(10.0+i/131072.0);
        sample->linear_acceleration.x = i;
        bounded.imu_cbk(sample);
      }
      bounded.livox_pcl_cbk(lidar());
      check(bounded.imu_buffer.size()==10000, "startup capacity");
      check(bounded.imu_buffer.front()->linear_acceleration.x == 2 &&
            bounded.imu_buffer.back()->linear_acceleration.x == 10001,
            "startup overflow must drop oldest");
    }
    std::cout << "PASS startup order, stale prefix, time offset, coverage, legacy gate, bounded overflow\n";
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "FAIL: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 2;
  }
}
