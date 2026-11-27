#include "LIVMapper.h"
#include <limits>
#include <stdexcept>
#include <vector>

namespace {
struct Sample {
  float x, y, z;
  bool keep;
};

void checkPoints(Preprocess& pre, const std::vector<Sample>& samples)
{
  auto scan = std::make_shared<livox_ros_driver2::msg::CustomMsg>();
  std::vector<int> expected, actual;
  for (size_t i = 0; i < samples.size(); ++i) {
    const auto& sample = samples[i];
    livox_ros_driver2::msg::CustomPoint point;
    point.line = 0;
    point.tag = 0x10;
    point.x = sample.x;
    point.y = sample.y;
    point.z = sample.z;
    point.reflectivity = i + 1;
    // Supply a full decimation group for each distance, including in the formal ROS config.
    for (int j = 0; j < pre.point_filter_num; ++j)
      scan->points.push_back(point);
    if (sample.keep) expected.push_back(point.reflectivity);
  }
  scan->point_num = scan->points.size();
  PointCloudXYZI::Ptr out(new PointCloudXYZI);
  pre.process(scan, out);
  for (const auto& point : out->points) actual.push_back(static_cast<int>(point.intensity));
  if (actual != expected) throw std::runtime_error("unexpected Livox point selection");
}
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try {
    Preprocess pre;
    checkPoints(pre, {{0.0099f,0,0,false}, {0.0101f,0,0,true}});
    pre.set(false, AVIA, 0.25, 1);
    checkPoints(pre, {{0,0.249f,0,false}, {0,0.25f,0,true}, {0,0.251f,0,true}});
    for (double invalid : {-1.0, std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::infinity(), std::numeric_limits<double>::max()}) {
      bool rejected = false;
      try { pre.setBlindDistance(invalid); }
      catch (const std::invalid_argument&) { rejected = true; }
      if (!rejected) throw std::runtime_error("invalid distance accepted");
    }
    checkPoints(pre, {{0.249f,0,0,false}, {0.25f,0,0,true}});
    pre.setBlindDistance(0);
    checkPoints(pre, {{0,0,0,true}});
    pre.set(true, AVIA, 0.5, 1);
    std::vector<Sample> near_line;
    for (int i = 0; i < 12; ++i) near_line.push_back({0.33f + 0.01f*i,0,0,false});
    checkPoints(pre, near_line);

    auto node = std::make_shared<rclcpp::Node>("blind_regression");
    LIVMapper mapper(node, "laserMapping");
    if (mapper.node->get_parameter("preprocess.blind").as_double() != 0.5 ||
        mapper.p_pre->point_filter_num != 3 || mapper.p_pre->feature_enabled)
      throw std::runtime_error("expected the formal Mid-360S configuration");
    checkPoints(*mapper.p_pre, {{0.499f,0,0,false}, {0,0,-0.5f,true}, {0,0.501f,0,true},
                                {0.30f,0,0.30f,false}, {0.375f,0,0.375f,true}});
    std::cout << "PASS constructor, set, invalid/zero, all-blind feature line and formal ROS Mid-360S boundary\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
