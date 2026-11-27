#include "LIVMapper.h"
#include "session_gravity.h"
#include "bag_input.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto nh = std::make_shared<rclcpp::Node>("fastlivo_transport");
  image_transport::ImageTransport it_(nh);
  try {
    LIVMapper mapper(nh, "laserMapping");
    mapper.initializeSubscribersAndPublishers(nh, it_);
    if (const char* path = std::getenv("FAST_LIVO2_OFFLINE_BAG")) {
      BagInput input(mapper, path);
      mapper.run(nh, [&] { return input.next(); });
      input.verify();
    } else {
      mapper.run(nh);
    }
  } catch (const std::exception& error) {
    RCLCPP_ERROR(nh->get_logger(), "%s", error.what());
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::shutdown();
  return 0;
}
