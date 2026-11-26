// Replay fixture for the exact deployed LIVMapper image callback.
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <cv_bridge/cv_bridge.h>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <iomanip>
#include <mutex>

inline double stamp2Sec(const builtin_interfaces::msg::Time& stamp) {
  return rclcpp::Time(stamp).seconds();
}

class LIVMapper {
 public:
  rclcpp::Node::SharedPtr node;
  bool img_en = true;
  bool hilti_en = false;
  double img_time_offset;
  double last_timestamp_lidar = -1.0;
  double last_timestamp_img = -1.0;
  std::mutex mtx_buffer;
  std::condition_variable sig_buffer;
  std::deque<cv::Mat> img_buffer;
  std::deque<double> img_time_buffer;
  cv::Mat getImageFromMsg(const sensor_msgs::msg::Image::ConstSharedPtr& image);
  void img_cbk(const sensor_msgs::msg::Image::ConstSharedPtr& image);
};

// Generated verbatim from the current deployed source, with its SHA recorded.
#include "deployed_image_callback.inc"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("official_time_callback_test");
  auto image_topic = node->declare_parameter<std::string>("common.img_topic", "/hik_camera/image_timeshare");
  auto lidar_topic = node->declare_parameter<std::string>("common.lid_topic", "/livox/lidar");
  const auto output_path = node->declare_parameter<std::string>("audit_output", "");
  if (output_path.empty()) throw std::runtime_error("audit_output is required");
  LIVMapper mapper;
  mapper.node = node;
  mapper.img_time_offset = node->declare_parameter<double>("time_offset.img_time_offset", 0.0);
  std::ofstream output;
  output.exceptions(std::ios::failbit | std::ios::badbit);
  output.open(output_path);
  output << "header_ns,offset_s,last_lidar_s,accepted,buffered_time_s,single_addition_matches\n";
  output << std::setprecision(17);
  auto lidar = node->create_subscription<livox_ros_driver2::msg::CustomMsg>(
      lidar_topic, 32, [&](const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr message) {
        mapper.last_timestamp_lidar = stamp2Sec(message->header.stamp);
      });
  auto images = node->create_subscription<sensor_msgs::msg::Image>(
      image_topic, 8, [&](const sensor_msgs::msg::Image::ConstSharedPtr image) {
        mapper.img_cbk(image);
        const bool accepted = !mapper.img_time_buffer.empty();
        output << rclcpp::Time(image->header.stamp).nanoseconds() << ','
               << mapper.img_time_offset << ',' << mapper.last_timestamp_lidar << ',' << accepted << ',';
        if (accepted) {
          const double buffered = mapper.img_time_buffer.front();
          output << buffered << ',' << (buffered == stamp2Sec(image->header.stamp) + mapper.img_time_offset);
          mapper.img_time_buffer.clear();
          mapper.img_buffer.clear();
        } else {
          output << ",0";
        }
        output << '\n';
        output.flush();
      });
  std::ofstream(output_path + ".ready") << "subscriptions ready\n";
  rclcpp::spin(node);
  output.close();
  rclcpp::shutdown();
}
