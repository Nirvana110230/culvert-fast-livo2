#pragma once

#include "LIVMapper.h"
#include <rosbag2_cpp/reader.hpp>
#include <map>

// Deliver the recorded CDR messages to the production callbacks in bag order.
// The caller drains every ready estimator update before asking for more input.
class BagInput {
public:
  BagInput(LIVMapper& mapper, const char* path) : mapper_(mapper) {
    reader_.open(path);
    rosbag2_storage::StorageFilter filter;
    filter.topics = {mapper.lid_topic, mapper.imu_topic};
    if (mapper.img_en) filter.topics.push_back(mapper.img_topic);
    const std::map<std::string, std::string> types = {
      {mapper.lid_topic, "livox_ros_driver2/msg/CustomMsg"},
      {mapper.imu_topic, "sensor_msgs/msg/Imu"}, {mapper.img_topic, "sensor_msgs/msg/Image"}};
    for (const auto& name : filter.topics) {
      for (const auto& topic : reader_.get_metadata().topics_with_message_count) {
        if (topic.topic_metadata.name == name && topic.topic_metadata.type == types.at(name))
          expected_[name] = topic.message_count;
      }
      if (!expected_.count(name) || expected_.at(name) == 0)
        throw std::runtime_error("Missing or incompatible bag topic " + name);
    }
    reader_.set_filter(filter);
  }

  bool next() {
    if (!reader_.has_next()) return false;
    const auto message = reader_.read_next();
    const auto& name = message->topic_name;
    if (name == mapper_.imu_topic) mapper_.imu_cbk(decode<sensor_msgs::msg::Imu>(*message));
    else if (name == mapper_.lid_topic)
      mapper_.livox_pcl_cbk(decode<livox_ros_driver2::msg::CustomMsg>(*message));
    else if (name == mapper_.img_topic) mapper_.img_cbk(decode<sensor_msgs::msg::Image>(*message));
    else throw std::runtime_error("Unexpected unfiltered bag topic " + name);
    ++counts_[name];
    return true;
  }

  void verify() const {
    if (counts_ != expected_) throw std::runtime_error("Bag did not deliver the expected topic counts");
  }

private:
  template<class T> static std::shared_ptr<T> decode(const rosbag2_storage::SerializedBagMessage& record) {
    auto output = std::make_shared<T>();
    rclcpp::SerializedMessage serialized(*record.serialized_data);
    rclcpp::Serialization<T>().deserialize_message(&serialized, output.get());
    return output;
  }
  LIVMapper& mapper_;
  rosbag2_cpp::Reader reader_;
  std::map<std::string, size_t> expected_, counts_;
};
