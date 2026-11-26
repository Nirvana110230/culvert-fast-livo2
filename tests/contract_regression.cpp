#include "LIVMapper.h"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>

namespace {
void require(bool value, const char* reason)
{
  if (!value) throw std::runtime_error(reason);
}

auto imuAt(double stamp)
{
  auto imu = std::make_shared<sensor_msgs::msg::Imu>();
  imu->header.stamp = sec2Stamp(stamp);
  imu->linear_acceleration.z = 9.81;
  return imu;
}

void checkSync(LIVMapper& mapper)
{
  LidarMeasureGroup group;
  auto sync = [&] { return mapper.sync_packages(group); };
  require(mapper.slam_mode_ == LIVO && !sync(), "empty LIVO startup");
  cv::Mat image(4, 4, CV_8UC1, cv::Scalar(42));
  mapper.img_buffer.push_back(image);
  mapper.img_time_buffer.push_back(10.125);
  mapper.imu_buffer.push_back(imuAt(10.0625));
  require(!sync() && group.lio_vio_flg == WAIT, "startup requires LiDAR");
  PointCloudXYZI::Ptr cloud(new PointCloudXYZI);
  PointType point{};
  cloud->push_back(point);
  point.curvature = 100;
  cloud->push_back(point);
  mapper.lid_raw_data_buffer.push_back(cloud);
  mapper.lid_header_time_buffer.push_back(10.0);
  mapper.imu_buffer.push_back(imuAt(10.125));
  require(!sync(), "camera time exceeds LiDAR coverage");
  cloud->points.back().curvature = 125;
  mapper.imu_buffer.pop_back();
  require(!sync(), "camera time exceeds IMU coverage");
  mapper.imu_buffer.clear();
  require(!sync(), "startup requires IMU");
  mapper.imu_buffer.push_back(imuAt(10.125));
  require(sync() && group.lio_vio_flg == LIO, "complete coverage must produce LIO");
  const double lio_time = group.measures.back().lio_time;
  require(lio_time == 10.125 && group.measures.back().imu.size() == 1 &&
          mapper.lid_raw_data_buffer.empty() && mapper.imu_buffer.empty(), "LIO cut at EOF");
  // LIO has finished processing this group. No future sensor messages will arrive.
  group.last_lio_update_time = lio_time;
  mapper.img_buffer.clear();
  require(!sync() && group.lio_vio_flg == LIO, "pending VIO requires image");
  mapper.img_buffer.push_back(image);
  require(sync() && group.lio_vio_flg == VIO && group.measures.size() == 1, "paired VIO at EOF");
  const auto& vio = group.measures.back();
  require(vio.vio_time == lio_time && vio.lio_time == lio_time && vio.img.data == image.data &&
          vio.imu.empty() && mapper.img_buffer.empty() && mapper.img_time_buffer.empty(), "paired VIO payload");
  require(!sync(), "no duplicate VIO at EOF");
  mapper.img_buffer.push_back(image);
  mapper.img_time_buffer.push_back(10.25);
  require(!sync() && group.lio_vio_flg == VIO && mapper.img_buffer.size() == 1,
          "new pair still requires new measurements after VIO");
  mapper.slam_mode_ = ONLY_LIO;
  mapper.img_en = 0;
  require(!sync(), "ONLY_LIO must still wait");
  mapper.lid_raw_data_buffer.push_back(cloud);
  mapper.lid_header_time_buffer.push_back(11);
  require(!sync(), "ONLY_LIO requires IMU");
  mapper.imu_buffer.push_back(imuAt(11.0625));
  mapper.last_timestamp_imu = 11.0625;
  require(!sync(), "ONLY_LIO requires IMU up to end of scan");
  mapper.imu_buffer.push_back(imuAt(11.125));
  mapper.last_timestamp_imu = 11.125;
  require(sync() && group.lio_vio_flg == LIO && group.measures.back().lio_time == 11.125,
          "complete ONLY_LIO package");
  mapper.slam_mode_ = ONLY_LO;
  mapper.imu_en = false;
  require(!sync(), "ONLY_LO must still wait");
  mapper.lid_raw_data_buffer.push_back(cloud);
  mapper.lid_header_time_buffer.push_back(12);
  require(sync() && group.lio_vio_flg == LO && group.measures.back().lio_time == 12.125,
          "complete ONLY_LO package without IMU");
  mapper.slam_mode_ = LIVO;
  mapper.img_en = 1;
  mapper.imu_en = true;
  group.lio_vio_flg = LIO;
  rclcpp::shutdown();
  require(!sync() && group.lio_vio_flg == LIO && mapper.img_buffer.size() == 1, "shutdown consumed an image");
}

void checkLio(LIVMapper& mapper)
{
  auto& lio = *mapper.voxelmap_manager;
  for (int x = 0; x < 4; ++x) {
    for (int y = 0; y < 4; ++y) {
      PointType point{};
      point.x = 3.12f + x * 0.03f;
      point.y = 1.12f + y * 0.04f;
      point.z = 2.12f;
      lio.feats_down_body_->push_back(point);
      const V3D world = lio.extR_ * V3D(point.x, point.y, point.z) + lio.extT_;
      point.x = world.x(); point.y = world.y(); point.z = world.z();
      lio.feats_down_world_->push_back(point);
    }
  }
  lio.feats_down_size_ = static_cast<int>(lio.feats_down_body_->size());
  lio.BuildVoxelMap();
  StatesGroup propagated = lio.state_;
  lio.StateEstimation(propagated);
  require(lio.effct_feat_num_ > 0 && lio.state_.cov.allFinite(), "synthetic plane did not match");
}

StatesGroup checkImu(LIVMapper& mapper)
{
  LidarMeasureGroup group;
  group.lio_vio_flg = LIO;
  MeasureGroup init;
  init.lio_time = 10;
  for (int i = 0; i < mapper.imu_int_frame + 1; ++i)
    init.imu.push_back(imuAt(10.0 - (mapper.imu_int_frame - i) * 0.001));
  group.measures.push_back(init);
  StatesGroup state;
  PointCloudXYZI::Ptr output(new PointCloudXYZI);
  mapper.p_imu->Process2(group, state, output);
  require(!mapper.p_imu->imu_need_init, "IMU initialization");
  state.cov.setZero(); // One real propagation from P=0 yields exactly Q.
  MeasureGroup next;
  next.lio_time = 10.125;
  next.imu.push_back(imuAt(next.lio_time));
  group.measures.clear();
  group.measures.push_back(next);
  mapper.p_imu->Process2(group, state, output);
  require(state.cov.allFinite(), "non-finite propagated covariance");
  return state;
}

double checkVio(LIVMapper& mapper)
{
  auto& vio = *mapper.vio_manager;
  const V3D world = vio.Rci.transpose() * (V3D(0, 0, 3) - vio.Pci);
  VisualPoint point(world);
  auto& sparse = *vio.visual_submap;
  sparse.voxel_points.push_back(&point);
  sparse.search_levels.push_back(0);
  sparse.warp_patch.emplace_back(vio.patch_size_total, 64.0f);
  sparse.errors.push_back(0);
  sparse.inv_expo_list.push_back(1.0);
  vio.total_points = 1;
  vio.compute_jacobian_time = vio.update_ekf_time = 0;
  cv::Mat image(vio.height, vio.width, CV_8UC1, cv::Scalar(128));
  vio.updateState(image, 0);
  require(vio.visual_update_steps > 0 && vio.H_T_H.allFinite(), "no accepted photometric update");
  sparse.reset();
  return vio.H_T_H(6, 6);
}

void jsonVector(std::ostream& out, const Eigen::Vector3d& vector)
{
  out << '[' << vector[0] << ',' << vector[1] << ',' << vector[2] << ']';
}
}  // namespace

int main(int argc, char **argv)
{
  try {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("contract_fixture");
    LIVMapper mapper(node, "laserMapping"); // No subscribers, no driver, no hardware access.
    checkLio(mapper);
    const StatesGroup imu = checkImu(mapper);
    const double photo = checkVio(mapper);
    const auto limit = mapper.node->get_parameter("lio.max_iterations").as_int();
    const bool exposure = mapper.node->get_parameter("vio.exposure_estimate_en").as_bool();
    const double gyr = mapper.node->get_parameter("imu.b_gyr_cov").as_double();
    const double acc = mapper.node->get_parameter("imu.b_acc_cov").as_double();
    checkSync(mapper); // Includes orderly shutdown with a pending VIO.
    std::cout << std::setprecision(17) << std::boolalpha
              << "CONTRACT {\"limit_readback\":" << limit
              << ",\"lio_root_size\":" << mapper.voxelmap_manager->config_setting_.max_voxel_size_
              << ",\"vio_root_size\":" << mapper.vio_manager->lidar_root_size
              << ",\"limit_consumed\":" << mapper.voxelmap_manager->config_setting_.max_iterations_
              << ",\"iterations_executed\":" << mapper.voxelmap_manager->last_iterations_
              << ",\"plane_matches\":" << mapper.voxelmap_manager->effct_feat_num_
              << ",\"exposure_readback\":" << exposure << ",\"gyr_bias_readback\":" << gyr
              << ",\"acc_bias_readback\":" << acc << ",\"photo_HtH_66\":" << photo
              << ",\"imu_dt\":0.125,\"imu_Q_66\":" << imu.cov(6, 6) << ",\"imu_Q_gyr_bias\":";
    jsonVector(std::cout, imu.cov.diagonal().segment<3>(10));
    std::cout << ",\"imu_Q_acc_bias\":";
    jsonVector(std::cout, imu.cov.diagonal().segment<3>(13));
    std::cout << ",\"sync_and_stop\":true}" << std::endl;
  } catch (const std::exception& error) {
    std::cerr << error.what() << std::endl;
    if (rclcpp::ok()) rclcpp::shutdown();
    return 2;
  }
  return 0;
}
