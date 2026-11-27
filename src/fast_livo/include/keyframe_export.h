#pragma once

#include "common_lib.h"
#include <memory>
#include <vikit/abstract_camera.h>

// Measurement-time snapshots from the estimator. No ROS publisher participates.
class KeyframeExport {
public:
  KeyframeExport(const std::string& directory, const std::string& calibration);
  ~KeyframeExport();
  void recordState(int64_t stamp, const char* stage, const StatesGroup& state);
  void lidar(int64_t stamp, const StatesGroup& state, const PointCloudXYZI& deskewed,
             const M3D& imu_from_lidar, const V3D& imu_from_lidar_t,
             size_t downsampled, size_t matches, double residual, bool expect_image);
  void image(int64_t stamp, const cv::Mat& bgr, const vk::AbstractCamera& camera,
             const M3D& camera_from_imu, const V3D& camera_from_imu_t, double blind);
  void finish();
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
