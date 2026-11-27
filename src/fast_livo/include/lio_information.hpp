#pragma once
#include <Eigen/Core>
#include <array>

namespace lio_information {
using Vec3 = Eigen::Vector3d;
using Mat6 = Eigen::Matrix<double, 6, 6>;

// Only sampled updates populate an accumulator, overwritten for each IEKF
// iteration. The final accepted linearization is the one written to disk.
struct MeasurementTerms {
  unsigned count = 0;
  double weight = 0, plane_variance = 0, point_variance = 0, inverse_weight = 0;
  double vertical_information = 0, yaw_information = 0, arm_squared = 0;
};

class Sample {
public:
  static bool due(double stamp);
  void frame(const Eigen::Matrix3d &rotation, const Vec3 &gravity, const Vec3 &lidar_origin);
  void add(const Vec3 &world_point, const Vec3 &normal, const Vec3 &rotation_jacobian_body,
           double arm_squared, double plane_variance, double point_variance, double weight);
  void write(double stamp, int iterations, const Mat6 &normal_matrix) const;

private:
  Eigen::Matrix3d rotation_;
  Vec3 up_, lidar_origin_, up_body_;
  std::array<MeasurementTerms,5> terms_{};
};
}
