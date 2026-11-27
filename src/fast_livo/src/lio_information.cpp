#include "lio_information.hpp"
#include "replay_timing_audit.hpp"
#include <Eigen/Eigenvalues>
#include <cmath>
#include <limits>

namespace lio_information {
bool Sample::due(double stamp)
{
  static double last = -std::numeric_limits<double>::infinity();
  if (!replay_timing::directory() || stamp < 0 || stamp-last < 1.0) return false;
  last = stamp;
  return true;
}

void Sample::frame(const Eigen::Matrix3d &rotation, const Vec3 &gravity, const Vec3 &lidar_origin)
{
  rotation_ = rotation;
  up_ = -gravity;
  if (up_.norm() > 1e-6) up_.normalize();
  else up_.setZero();
  up_body_ = rotation_.transpose() * up_;
  lidar_origin_ = lidar_origin;
}

void Sample::add(const Vec3 &world_point, const Vec3 &normal, const Vec3 &rotation_jacobian_body,
                 double arm_squared, double plane_variance, double point_variance, double weight)
{
  const double norm = normal.norm();
  int category = 4;
  if (up_.squaredNorm() > 0 && norm > 1e-6) {
    const double cosine = std::abs(normal.dot(up_)) / norm;
    const double height = (world_point-lidar_origin_).dot(up_);
    if (cosine >= 0.9396926207859084 && height < -0.20) category=1;
    else if (cosine >= 0.9396926207859084 && height > 0.20) category=2;
    else if (cosine <= 0.3420201433256687) category=3;
  }
  const double vertical = normal.dot(up_);
  const double yaw = rotation_jacobian_body.dot(up_body_);
  for (const int i : {0,category}) {
    auto &t=terms_[i];
    ++t.count;
    t.weight += weight;
    t.plane_variance += plane_variance;
    t.point_variance += point_variance;
    t.inverse_weight += 1.0/weight;
    t.arm_squared += arm_squared;
    t.vertical_information += weight * vertical * vertical;
    t.yaw_information += weight * yaw * yaw;
  }
}

void Sample::write(double stamp, int iterations, const Mat6 &normal_matrix) const
{
  static auto source = replay_timing::csv("measurement_weight_sources.csv",
    "cut_s,iterations,class,count,weight_sum,sigma_plane_sum_m2,sigma_point_sum_m2,inverse_weight_sum_m2,fixed_variance_sum_m2,arm_squared_sum_m2,vertical_translation_information,yaw_rotation_information");
  const char *names[]={"all","horizontal_below","horizontal_above","vertical","other"};
  for (std::size_t i=0; i<terms_.size(); ++i) {
    const auto &t=terms_[i];
    source << stamp << ',' << iterations << ',' << names[i] << ',' << t.count << ',' << t.weight << ',' << t.plane_variance
           << ',' << t.point_variance << ',' << t.inverse_weight << ',' << t.count*0.001 << ',' << t.arm_squared
           << ',' << t.vertical_information << ',' << t.yaw_information << '\n';
  }
  source.flush();
  static auto geometry=replay_timing::csv("lio_information_6d.csv",
    "cut_s,iterations,points,valid,rms_arm_m,weight_sum,up_x,up_y,up_z,world_tup_information,world_rup_scaled_information,weak_translation_up,weak_scaled_rotation_up,lambda0,lambda1,lambda2,lambda3,lambda4,lambda5,weak_rx,weak_ry,weak_rz,weak_tx,weak_ty,weak_tz,h00,h01,h02,h03,h04,h05,h11,h12,h13,h14,h15,h22,h23,h24,h25,h33,h34,h35,h44,h45,h55");
  const auto &all=terms_[0];
  const double arm=all.count ? std::sqrt(all.arm_squared/all.count) : 0.0;
  Mat6 info = Mat6::Zero();
  bool valid=arm>1e-8 && all.weight>0 && up_.squaredNorm()>0 && normal_matrix.allFinite();
  Eigen::Matrix<double,6,1> values=Eigen::Matrix<double,6,1>::Zero(), weak=values;
  if (valid) {
    // Native Jacobian takes rotation in body axes and translation in world.
    // Express both in world axes with eta=[L*dtheta_world, delta_x_world].
    Mat6 basis=Mat6::Identity();
    basis.topLeftCorner<3,3>()=rotation_.transpose()/arm;
    info=basis.transpose()*normal_matrix*basis;
    info=(0.5*(info+info.transpose())).eval();
    Eigen::SelfAdjointEigenSolver<Mat6> eig(info);
    valid=eig.info()==Eigen::Success;
    if (valid) {
      values=eig.eigenvalues();
      weak=eig.eigenvectors().col(0);
      Eigen::Index pivot;
      weak.cwiseAbs().maxCoeff(&pivot);
      if (weak[pivot]<0) weak=-weak;
    }
  }
  geometry<<stamp<<','<<iterations<<','<<all.count<<','<<valid<<','<<arm<<','<<all.weight;
  for (int i=0;i<3;++i) geometry<<','<<up_[i];
  geometry<<','<<up_.dot(info.bottomRightCorner<3,3>()*up_)<<','<<up_.dot(info.topLeftCorner<3,3>()*up_)
          <<','<<weak.tail<3>().dot(up_)<<','<<weak.head<3>().dot(up_);
  for (int i=0;i<6;++i) geometry<<','<<values[i];
  for (int i=0;i<6;++i) geometry<<','<<weak[i];
  for (int i=0;i<6;++i) for (int j=i;j<6;++j) geometry<<','<<info(i,j);
  geometry<<std::endl;
}
}
