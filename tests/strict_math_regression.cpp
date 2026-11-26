#include <iomanip>
#include <iostream>
#include <memory>
#include "vio.h"

int main()
{
  std::cout << std::setprecision(17);
  std::cout << "Eigen=" << EIGEN_WORLD_VERSION << '.' << EIGEN_MAJOR_VERSION << '.' << EIGEN_MINOR_VERSION
            << " sizeof_SE3=" << sizeof(Sophus::SE3<double>)
            << " alignof_SE3=" << alignof(Sophus::SE3<double>) << '\n';
#ifdef EIGEN_VECTORIZE_NEON
  std::cout << "EIGEN_VECTORIZE_NEON=1\n";
#endif
#ifdef __FAST_MATH__
  std::cout << "FAST_MATH=1\n";
#endif
  int failures = 0;
  const auto expect_small = [&failures](const char *label, double value, double tolerance) {
    std::cout << label << '=' << value << '\n';
    if (!(value <= tolerance)) {
      ++failures;
      std::cerr << "FAIL " << label << " exceeds " << tolerance << '\n';
    }
  };

  // Frozen calibration_v1, copied verbatim. IMU-to-LiDAR base rotation is identity.
  M3D rcl;
  rcl << -0.010020297203748349, -0.9998641757830284, 0.013085244731187556,
          0.3631629997011949, -0.015831249198199115, -0.9315911158855339,
          0.9316717390218398, -0.0045827431251422995, 0.3632723071931998;
  const V3D pcl(0.005266584459228521, -0.09118587453180663, 0.00029624756364787785);
  const V3D lidar_to_imu_translation(-0.011, -0.02329, 0.04412);
  const M3D rci = Eigen::Quaterniond(rcl).normalized().toRotationMatrix();
  const V3D pci = pcl - rcl * lidar_to_imu_translation;
  const Eigen::Quaterniond quaternion(rci);
  const Eigen::Quaterniond conjugated = quaternion.conjugate();
  const Eigen::Quaterniond inverted_quaternion = quaternion.inverse();
  const Eigen::Quaterniond scalar_conjugated(quaternion.w(), -quaternion.x(), -quaternion.y(), -quaternion.z());
  std::cout << "quaternion_xyzw=" << quaternion.coeffs().transpose()
            << " conjugated_xyzw=" << conjugated.coeffs().transpose()
            << " scalar_conjugated_xyzw=" << scalar_conjugated.coeffs().transpose() << '\n';
  expect_small("quaternion_conjugate_coeff_error", (conjugated.coeffs() - scalar_conjugated.coeffs()).norm(), 1e-12);
  expect_small("quaternion_inverse_rotation_error", (inverted_quaternion.toRotationMatrix() - rci.transpose()).norm(), 1e-12);

  const Sophus::SO3<double> rotation(quaternion);
  expect_small("SO3_inverse_rotation_error", (rotation.inverse().matrix() - rci.transpose()).norm(), 1e-12);
  const Sophus::SE3<double> transform(rci, pci);
  const Sophus::SE3<double> inverse = transform.inverse();
  Eigen::Matrix4d manual_inverse = Eigen::Matrix4d::Identity();
  manual_inverse.block<3, 3>(0, 0) = rci.transpose();
  manual_inverse.block<3, 1>(0, 3) = -rci.transpose() * pci;
  expect_small("SE3_inverse_manual_matrix_error", (inverse.matrix() - manual_inverse).norm(), 1e-12);
  expect_small("SE3_right_inverse_identity_error", ((transform * inverse).matrix() - Eigen::Matrix4d::Identity()).norm(), 1e-12);
  expect_small("SE3_left_inverse_identity_error", ((inverse * transform).matrix() - Eigen::Matrix4d::Identity()).norm(), 1e-12);
  std::cout << "transform_R=\n" << transform.rotationMatrix()
            << "\ninverse_R=\n" << inverse.rotationMatrix()
            << "\nmanual_inverse_R=\n" << manual_inverse.block<3, 3>(0, 0)
            << "\nrelative_R=\n" << (transform * inverse).rotationMatrix() << '\n';

  vk::PinholeCamera camera(1280, 1024, 1.0, 1297.754900243953, 1299.166933102416,
                          655.3979063879893, 501.2026449669307,
                          -0.08265631931699047, 0.13391147021521527,
                          -0.00032649255292178974, 0.00029400314258161474, 0.0);
  cv::Mat image(1024, 1280, CV_8UC1, cv::Scalar(128));
  VIOManager manager;
  manager.cam = &camera;
  manager.Rci = rci;
  manager.Pci = pci;
  manager.new_frame_.reset(new Frame(&camera, image));
  StatesGroup state;
  manager.updateFrameState(state);
  expect_small("compiled_updateFrameState_matrix_error", (manager.new_frame_->T_f_w_.matrix() - transform.matrix()).norm(), 1e-12);

  const V2D reference_pixel(500.25, 450.75);
  V3D reference_camera_point = camera.cam2world(reference_pixel);
  reference_camera_point *= 3.0 / reference_camera_point[2];
  const V3D world_point = rci.transpose() * (reference_camera_point - pci);
  float *reference_patch = new float[64]();
  Feature reference(nullptr, reference_patch, reference_pixel, camera.cam2world(reference_pixel), manager.new_frame_->T_f_w_, 0);
  const Sophus::SE3<double> relative = manager.new_frame_->T_f_w_ * reference.T_f_w_.inverse();
  expect_small("Frame_Feature_static_relative_identity_error", (relative.matrix() - Eigen::Matrix4d::Identity()).norm(), 1e-12);
  const V3D actual_reference_point = reference.T_f_w_ * world_point;
  expect_small("reference_pixel_roundtrip_error", (camera.world2cam(actual_reference_point) - reference_pixel).norm(), 0.001);
  Matrix2d static_warp;
  manager.getWarpMatrixAffineHomography(camera, reference_pixel, actual_reference_point,
                                        V3D(0.0, 0.0, 1.0), relative, 0, static_warp);
  std::cout << "static_nonidentity_absolute_warp=\n" << static_warp << "\nstatic_warp_det=" << static_warp.determinant() << '\n';
  expect_small("static_nonidentity_absolute_warp_error", (static_warp - Matrix2d::Identity()).norm(), 0.001);
  Matrix2d manual_static_warp;
  manager.getWarpMatrixAffineHomography(camera, reference_pixel, actual_reference_point,
                                        V3D(0.0, 0.0, 1.0), Sophus::SE3<double>(), 0, manual_static_warp);
  expect_small("manual_identity_warp_error", (manual_static_warp - Matrix2d::Identity()).norm(), 0.001);

  state.rot_end = Exp(V3D(0.001, -0.002, 0.0015));
  state.pos_end = V3D(0.002, -0.001, 0.0005);
  manager.updateFrameState(state);
  const M3D manual_relative_rotation = manager.new_frame_->T_f_w_.rotationMatrix() * reference.T_f_w_.rotationMatrix().transpose();
  const V3D manual_relative_translation = manager.new_frame_->T_f_w_.translation() - manual_relative_rotation * reference.T_f_w_.translation();
  const Sophus::SE3<double> actual_relative = manager.new_frame_->T_f_w_ * reference.T_f_w_.inverse();
  expect_small("tiny_motion_relative_rotation_error", (actual_relative.rotationMatrix() - manual_relative_rotation).norm(), 1e-12);
  expect_small("tiny_motion_relative_translation_error", (actual_relative.translation() - manual_relative_translation).norm(), 1e-12);
  std::cout << "tiny_motion_relative_angle=" << Eigen::AngleAxisd(actual_relative.rotationMatrix()).angle()
            << " manual_angle=" << Eigen::AngleAxisd(manual_relative_rotation).angle()
            << " actual_translation=" << actual_relative.translation().transpose()
            << " manual_translation=" << manual_relative_translation.transpose() << '\n';
  std::cout << "vio_transform_test failures=" << failures << '\n';
  return failures ? 1 : 0;
}
