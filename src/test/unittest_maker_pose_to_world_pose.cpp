#include <gtest/gtest.h>

#include <Eigen/Geometry>

TEST(pose_estimation, marker_pose_to_world_pose_using_quaternion)
{
  // marker is detected by an camera, then pose is estimated relative from camera perspective
  // note: uses Pythagoras for ground truth
  // pose marker relative from camera
  const Eigen::Vector3d pos_m(std::sqrt(2 * 2 + 2 * 2), 0, 0);
  const Eigen::Quaterniond rot_m(Eigen::AngleAxisd(-M_PI_4, Eigen::Vector3d::UnitZ()));

  // pose camera absolute in world
  const Eigen::Vector3d pos_w(2.0, 2.0, 0.0);
  const Eigen::Quaterniond rot_w(Eigen::AngleAxisd(M_PI_4, Eigen::Vector3d::UnitZ()));

  // transforming marker pose seen by camera into world coordinates
  // P_m = T_m * R_m
  // P_m^-1 = (T_m * R_m)^-1 = R_m^-1 * T_m^-1
  // T_m can be seen as a vector instead of translation matrix, therefore 
  const Eigen::Vector3d pos_m_inv = pos_m;// * -1.0;
  const Eigen::Quaterniond rot_m_inv = rot_m.inverse();

  const Eigen::Vector3d pos_w_calc = rot_m_inv * pos_m_inv;

  std::cout << "pos_w_calc:\n" << pos_w_calc << std::endl;
}

TEST(pose_estimation, marker_pose_to_world_pose_using_matrix)
{
  // marker is detected by an camera, then pose is estimated relative from camera perspective
  // note: uses Pythagoras for ground truth
  // pose marker relative from camera
  const Eigen::Vector3d pos_m(std::sqrt(2 * 2 + 2 * 2), 0, 0);
  const Eigen::Quaterniond rot_m(Eigen::AngleAxisd(-M_PI_4, Eigen::Vector3d::UnitZ()));

  // pose camera absolute in world
  const Eigen::Vector3d pos_w(2.0, 2.0, 0.0);
  const Eigen::Quaterniond rot_w(Eigen::AngleAxisd(M_PI_4, Eigen::Vector3d::UnitZ()));

  // transforming marker pose seen by camera into world coordinates
  // P_m = T_m * R_m
  Eigen::Matrix4d T_m;
  T_m << 1.0, 0.0, 0.0, pos_m.x(),
         0.0, 1.0, 0.0, pos_m.y(),
         0.0, 0.0, 1.0, pos_m.z(),
         0.0, 0.0, 0.0,       1.0;
  
  Eigen::Matrix4d R_m = Eigen::Matrix4d::Identity();
  R_m.block<3, 3>(0, 0) = rot_m.toRotationMatrix();

  std::cout << "T_m:\n" << T_m << std::endl;
  std::cout << "R_m:\n" << R_m << std::endl;
  
  const Eigen::Matrix4d P_m = T_m * R_m;
  const Eigen::Matrix4d P_m_inv = P_m.inverse();

  std::cout << "P_m_inv:\n" << P_m_inv << std::endl;

  const Eigen::Quaterniond q_180(
    Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) * 
    Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) * 
    Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ())
  );
  std::cout << "R_180:\n" << q_180.toRotationMatrix() << std::endl;
}