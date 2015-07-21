/*
 *  Copyright 2015 Aldebaran. All rights reserved.
 *
 */

#ifndef LIB_ALMATH_SCENEGRAPH_URDFEIGEN_H
#define LIB_ALMATH_SCENEGRAPH_URDFEIGEN_H

#include <almath/scenegraph/urdf.h>
#include <Eigen/Geometry>

namespace AL {
namespace Math {

inline Eigen::Quaterniond eigenQuaternionFromUrdfRpy(
    const AL::urdf::Array3d &rpy) {
  return Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX());
}

template <typename T>
AL::urdf::Array3d urdfRpyFromEigenMatrix3(const T &t) {
  Eigen::Vector3d ypr = t.eulerAngles(2, 1, 0);
  AL::urdf::Array3d result = {{ypr[2], ypr[1], ypr[0]}};
  return result;
}

inline Eigen::Isometry3d toEigenTransform(const AL::urdf::Pose &p) {
  typedef Eigen::Map<const Eigen::Vector3d> Vector3dMap;
  const AL::urdf::Array3d xyz = p.xyz();
  return Eigen::Translation3d(Vector3dMap(xyz.data())) *
         eigenQuaternionFromUrdfRpy(p.rpy());
}

inline Eigen::Matrix3d toEigenMatrix3(const AL::urdf::Inertial &inertial) {
  Eigen::Matrix3d m;
  m(0, 0) = inertial.ixx();
  m(0, 1) = m(1, 0) = inertial.ixy();
  m(0, 2) = m(2, 0) = inertial.ixz();
  m(1, 1) = inertial.iyy();
  m(1, 2) = m(2, 1) = inertial.iyz();
  m(2, 2) = inertial.izz();
  return m;
}

// TODO: implement this algorithm on Mass type?
template <typename Scalar, typename Derived0, typename Derived1>
void addPointMassInertia(Scalar mass,
                         const Eigen::MatrixBase<Derived0> &position,
                         Eigen::MatrixBase<Derived1> &output) {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived0, 3, 1)
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived1, 3, 3)
  EIGEN_STATIC_ASSERT(
      (Eigen::internal::is_same<Scalar, typename Derived0::Scalar>::value),
      YOU_MIXED_DIFFERENT_NUMERIC_TYPES__YOU_NEED_TO_USE_THE_CAST_METHOD_OF_MATRIXBASE_TO_CAST_NUMERIC_TYPES_EXPLICITLY)
  EIGEN_STATIC_ASSERT(
      (Eigen::internal::is_same<Scalar, typename Derived1::Scalar>::value),
      YOU_MIXED_DIFFERENT_NUMERIC_TYPES__YOU_NEED_TO_USE_THE_CAST_METHOD_OF_MATRIXBASE_TO_CAST_NUMERIC_TYPES_EXPLICITLY)
  typedef Eigen::Array<Scalar, 3, 1> Array3;
  Array3 position2 = position.array().square();
  output(0, 0) += mass * (position2[1] + position2[2]);
  output(1, 1) += mass * (position2[0] + position2[2]);
  output(2, 2) += mass * (position2[0] + position2[1]);

  output(0, 1) += -mass * position[0] * position[1];
  output(1, 0) += -mass * position[0] * position[1];

  output(0, 2) += -mass * position[0] * position[2];
  output(2, 0) += -mass * position[0] * position[2];

  output(1, 2) += -mass * position[1] * position[2];
  output(2, 1) += -mass * position[1] * position[2];
}

// squash inertial `a` and `b` to create inertial `c`.
// All of them follow the urdf convention: frame origin is the center of mass.
// pose_a and pose_b shall be defined with respect to the same frame.
// pose_c is defined such that
// 1/ its origin is the center of mass of the compound body
// 2/ its basis is the same as pose_a one.
template <typename Scalar, typename EPose, typename Derived1>
void squashInertial(Scalar mass_a, const EPose &pose_a,
                    const Eigen::MatrixBase<Derived1> &inertia_a, Scalar mass_b,
                    const EPose &pose_b,
                    const Eigen::MatrixBase<Derived1> &inertia_b,
                    Scalar &mass_c, EPose &pose_c,
                    Eigen::MatrixBase<Derived1> &inertia_c) {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived1, 3, 3)
  EIGEN_STATIC_ASSERT(
      (Eigen::internal::is_same<Scalar, typename EPose::Scalar>::value),
      YOU_MIXED_DIFFERENT_NUMERIC_TYPES__YOU_NEED_TO_USE_THE_CAST_METHOD_OF_MATRIXBASE_TO_CAST_NUMERIC_TYPES_EXPLICITLY)
  EIGEN_STATIC_ASSERT(
      (Eigen::internal::is_same<Scalar, typename Derived1::Scalar>::value),
      YOU_MIXED_DIFFERENT_NUMERIC_TYPES__YOU_NEED_TO_USE_THE_CAST_METHOD_OF_MATRIXBASE_TO_CAST_NUMERIC_TYPES_EXPLICITLY)
  typedef Eigen::Transform<Scalar, 3, Eigen::AffineCompact, Eigen::DontAlign> T;
  typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;
  typedef Eigen::Matrix<Scalar, 3, 1> Vector3;

  // define total mass
  mass_c = mass_a + mass_b;

  // define the new frame
  pose_c.translation() =
      (mass_a * pose_a.translation() + mass_b * pose_b.translation()) / mass_c;
  pose_c.linear() = pose_a.linear();

  // add inertia from a
  inertia_c = inertia_a;
  Vector3 displ = pose_c.translation() - pose_a.translation();
  addPointMassInertia(mass_a, pose_c.linear().transpose() * displ, inertia_c);

  // add inertia from b
  Matrix3 rot_cb = pose_c.linear().transpose() * pose_b.linear();
  inertia_c += rot_cb.transpose() * inertia_b * rot_cb;
  displ = pose_c.translation() - pose_b.translation();
  addPointMassInertia(mass_b, pose_c.linear().transpose() * displ, inertia_c);
}

template <typename Scalar, typename Derived0, typename Derived1>
void urdfInertialToEigen(const AL::urdf::Inertial &inertial, Scalar &mass,
                         Eigen::MatrixBase<Derived0> &com_a,
                         Eigen::MatrixBase<Derived1> &inertia_a) {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived0, 3, 1)
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived1, 3, 3)
  EIGEN_STATIC_ASSERT(
      (Eigen::internal::is_same<Scalar, typename Derived0::Scalar>::value),
      YOU_MIXED_DIFFERENT_NUMERIC_TYPES__YOU_NEED_TO_USE_THE_CAST_METHOD_OF_MATRIXBASE_TO_CAST_NUMERIC_TYPES_EXPLICITLY)
  EIGEN_STATIC_ASSERT(
      (Eigen::internal::is_same<Scalar, typename Derived1::Scalar>::value),
      YOU_MIXED_DIFFERENT_NUMERIC_TYPES__YOU_NEED_TO_USE_THE_CAST_METHOD_OF_MATRIXBASE_TO_CAST_NUMERIC_TYPES_EXPLICITLY)
  typedef Eigen::Transform<Scalar, 3, Eigen::AffineCompact, Eigen::DontAlign> T;
  typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;
  typedef Eigen::Matrix<Scalar, 3, 1> Vector3;

  mass = inertial.mass();
  T t_ab = toEigenTransform(inertial.origin()).cast<Scalar>();
  com_a = t_ab.translation();
  Matrix3 inertia_b = toEigenMatrix3(inertial).cast<Scalar>();
  inertia_a = t_ab.linear().transpose() * inertia_b * t_ab.linear();
}
}
}
#endif
