/*
 * Copyright (c) 2015 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */

#pragma once
#ifndef _LIBALMATH_ALMATH_SCENEGRAPH_ALMATHEIGEN_H_
#define _LIBALMATH_ALMATH_SCENEGRAPH_ALMATHEIGEN_H_

#include <almath/types/altransform.h>
#include <almath/types/alposition3d.h>
#include <almath/types/alvelocity6d.h>
#include <boost/range/begin.hpp>
#include <boost/range/size.hpp>
#include <Eigen/Dense>

namespace AL {
namespace Math {

typedef Eigen::Matrix<float, 3, 4> Matrix34f;
typedef Eigen::Matrix<float, 3, 4, Eigen::RowMajor> Matrix34frm;
typedef Eigen::Matrix<float, 6, 1> Vector6f;

inline
Eigen::Map<const Matrix34frm> toEigenMapMatrix34(const Math::Transform &tr) {
  return Eigen::Map<const Matrix34frm>(&tr.r1_c1);
}

inline Matrix34f toEigenMatrix34(const Math::Transform &tr) {
  return Matrix34f(Eigen::Map<const Matrix34frm>(&tr.r1_c1));
}

inline Eigen::Matrix3f toEigenMatrix3(const Math::Transform &tr) {
  return Eigen::Matrix3f(
      Eigen::Map<const Matrix34frm>(&tr.r1_c1).block<3, 3>(0, 0));
}

template <typename T>
inline Eigen::Quaternion<T> toEigenQuaternion(const Math::Transform &tr) {
  return Eigen::Quaternion<T>(
      Eigen::Map<const Matrix34frm>(&tr.r1_c1).block<3, 3>(0, 0));
}

inline Eigen::Vector3f toEigenVector3(const Math::Position3D &v) {
  return Eigen::Vector3f(Eigen::Map<const Eigen::Vector3f>(&v.x));
}

inline Eigen::AffineCompact3f toEigenAffineCompact3(
    const Math::Transform &tr) {
  return Eigen::AffineCompact3f(Eigen::Map<const Matrix34frm>(&tr.r1_c1));
}

template <typename Derived0>
void toALMathTransform(const Eigen::MatrixBase<Derived0> &in, Transform &out) {
  EIGEN_STATIC_ASSERT(
      (Eigen::internal::is_same<float, typename Derived0::Scalar>::value),
      YOU_MIXED_DIFFERENT_NUMERIC_TYPES__YOU_NEED_TO_USE_THE_CAST_METHOD_OF_MATRIXBASE_TO_CAST_NUMERIC_TYPES_EXPLICITLY)
  // will fail if in has the wrong size
  Eigen::Map<Matrix34frm>(&out.r1_c1) = in;
}

template <typename Derived0>
Transform toALMathTransform(const Eigen::MatrixBase<Derived0> &m) {
  Transform tr;
  toALMathTransform(m, tr);
  return tr;
}

template <int _Mode, int _Options>
void toALMathTransform(const Eigen::Transform<float, 3, _Mode, _Options> &in,
                       Transform &out) {
  Eigen::Map<Matrix34frm>(&out.r1_c1) = in.affine();
}

template <int _Mode, int _Options>
Transform toALMathTransform(
    const Eigen::Transform<float, 3, _Mode, _Options> &tr) {
  Math::Transform out;
  toALMathTransform(tr, out);
  return out;
}

template <typename Derived0>
Position3D toALMathPosition3D(const Eigen::MatrixBase<Derived0> &in) {
  EIGEN_STATIC_ASSERT(
      (Eigen::internal::is_same<float, typename Derived0::Scalar>::value),
      YOU_MIXED_DIFFERENT_NUMERIC_TYPES__YOU_NEED_TO_USE_THE_CAST_METHOD_OF_MATRIXBASE_TO_CAST_NUMERIC_TYPES_EXPLICITLY)
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived0, 3, 1)
  return Position3D(in[0], in[1], in[2]);
}

template <typename Derived0>
void toALMathVelocity6D(const Eigen::MatrixBase<Derived0> &in,
                        Velocity6D &out) {
  EIGEN_STATIC_ASSERT(
      (Eigen::internal::is_same<float, typename Derived0::Scalar>::value),
      YOU_MIXED_DIFFERENT_NUMERIC_TYPES__YOU_NEED_TO_USE_THE_CAST_METHOD_OF_MATRIXBASE_TO_CAST_NUMERIC_TYPES_EXPLICITLY)
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived0, 6, 1)
  assert((in.rows() == 6) && (in.cols() == 1));
  Eigen::Map<Vector6f>(&out.xd) = in;
}

template <typename Derived0>
Velocity6D toALMathVelocity6D(const Eigen::MatrixBase<Derived0> &in) {
  EIGEN_STATIC_ASSERT(
      (Eigen::internal::is_same<float, typename Derived0::Scalar>::value),
      YOU_MIXED_DIFFERENT_NUMERIC_TYPES__YOU_NEED_TO_USE_THE_CAST_METHOD_OF_MATRIXBASE_TO_CAST_NUMERIC_TYPES_EXPLICITLY)
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived0, 6, 1)
  return Velocity6D(in[0], in[1], in[2], in[3], in[4], in[5]);
}

// Compute the average of a range of AL::Math::Transform.
// From: http://wiki.unity3d.com/index.php/Averaging_Quaternions_and_Vectors
// Throw if the range is empty.
template <class RandomAccessRange>
Math::Transform averageTransforms(const RandomAccessRange &range)
{
  const int numCumulated = boost::size(range);
  if (numCumulated == 0)
  {
    throw std::runtime_error("Invalid empty range");
  }
  float averageFactor = 1.f / static_cast<float>(numCumulated);

  const auto firstQuat = Math::toEigenQuaternion<float>(*boost::begin(range));
  Eigen::Vector3f cumulatedPosition = Eigen::Vector3f::Zero();
  Eigen::Quaternion<float> cumulatedQuaternion{0.f, 0.f, 0.f, 0.f};
  for (const auto &tf : range)
  {
    // If the new quaternion is not close enough to the first rotation
    // of the range of transforms, they cannot be averaged, so we consider -q
    // instead of q.
    auto newRotation = Math::toEigenQuaternion<float>(tf);
    const auto areQuaternionsClose = newRotation.dot(firstQuat) >= 0.f;
    if (!areQuaternionsClose)
    {
      // Consider -q instead of q.
      newRotation.coeffs() = -newRotation.coeffs();
    }
    cumulatedQuaternion.coeffs() += newRotation.coeffs();
    cumulatedPosition +=
        Eigen::Map<const Math::Matrix34frm>(&tf.r1_c1).block<3, 1>(0, 3);
  }
  cumulatedQuaternion.normalize();
  Math::Transform ret;
  Eigen::Map<Matrix34frm> retm(&ret.r1_c1);
  retm.block<3, 3>(0, 0) = cumulatedQuaternion.toRotationMatrix();
  retm.block<3, 1>(0, 3) = cumulatedPosition * averageFactor;
  return ret;
}
}
}
#endif
