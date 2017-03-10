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
#include <Eigen/Dense>

namespace AL {
namespace Math {

typedef Eigen::Matrix<float, 3, 4> Matrix34f;
typedef Eigen::Matrix<float, 3, 4, Eigen::RowMajor> Matrix34frm;
typedef Eigen::Matrix<float, 6, 1> Vector6f;

inline Matrix34f toEigenMatrix34(const AL::Math::Transform &tr) {
  return Matrix34f(Eigen::Map<const Matrix34frm>(&tr.r1_c1));
}

inline Eigen::Matrix3f toEigenMatrix3(const AL::Math::Transform &tr) {
  return Eigen::Matrix3f(
      Eigen::Map<const Matrix34frm>(&tr.r1_c1).block<3, 3>(0, 0));
}

template <typename T>
inline Eigen::Quaternion<T> toEigenQuaternion(const AL::Math::Transform &tr) {
  return Eigen::Quaternion<T>(
      Eigen::Map<const Matrix34frm>(&tr.r1_c1).block<3, 3>(0, 0));
}

inline Eigen::Vector3f toEigenVector3(const AL::Math::Position3D &v) {
  return Eigen::Vector3f(Eigen::Map<const Eigen::Vector3f>(&v.x));
}

inline Eigen::AffineCompact3f toEigenAffineCompact3(
    const AL::Math::Transform &tr) {
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
  AL::Math::Transform out;
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
}
}
#endif
