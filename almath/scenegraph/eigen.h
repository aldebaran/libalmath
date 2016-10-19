/*
 * Copyright (c) 2017 SoftBank Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */

#pragma once
#ifndef _LIBALMATH_ALMATH_SCENEGRAPH_EIGEN_H_
#define _LIBALMATH_ALMATH_SCENEGRAPH_EIGEN_H_

#include <Eigen/Dense>

namespace AL {
namespace Math {

enum struct TwistPolarity { AngularFirst = 0, LinearFirst = 1 };

template <typename Scalar_, TwistPolarity Polarity_, int Options_>
class Twist : public Eigen::Matrix<Scalar_, 6, 1, Options_> {
 private:
  typedef Eigen::Matrix<Scalar_, 6, 1, Options_> Derived;
  // Derived m_matrix;
 public:
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF_VECTORIZABLE_FIXED_SIZE(Scalar_, 6)

  typedef Derived MatrixType;
  typedef Scalar_ Scalar;

  typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
  typedef Eigen::VectorBlock<Derived, 3> Part;
  typedef const Eigen::VectorBlock<const Derived, 3> ConstPart;

  enum {
    PolarityIsAngularFirst = (Polarity_ == TwistPolarity::AngularFirst),
    LinearIndex = PolarityIsAngularFirst ? 3 : 0,
    AngularIndex = PolarityIsAngularFirst ? 0 : 3
  };
  inline Part linear() { return this->template segment<3>(LinearIndex); }
  inline ConstPart linear() const {
    return this->template segment<3>(LinearIndex);
  }
  inline Part angular() { return this->template segment<3>(AngularIndex); }
  inline ConstPart angular() const {
    return this->template segment<3>(AngularIndex);
  }
  inline MatrixType matrix() { return *this; }
  inline MatrixType const matrix() const { return *this; }
};

typedef Twist<float, TwistPolarity::AngularFirst, 0> Twistf;
typedef Twist<double, TwistPolarity::AngularFirst, 0> Twistd;

template <typename T, typename Derived0, typename Derived1>
T exp(const Eigen::MatrixBase<Derived0> &v,
      const Eigen::MatrixBase<Derived1> &w) {
  // implementation copied from AL::Math::velocityExponentialInPlace
  typedef typename Derived0::Scalar Scalar;
  EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived0);
  EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived1);

  typedef Eigen::Transform<Scalar, 3, Eigen::AffineCompact> AffineCompact3;
  typedef Eigen::Array<Scalar, 1, 3> Array3;

  Scalar t = w.norm();
  Array3 w2 = w.array().square();

  Scalar CC = 0, SC = 0, dSC = 0;

  if (t >= Eigen::NumTraits<Scalar>::dummy_precision()) {
    CC = (1 - std::cos(t)) / (t * t);  // motionCos cardinal cosinus
    SC = std::sin(t) / t;              // motionSin cardinal sinus
    dSC = (t - std::sin(t)) /
          std::pow(t, 3);  // motionSin cardinal sinus derivative std::pow
  } else {
    CC = 0.5;
    SC = 1.0 - t * t / 6;
    dSC = 1. / 6;
  }

  AffineCompact3 res;

  // Maxima
  res.linear()(0, 0) = 1. - CC * (w2[2] + w2[1]);
  res.linear()(0, 1) = -SC * w[2] + CC * w[0] * w[1];
  res.linear()(0, 2) = SC * w[1] + CC * w[0] * w[2];

  res.linear()(1, 0) = SC * w[2] + CC * w[0] * w[1];
  res.linear()(1, 1) = 1. - CC * (w2[0] + w2[2]);
  res.linear()(1, 2) = -SC * w[0] + CC * w[1] * w[2];

  res.linear()(2, 0) = -SC * w[1] + CC * w[0] * w[2];
  res.linear()(2, 1) = SC * w[0] + CC * w[1] * w[2];
  res.linear()(2, 2) = 1. - CC * (w2[0] + w2[1]);

  res.translation()(0) = (SC + dSC * w2[0]) * v[0] +
                         (-CC * w[2] + dSC * w[0] * w[1]) * v[1] +
                         (+CC * w[1] + dSC * w[0] * w[2]) * v[2];
  res.translation()(1) = (CC * w[2] + dSC * w[1] * w[0]) * v[0] +
                         (SC + dSC * w2[1]) * v[1] +
                         (-CC * w[0] + dSC * w[1] * w[2]) * v[2];
  res.translation()(2) = (-CC * w[1] + dSC * w[2] * w[0]) * v[0] +
                         (CC * w[0] + dSC * w[2] * w[1]) * v[1] +
                         (SC + dSC * w2[2]) * v[2];
  return res;
}

template <typename Derived0>
Eigen::Matrix3f skew(const Eigen::MatrixBase<Derived0> &in)
{
  EIGEN_STATIC_ASSERT(
      (Eigen::internal::is_same<float, typename Derived0::Scalar>::value),
      YOU_MIXED_DIFFERENT_NUMERIC_TYPES__YOU_NEED_TO_USE_THE_CAST_METHOD_OF_MATRIXBASE_TO_CAST_NUMERIC_TYPES_EXPLICITLY)
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived0, 3, 1)
  Eigen::Matrix3f ret = Eigen::Matrix3f::Zero();
  ret(1, 2) = -in(0);
  ret(2, 1) = in(0);
  ret(0, 2) = in(1);
  ret(2, 0) = -in(1);
  ret(0, 1) = -in(2);
  ret(1, 0) = in(2);
  return ret;
}

// given a matrix pose_a_b such that
//   p_a = pose_a_b * p_b,
// compute adjoint_a_b, such that
//   twist_a = adjoint_a_b * twist_b
template <TwistPolarity Polarity_, typename Scalar_>
inline Eigen::Matrix<Scalar_, 6, 6> adjoint(
    const Eigen::Matrix<Scalar_, 3, 4> &pose_a_b)
{
  Eigen::Block<const Eigen::Matrix<Scalar_, 3, 4>, 3, 3, false> rot_a_b =
      pose_a_b.template block<3, 3>(0, 0);
  Eigen::Block<const Eigen::Matrix<Scalar_, 3, 4>, 3, 1, true> p_a_b =
      pose_a_b.template rightCols<1>();
  Eigen::Matrix<Scalar_, 6, 6> adjoint_a_b;
  if (Polarity_ == TwistPolarity::AngularFirst)
  {
    adjoint_a_b << rot_a_b,                 Eigen::Matrix3f::Zero(),
                   (skew(p_a_b) * rot_a_b), rot_a_b;
  }
  else
  {
    adjoint_a_b << rot_a_b,                 (skew(p_a_b) * rot_a_b),
                   Eigen::Matrix3f::Zero(), rot_a_b;
  }
  return adjoint_a_b;
}
}  // namespace Math
}  // namespace AL
#endif
