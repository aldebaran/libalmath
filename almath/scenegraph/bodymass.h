/*
 *  Copyright 2015 Aldebaran. All rights reserved.
 *
 */
#ifndef LIB_ALMATH_SCENEGRAPH_BODYMASS_H
#define LIB_ALMATH_SCENEGRAPH_BODYMASS_H

#include <Eigen/Dense>
#include <boost/math/special_functions/pow.hpp>

namespace AL {
namespace Math {


// add the rotational inertial of a point mass located at position, to output.
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

// Mass of a rigid body, described in an implicitly-defined "body frame".
template <typename T>
struct BodyMass {
  typedef T Scalar;
  typedef Eigen::Matrix<Scalar, 3, 1, Eigen::DontAlign> Vector3;
  typedef Eigen::Matrix<Scalar, 3, 3, Eigen::DontAlign> Matrix3;

  Scalar mass;
  // center of mass, expressed in the body frame.
  Vector3 center_of_mass;
  // rotational inertia, expressed at the body center of mass,
  // in the body frame basis.
  Matrix3 rotational_inertia;

  static BodyMass Zero() {
    return BodyMass{0, Vector3::Zero(), Matrix3::Zero()};
  }

  template <typename S>
  BodyMass<S> cast() const {
    return BodyMass<S>{static_cast<S>(mass), center_of_mass.template cast<S>(),
                       rotational_inertia.template cast<S>()};
  }

  // return the rotational inertial expressed at the given point,
  // in the body frame basis.
  // The point is given in the body frame.
  Matrix3 get_rotational_inertia_at(const Vector3 &point) const {
    Matrix3 ret = rotational_inertia;
    addPointMassInertia(mass, point - center_of_mass, ret);
    return ret;
  }
};

template <typename T>
bool operator==(const BodyMass<T> &lhs, const BodyMass<T> &rhs) {
  return lhs.mass == rhs.mass &&
         lhs.center_of_mass == rhs.center_of_mass &&
         lhs.rotational_inertia == rhs.rotational_inertia;
}

template <typename T, typename BodyMassRange>
BodyMass<T> squashBodyMasses(const BodyMassRange &bodymasses) {
  using Scalar = typename BodyMass<T>::Scalar;
  using Vector3 = typename BodyMass<T>::Vector3;
  using Matrix3 = typename BodyMass<T>::Matrix3;
  Scalar mass = 0;
  Vector3 center_of_mass = Vector3::Zero();
  for (const BodyMass<T> &bodymass: bodymasses) {
    mass += bodymass.mass;
    center_of_mass += bodymass.mass * bodymass.center_of_mass;
  }
  if (mass != static_cast<Scalar>(0))
    center_of_mass /= mass;

  Matrix3 rotational_inertia = Matrix3::Zero();
  for (const BodyMass<T> &bodymass: bodymasses) {
    rotational_inertia += bodymass.rotational_inertia;
    addPointMassInertia(bodymass.mass,
                        center_of_mass - bodymass.center_of_mass,
                        rotational_inertia);
  }
  return BodyMass<T>{mass, center_of_mass, rotational_inertia};
}

// squash Body masses
template <typename T>
BodyMass<T> operator+(const BodyMass<T> &lhs, const BodyMass<T> &rhs) {
  using Scalar = typename BodyMass<T>::Scalar;
  using Vector3 = typename BodyMass<T>::Vector3;
  using Matrix3 = typename BodyMass<T>::Matrix3;
  const Scalar mass = lhs.mass + rhs.mass;
  Vector3 center_of_mass = lhs.mass * lhs.center_of_mass +
                           rhs.mass * rhs.center_of_mass;
  if (mass != static_cast<Scalar>(0))
    center_of_mass /= mass;

  Matrix3 rotational_inertia = lhs.rotational_inertia +
                               rhs.rotational_inertia;
  for (const BodyMass<T> &bodymass: {lhs, rhs}) {
    addPointMassInertia(bodymass.mass,
                        center_of_mass - bodymass.center_of_mass,
                        rotational_inertia);
  }
  return BodyMass<T>{mass, center_of_mass, rotational_inertia};
}

}
}

#endif
