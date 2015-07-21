/*
 *  Copyright 2015 Aldebaran. All rights reserved.
 *
 */
#ifndef LIB_ALMATH_SCENEGRAPH_BODYMASS_H
#define LIB_ALMATH_SCENEGRAPH_BODYMASS_H

#include <Eigen/Dense>

namespace AL {
namespace Math {

// Mass of a rigid body, described in an implcitly-defined "body frame".
template <typename T>
struct BodyMass {
  typedef T Scalar;
  typedef Eigen::Matrix<Scalar, 3, 1, Eigen::DontAlign> Vector3;
  typedef Eigen::Matrix<Scalar, 3, 3, Eigen::DontAlign> Matrix3;

  Scalar mass;
  // center of mass, expressed in the body frame.
  Vector3 center_of_mass;
  // rotational inertia, expressed at the body center of mass, the body frame
  // basis.
  Matrix3 rotational_inertia;

  template <typename S>
  BodyMass<S> cast() const {
    return BodyMass<S>{static_cast<S>(mass), center_of_mass.template cast<S>(),
                       rotational_inertia.template cast<S>()};
  }
};
}
}

#endif
