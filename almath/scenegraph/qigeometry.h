/**
 * Copyright 2016 Aldebaran. All rights reserved.
 */
#pragma once
#ifndef LIBALMATH_SCENEGRAPH_QIGEOMETRY_H
#define LIBALMATH_SCENEGRAPH_QIGEOMETRY_H

// the user is expected to
//  #include <qi/geometry/geometry.hpp>
// and link with qigeometry

#include <Eigen/Geometry>

namespace qi
{
namespace geometry
{
inline Vector3 makeVector3(double x, double y, double z)
{
  return Vector3{x, y, z};
}

inline std::ostream& operator<<(std::ostream &o, const Vector3 &t)
{
  return o << "Vector3(" << t.x << ", " << t.y << ", " << t.z << ")";
}


inline Quaternion makeQuaternion(double x, double y, double z, double w)
{
  return Quaternion{x, y, z, w};
}

inline std::ostream& operator<<(std::ostream &o, const Quaternion &r)
{
  return o << "Quaternion(" << r.x << ", " << r.y << ", " << r.z << ", "
           << r.w << ")";
}

inline double norm(const Quaternion &r)
{
  return Eigen::Map<const Eigen::Quaterniond>(&r.x).norm();
}

inline bool isNormalized(const Quaternion &r, double epsilon)
{
  return std::abs(norm(r) - 1.0) < epsilon;
}

// Normalize a quaternion in place
inline void normalize(Quaternion &r)
{
  Eigen::Map<Eigen::Quaterniond>(&r.x).normalize();
}

// Return a normalized copy of a quaternion
inline Quaternion normalized(const Quaternion &r)
{
  Quaternion result = r;
  normalize(result);
  return result;
}

inline
Transform makeTransform(const Quaternion &rotation, const Vector3 &translation)
{
  return Transform{rotation, translation};
}


inline std::ostream& operator<<(std::ostream &o, const Transform &tf)
{
  return o << "Transform(" << tf.rotation << ", " << tf.translation << ")";
}


inline Eigen::Affine3d toEigenAffine3d(const Transform &tf)
{
  assert(isNormalized(tf.rotation, 1e-5));
  const auto& t = tf.translation;
  const auto& r = tf.rotation;
  return Eigen::Affine3d(Eigen::Translation3d(t.x, t.y, t.z) *
                         Eigen::Quaterniond(r.w, r.x, r.y, r.z));
}

// assumes lhs and rhs have normalized quaternions
inline Transform operator*(const Transform &lhs, const Transform &rhs)
{
  Transform result;
  assert(isNormalized(lhs.rotation, 1e-5));
  assert(isNormalized(rhs.rotation, 1e-5));
  auto lv = Eigen::Map<const Eigen::Vector3d>(&lhs.translation.x);
  auto rv = Eigen::Map<const Eigen::Vector3d>(&rhs.translation.x);
  auto lq = Eigen::Map<const Eigen::Quaterniond>(&lhs.rotation.x);
  auto rq = Eigen::Map<const Eigen::Quaterniond>(&rhs.rotation.x);
  Eigen::Map<Eigen::Quaterniond>(&result.rotation.x) = lq * rq;
  Eigen::Map<Eigen::Vector3d>(&result.translation.x) = lv + lq * rv;
  return result;
}

// assumes lhs and rhs have normalized quaternions
inline bool isNear(const Transform &lhs, const Transform &rhs, double epsilon)
{
  auto la = toEigenAffine3d(lhs);
  auto ra = toEigenAffine3d(rhs);
  return la.isApprox(ra, epsilon);
}


// assumes tf has normalized quaternion
inline Transform inverse(const Transform &tf)
{
  Transform result;
  auto inv = toEigenAffine3d(tf).inverse();
  Eigen::Map<Eigen::Quaterniond>(&result.rotation.x) = inv.rotation();
  Eigen::Map<Eigen::Vector3d>(&result.translation.x) = inv.translation();
  return result;
}

}
}

#endif
