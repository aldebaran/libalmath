/**
 * Copyright 2016 Aldebaran. All rights reserved.
 */

#include <almath/geometry/geometry.hpp>
#include <Eigen/Geometry>

namespace qi
{
namespace geometry
{
double norm(const Quaternion &r)
{
  return Eigen::Map<const Eigen::Quaterniond>(&r.x).norm();
}

void normalize(Quaternion &r)
{
  Eigen::Map<Eigen::Quaterniond>(&r.x).normalize();
}

Quaternion normalized(const Quaternion &r)
{
  Quaternion result = r;
  normalize(result);
  return result;
}

bool isNormalized(const Quaternion &r, double epsilon)
{
  return std::abs(norm(r) - 1.0) < epsilon;
}

inline Eigen::Affine3d toEigenAffine3d(const Transform &tf)
{
  assert(isNormalized(tf.r, 1e-5));
  return Eigen::Affine3d(Eigen::Translation3d(tf.t.x, tf.t.y, tf.t.z) *
                         Eigen::Quaterniond(tf.r.w, tf.r.x, tf.r.y, tf.r.z));
}

Transform operator*(const Transform &lhs, const Transform &rhs)
{
  assert(isNormalized(lhs.r, 1e-5));
  assert(isNormalized(rhs.r, 1e-5));
  auto lv = Eigen::Map<const Eigen::Vector3d>(&lhs.t.x);
  auto rv = Eigen::Map<const Eigen::Vector3d>(&rhs.t.x);
  auto lq = Eigen::Map<const Eigen::Quaterniond>(&lhs.r.x);
  auto rq = Eigen::Map<const Eigen::Quaterniond>(&rhs.r.x);
  Transform result;
  Eigen::Map<Eigen::Quaterniond>(&result.r.x) = lq * rq;
  Eigen::Map<Eigen::Vector3d>(&result.t.x) = lv + lq * rv;
  return result;
}

bool isNear(const Transform &lhs, const Transform &rhs, double epsilon)
{
  auto la = toEigenAffine3d(lhs);
  auto ra = toEigenAffine3d(rhs);
  return la.isApprox(ra, epsilon);
}

Transform inverse(const Transform &tf)
{
  auto inv = toEigenAffine3d(tf).inverse();
  Transform result;
  Eigen::Map<Eigen::Quaterniond>(&result.r.x) = inv.rotation();
  Eigen::Map<Eigen::Vector3d>(&result.t.x) = inv.translation();
  return result;
}
}
}
