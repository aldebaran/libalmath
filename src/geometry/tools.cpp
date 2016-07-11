/**
 * Copyright 2016 Aldebaran. All rights reserved.
 */

#include <almath/geometry/tools.hpp>
#include <Eigen/Geometry>

namespace qi
{
namespace geometry
{
Vector3 makeVector3(double x, double y, double z)
{
  // TODO: #34174 qilang does not let us define our constructors
  Vector3 v;
  v.x = x;
  v.y = y;
  v.z = z;
  return v;
}

Quaternion makeQuaternion(double x, double y, double z, double w)
{
  // TODO: #34174 qilang does not let us define our constructors
  Quaternion q;
  q.x = x;
  q.y = y;
  q.z = z;
  q.w = w;
  return q;
}

bool isNormalized(const Quaternion &r, double epsilon)
{
  return std::abs(norm(r) - 1.0) < epsilon;
}

Quaternion makeNormalizedQuaternion(double x, double y, double z, double w)
{
  auto q = makeQuaternion(x, y, z, w);
  if (!isNormalized(q, 1e-5f))
  {
    std::stringstream err;
    err << "Quaternion(" << x << ", " << y << ", " << z << ", " << w
        << ") is not normalized";
    throw std::runtime_error(err.str());
  }
  return q;
}

Transform makeTransform(const Quaternion &rotation, const Vector3 &translation)
{
  // TODO: #34174 qilang does not let us define our constructors
  Transform tf;
  tf.translation = translation;
  tf.rotation = rotation;
  return tf;
}

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

inline Eigen::Affine3d toEigenAffine3d(const Transform &tf)
{
  assert(isNormalized(tf.rotation, 1e-5));
  const auto& t = tf.translation;
  const auto& r = tf.rotation;
  return Eigen::Affine3d(Eigen::Translation3d(t.x, t.y, t.z) *
                         Eigen::Quaterniond(r.w, r.x, r.y, r.z));
}

Transform operator*(const Transform &lhs, const Transform &rhs)
{
  assert(isNormalized(lhs.rotation, 1e-5));
  assert(isNormalized(rhs.rotation, 1e-5));
  auto lv = Eigen::Map<const Eigen::Vector3d>(&lhs.translation.x);
  auto rv = Eigen::Map<const Eigen::Vector3d>(&rhs.translation.x);
  auto lq = Eigen::Map<const Eigen::Quaterniond>(&lhs.rotation.x);
  auto rq = Eigen::Map<const Eigen::Quaterniond>(&rhs.rotation.x);
  Transform result;
  Eigen::Map<Eigen::Quaterniond>(&result.rotation.x) = lq * rq;
  Eigen::Map<Eigen::Vector3d>(&result.translation.x) = lv + lq * rv;
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
  Eigen::Map<Eigen::Quaterniond>(&result.rotation.x) = inv.rotation();
  Eigen::Map<Eigen::Vector3d>(&result.translation.x) = inv.translation();
  return result;
}
}
}
