/**
 * Copyright 2016 Aldebaran. All rights reserved.
 */

#include <almath/geometry/tools.hpp>
#include <Eigen/Geometry>

namespace qi
{
namespace geometry
{
Vector3 make_vector3(double x, double y, double z)
{
  // TODO: #34174 qilang does not let us define our constructors
  Vector3 v;
  v.x = x;
  v.y = y;
  v.z = z;
  return v;
}

Quaternion make_quaternion(double x, double y, double z, double w)
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

Quaternion make_normalized_quaternion(double x, double y, double z, double w)
{
  auto q = make_quaternion(x, y, z, w);
  if (!isNormalized(q, 1e-5f))
  {
    std::stringstream err;
    err << "Quaternion(" << x << ", " << y << ", " << z << ", " << w
        << ") is not normalized";
    throw std::runtime_error(err.str());
  }
  return q;
}

Transform make_transform(const Quaternion &r, const Vector3 &t)
{
  // TODO: #34174 qilang does not let us define our constructors
  Transform tf;
  tf.t = t;
  tf.r = r;
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
