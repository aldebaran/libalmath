/**
 * Copyright 2016 Aldebaran. All rights reserved.
 */

#ifndef _LIBALMATH_ALMATH_GEOMETRY_TOOLS_HPP_
#define _LIBALMATH_ALMATH_GEOMETRY_TOOLS_HPP_

#include <almath/api.h>
#include <qi/geometry/geometry.hpp>

namespace qi
{
namespace geometry
{
ALMATH_API Vector3 makeVector3(double x, double y, double z);

ALMATH_API Quaternion makeQuaternion(double x, double y, double z, double w);

// Check if the quaternion made out of x, y, z, w is normalized, and
// throw if it is not the case.
ALMATH_API
Quaternion makeNormalizedQuaternion(double x, double y, double z, double w);

ALMATH_API
Transform makeTransform(const Quaternion &rotation, const Vector3 &translation);

// assumes lhs and rhs have normalized quaternions
ALMATH_API bool isNear(const Transform &lhs, const Transform &rhs, double epsilon);

// assumes lhs and rhs have normalized quaternions
ALMATH_API Transform operator*(const Transform &lhs, const Transform &rhs);

// assumes tf has normalized quaternion
ALMATH_API Transform inverse(const Transform &tf);

ALMATH_API double norm(const Quaternion &r);

ALMATH_API void normalize(Quaternion &r);

ALMATH_API Quaternion normalized(const Quaternion &r);

inline std::ostream& operator<<(std::ostream &o, const Quaternion &r)
{
  return o << "Quaternion(" << r.x << ", " << r.y << ", " << r.z << ", "
           << r.w << ")";
}

inline std::ostream& operator<<(std::ostream &o, const Vector3 &t)
{
  return o << "Vector3(" << t.x << ", " << t.y << ", " << t.z << ")";
}

inline std::ostream& operator<<(std::ostream &o, const Transform &tf)
{
  return o << "Transform(" << tf.rotation << ", " << tf.translation << ")";
}
}
}

#endif
