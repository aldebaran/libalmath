/**
 * Copyright 2016 Aldebaran. All rights reserved.
 */

#ifndef LIB_ALMATH_GEOMETRY_GEOMETRY_H
#define LIB_ALMATH_GEOMETRY_GEOMETRY_H

#include <qi/anyobject.hpp>

namespace qi
{
namespace geometry
{
struct Vector3 {
    Vector3() : Vector3 {0., 0., 0.} {}
    Vector3(double x, double y, double z) : x(x), y(y), z(z) {}

    double x, y, z;
};

struct Quaternion {
    Quaternion() : Quaternion {0., 0., 0., 1.} {}
    Quaternion(double x, double y, double z, double w) :
      x(x), y(y), z(z), w(w) {}

    double x, y, z, w;
};

struct Transform {
    Transform() {}
    Transform(const Quaternion &r, const Vector3 &t) : r(r), t(t) {}

  Quaternion r;
  Vector3 t;
};

struct TransformTime {
    Transform transform;
    qi::ClockTimePoint time;
};

bool isNear(const Transform &lhs, const Transform &rhs, double epsilon);

Transform operator*(const Transform &lhs, const Transform &rhs);

Transform inverse(const Transform &tf);

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
  return o << "Transform(" << tf.r << ", " << tf.t << ")";
}
}
}

QI_TYPE_STRUCT(qi::geometry::Vector3, x, y, z)
QI_TYPE_STRUCT(qi::geometry::Quaternion, x, y, z, w)
QI_TYPE_STRUCT(qi::geometry::Transform, r, t)
QI_TYPE_STRUCT(qi::geometry::TransformTime, transform, time)

#endif
