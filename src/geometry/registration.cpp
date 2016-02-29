/**
 * Copyright 2016 Aldebaran. All rights reserved.
 */

#include <almath/geometry/geometry.hpp>
#include <qi/anymodule.hpp>

namespace qi
{
namespace sdk
{
Vector3 vector3(double x, double y, double z)
{
  return Vector3(x, y, z);
}

Quaternion quaternion(double x, double y, double z, double w)
{
  return Quaternion(x, y, z, w);
}

Transform transform(const Quaternion &r, const Vector3 &t)
{
  return Transform(r, t);
}

void register_geometry_module(qi::ModuleBuilder* mb)
{
  mb->advertiseMethod("vector3", &vector3);
  mb->advertiseMethod("quaternion", &quaternion);
  mb->advertiseMethod("transform", &transform);
  mb->advertiseMethod("inverse", &inverse);
  mb->advertiseMethod("multiply", &operator *);
}

QI_REGISTER_MODULE("geometry_module", &register_geometry_module)
}
}
