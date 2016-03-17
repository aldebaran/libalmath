/**
 * Copyright 2016 Aldebaran. All rights reserved.
 */

#include <almath/geometry/tools.hpp>
#include <qi/anymodule.hpp>

namespace qi
{
namespace geometry
{
void register_geometry_module(qi::ModuleBuilder* mb)
{
  mb->advertiseMethod("make_vector3", &make_vector3);
  mb->advertiseMethod("make_quaternion", &make_quaternion);
  mb->advertiseMethod("make_transform", &make_transform);
  mb->advertiseMethod("inverse", &inverse);
  mb->advertiseMethod("multiply", &operator *);
  mb->advertiseMethod("norm", &norm);
  mb->advertiseMethod("normalized", &normalized);
}

QI_REGISTER_MODULE("geometry_module", &register_geometry_module)
}
}
