/**
 * Copyright 2016 Aldebaran. All rights reserved.
 */

#include <qi/geometry/geometry.hpp>
#include <almath/scenegraph/qigeometry.h>
#include <qi/anymodule.hpp>

namespace qi
{
namespace geometry
{
void register_geometry_module(qi::ModuleBuilder* mb)
{
  mb->advertiseMethod("makeVector3", &makeVector3);
  mb->advertiseMethod("makeQuaternion", &makeQuaternion);
  mb->advertiseMethod("makeTransform", &makeTransform);
  mb->advertiseMethod("inverse", &inverse);
  mb->advertiseMethod("multiply", &operator *);
  mb->advertiseMethod("norm", &norm);
  mb->advertiseMethod("normalized", &normalized);
}

QI_REGISTER_MODULE("geometry_module", &register_geometry_module)
}
}
