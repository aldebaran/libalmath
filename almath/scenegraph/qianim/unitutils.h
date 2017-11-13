/*
 * Copyright (c) 2017 Softbank Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */

#ifndef LIBALMATH_ALMATH_SCENEGRAPH_QIANIM_UTILS_H
#define LIBALMATH_ALMATH_SCENEGRAPH_QIANIM_UTILS_H

#include <almath/api.h>
#include <almath/scenegraph/qianim/unit.h>
#include <boost/math/constants/constants.hpp>

namespace AL {
namespace qianim {

// example usage:
// float val_from = ...;
// auto val_to = getUnitConversionFactor<float>(to, from) * val_from;
template <typename Scalar>
typename std::enable_if<std::is_floating_point<Scalar>::value, Scalar>::type
getUnitConversionFactor(Unit to, Unit from) {
  if (from == to)
    return static_cast<Scalar>(1);
  if (to == Unit::radian && from == Unit::degree)
    return boost::math::constants::pi<Scalar>()/180;
  if (to == Unit::degree && from == Unit::radian)
    return 180/boost::math::constants::pi<Scalar>();
  // units are not commensurable
  throw std::invalid_argument("units are not of the same dimension");
}

// example usage:
// float val_from = ...;
// auto val_si = getSIConversionFactor<float>(from) * val_from;
template <typename Scalar>
typename std::enable_if<std::is_floating_point<Scalar>::value, Scalar>::type
getSIConversionFactor(Unit from) {
  return (from == Unit::degree) ? boost::math::constants::pi<Scalar>()/180
                                    : static_cast<Scalar>(1);
}
}
}

#endif
