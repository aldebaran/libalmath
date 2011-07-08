/*
** Author(s):
**  - Chris Kilner
**  - Cyrille Collette
**  - David Gouaillier
**
** Copyright (C) 2011 Aldebaran Robotics
*/

#pragma once

#ifndef _LIB_ALMATH_ALMATH_AVOIDFOOTCOLLISION_H_
#define _LIB_ALMATH_ALMATH_AVOIDFOOTCOLLISION_H_

#include <almath/types/alpose2d.h>
#include <vector>

namespace AL {
namespace Math {

/// <summary>
/// Compute the best position(orientation) of the foot to avoid collision.
/// </summary>
/// <param name="pLFootBoundingBoxe"> vector<Pose2D> of the left footBoundingBox.</param>
/// <param name="pRFootBoundingBoxe"> vector<Pose2D> of the right footBoundingBox.</param>
/// <param name="pIsLeftSupport">     Bool true if left is the support leg. </param>
/// <param name="pMove">              the desired and return Pose2D. </param>
/// <returns>
/// true if pMove is clamped.
/// </returns>
/// \ingroup Tools
const bool avoidFootCollision(
  const std::vector<Pose2D>&  pLFootBoundingBox,
  const std::vector<Pose2D>&  pRFootBoundingBox,
  const bool&                 pIsLeftSupport,
  Pose2D&                     pMove);

} // namespace Math
} // namespace AL

#endif  // _LIB_ALMATH_ALMATH_AVOIDFOOTCOLLISION_H_

