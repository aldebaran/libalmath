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
    // <summary> Compute the best position(orientation) of the foot to avoid collision. </summary>
    // <param name="pLFootBoundingBoxe"> vector<Pose2D> of the left footBoundingBoxe. </param>
    // <param name="pRFootBoundingBoxe"> vector<Pose2D> of the right footBoundingBoxe. </param>
    // <param name="pMove">              Pose2D the initial move. </param>
    // <param name="pIsLeftSupport">     Bool true if left is the support leg. </param>
    // <returns> Pose2D, the best position without collsion. </returns>
    const bool avoidFootCollision(
      const std::vector<AL::Math::Pose2D>&  pLFootBoundingBoxe,
      const std::vector<AL::Math::Pose2D>&  pRFootBoundingBoxe,
      const bool&                           pIsLeftSupport,
      AL::Math::Pose2D&                     pMove );
  } // namespace Math

} // namespace AL

#endif  // _LIB_ALMATH_ALMATH_AVOIDFOOTCOLLISION_H_

