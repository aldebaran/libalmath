/*
** Author(s):
**  - Chris Kilner
**  - Cyrille Collette
**  - David Gouaillier
**
** Copyright (C) 2011 Aldebaran Robotics
*/

#pragma once

#ifndef _LIB_ALMATH_ALMATH_ALDUBINSCURVE_H_
#define _LIB_ALMATH_ALMATH_ALDUBINSCURVE_H_

#include <almath/types/alpose2d.h>
#include <vector>

namespace AL {
  namespace Math {

    /// <summary> Get the dubins solutions. </summary>
    /// <param name="pTargetPose">   The target pose. </param>
    /// <param name="pCircleRadius"> The circle radius. </param>
    /// <return> The dubins solution. </return>
    /// \ingroup Tools
    std::vector<AL::Math::Pose2D> getDubinsSolutions(
      const AL::Math::Pose2D& pTargetPose,
      const float             pCircleRadius);

  }
}
#endif  // _LIB_ALMATH_ALMATH_ALDUBINSCURVE_H_
