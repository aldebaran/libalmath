/*
** Author(s):
**  - Chris Kilner
**  - Cyrille Collette
**  - David Gouaillier
**
** Copyright (C) 2011 Aldebaran Robotics
*/

#pragma once

#ifndef _LIB_ALMATH_ALMATH_ALINTERPOLATIONUTILS_H_
#define _LIB_ALMATH_ALMATH_ALINTERPOLATIONUTILS_H_

#include <almath/types/altransform.h>

namespace AL {
  namespace Math {

    void computeFinalTimeInterpolation(
      const float& pPeriod,
      float&       pFinalTime);

    /// <summary> Get Final Time (relative). </summary>
    float getTimeFinalJoint(
      const float& pPointInit,
      const float& pPointFinal,
      const float& pVelocityInit,
      const float& pVelocityFinal,
      const float& pVelocityMaxAbs,
      const float& pPeriod);

      float getTimeFinalCartesian(
        const AL::Math::Transform& pHInit,
        const AL::Math::Transform& pHFinal,
        const float&               pMaxVelocity,
        const float&               pPeriod);
  } // namespace Math
} // namespace AL
#endif  // _LIB_ALMATH_ALMATH_ALINTERPOLATIONUTILS_H_
