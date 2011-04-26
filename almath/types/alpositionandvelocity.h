/**
* Copyright (c) Aldebaran Robotics 2010 All Rights Reserved
*/

#pragma once

#ifndef _LIB_ALMATH_ALMATH_ALPOSITIONANDVELOCITY_H_
#define _LIB_ALMATH_ALMATH_ALPOSITIONANDVELOCITY_H_

namespace AL {
  namespace Math {

    /// <summary> Struct composed of a 1D position and a velocity </summary>
    struct PositionAndVelocity
    {
      float q, dq;

      PositionAndVelocity(
          const float pq  = 0.0f,
          const float pdq = 0.0f) :
          q(pq),
          dq(pdq) {}

      bool isNear(
        const PositionAndVelocity& pDat,
        const float&               pEpsilon=0.0001f) const;

    };

  }
}
#endif  // _LIB_ALMATH_ALMATH_ALPOSITIONANDVELOCITY_H_
