/*
** Author(s):
**  - Chris Kilner
**  - Cyrille Collette
**  - David Gouaillier
**
** Copyright (C) 2011 Aldebaran Robotics
*/

#pragma once

#ifndef _LIB_ALMATH_ALMATH_ALPOSITIONANDVELOCITY_H_
#define _LIB_ALMATH_ALMATH_ALPOSITIONANDVELOCITY_H_

namespace AL {
  namespace Math {

    /// <summary>
    /// Create and play with a PositionAndVelocity.
    ///
    /// A PositionAndVelocity is just defined by q and dq.
    /// </summary>
    /// \ingroup Types
    struct PositionAndVelocity
    {
      float q, dq;

      /// <summary>
      /// Create a PositionAndVelocity initialize with explicit value.
      /// </summary>
      /// <param name="pq"> the float value for q (default value = 0.0f) </param>
      /// <param name="pdq"> the float value for dq (default value = 0.0f) </param>
      PositionAndVelocity(
          const float pq  = 0.0f,
          const float pdq = 0.0f) :
          q(pq),
          dq(pdq) {}

      /// <summary>
      /// Check if the actual PositionAndVelocity is near the one
      /// give in argument.
      ///
      /// </summary>
      /// <param name="pDat2"> the second PositionAndVelocity </param>
      /// <param name="pEpsilon"> an optional epsilon distance (default = 0.0001f) </param>
      /// <returns>
      /// true if the difference of each float of the two PositionAndVelocity is less than pEpsilon
      /// </returns>
      bool isNear(
        const PositionAndVelocity& pDat2,
        const float&               pEpsilon=0.0001f) const;
    };

  }
}
#endif  // _LIB_ALMATH_ALMATH_ALPOSITIONANDVELOCITY_H_
