/**
* Copyright (c) Aldebaran Robotics 2010 All Rights Reserved
*/

#include <almath/types/alpositionandvelocity.h>
#include <cmath>

namespace AL {
  namespace Math {

  PositionAndVelocity::PositionAndVelocity(
      const float pq,
      const float pdq) :
      q(pq),
      dq(pdq) {}

    bool PositionAndVelocity::isNear(
      const PositionAndVelocity& pDat,
      const float&               pEpsilon) const
    {
      if (
        (fabsf(q - pDat.q) > pEpsilon) ||
        (fabsf(dq - pDat.dq) > pEpsilon))
      {
        return false;
      }
      else
      {
        return true;
      }
    }

  }
}
