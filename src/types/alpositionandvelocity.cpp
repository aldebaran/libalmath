/*
 * Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
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
      return (std::abs(q - pDat.q) <= pEpsilon &&
              std::abs(dq - pDat.dq) <= pEpsilon);
    }

    void PositionAndVelocity::toVector(std::vector<float>& pReturnVector) const
    {
      pReturnVector.resize(2);
      pReturnVector[0] = q;
      pReturnVector[1] = dq;
    }

    std::vector<float> PositionAndVelocity::toVector(void) const
    {
      std::vector<float> returnVector(2, 0.0f);
      this->toVector(returnVector);
      return returnVector;
    }

  }
}
