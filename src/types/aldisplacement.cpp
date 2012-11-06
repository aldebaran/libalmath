/*
 * Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */

#include<almath/types/aldisplacement.h>

namespace AL {
  namespace Math {
    bool Displacement::isNear(
      const Displacement& pDisp,
      const float         pEpsilon) const
    {
      if (
        P.isNear(pDisp.P, pEpsilon) &&
        Q.isNear(pDisp.Q, pEpsilon))
      {
        return true;
      }
      return false;
    }
  }
}

