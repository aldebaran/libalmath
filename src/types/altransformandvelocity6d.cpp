/*
 * Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */

#include<almath/types/altransformandvelocity6d.h>

namespace AL {
  namespace Math {


    TransformAndVelocity6D::TransformAndVelocity6D(): T(), V() {}

    TransformAndVelocity6D::TransformAndVelocity6D(
        const AL::Math::Transform& pT,
        const AL::Math::Velocity6D& pV): T(pT), V(pV){}

    bool TransformAndVelocity6D::isNear(
        const TransformAndVelocity6D& pDat,
        const float&                  pEpsilon) const
    {
      return (T.isNear(pDat.T, pEpsilon) &&
              V.isNear(pDat.V, pEpsilon));
    }


  }
}
