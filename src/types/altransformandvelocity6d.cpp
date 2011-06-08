/**
* Copyright (c) Aldebaran Robotics 2010 All Rights Reserved
*/

#include<almath/types/altransformandvelocity6d.h>

namespace AL {
  namespace Math {


    bool TransformAndVelocity6D::isNear(
      const TransformAndVelocity6D& pDat,
      const float&                  pEpsilon) const
    {

      if (
        H.isNear(pDat.H, pEpsilon) &&
        V.isNear(pDat.V, pEpsilon))
      {
        return true;
      }
      else
      {
        return false;
      }
    }


  }
}
