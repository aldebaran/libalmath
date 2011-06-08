/*
** Author(s):
**  - Chris Kilner
**  - Cyrille Collette
**  - David Gouaillier
**
** Copyright (C) 2011 Aldebaran Robotics
*/

#pragma once

#ifndef _LIB_ALMATH_ALMATH_ALTRANSFORMANDVELOCITY6D_H_
#define _LIB_ALMATH_ALMATH_ALTRANSFORMANDVELOCITY6D_H_

#include<almath/types/altransform.h>
#include<almath/types/alvelocity6d.h>

namespace AL {
  namespace Math {

    /// <summary> Struct composed of an Transform and a Velocity6D</summary>
    struct TransformAndVelocity6D
    {
      AL::Math::Transform  H;
      AL::Math::Velocity6D V;

      bool isNear(
        const TransformAndVelocity6D& pDat,
        const float&                  pEpsilon=0.0001f) const;

    };

  }
}
#endif  // _LIB_ALMATH_ALMATH_ALTRANSFORMANDVELOCITY6D_H_

