/*
 * Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */

#include <almath/types/aldisplacement.h>
#include <almath/tools/almath.h>

namespace AL {
  namespace Math {
    Displacement::Displacement() : P(), Q() {}

    Displacement::Displacement(
      const Position3D& pos3d,
      const Quaternion& quat) : P(pos3d), Q(quat) {}


    Displacement& Displacement::operator*=(const Displacement& pDisp)
    {
      P += Q * (pDisp.P);
      Q *= pDisp.Q;
      return *this;
    }

    bool Displacement::isNear(
      const Displacement& pDisp,
      const float         pEpsilon) const
    {
      return (P.isNear(pDisp.P, pEpsilon) &&
              Q.isNear(pDisp.Q, pEpsilon));
    }

    Displacement Displacement::operator*(const Displacement& pDisp)
    {
      Displacement copy = *this;
      copy *= pDisp;
      return copy;
    }
  }
}

