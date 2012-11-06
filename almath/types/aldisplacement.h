/*
 * Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */


#pragma once
#ifndef ALDISPLACEMENT_H
#define ALDISPLACEMENT_H

#include <almath/types/alposition3d.h>
#include <almath/types/alquaternion.h>

namespace AL {
  namespace Math {

    /// <summary>
    /// Struct composed of a Position3D and a Quaternion
    /// </summary>
    struct Displacement
    {
      /// <summary> Translation of the Displacement </summary>
      Position3D P;
      /// <summary> Rotation of the Displacement </summary>
      Quaternion Q;

      /// <summary>
      /// Check if the current Displacement is Near the one
      /// given in argument.
      /// </summary>
      /// <param name="pDisp2"> the second Displacement</param>
      /// <param name="pEpsilon"> an optional epsilon distance</param>
      /// <returns>
      /// true if the distance between the two Displacement is less than pEpsilon
      /// </returns>
      bool isNear(
        const Displacement&          pDisp2,
        const float                  pEpsilon=0.0001f) const;

    };

  }
}

#endif // ALDISPLACEMENT_H
