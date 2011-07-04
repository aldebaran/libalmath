/*
** Author(s):
**  - Chris Kilner
**  - Cyrille Collette
**  - David Gouaillier
**
** Copyright (C) 2011 Aldebaran Robotics
*/

#pragma once

#ifndef _LIB_ALMATH_ALMATH_ALROTATION3D_H_
#define _LIB_ALMATH_ALMATH_ALROTATION3D_H_

#include <vector>

namespace AL {
  namespace Math {

    struct Rotation3D {
      float wx, wy, wz;

      Rotation3D(): wx(0.0f),
        wy(0.0f),
        wz(0.0f) {}

      explicit Rotation3D(float pInit): wx(pInit),
        wy(pInit),
        wz(pInit) {}

      Rotation3D(
        float pWx,
        float pWy,
        float pWz): wx(pWx),
        wy(pWy),
        wz(pWz) {}

      /**
      * CONSTRUCTOR: create a Rotation3D from an std vector.
      */
      Rotation3D (const std::vector<float>& pFloats)
      {
        if (pFloats.size() == 3)
        {
          wx = pFloats[0];
          wy = pFloats[1];
          wz = pFloats[2];
        }
        else
        {
          wx = 0.0f;
          wy = 0.0f;
          wz = 0.0f;
        }
      }

      Rotation3D operator+ (const Rotation3D& pRot2) const;
      Rotation3D operator- (const Rotation3D& pRot2) const;

      Rotation3D& operator+= (const Rotation3D& pRot2);
      Rotation3D& operator-= (const Rotation3D& pRot2);

      bool operator== (const Rotation3D& pRot2) const;
      bool operator!= (const Rotation3D& pRot2) const;

      Rotation3D operator* (const float pVal) const;
      Rotation3D operator/ (const float pVal) const;
      Rotation3D& operator*= (const float pVal);
      Rotation3D& operator/= (const float pVal);

      bool isNear(
        const Rotation3D& pRot,
        const float&      pEpsilon=0.0001f) const;

      float norm() const;

      std::vector<float> toVector() const;

    };

    float norm(const Rotation3D& p);

  } // end namespace Math
} // end namespace AL
#endif  // _LIB_ALMATH_ALMATH_ALROTATION3D_H_
