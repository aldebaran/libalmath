/*
** Author(s):
**  - Chris Kilner
**  - Cyrille Collette
**  - David Gouaillier
**
** Copyright (C) 2011 Aldebaran Robotics
*/

#pragma once

#ifndef _LIB_ALMATH_ALMATH_ALPOSITION6D_H_
#define _LIB_ALMATH_ALMATH_ALPOSITION6D_H_

#include <vector>

namespace AL {
  namespace Math {

    struct Position6D {
      float x, y, z, wx, wy, wz;

      Position6D() : x(0.0f),
        y(0.0f),
        z(0.0f),
        wx(0.0f),
        wy(0.0f),
        wz(0.0f) {}

      explicit Position6D(float pInit) : x(pInit),
        y(pInit),
        z(pInit),
        wx(pInit),
        wy(pInit),
        wz(pInit) {}

      Position6D(
        float pX,
        float pY,
        float pZ,
        float pWx,
        float pWy,
        float pWz) : x(pX),
        y(pY),
        z(pZ),
        wx(pWx),
        wy(pWy),
        wz(pWz) {}

      /**
      * CONSTRUCTOR: create a Position6D from an std vector.
      */
      Position6D(const std::vector<float>& pFloats)
      {
        if (pFloats.size() == 6)
        {
          x = pFloats[0];
          y = pFloats[1];
          z = pFloats[2];

          wx = pFloats[3];
          wy = pFloats[4];
          wz = pFloats[5];
        }
        else
        {
          x = 0.0f;
          y = 0.0f;
          z = 0.0f;

          wx = 0.0f;
          wy = 0.0f;
          wz = 0.0f;
        }
      }

      Position6D operator+ (const Position6D& pPos2) const;
      Position6D operator- (const Position6D& pPos2) const;

      Position6D operator+ () const;
      Position6D operator- () const;

      Position6D& operator+= (const Position6D& pPos2);
      Position6D& operator-= (const Position6D& pPos2);

      bool isNear(
        const Position6D& pPos,
        const float&      pEpsilon=0.0001f) const;

      Position6D operator* (float pM) const;
      Position6D operator/ (float pM) const;
      Position6D& operator*= (float pM);
      Position6D& operator/= (float pM);

      float distance(const Position6D& pPos) const;
      float distanceSquared(const Position6D& pPos) const;
      float norm() const;

      std::vector<float> toVector () const;

    }; // end struct


    float distanceSquared(
      const Position6D& pPos1,
      const Position6D& pPos2);

    float distance(
      const Position6D& pPos1,
      const Position6D& pPos2);

    float norm(const Position6D& p);

    Position6D normalize(const Position6D& p);
  } // end namespace math
} // end namespace al
#endif  // _LIB_ALMATH_ALMATH_ALPOSITION6D_H_
