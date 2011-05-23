/*
** Author(s):
**  - Chris Kilner
**  - Cyrille Collette
**  - David Gouaillier
**
** Copyright (C) 2011 Aldebaran Robotics
*/

#pragma once

#ifndef _LIB_ALMATH_ALMATH_ALPOSITION3D_H_
#define _LIB_ALMATH_ALMATH_ALPOSITION3D_H_

#include <vector>

namespace AL {
  namespace Math {

    struct Position3D {
      float x, y, z;

      Position3D() : x(0.0f), y(0.0f), z(0.0f) {}
      explicit Position3D(float pInit) : x(pInit), y(pInit), z(pInit) {}
      Position3D(float pX, float pY, float pZ) : x(pX), y(pY), z(pZ) {}

      /**
      * CONSTRUCTOR: create a Position3D from an std vector.
      */
      Position3D (const std::vector<float>& pFloats)
      {
        if (pFloats.size() == 3)
        {
          x = pFloats[0];
          y = pFloats[1];
          z = pFloats[2];
        }
        else
        {
          x = 0.0f;
          y = 0.0f;
          z = 0.0f;
        }
      }

      Position3D operator+ (const Position3D& pPos2) const;
      Position3D operator- (const Position3D& pPos2) const;

      Position3D operator+ () const;
      Position3D operator- () const;

      Position3D& operator+= (const Position3D& pPos2);
      Position3D& operator-= (const Position3D& pPos2);

      bool operator== (const Position3D& pPos2) const;
      bool operator!= (const Position3D& pPos2) const;

      bool isNear(
        const Position3D& pPos,
        const float&      pEpsilon=0.0001f) const;

      Position3D operator* (float pM) const;
      Position3D operator/ (float pM) const;
      Position3D& operator*= (float pM);
      Position3D& operator/= (float pM);

      float distanceSquared(const Position3D& pPos) const;
      float distance(const Position3D& pPos2) const;
      float norm() const;
      Position3D normalize() const;
      float dotProduct(const Position3D& p) const;
      Position3D crossProduct(const Position3D& p) const;

      std::vector<float> toVector() const;
    };

    Position3D operator* (
      const float       pM,
      const Position3D& pPos1);

    // Does it make sense to define 2.0 / Position3D() ??
    Position3D operator/ (
      const float       pM,
      const Position3D& pPos1);

    float distanceSquared(
      const Position3D& pPos1,
      const Position3D& pPos2);

    float distance(
      const Position3D& pPos1,
      const Position3D& pPos2);

    float norm(const Position3D& p);

    Position3D normalize(const Position3D& p);

    float dotProduct(
      const Position3D& p1,
      const Position3D& p2);

    void crossProduct(
      const Position3D& p1,
      const Position3D& p2,
      Position3D&       pRes);

    Position3D crossProduct(
      const Position3D& p1,
      const Position3D& p2);

  } // end namespace math
} // end namespace al
#endif  // _LIB_ALMATH_ALMATH_ALPOSITION3D_H_
