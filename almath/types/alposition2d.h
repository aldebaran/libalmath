/**
* @author Chris Kilner
* Copyright (c) Aldebaran Robotics 2009 All Rights Reserved
*/

#pragma once

#ifndef _LIB_ALMATH_ALMATH_ALPOSITION2D_H_
#define _LIB_ALMATH_ALMATH_ALPOSITION2D_H_

#include <vector>

namespace AL {
  namespace Math {

    struct Position2D
    {
      float x, y;

      Position2D() : x(0.0f), y(0.0f) {}
      Position2D(float pX, float pY) : x(pX), y(pY) {}

      /**
      * CONSTRUCTOR: create a Position2D from an std vector.
      */
      Position2D (const std::vector<float>& pFloats)
      {
        if (pFloats.size() == 2)
        {
          x = pFloats[0];
          y = pFloats[1];
        }
        else
        {
          x = 0.0f;
          y = 0.0f;
        }
      }

      Position2D operator+ (const Position2D& pPos2) const;
      Position2D operator- (const Position2D& pPos2) const;

      Position2D& operator+= (const Position2D& pPos2);

      bool operator!=(const Position2D& pPos2) const;

      bool isNear(
        const Position2D& pPos,
        const float&      pEpsilon=0.0001f) const;

      bool operator==(const Position2D& pPos) const;

      Position2D operator* (float pM) const;
      Position2D operator/ (float pM) const;
      Position2D& operator*= (float pM);
      Position2D& operator/= (float pM);

      float distanceSquared(const Position2D& pPos) const;
      float distance(const Position2D& pPos) const;
      float norm() const;
      Position2D normalize() const;
      float crossProduct(const Position2D& pPos) const;

      std::vector<float> toVector() const;
    };

    Position2D operator* (
      const float       pM,
      const Position2D& pPos1);

    float distanceSquared(
      const Position2D& pPos1,
      const Position2D& pPos2);

    float distance(
      const Position2D& pPos1,
      const Position2D& pPos2);

    float norm(const Position2D& p);

    Position2D normalize(const Position2D& p);

    /*
    * Produit vectoriel de 2 vecteurs V1 et V2
    */
    float crossProduct(
      const Position2D& pPos1,
      const Position2D& pPos2);

    void crossProduct(
      const Position2D& pPos1,
      const Position2D& pPos2,
      float&            result);

  } // end namespace math
} // end namespace al
#endif  // _LIB_ALMATH_ALMATH_ALPOSITION2D_H_
