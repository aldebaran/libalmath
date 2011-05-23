/*
** Author(s):
**  - Chris Kilner
**  - Cyrille Collette
**  - David Gouaillier
**
** Copyright (C) 2011 Aldebaran Robotics
*/

#pragma once

#ifndef _LIB_ALMATH_ALMATH_ALPOSITION2D_H_
#define _LIB_ALMATH_ALMATH_ALPOSITION2D_H_

#include <vector>

namespace AL {
  namespace Math {

    /// <summary>
    /// Create and play with a Position2D.
    ///
    /// A positio2D is just defined by x and y.
    /// </summary>
    /// \ingroup Types
    struct Position2D
    {
      float x;
      float y;

      /// <summary>
      /// create a Position2D initialize with 0.0f.
      /// </summary>
      Position2D() : x(0.0f), y(0.0f) {}

      /// <summary>
      /// create a Position2D initialize with explicit value.
      /// </summary>
      /// <param name="pX"> the float value for x </param>
      /// <param name="pY"> the float value for y </par
      Position2D(float pX, float pY) : x(pX), y(pY) {}

      /// <summary>
      /// create a Position2D with an std::vector.
      /// </summary>
      /// <param name="pFloats">
      /// An std::vector<float> of size 2 for respectively :
      /// x and y
      /// </param>
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

      Position2D operator+ () const;
      Position2D operator- () const;

      Position2D& operator+= (const Position2D& pPos2);
      Position2D& operator-= (const Position2D& pPos2);

      bool operator!=(const Position2D& pPos2) const;

      bool operator==(const Position2D& pPos) const;

      Position2D operator* (float pM) const;
      Position2D operator/ (float pM) const;
      Position2D& operator*= (float pM);
      Position2D& operator/= (float pM);

      float distanceSquared(const Position2D& pPos) const;
      float distance(const Position2D& pPos) const;
      bool isNear(
        const Position2D& pPos,
        const float&      pEpsilon=0.0001f) const;
      float norm() const;
      Position2D normalize() const;
      float crossProduct(const Position2D& pPos) const;

      std::vector<float> toVector() const;
    };

    Position2D operator* (
      const float       pM,
      const Position2D& pPos1);

    /// <summary>
    /// compute the squared distance between two Position2D
    ///
    /// (pA.x-pB.x)²+(pA.y-pB-y)²
    /// </summary>
    /// <param name="pPos1"> the first Position2D </param>
    /// <param name="pPos2"> the second Position2D </param>
    /// <returns>
    /// the float squared distance between the two Position2D
    /// </returns>
    /// \ingroup Types
    float distanceSquared(
      const Position2D& pPos1,
      const Position2D& pPos2);

    /// <summary>
    /// compute the distance between two Position2D
    ///
    /// sqrt((pA.x-pB.x)²+(pA.y-pB-y)²)
    /// </summary>
    /// <param name="pPos1"> the first Position2D </param>
    /// <param name="pPos2"> the second Position2D </param>
    /// <returns>
    /// the float distance between the two Position2D
    /// </returns>
    /// \ingroup Types
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
