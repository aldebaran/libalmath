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

    /// <summary>
    /// Create and play with a Position3D.
    ///
    /// A Position3D is just defined by x, y and z.
    /// </summary>
    /// \ingroup Types
    struct Position3D {
      float x;
      float y;
      float z;

      /// <summary>
      /// create a Position3D initialize with 0.0f.
      /// </summary>
      Position3D() : x(0.0f), y(0.0f), z(0.0f) {}

      /// <summary>
      /// create a Position3D initialize with the same float.
      /// </summary>
      /// <param name="pInit"> the float value for each member </param>
      /// </summary>
      Position3D(float pInit) : x(pInit), y(pInit), z(pInit) {}

      /// <summary>
      /// create a Position3D initialize with explicit value.
      /// </summary>
      /// <param name="pX"> the float value for x </param>
      /// <param name="pY"> the float value for y </param>
      /// <param name="pZ"> the float value for z </param>
      Position3D(float pX, float pY, float pZ) : x(pX), y(pY), z(pZ) {}

      /// <summary>
      /// create a Positio3D with an std::vector.
      /// </summary>
      /// <param name="pFloats">
      /// An std::vector<float> of size 3 for respectively :
      /// x, y and z
      /// </param>
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

      Position3D operator* (float pM) const;
      Position3D operator/ (float pM) const;
      Position3D& operator*= (float pM);
      Position3D& operator/= (float pM);

      /// <summary>
      /// compute the squared distance between the actual
      /// Position3D and the one give in argument
      ///
      /// (pPos1.x-pPos2.x)²+(pPos1.y-pPos2-y)²+(pPos1.z-pPos2.z)²
      /// </summary>
      /// <param name="pPos2"> the second Position3D </param>
      /// <returns>
      /// the float squared distance between the two Position3D
      /// </returns>
      /// \ingroup Types
      float distanceSquared(const Position3D& pPos2) const;

      /// <summary>
      /// compute the distance between the actual
      /// Position3D and the one give in argument
      ///
      /// sqrt((pPos1.x-pPos2.x)²+(pPos1.y-pPos2-y)²+(pPos1.z-pPos2.z)²)
      /// </summary>
      /// <param name="pPos2"> the second Position3D </param>
      /// <returns>
      /// the float distance between the two Position3D
      /// </returns>
      /// \ingroup Types
      float distance(const Position3D& pPos2) const;

      /// <summary>
      /// check if the actual Position3D is Near the one
      /// give in argument.
      ///
      /// </summary>
      /// <param name="pPos2"> the second Position3D </param>
      /// <param name="pEpsilon"> an optionnal epsilon distance </param>
      /// <returns>
      /// true if the distance between the two Position3D is less than pEpsilon
      /// </returns>
      bool isNear(
        const Position3D& pPos2,
        const float&      pEpsilon=0.0001f) const;

      /// <summary>
      /// compute the norm of the actual Position3D
      ///
      /// sqrt((pPosx-pPos.x)²+(pPos.y-pPos-y)²+(pPos.z-pPos-z)²)
      /// </summary>
      /// <returns>
      /// the float norm of the Position3D
      /// </returns>
      /// \ingroup Types
      float norm() const;

      /// <summary>
      /// normalize the actual Position3D
      ///
      /// result = pPos/ norm(pPos)
      /// </summary>
      /// <returns>
      /// the Position3D normalized
      /// </returns>
      /// \ingroup Types
      Position3D normalize() const;

      /// <summary>
      /// compute the dot Product between the actual
      /// Position3D and the one give in argument
      ///
      /// result = (pPos1.x*pPos2.x + pPos1.y*pPos2.y + pPos1.z*pPos2.z)
      /// </summary>
      /// <param name="pPos2"> the second Position3D </param>
      /// <returns>
      /// the float dot product between the two Position3D
      /// </returns>
      float dotProduct(const Position3D& pPos2) const;

      /// <summary>
      /// compute the cross Product between the actual
      /// Position3D and the one give in argument
      ///
      /// pRes.x = pPos1.y*pPos2.z - pPos1.z*pPos2.y;
      /// pRes.y = pPos1.z*pPos2.x - pPos1.x*pPos2.z;
      /// pRes.z = pPos1.x*pPos2.y - pPos1.y*pPos2.x;
      /// </summary>
      /// <param name="pPos2"> the second Position3D </param>
      /// <returns>
      /// the Position3D cross product between the two Position3D
      /// </returns>
      Position3D crossProduct(const Position3D& pPos2) const;

      /// <summary>
      /// return the Position3D as a vector of float [x, y, z]
      /// </summary>
      std::vector<float> toVector() const;
    };

    // TODO : Need this ?
//    Position3D operator* (
//      const float       pM,
//      const Position3D& pPos1);

//    // Does it make sense to define 2.0 / Position3D() ??
//    Position3D operator/ (
//      const float       pM,
//      const Position3D& pPos1);

    /// <summary>
    /// compute the squared distance between two Position3D
    ///
    /// (pPos1.x-pPos2.x)²+(pPos1.y-pPos2-y)²+(pPos1.z-pPos2.z)²
    /// </summary>
    /// <param name="pPos1"> the first Position3D </param>
    /// <param name="pPos2"> the second Position3D </param>
    /// <returns>
    /// the float squared distance between the two Position3D
    /// </returns>
    /// \ingroup Types
    float distanceSquared(
      const Position3D& pPos1,
      const Position3D& pPos2);

    /// <summary>
    /// compute the distance between two Position3D
    ///
    /// sqrt((pPos1.x-pPos2.x)²+(pPos1.y-pPos2-y)²+(pPos1.z-pPos2.z)²)
    /// </summary>
    /// <param name="pPos1"> the first Position3D </param>
    /// <param name="pPos2"> the second Position3D </param>
    /// <returns>
    /// the float distance between the two Position3D
    /// </returns>
    /// \ingroup Types
    float distance(
      const Position3D& pPos1,
      const Position3D& pPos2);

    /// <summary>
    /// compute the norm of a Position3D
    ///
    /// sqrt((pPosx-pPos.x)²+(pPos.y-pPos-y)²+(pPos.z-pPos-z)²)
    /// </summary>
    /// <param name="pPos"> the given Position3D </param>
    /// <returns>
    /// the float norm of the given Position3D
    /// </returns>
    /// \ingroup Types
    float norm(const Position3D& pPos);

    /// <summary>
    /// normalize a Position3D
    ///
    /// pRes = pPos/ norm(pPos)
    /// </summary>
    /// <param name="pPos"> the given Position3D </param>
    /// <returns>
    /// the given Position3D normalized
    /// </returns>
    /// \ingroup Types
    Position3D normalize(const Position3D& pPos);

    /// <summary>
    /// compute the dot Product between two Position3D
    ///
    /// result = (pPos1.x*pPos2.x + pPos1.y*pPos2.y + pPos1.z*pPos2.z)
    /// </summary>
    /// <param name="pPos1"> the first Position3D </param>
    /// <param name="pPos2"> the second Position3D </param>
    /// <returns>
    /// the float dot product between the two Position3D
    /// </returns>
    float dotProduct(
      const Position3D& pPos1,
      const Position3D& pPos2);

    /// <summary>
    /// compute the cross Product between two Position3D
    ///
    /// pRes.x = pPos1.y*pPos2.z - pPos1.z*pPos2.y;
    /// pRes.y = pPos1.z*pPos2.x - pPos1.x*pPos2.z;
    /// pRes.z = pPos1.x*pPos2.y - pPos1.y*pPos2.x;
    /// </summary>
    /// <param name="pPos1"> the first Position3D </param>
    /// <param name="pPos2"> the second Position3D </param>
    /// <returns>
    /// the Position3D cross product between the two Position3D
    /// </returns>
    Position3D crossProduct(
      const Position3D& pPos1,
      const Position3D& pPos2);

    /// <summary>
    /// compute the cross Product between two Position3D
    ///
    /// pRes.x = pPos1.y*pPos2.z - pPos1.z*pPos2.y;
    /// pRes.y = pPos1.z*pPos2.x - pPos1.x*pPos2.z;
    /// pRes.z = pPos1.x*pPos2.y - pPos1.y*pPos2.x;
    /// </summary>
    /// <param name="pPos1"> the first Position3D </param>
    /// <param name="pPos2"> the second Position3D </param>
    /// <param name="pRes">
    /// the Position3D cross product between the two Position3D </param>
    void crossProduct(
      const Position3D& pPos1,
      const Position3D& pPos2,
      Position3D&       pRes);

  } // end namespace math
} // end namespace al
#endif  // _LIB_ALMATH_ALMATH_ALPOSITION3D_H_
