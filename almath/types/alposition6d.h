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

    /// <summary>
    /// Create and play with a Position6D.
    ///
    /// A Position6D is just defined by x, y, z, wx, wy and wz.
    /// </summary>
    /// \ingroup Types
    struct Position6D {
      float x, y, z, wx, wy, wz;

      /// <summary>
      /// create a Position6D initialize with 0.0f.
      /// </summary>
      Position6D() : x(0.0f),
        y(0.0f),
        z(0.0f),
        wx(0.0f),
        wy(0.0f),
        wz(0.0f) {}

      /// <summary>
      /// create a Position6D initialize with the same float.
      /// </summary>
      /// <param name="pInit"> the float value for each member </param>
      /// </summary>
      explicit Position6D(float pInit) : x(pInit),
        y(pInit),
        z(pInit),
        wx(pInit),
        wy(pInit),
        wz(pInit) {}

      /// <summary>
      /// create a Position6D initialize with explicit value.
      /// </summary>
      /// <param name="pX"> the float value for x </param>
      /// <param name="pY"> the float value for y </param>
      /// <param name="pZ"> the float value for z </param>
      /// <param name="pWx"> the float value for wx </param>
      /// <param name="pWy"> the float value for wy </param>
      /// <param name="pWz"> the float value for wz </param>
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

      /// <summary>
      /// create a Positio6D with an std::vector.
      /// </summary>
      /// <param name="pFloats">
      /// An std::vector<float> of size 6 for respectively:
      /// x, y, z, wx, wy and wz
      /// </param>
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

      Position6D operator* (float pM) const;
      Position6D operator/ (float pM) const;
      Position6D& operator*= (float pM);
      Position6D& operator/= (float pM);

      /// <summary>
      /// check if the actual Position6D is Near the one
      /// give in argument.
      ///
      /// </summary>
      /// <param name="pPos2"> the second Position6D </param>
      /// <param name="pEpsilon"> an optional epsilon distance </param>
      /// <returns>
      /// true if the difference of each float of the two Position6D is less than pEpsilon
      /// </returns>
      bool isNear(
        const Position6D& pPos2,
        const float&      pEpsilon=0.0001f) const;

      /// <summary>
      /// compute the squared distance of translation part (x, y and z)
      /// between the actual Position6D and the one give in argument
      ///
      /// (pPos1.x-pPos2.x)²+(pPos1.y-pPos2.y)²+(pPos1.z-pPos2.z)²
      /// </summary>
      /// <param name="pPos2"> the second Position6D </param>
      /// <returns>
      /// the float squared distance between the two Position6D
      /// </returns>
      float distanceSquared(const Position6D& pPos2) const;

      /// <summary>
      /// compute the distance of translation part (x, y and z) between the actual
      /// Position6D and the one give in argument
      ///
      /// sqrt((pPos1.x-pPos2.x)²+(pPos1.y-pPos2.y)²+(pPos1.z-pPos2.z)²)
      /// </summary>
      /// <param name="pPos2"> the second Position6D </param>
      /// <returns>
      /// the float distance between the two Position6D
      /// </returns>
      float distance(const Position6D& pPos2) const;

      /// <summary>
      /// compute the norm of the actual Position6D
      ///
      /// sqrt(pPos.x² + pPos.y² + pPos.z² + pPos.wx² + pPos.wy² + pPos.wz²)
      /// </summary>
      /// <returns>
      /// the float norm of the Position6D
      /// </returns>
      float norm() const;

      /// <summary>
      /// return the Position6D as a vector of float [x, y, z, wx, wy, wz]
      /// </summary>
      std::vector<float> toVector () const;
    }; // end struct


    /// <summary>
    /// compute the squared distance of translation part (x, y and z)
    /// between two Position6D
    ///
    /// (pPos1.x-pPos2.x)²+(pPos1.y-pPos2.y)²+(pPos1.z-pPos2.z)²
    /// </summary>
    /// <param name="pPos1"> the first Position6D </param>
    /// <param name="pPos2"> the second Position6D </param>
    /// <returns>
    /// the float squared distance between the two Position6D
    /// </returns>
    /// \ingroup Types
    float distanceSquared(
      const Position6D& pPos1,
      const Position6D& pPos2);


    /// <summary>
    /// compute the distance of translation part (x, y and z) between two Position6D
    ///
    /// sqrt((pPos1.x-pPos2.x)²+(pPos1.y-pPos2.y)²+(pPos1.z-pPos2.z)²)
    /// </summary>
    /// <param name="pPos1"> the first Position6D </param>
    /// <param name="pPos2"> the second Position6D </param>
    /// <returns>
    /// the float distance between the two Position6D
    /// </returns>
    /// \ingroup Types
    float distance(
      const Position6D& pPos1,
      const Position6D& pPos2);

    /// <summary>
    /// compute the norm of a Position6D
    ///
    /// sqrt(pPos.x² + pPos.y² + pPos.z² + pPos.wx² + pPos.wy² + pPos.wz²)
    /// </summary>
    /// <param name="pPos"> the given Position6D </param>
    /// <returns>
    /// the float norm of the given Position6D
    /// </returns>
    /// \ingroup Types
    float norm(const Position6D& pPos);

    /// <summary>
    /// normalize a Position6D
    ///
    /// pRes = pPos/ norm(pPos)
    /// </summary>
    /// <param name="pPos"> the given Position6D </param>
    /// <returns>
    /// the given Position6D normalized
    /// </returns>
    /// \ingroup Types
    Position6D normalize(const Position6D& pPos);
  } // end namespace math
} // end namespace al
#endif  // _LIB_ALMATH_ALMATH_ALPOSITION6D_H_
