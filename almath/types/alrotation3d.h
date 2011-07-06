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

  /// <summary>
  /// A Rotation3D give 3 composed angle in radian
  /// </summary>
  /// \ingroup Types
    struct Rotation3D {
      float wx, wy, wz;

      /// <summary>
      /// create a Rotation3D initialize with 0.0f.
      /// </summary>
      Rotation3D(): wx(0.0f),
        wy(0.0f),
        wz(0.0f) {}

      /// <summary>
      /// create a Rotation3D initialize with the same float.
      /// </summary>
      /// <param name="pInit"> the float value for each member </param>
      /// </summary>
      explicit Rotation3D(float pInit): wx(pInit),
        wy(pInit),
        wz(pInit) {}

      /// <summary>
      /// create a Rotation3D initialize with explicit value.
      /// </summary>
      /// <param name="pWx"> the float value for wx </param>
      /// <param name="pWy"> the float value for wy </param>
      /// <param name="pWz"> the float value for wz </param>
      Rotation3D(
        float pWx,
        float pWy,
        float pWz): wx(pWx),
        wy(pWy),
        wz(pWz) {}

      /// <summary>
      /// create a Rotation3D with an std::vector.
      /// </summary>
      /// <param name="pFloats">
      /// An std::vector<float> of size 3 for respectively:
      /// wx, wy and wz
      /// </param>
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

      /// <summary>
      /// overloading of operator + for Rotation3D.
      /// </summary>
      /// <param name="pRot2"> the second Rotation3D </param>
      Rotation3D operator+ (const Rotation3D& pRot2) const;

      /// <summary>
      /// overloading of operator - for Rotation3D.
      /// </summary>
      /// <param name="pRot2"> the second Rotation3D </param>
      Rotation3D operator- (const Rotation3D& pRot2) const;

      /// <summary>
      /// overloading of operator += for Rotation3D.
      /// </summary>
      /// <param name="pRot2"> the second Rotation3D </param>
      Rotation3D& operator+= (const Rotation3D& pRot2);

      /// <summary>
      /// overloading of operator -= for Rotation3D.
      /// </summary>
      /// <param name="pRot2"> the second Rotation3D </param>
      Rotation3D& operator-= (const Rotation3D& pRot2);

      /// <summary>
      /// overloading of operator == for Rotation3D.
      /// </summary>
      /// <param name="pRot2"> the second Rotation3D </param>
      bool operator== (const Rotation3D& pRot2) const;

      /// <summary>
      /// overloading of operator != for Rotation3D.
      /// </summary>
      /// <param name="pRot2"> the second Rotation3D </param>
      bool operator!= (const Rotation3D& pRot2) const;

      /// <summary>
      /// overloading of operator * for Rotation3D.
      /// </summary>
      /// <param name="pVal"> the float factor </param>
      Rotation3D operator* (const float pVal) const;

      /// <summary>
      /// overloading of operator / for Rotation3D.
      /// </summary>
      /// <param name="pVal"> the float factor </param>
      Rotation3D operator/ (const float pVal) const;

      /// <summary>
      /// overloading of operator *= for Rotation3D.
      /// </summary>
      /// <param name="pVal"> the float factor </param>
      Rotation3D& operator*= (const float pVal);

      /// <summary>
      /// overloading of operator /= for Rotation3D.
      /// </summary>
      /// <param name="pVal"> the float factor </param>
      Rotation3D& operator/= (const float pVal);

      /// <summary>
      /// check if the actual Rotation3D is Near the one
      /// give in argument.
      ///
      /// </summary>
      /// <param name="pRot2"> the second Rotation3D </param>
      /// <param name="pEpsilon"> an optional epsilon distance </param>
      /// <returns>
      /// true if the difference of each float of the two Rotation3D is less than pEpsilon
      /// </returns>
      bool isNear(
        const Rotation3D& pRot2,
        const float&      pEpsilon=0.0001f) const;

      /// <summary>
      /// compute the norm of the actual Position6D
      ///
      /// \f$\sqrt{pRot.wx^2 + pRot.wy^2 + pRot.wz^2}\f$
      /// </summary>
      /// <returns>
      /// the float norm of the Position6D
      /// </returns>
      float norm() const;

      /// <summary>
      /// return the Rotation3D as a vector of float [wx, wy, wz]
      /// </summary>
      std::vector<float> toVector() const;
    };

    /// <summary>
    /// compute the norm of a Rotation3D
    ///
    /// \f$\sqrt{pRot.wx^2 + pRot.wy^2 + pRot.wz^2}\f$
    /// </summary>
    /// <param name="pRot"> the given Rotation3D </param>
    /// <returns>
    /// the float norm of the given Rotation3D
    /// </returns>
    /// \ingroup Types
    float norm(const Rotation3D& pRot);

  } // end namespace Math
} // end namespace AL
#endif  // _LIB_ALMATH_ALMATH_ALROTATION3D_H_
