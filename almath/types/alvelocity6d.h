/*
** Author(s):
**  - Chris Kilner
**  - Cyrille Collette
**  - David Gouaillier
**
** Copyright (C) 2011 Aldebaran Robotics
*/

#pragma once

#ifndef _LIB_ALMATH_ALMATH_ALVELOCITY6D_H_
#define _LIB_ALMATH_ALMATH_ALVELOCITY6D_H_

#include <vector>

namespace AL {
  namespace Math {

    /// <summary>
    /// Create and play with a Velocity6D.
    ///
    /// A Position3D is just defined by xd, yd, zd, wxd, wyd and wzd.
    /// </summary>
    /// \ingroup Types
    struct Velocity6D {
      float xd, yd, zd, wxd, wyd, wzd;

      /// <summary>
      /// create a Velocity6D initialize with 0.0f.
      /// </summary>
      Velocity6D():
        xd(0.0f),
        yd(0.0f),
        zd(0.0f),
        wxd(0.0f),
        wyd(0.0f),
        wzd(0.0f) {}

      /// <summary>
      /// create a Velocity6D initialize with the same float.
      /// </summary>
      /// <param name="pInit"> the float value for each member </param>
      /// </summary>
      explicit Velocity6D(float pInit):
        xd(pInit),
        yd(pInit),
        zd(pInit),
        wxd(pInit),
        wyd(pInit),
        wzd(pInit) {}

      /// <summary>
      /// create a Velocity6D initialize with explicit value.
      /// </summary>
      /// <param name="pXd"> the float value for xd </param>
      /// <param name="pYd"> the float value for yd </param>
      /// <param name="pZd"> the float value for zd </param>
      /// <param name="pWxd"> the float value for wxd </param>
      /// <param name="pWyd"> the float value for wyd </param>
      /// <param name="pWzd"> the float value for wzd </param>
      Velocity6D(
        float pXd,
        float pYd,
        float pZd,
        float pWxd,
        float pWyd,
        float pWzd):
        xd(pXd),
        yd(pYd),
        zd(pZd),
        wxd(pWxd),
        wyd(pWyd),
        wzd(pWzd) {}

      /// <summary>
      /// create a Positio3D with an std::vector.
      /// </summary>
      /// <param name="pFloats">
      /// An std::vector<float> of size 3 for respectively:
      /// xd, yd, zd, wxd, wyd and wzd
      /// </param>
      Velocity6D(const std::vector<float>& pFloats)
      {
        if (pFloats.size() == 6)
        {
          xd  = pFloats[0];
          yd  = pFloats[1];
          zd  = pFloats[2];
          wxd = pFloats[3];
          wyd = pFloats[4];
          wzd = pFloats[5];
        }
        else
        {
          xd  = 0.0f;
          yd  = 0.0f;
          zd  = 0.0f;
          wxd = 0.0f;
          wyd = 0.0f;
          wzd = 0.0f;
        }
      }

      Velocity6D operator+ (const Velocity6D& pVel2) const;
      Velocity6D operator- (const Velocity6D& pVel2) const;

      Velocity6D operator+ () const;
      Velocity6D operator- () const;

      Velocity6D operator* (const float pVal) const;
      Velocity6D operator/ (const float pVal) const;
      Velocity6D& operator*= (const float pM);
      Velocity6D& operator/= (const float pM);

      /// <summary>
      /// check if the actual Velocity6D is Near the one
      /// give in argument.
      ///
      /// </summary>
      /// <param name="pVel2"> the second Velocity6D </param>
      /// <param name="pEpsilon"> an optional epsilon distance </param>
      /// <returns>
      /// true if the difference of each float of the two Velocity6D is less than pEpsilon
      /// </returns>
      bool isNear(
        const Velocity6D& pVel2,
        const float&      pEpsilon=0.0001f) const;

      /// <summary>
      /// compute the norm of the actual Velocity6D
      ///
      /// sqrt(pVel.xd² + pVel.yd² + pVel.zd² + pVel.wxd² + pVel.wyd² + pVel.wzd²)
      /// </summary>
      /// <returns>
      /// the float norm of the Velocity6D
      /// </returns>
      float norm() const;

      /// <summary>
      /// normalize the actual Velocity6D
      ///
      /// result = pVel/ norm(pVel)
      /// </summary>
      /// <returns>
      /// the Velocity6D normalized
      /// </returns>
      Velocity6D normalize() const;

      /// <summary>
      /// return the Velocity6D as a vector of float [xd, yd, zd, wxd, wyd, wzd]
      /// </summary>
      std::vector<float> toVector() const;
    }; // end struct

    /**
    * Left multiplication
    */
    Velocity6D operator* (
      const float       pVal,
      const Velocity6D& pVel);

    /// <summary>
    /// compute the norm of a Velocity6D
    ///
    /// sqrt(pVel.xd² + pVel.yd² + pVel.zd² + pVel.wxd² + pVel.wyd² + pVel.wzd²)
    /// </summary>
    /// <param name="pVel"> the given Velocity6D </param>
    /// <returns>
    /// the float norm of the given Velocity6D
    /// </returns>
    /// \ingroup Types
    float norm(const Velocity6D& pVel);

    /// <summary>
    /// normalize a Velocity6D
    ///
    /// pRes = pVel/ norm(pVel)
    /// </summary>
    /// <param name="pVel"> the given Velocity6D </param>
    /// <returns>
    /// the given Velocity6D normalized
    /// </returns>
    /// \ingroup Types
    Velocity6D normalize(const Velocity6D& pVel);

  } // end namespace Math
} // end namespace AL
#endif  // _LIB_ALMATH_ALMATH_ALVELOCITY6D_H_
