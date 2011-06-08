/*
** Author(s):
**  - Chris Kilner
**  - Cyrille Collette
**  - David Gouaillier
**
** Copyright (C) 2011 Aldebaran Robotics
*/

#pragma once

#ifndef _LIB_ALMATH_ALMATH_ALVELOCITY3D_H_
#define _LIB_ALMATH_ALMATH_ALVELOCITY3D_H_

#include <vector>

namespace AL {
  namespace Math {

    /// <summary>
    /// Create and play with a Velocity3D.
    ///
    /// A Velocity3D is just defined by xd, yd and zd.
    /// </summary>
    /// \ingroup Types
    struct Velocity3D {
      float xd, yd, zd;

      /// <summary>
      /// create a Velocity3D initialize with 0.0f.
      /// </summary>
      Velocity3D(): xd(0.0f),
        yd(0.0f),
        zd(0.0f) {}

      /// <summary>
      /// create a Velocity3D initialize with the same float.
      /// </summary>
      /// <param name="pInit"> the float value for each member </param>
      /// </summary>
      explicit Velocity3D(float pInit): xd(pInit),
        yd(pInit),
        zd(pInit) {}

      /// <summary>
      /// create a Velocity3D initialize with explicit value.
      /// </summary>
      /// <param name="pXd"> the float value for xd </param>
      /// <param name="pYd"> the float value for yd </param>
      /// <param name="pZd"> the float value for zd </param>
      Velocity3D(
        float pXd,
        float pYd,
        float pZd): xd(pXd),
        yd(pYd),
        zd(pZd) {}

      /// <summary>
      /// create a Velocity3D with an std::vector.
      /// </summary>
      /// <param name="pFloats">
      /// An std::vector<float> of size 3 for respectively:
      /// xd, yd, zd
      /// </param>
      Velocity3D(const std::vector<float>& pFloats)
      {
        if (pFloats.size() == 3)
        {
          xd = pFloats[0];
          yd = pFloats[1];
          zd = pFloats[2];
        }
        else
        {
          xd = 0.0f;
          yd = 0.0f;
          zd = 0.0f;
        }
      }

      Velocity3D operator+ (const Velocity3D& pVel2) const;
      Velocity3D operator- (const Velocity3D& pVel2) const;

      Velocity3D operator+ () const;
      Velocity3D operator- () const;

      Velocity3D& operator+= (const Velocity3D& pVel2);
      Velocity3D& operator-= (const Velocity3D& pVel2);

      Velocity3D operator* (const float pM) const;
      Velocity3D operator/ (const float pM) const;
      Velocity3D& operator*= (const float pM);
      Velocity3D& operator/= (const float pM);

      /// <summary>
      /// check if the actual Velocity3D is Near the one
      /// give in argument.
      ///
      /// </summary>
      /// <param name="pVel2"> the second Velocity3D </param>
      /// <param name="pEpsilon"> an optional epsilon distance </param>
      /// <returns>
      /// true if the difference of each float of the two Velocity3D is less than pEpsilon
      /// </returns>
      bool isNear(
        const Velocity3D& pVel2,
        const float&      pEpsilon=0.0001f) const;


      /// <summary>
      /// compute the norm of the actual Velocity3D
      ///
      /// sqrt(pVel.xd² + pVel.yd² + pVel.zd²)
      /// </summary>
      /// <returns>
      /// the float norm of the Velocity3D
      /// </returns>
      float norm () const;

      /// <summary>
      /// normalize the actual Velocity3D
      ///
      /// result = pVel/ norm(pVel)
      /// </summary>
      /// <returns>
      /// the Velocity3D normalized
      /// </returns>
      Velocity3D normalize() const;

      /// <summary>
      /// return the Velocity3D as a vector of float [xd, yd, zd]
      /// </summary>
      std::vector<float> toVector() const;
    };

    Velocity3D operator* (
      const float       pM,
      const Velocity3D& pVel1);

    /// <summary>
    /// compute the norm of a Velocity3D
    ///
    /// sqrt(pVel.xd² + pVel.yd² + pVel.zd²)
    /// </summary>
    /// <param name="pPos"> the given Velocity3D </param>
    /// <returns>
    /// the float norm of the given Velocity3D
    /// </returns>
    /// \ingroup Types
    float norm (const Velocity3D& pVel);

    /// <summary>
    /// normalize a Velocity3D
    ///
    /// pRes = pVel/ norm(pVel)
    /// </summary>
    /// <param name="pVel"> the given Velocity3D </param>
    /// <returns>
    /// the given Velocity3D normalized
    /// </returns>
    /// \ingroup Types
    Velocity3D normalize(const Velocity3D& pVel);

  } // end namespace Math
} // end namespace AL
#endif  // _LIB_ALMATH_ALMATH_ALVELOCITY3D_H_
