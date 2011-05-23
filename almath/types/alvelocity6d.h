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

    struct Velocity6D {
      float xd, yd, zd, wxd, wyd, wzd;

      Velocity6D(): xd(0.0f),
        yd(0.0f),
        zd(0.0f),
        wxd(0.0f),
        wyd(0.0f),
        wzd(0.0f) {}

      explicit Velocity6D(float pInit): xd(pInit),
        yd(pInit),
        zd(pInit),
        wxd(pInit),
        wyd(pInit),
        wzd(pInit) {}

      Velocity6D(
        float pXd,
        float pYd,
        float pZd,
        float pWxd,
        float pWyd,
        float pWzd): xd(pXd),
        yd(pYd),
        zd(pZd),
        wxd(pWxd),
        wyd(pWyd),
        wzd(pWzd) {}

      /**
      * CONSTRUCTOR: create a Velocity6D from an std vector.
      */
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

      bool isNear(
        const Velocity6D& pVel,
        const float&      pEpsilon=0.0001f) const;

      Velocity6D operator* (const float pVal) const;
      Velocity6D operator/ (const float pVal) const;
      Velocity6D& operator*= (const float pM);
      Velocity6D& operator/= (const float pM);

      float norm() const;
      Velocity6D normalize() const;

      std::vector<float> toVector() const;

    }; // end struct

    /**
    * Left multiplication
    */
    Velocity6D operator* (
      const float       pVal,
      const Velocity6D& pVel);

    float norm(const Velocity6D& p);

    Velocity6D normalize(const Velocity6D& p);

  } // end namespace Math
} // end namespace AL
#endif  // _LIB_ALMATH_ALMATH_ALVELOCITY6D_H_
