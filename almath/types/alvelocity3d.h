/**
* @author Chris Kilner
* Copyright (c) Aldebaran Robotics 2009 All Rights Reserved
*/

#pragma once

#ifndef _LIB_ALMATH_ALMATH_ALVELOCITY3D_H_
#define _LIB_ALMATH_ALMATH_ALVELOCITY3D_H_

#include <vector>

namespace AL {
  namespace Math {

    struct Velocity3D {
      float xd, yd, zd;

      Velocity3D(): xd(0.0f),
        yd(0.0f),
        zd(0.0f) {}

      explicit Velocity3D(float pInit): xd(pInit),
        yd(pInit),
        zd(pInit) {}

      Velocity3D(
        float pXd,
        float pYd,
        float pZd): xd(pXd),
        yd(pYd),
        zd(pZd) {}

      /**
      * CONSTRUCTOR: create a Velocity3D from an std vector.
      */
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
      Velocity3D& operator+= (const Velocity3D& pVel2);

      bool isNear(
        const Velocity3D& pVel,
        const float&      pEpsilon=0.0001f) const;

      Velocity3D operator* (const float pM) const;
      Velocity3D operator/ (const float pM) const;
      Velocity3D& operator*= (const float pM);
      Velocity3D& operator/= (const float pM);

      float norm () const;

      Velocity3D normalize() const;

      std::vector<float> toVector() const;
    };

    Velocity3D operator* (
      const float       pM,
      const Velocity3D& pVel1);

    float norm (const Velocity3D& p);

    Velocity3D normalize(const Velocity3D& p);

  } // end namespace Math
} // end namespace AL
#endif  // _LIB_ALMATH_ALMATH_ALVELOCITY3D_H_
