/*
** Author(s):
**  - Chris Kilner
**  - Cyrille Collette
**  - David Gouaillier
**
** Copyright (C) 2011 Aldebaran Robotics
*/

#pragma once

#ifndef _LIB_ALMATH_ALMATH_ALROTATION2D_H_
#define _LIB_ALMATH_ALMATH_ALROTATION2D_H_

#include <vector>

namespace AL {
  namespace Math {

    struct Rotation2D
    {
      float r1_c1, r1_c2,
        r2_c1, r2_c2;

      Rotation2D(): r1_c1(1.0f), r1_c2(0.0f),
        r2_c1(0.0f), r2_c2(1.0f){}

      /**
      * Create a Rotation2D from an std vector.
      */
      Rotation2D (const std::vector<float>& pFloats)
      {
        if (pFloats.size() == 4)
        {
          r1_c1 = pFloats[0];
          r1_c2 = pFloats[1];
          r2_c1 = pFloats[2];
          r2_c2 = pFloats[3];
        }
        else
        {
          r1_c1 = 0.0f;
          r1_c2 = 0.0f;
          r2_c1 = 0.0f;
          r2_c2 = 0.0f;
        }
      }

      Rotation2D& operator*= (const Rotation2D& pRot2);
      Rotation2D operator* (const Rotation2D& pRot2) const;

      bool isNear(
        const Rotation2D& pRot,
        const float&      pEpsilon=0.0001f) const;

      Rotation2D transpose() const;
      float determinant() const;

      static Rotation2D fromAngle(const float pTheta);

      std::vector<float> toVector() const;
    };

   /**
    * Function Transpose : transpose rotation
    * @param  Rotation R
    * @return Transpose Rotation
    **/
    Rotation2D transpose(const Rotation2D& pIn);

    /**
    * Function Determiant : compute determinant of rotation part of Transform
    * @param  Rotation R
    * @return float det
    **/
    float determinant(const Rotation2D& pM);

  }
}
#endif  // _LIB_ALMATH_ALMATH_ALROTATION2D_H_
