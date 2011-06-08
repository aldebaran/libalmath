/*
** Author(s):
**  - Chris Kilner
**  - Cyrille Collette
**  - David Gouaillier
**
** Copyright (C) 2011 Aldebaran Robotics
*/

#pragma once

#ifndef _LIB_ALMATH_ALMATH_ALROTATION_H_
#define _LIB_ALMATH_ALMATH_ALROTATION_H_

#include <vector>

namespace AL {
  namespace Math {

    struct Rotation
    {
      float r1_c1, r1_c2, r1_c3,
        r2_c1, r2_c2, r2_c3,
        r3_c1, r3_c2, r3_c3;

      Rotation(): r1_c1(1.0f), r1_c2(0.0f), r1_c3(0.0f),
        r2_c1(0.0f), r2_c2(1.0f), r2_c3(0.0f),
        r3_c1(0.0f), r3_c2(0.0f), r3_c3(1.0f){}

      /**
      * CONSTRUCTOR: create a Rotation from an std vector.
      */
      Rotation (const std::vector<float>& pFloats)
      {
        if (pFloats.size() == 9)
        {
          r1_c1 = pFloats[0];
          r1_c2 = pFloats[1];
          r1_c3 = pFloats[2];
          r2_c1 = pFloats[3];
          r2_c2 = pFloats[4];
          r2_c3 = pFloats[5];
          r3_c1 = pFloats[6];
          r3_c2 = pFloats[7];
          r3_c3 = pFloats[8];
        }
        else if ((pFloats.size() == 12) || (pFloats.size() == 16))
        {
          // if we give a transform, it take rotation part
          r1_c1 = pFloats[0];
          r1_c2 = pFloats[1];
          r1_c3 = pFloats[2];

          r2_c1 = pFloats[4];
          r2_c2 = pFloats[5];
          r2_c3 = pFloats[6];

          r3_c1 = pFloats[8];
          r3_c2 = pFloats[9];
          r3_c3 = pFloats[10];
        }
        else
        {
          r1_c1 = 1.0f;
          r1_c2 = 0.0f;
          r1_c3 = 0.0f;

          r2_c1 = 0.0f;
          r2_c2 = 1.0f;
          r2_c3 = 0.0f;

          r3_c1 = 0.0f;
          r3_c2 = 0.0f;
          r3_c3 = 1.0f;
        }
      }

      Rotation& operator*= (const Rotation& pT2);
      Rotation operator* (const Rotation& pT2) const;

      bool isNear(
        const Rotation& pRot,
        const float&    pEpsilon=0.0001f) const;

      Rotation transpose() const;
      float determinant() const;

      static Rotation fromQuaternion(
        const float pA,
        const float pB,
        const float pC,
        const float pD);

      static Rotation fromAngleDirection(
        const float pAngle,
        const float pX,
        const float pY,
        const float pZ);

      static Rotation fromRotX(const float pTheta);
      static Rotation fromRotY(const float pTheta);
      static Rotation fromRotZ(const float pTheta);

      static Rotation from3DRotation(
        const float& pWX,
        const float& pWY,
        const float& pWZ);

      std::vector<float> toVector() const;

    };


    /**
    * Function Transpose : transpose rotation
    * @param  Rotation R
    * @return Transpose Rotation
    **/
    Rotation Transpose(const Rotation& pIn);


    /**
    * Function Determiant : compute determinant of rotation part of Transform
    * @param  Rotation R
    * @return float det
    **/
    float Determinant(const Rotation& pM);

    /**
    * Creates a 3*3 Rotation Matrix from a normalized quaternion ( |a + bi + cj + dk| = 1)
    * @param pA Coefficient a of the normalized quaternion
    * @param pB Coefficient b of the normalized quaternion
    * @param pC Coefficient c of the normalized quaternion
    * @param pD Coefficient d of the normalized quaternion
    */
    Rotation RotationFromQuaternion(
      const float pA,
      const float pB,
      const float pC,
      const float pD);

    /**
    * Creates a 3*3 Rotation Matrix from a an angle and a normalized direction( |pX, pY, pZ| = 1)
    * @param pAngle Coefficient a of the normalized quaternion
    * @param pX X direction of the vector of the rotation
    * @param pY Y direction of the vector of the rotation
    * @param pZ Z direction of the vector of the rotation
    */
    Rotation RotationFromAngleDirection(
      const float pAngle,
      const float pX,
      const float pY,
      const float pZ);


    void ApplyRotation(
      AL::Math::Rotation& pRotation,
      float&              pX,
      float&              pY,
      float&              pZ);

    /**
    * Creates a 3*3 Matrix from a rotation
    * about the X axis.
    * @param pTheta The rotation
    */
    Rotation RotationFromRotX(const float pTheta);

    /**
    * Creates a 3*3 Matrix from a rotation
    * about the Y axis
    * @param pTheta The rotation
    */
    Rotation RotationFromRotY(const float pTheta);

    /**
    * Creates a 3*3 Matrix from a rotation
    *  about the Z axis
    * @param pTheta The rotation
    */
    Rotation RotationFromRotZ(const float pTheta);

    /**
    * Creates a 3*3 Matrix from a Roll, Pitch and yaw Angle in radian
    * @param pWX x rotation in radians
    * @param pWY y rotation in radians
    * @param pWZ z rotation in radians
    */
    Rotation RotationFrom3DRotation(
      const float& pWX,
      const float& pWY,
      const float& pWZ);

  }
}
#endif  // _LIB_ALMATH_ALMATH_ALROTATION_H_
