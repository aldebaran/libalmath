/*
** Author(s):
**  - Chris Kilner
**  - Cyrille Collette
**  - David Gouaillier
**
** Copyright (C) 2011 Aldebaran Robotics
*/

#pragma once

#ifndef _LIB_ALMATH_ALMATH_ALTRANSFORM_H_
#define _LIB_ALMATH_ALMATH_ALTRANSFORM_H_

#include <vector>

namespace AL {
  namespace Math {

    /// <summary>
    /// Create and play with a Transform.
    ///
    /// A Position3D is just defined by x, y and z.
    /// </summary>
    /// <A HREF="http://en.wikipedia.org/wiki/Transformation_matrix">more information</A>
    /// \ingroup Types
    struct Transform {
      float r1_c1, r1_c2, r1_c3, r1_c4;
      float r2_c1, r2_c2, r2_c3, r2_c4;
      float r3_c1, r3_c2, r3_c3, r3_c4;

      /**
      * DEFAULT CONSTRUCTOR: create an identity matrix
      * (null Transform does not exist)
      * g Standards </A>
      */
      Transform(): r1_c1(1.0f), r1_c2(0.0f), r1_c3(0.0f), r1_c4(0.0f),
        r2_c1(0.0f), r2_c2(1.0f), r2_c3(0.0f), r2_c4(0.0f),
        r3_c1(0.0f), r3_c2(0.0f), r3_c3(1.0f), r3_c4(0.0f) {}

      /**
      * CONSTRUCTOR: create an Transform from a std vector.
      */
      explicit Transform(const std::vector<float>& pFloats)
      {
        if (
          (pFloats.size() == 12) ||
          (pFloats.size() == 16))
        {
          r1_c1 = pFloats[0];
          r1_c2 = pFloats[1];
          r1_c3 = pFloats[2];
          r1_c4 = pFloats[3];

          r2_c1 = pFloats[4];
          r2_c2 = pFloats[5];
          r2_c3 = pFloats[6];
          r2_c4 = pFloats[7];

          r3_c1 = pFloats[8];
          r3_c2 = pFloats[9];
          r3_c3 = pFloats[10];
          r3_c4 = pFloats[11];
        }
        else
        {
          r1_c1 = 1.0f;
          r1_c2 = 0.0f;
          r1_c3 = 0.0f;
          r1_c4 = 0.0f;

          r2_c1 = 0.0f;
          r2_c2 = 1.0f;
          r2_c3 = 0.0f;
          r2_c4 = 0.0f;

          r3_c1 = 0.0f;
          r3_c2 = 0.0f;
          r3_c3 = 1.0f;
          r3_c4 = 0.0f;
        }
      }

      /**
      * CONSTRUCTOR: create a Transform from an 3d position.
      */
      Transform(
        const float& pPosX,
        const float& pPosY,
        const float& pPosZ)
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

        r1_c4 = pPosX;
        r2_c4 = pPosY;
        r3_c4 = pPosZ;
      }

      Transform& operator*= (const Transform& pT2);
      Transform operator* (const Transform& pT2) const;

      bool operator==(const Transform& pH) const;
      bool operator!=(const Transform& pH) const;

      bool isNear(
        const Transform& pH,
        const float&     pEpsilon=0.0001f) const;

      bool isTransform(
          const float& pEpsilon=0.0001f) const;

      float norm() const;

      float determinant() const;
      Transform inverse() const;


      static Transform fromRotX(const float rotx);
      static Transform fromRotY(const float roty);
      static Transform fromRotZ(const float rotz);


      static Transform from3DRotation(
        const float& pWX,
        const float& pWY,
        const float& pWZ);

      static Transform fromPosition(
        const float x,
        const float y,
        const float z);

      static Transform fromPosition(
        const float& pX,
        const float& pY,
        const float& pZ,
        const float& pWX,
        const float& pWY,
        const float& pWZ);


      Transform diff(const Transform &pT2) const;
      float squaredDistance(const Transform& pT2) const;
      float distance(const Transform& pT2) const;

      std::vector<float> toVector() const;

    }; // end struct

    void TransformPreMultiply(
      const Transform& pT1,
      Transform&       pResult);

    float norm(const Transform& pH);

    void TransformToFloatVector(
      const Transform&  pT,
      std::vector<float>& pOut);

    std::vector<float> TransformToFloatVector(
      const Transform&  pT);


    /** cyrille - 18/05/2009
    * Function Determiant : compute determinant of rotation part of Transform
    * @param  Transform H  = [R      r ; 0 0 0 1]
    * @return float det
    **/
    float Determinant(const Transform& pH);

    float Determinant(const std::vector<float>& pH);

    /**
    * Function Inverse of n Transform
    * @param  Transform H  = [R      r ; 0 0 0 1]
    * @return Transform Hi = [R' -R'*r ; 0 0 0 1];
    **/
    void TransformInverse(
      const Transform& pIn,
      Transform&       pOut);

    Transform TransformInverse(const Transform& pIn);

    /**
    * Creates a 4*4 Homogeneous Matrix from a rotation
    * about the X axis.
    * @param pTheta The rotation
    */
    Transform TransformFromRotX(const float pTheta);

    /**
    * Creates a 4*4 Homogeneous Matrix from a rotation
    * about the Y axis
    * @param pTheta The rotation
    */
    Transform TransformFromRotY(const float pTheta);

    /**
    * Creates a 4*4 Homogeneous Matrix from a rotation
    *  about the Z axis
    * @param pTheta The rotation
    */
    Transform TransformFromRotZ(const float pTheta);

    /**
    * Creates a 4*4 Homogeneous Matrix from a Roll, Pitch and yaw Angle in radian
    * pH = rotYaw(pWZ)*rotPitch(pWY)*rotRoll(pWX)
    * @param pWX x rotation in radians
    * @param pWY y rotation in radians
    * @param pWZ z rotation in radians
    */
    Transform TransformFrom3DRotation(
      const float& pWX,
      const float& pWY,
      const float& pWZ);

    /**
    * Creates a 4*4 transform matrix from XYZ coordinates
    * indicating the translational offsets.
    */
    Transform TransformFromPosition(
      const float x,
      const float y,
      const float z);

    /**
    * Creates a 4*4 transform matrix from a rotation(RPY) and a position
    * indicating the translational offsets.
    */
    Transform TransformFromPosition(
      const float& pX,
      const float& pY,
      const float& pZ,
      const float& pWX,
      const float& pWY,
      const float& pWZ);

    void TransformInvertInPlace(Transform& pT);

    /* Alternative name for Inverse */
    Transform pinv(const Transform& pT);

    Transform TransformDiff(
      const Transform& pT1,
      const Transform &pT2);

    float TransformSqaredDistance(
      const Transform& pT1,
      const Transform& pT2);

    float TransformDistance(
      const Transform& pT1,
      const Transform& pT2);

  } // end namespace Math
} // end namespace AL
#endif  // _LIB_ALMATH_ALMATH_ALTRANSFORM_H_
