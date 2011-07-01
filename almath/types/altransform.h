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

      /// <summary>
      /// create a Transform initialize to identity.
      /// </summary>
      Transform(): r1_c1(1.0f), r1_c2(0.0f), r1_c3(0.0f), r1_c4(0.0f),
        r2_c1(0.0f), r2_c2(1.0f), r2_c3(0.0f), r2_c4(0.0f),
        r3_c1(0.0f), r3_c2(0.0f), r3_c3(1.0f), r3_c4(0.0f) {}

      /// <summary>
      /// create a Transform with an std::vector.
      /// </summary>
      /// <param name="pFloats">
      /// An std::vector<float> of size 12 or 16 for respectively:
      /// r1_c1, r1_c2, r1_c3, r1_c4,
      /// r2_c1, r2_c2, r2_c3, r2_c4,
      /// r3_c1, r3_c2, r3_c3, r3_c4,
      /// 0.0f, 0.0f, 0.0f and 1.0f
      /// </param>
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

      /// <summary>
      /// create a Transform initialize with explicit value for translation part. Rotation part is set to identity.
      /// </summary>
      /// <param name="pPosX"> the float value for translation x </param>
      /// <param name="pPosY"> the float value for translation y </param>
      /// <param name="pPosZ"> the float value for translation z </param>
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

      bool operator==(const Transform& pT) const;
      bool operator!=(const Transform& pT) const;


      /// <summary>
      /// check if the actual Transform is Near the one
      /// give in argument.
      ///
      /// </summary>
      /// <param name="pT2"> the second Transform </param>
      /// <param name="pEpsilon"> an optionnal epsilon distance </param>
      /// <returns>
      /// true if the distance between the two Transform is less than pEpsilon
      /// </returns>
      bool isNear(
        const Transform& pT2,
        const float&     pEpsilon=0.0001f) const;

      /// <summary>
      /// check if the rotation part is correct
      /// The condition check are:
      /// R' * R = Identity
      /// and
      /// det(R) = 1
      ///
      /// </summary>
      /// <param name="pEpsilon"> an optionnal epsilon distance </param>
      /// <returns>
      /// true if the Transform is correct
      /// </returns>
      bool isTransform(
          const float& pEpsilon=0.0001f) const;

      /// <summary>
      /// compute the norm translation part of the actual Transform
      ///
      /// \f$\sqrt{pT.r1_c4²+pT.r2_c4²+pT.r3_c4²}\f$
      /// </summary>
      /// <returns>
      /// the float norm of the Transform
      /// </returns>
      float norm() const;

      /// <summary>
      /// compute the determinant of rotation part of the given Transform
      ///
      /// \f$pT.r1_c1*pT.r2_c2*pT.r3_c3 + pT.r1_c2*pT.r2_c3*pT.r3_c1 + pT.r1_c3*pT.r2_c1 * pT.r3_c2 - pT.r1_c1*pT.r2_c3*pT.r3_c2 - pT.r1_c2*pT.r2_c1*pT.r3_c3 - pT.r1_c3*pT.r2_c2*pT.r3_c1\f$
      /// </summary>
      /// <returns>
      /// the float determinant of rotation Transform part
      /// </returns>
      float determinant() const;

      /// <summary>
      /// compute the transform inverse of the given Transform
      /// pT    = [R        r ; 0 0 0 1]
      /// pTOut = [R' (-R'*r) ; 0 0 0 1]
      ///
      /// </summary>
      /// <returns>
      /// the Transform inverse
      /// </returns>
      Transform inverse() const;

      /// <summary>
      /// create a Transform initialize with explicit rotation around x axis.
      /// 1.0 0.0        0.0         0.0
      /// 0.0 cos(pRotX) -sin(pRotX) 0.0
      /// 0.0 sin(pRotX) cos(pRotX)  0.0
      /// 0.0 0.0        0.0         1.0
      ///
      /// </summary>
      /// <param name="pRotX"> the float value for angle rotation in radian around x axis </param>
      static Transform fromRotX(const float pRotX);

      /// <summary>
      /// create a Transform initialize with explicit rotation around y axis.
      /// cos(pRotY)  0.0 sin(pRotY) 0.0
      /// 0.0         1.0 0.0        0.0
      /// -sin(pRotY) 0.0 cos(pRotY) 0.0
      /// 0.0         0.0 0.0        1.0
      ///
      /// </summary>
      /// <param name="pRotY"> the float value for angle rotation in radian around y axis </param>
      static Transform fromRotY(const float pRotY);

      /// <summary>
      /// create a Transform initialize with explicit rotation around z axis.
      /// cos(pRotZ) -sin(pRotZ) 0.0 0.0
      /// sin(pRotZ) cos(pRotZ)  0.0 0.0
      /// 0.0        0.0         1.0 0.0
      /// 0.0        0.0         0.0 1.0
      ///
      /// </summary>
      /// <param name="pRotZ"> the float value for angle rotation in radian around z axis </param>
      static Transform fromRotZ(const float pRotZ);


      /// <summary>
      /// create a Transform initialize with euler angle.
      /// H = fromRotZ(pWZ)*fromRotY(pWY)*fromRotX(pWX)
      ///
      /// </summary>
      /// <param name="pWX"> the float value for euler angle x in radian </param>
      /// <param name="pWY"> the float value for euler angle y in radian </param>
      /// <param name="pWZ"> the float value for euler angle z in radian </param>
      static Transform from3DRotation(
        const float& pWX,
        const float& pWY,
        const float& pWZ);


      /// <summary>
      /// create a Transform initialize with explicit value for translation part.
      /// 1.0 0.0 0.0 pX
      /// 0.0 1.0 0.0 pY
      /// 0.0 0.0 1.0 pZ
      /// 0.0 0.0 0.0 1.0
      ///
      /// </summary>
      /// <param name="pX"> the float value for translation axis x in meter (r1_c4) </param>
      /// <param name="pY"> the float value for translation axis y in meter (r2_c4) </param>
      /// <param name="pZ"> the float value for translation axis z in meter (r3_c4) </param>
      static Transform fromPosition(
        const float pX,
        const float pY,
        const float pZ);

      /// <summary>
      /// create a Transform initialize with explicit value for translation part and euler angle.
      /// H = fromRotZ(pWZ)*fromRotY(pWY)*fromRotX(pWX)
      /// H.r1_c4 = pX
      /// H.r2_c4 = pY
      /// H.r3_c4 = pZ
      ///
      /// </summary>
      /// <param name="pX"> the float value for translation axis x in meter (r1_c4) </param>
      /// <param name="pY"> the float value for translation axis y in meter (r2_c4) </param>
      /// <param name="pZ"> the float value for translation axis z in meter (r3_c4) </param>
      /// <param name="pWX"> the float value for euler angle x in radian </param>
      /// <param name="pWY"> the float value for euler angle y in radian </param>
      /// <param name="pWZ"> the float value for euler angle z in radian </param>
      static Transform fromPosition(
        const float& pX,
        const float& pY,
        const float& pZ,
        const float& pWX,
        const float& pWY,
        const float& pWZ);

      /// <summary>
      /// compute the Transform between the actual
      /// Transform and the one give in argument
      /// result: inverse(pT1)*pT2
      ///
      /// </summary>
      /// <param name="pT2"> the second transform </param>
      Transform diff(const Transform& pT2) const;


      /// <summary>
      /// compute the squared distance between the actual
      /// Transform and the one give in argument (translation part)
      ///
      /// \f$(pT1.r1_c4-pT2.r1_C4)²+(pT1.r2_c4-pT2.r2_C4)²+(pT1.r3_c4-pT2.r3_C4)²\f$
      /// </summary>
      /// <param name="pT2"> the second Transform </param>
      /// <returns>
      /// the float squared distance between the two Transform: translation part
      /// </returns>
      float distanceSquared(const Transform& pT2) const;


      /// <summary>
      /// compute the distance between the actual
      /// Transform and the one give in argument
      ///
      /// \f$\sqrt{(pT1.r1_c4-pT2.r1_c4)²+(pT1.r2_c4-pT2.r2_c4)²+(pT1.r3_c4-pT2.r3_c4)²}\f$
      /// </summary>
      /// <param name="pT2"> the second Transform </param>
      /// <returns>
      /// the float distance between the two Transform
      /// </returns>
      float distance(const Transform& pT2) const;

      /// <summary>
      /// return the Transform as a vector of float
      /// [r1_c1, r1_c2, r1_c3, r1_c4,
      ///  r2_c1, r2_c2, r2_c3, r2_c4,
      ///  r3_c1, r3_c2, r3_c3, r3_c4,
      ///  0.0, 0.0, 0.0, 1.0]
      /// </summary>
      std::vector<float> toVector() const;

    }; // end struct

    /// <summary>
    /// pTOut = pT*pTOut
    ///
    /// </summary>
    /// <param name="pT"> the first constant Transform </param>
    /// <param name="pTOut"> the second modified Transform </param>
    /// \ingroup Types
    void TransformPreMultiply(
      const Transform& pT,
      Transform&       pTOut);

    /// <summary>
    /// compute the norm translation part of the actual Transform
    ///
    /// \f$\sqrt{pT.r1_c4²+pT.r2_c4²+pT.r3_c4²}\f$
    /// </summary>
    /// <param name="pT"> the given Transform </param>
    /// <returns>
    /// the float norm of the given Transform
    /// </returns>
    /// \ingroup Types
    float norm(const Transform& pT);


    /// <summary>
    /// copy the Transform in a vector of float
    /// [r1_c1, r1_c2, r1_c3, r1_c4,
    ///  r2_c1, r2_c2, r2_c3, r2_c4,
    ///  r3_c1, r3_c2, r3_c3, r3_c4,
    ///  0.0, 0.0, 0.0, 1.0]
    /// </summary>
    /// <param name="pT"> the given Transform </param>
    /// <param name="pTOut"> the vector of float update to given transform value </param>
    /// \ingroup Types
    void TransformToFloatVector(
      const Transform&    pT,
      std::vector<float>& pTOut);

    /// <summary>
    /// return the Transform in a vector of float
    /// [r1_c1, r1_c2, r1_c3, r1_c4,
    ///  r2_c1, r2_c2, r2_c3, r2_c4,
    ///  r3_c1, r3_c2, r3_c3, r3_c4,
    ///  0.0, 0.0, 0.0, 1.0]
    /// </summary>
    /// <param name="pT"> the given Transform </param>
    /// <returns>
    /// the vector of float update to given transform value
    /// </returns>
    /// \ingroup Types
    std::vector<float> TransformToFloatVector(
      const Transform& pT);


    /// <summary>
    /// compute the determinant of rotation part of the given Transform
    ///
    /// \f$pT.r1_c1*pT.r2_c2*pT.r3_c3 + pT.r1_c2*pT.r2_c3*pT.r3_c1 + pT.r1_c3*pT.r2_c1 * pT.r3_c2 - pT.r1_c1*pT.r2_c3*pT.r3_c2 - pT.r1_c2*pT.r2_c1*pT.r3_c3 - pT.r1_c3*pT.r2_c2*pT.r3_c1\f$
    /// </summary>
    /// <param name="pT"> the given Transform </param>
    /// <returns>
    /// the float determinant of rotation Transform part
    /// </returns>
    /// \ingroup Types
    float Determinant(const Transform& pT);

    /// <summary>
    /// compute the determinant of rotation part of the given vector of floats
    ///
    /// \f$pT[0]*pT[5]*pT[10] + pT[1]*pT[6]*pT[8] + pT[2]*pT[4]*pT[9] - pT[0]*pT[6]*pT[9] - pT[1]*pT[4]*pT[10] - pT[2]*pT[5]*pT[8]\f$
    /// </summary>
    /// <param name="pFloats"> the given vector of floats </param>
    /// <returns>
    /// the float determinant of rotation Transform part
    /// </returns>
    /// \ingroup Types
    float Determinant(const std::vector<float>& pFloats);

    /// <summary>
    /// return the transform inverse of the given Transform
    /// pT    = [R        r ; 0 0 0 1]
    /// pTOut = [R' (-R'*r) ; 0 0 0 1]
    ///
    /// </summary>
    /// <param name="pT"> the given Transform </param>
    /// <param name="pTOut"> the inverse of the given Transform </param>
    /// \ingroup Types
    void TransformInverse(
      const Transform& pT,
      Transform&       pTOut);

    /// <summary>
    /// return the transform inverse of the given Transform
    /// pT    = [R        r ; 0 0 0 1]
    /// pTOut = [R' (-R'*r) ; 0 0 0 1]
    ///
    /// </summary>
    /// <param name="pT"> the given Transform </param>
    /// <returns>
    /// the Transform inverse
    /// </returns>
    /// \ingroup Types
    Transform TransformInverse(const Transform& pT);


    /// <summary>
    /// create a Transform initialize with explicit rotation around x axis.
    /// 1.0 0.0        0.0         0.0
    /// 0.0 cos(pRotX) -sin(pRotX) 0.0
    /// 0.0 sin(pRotX) cos(pRotX)  0.0
    /// 0.0 0.0        0.0         1.0
    ///
    /// </summary>
    /// <param name="pRotX"> the float value for angle rotation in radian around x axis </param>
    /// <returns>
    /// the Transform
    /// </returns>
    /// \ingroup Types
    Transform TransformFromRotX(const float pRotX);

    /// <summary>
    /// create a Transform initialize with explicit rotation around y axis.
    /// cos(pRotY)  0.0 sin(pRotY) 0.0
    /// 0.0         1.0 0.0        0.0
    /// -sin(pRotY) 0.0 cos(pRotY) 0.0
    /// 0.0         0.0 0.0        1.0
    ///
    /// </summary>
    /// <param name="pRotY"> the float value for angle rotation in radian around y axis </param>
    /// <returns>
    /// the Transform
    /// </returns>
    /// \ingroup Types
    Transform TransformFromRotY(const float pRotY);

    /// <summary>
    /// create a Transform initialize with explicit rotation around z axis.
    /// cos(pRotZ) -sin(pRotZ) 0.0 0.0
    /// sin(pRotZ) cos(pRotZ)  0.0 0.0
    /// 0.0        0.0         1.0 0.0
    /// 0.0        0.0         0.0 1.0
    ///
    /// </summary>
    /// <param name="pRotZ"> the float value for angle rotation in radian around z axis </param>
    /// <returns>
    /// the Transform
    /// </returns>
    /// \ingroup Types
    Transform TransformFromRotZ(const float pRotZ);


    /// <summary>
    /// create a Transform initialize with euler angle.
    /// H = fromRotZ(pWZ)*fromRotY(pWY)*fromRotX(pWX)
    ///
    /// </summary>
    /// <param name="pWX"> the float value for euler angle x in radian </param>
    /// <param name="pWY"> the float value for euler angle y in radian </param>
    /// <param name="pWZ"> the float value for euler angle z in radian </param>
    /// <returns>
    /// the Transform
    /// </returns>
    /// \ingroup Types
    Transform TransformFrom3DRotation(
      const float& pWX,
      const float& pWY,
      const float& pWZ);

    /// <summary>
    /// create a Transform initialize with explicit value for translation part.
    /// 1.0 0.0 0.0 pX
    /// 0.0 1.0 0.0 pY
    /// 0.0 0.0 1.0 pZ
    /// 0.0 0.0 0.0 1.0
    ///
    /// </summary>
    /// <param name="pX"> the float value for translation axis x in meter (r1_c4) </param>
    /// <param name="pY"> the float value for translation axis y in meter (r2_c4) </param>
    /// <param name="pZ"> the float value for translation axis z in meter (r3_c4) </param>
    /// <returns>
    /// the Transform
    /// </returns>
    /// \ingroup Types
    Transform TransformFromPosition(
      const float& pX,
      const float& pY,
      const float& pZ);

    /// <summary>
    /// create a Transform initialize with explicit value for translation part and euler angle.
    /// H = fromRotZ(pWZ)*fromRotY(pWY)*fromRotX(pWX)
    /// H.r1_c4 = pX
    /// H.r2_c4 = pY
    /// H.r3_c4 = pZ
    ///
    /// </summary>
    /// <param name="pX"> the float value for translation axis x in meter (r1_c4) </param>
    /// <param name="pY"> the float value for translation axis y in meter (r2_c4) </param>
    /// <param name="pZ"> the float value for translation axis z in meter (r3_c4) </param>
    /// <param name="pWX"> the float value for euler angle x in radian </param>
    /// <param name="pWY"> the float value for euler angle y in radian </param>
    /// <param name="pWZ"> the float value for euler angle z in radian </param>
    /// <returns>
    /// the Transform
    /// </returns>
    /// \ingroup Types
    Transform TransformFromPosition(
      const float& pX,
      const float& pY,
      const float& pZ,
      const float& pWX,
      const float& pWY,
      const float& pWZ);


    /// <summary>
    /// inverse the given Transform in place
    ///
    /// </summary>
    /// <param name="pT"> the given Transform </param>
    /// \ingroup Types
    void TransformInvertInPlace(Transform& pT);


    /// <summary>
    /// alternative name for inverse: return the transform inverse of the given Transform
    /// pT    = [R        r ; 0 0 0 1]
    /// pTOut = [R' (-R'*r) ; 0 0 0 1]
    ///
    /// </summary>
    /// <param name="pT"> the given Transform </param>
    /// <param name="pTOut"> the inverse of the given Transform </param>
    /// \ingroup Types
    Transform pinv(const Transform& pT);


    /// <summary>
    /// compute the Transform between the actual
    /// Transform and the one give in argument
    /// result: inverse(pT1)*pT2
    ///
    /// </summary>
    /// <param name="pT1"> the first transform </param>
    /// <param name="pT2"> the second transform </param>
    /// <returns>
    /// the Transform
    /// </returns>
    /// \ingroup Types
    Transform TransformDiff(
      const Transform& pT1,
      const Transform& pT2);

    /// <summary>
    /// compute the squared distance between the actual
    /// Transform and the one give in argument (translation part)
    ///
    /// \f$(pT1.r1_c4-pT2.r1_C4)²+(pT1.r2_c4-pT2.r2_C4)²+(pT1.r3_c4-pT2.r3_C4)²\f$
    /// </summary>
    /// <param name="pT1"> the first Transform </param>
    /// <param name="pT2"> the second Transform </param>
    /// <returns>
    /// the float squared distance between the two Transform: translation part
    /// </returns>
    /// \ingroup Types
    float TransformDistanceSquared(
      const Transform& pT1,
      const Transform& pT2);


    /// <summary>
    /// compute the distance between the actual
    /// Transform and the one give in argument
    ///
    /// \f$\sqrt{(pT1.r1_c4-pT2.r1_c4)²+(pT1.r2_c4-pT2.r2_c4)²+(pT1.r3_c4-pT2.r3_c4)²}\f$
    /// </summary>
    /// <param name="pT1"> the first Transform </param>
    /// <param name="pT2"> the second Transform </param>
    /// <returns>
    /// the float distance between the two Transform
    /// </returns>
    /// \ingroup Types
    float TransformDistance(
      const Transform& pT1,
      const Transform& pT2);

  } // end namespace Math
} // end namespace AL
#endif  // _LIB_ALMATH_ALMATH_ALTRANSFORM_H_
