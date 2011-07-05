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
    /// A homogenous transformation matrix
    ///
    /// </summary>
    /// <A HREF="http://en.wikipedia.org/wiki/Transformation_matrix">more information</A>
    /// \ingroup Types
    struct Transform {
      float r1_c1, r1_c2, r1_c3, r1_c4;
      float r2_c1, r2_c2, r2_c3, r2_c4;
      float r3_c1, r3_c2, r3_c3, r3_c4;

      /// <summary>
      /// Create a Transform initialize to identity.
      /// </summary>
      Transform(): r1_c1(1.0f), r1_c2(0.0f), r1_c3(0.0f), r1_c4(0.0f),
        r2_c1(0.0f), r2_c2(1.0f), r2_c3(0.0f), r2_c4(0.0f),
        r3_c1(0.0f), r3_c2(0.0f), r3_c3(1.0f), r3_c4(0.0f) {}

      /// <summary>
      /// Create a Transform with an std::vector.
      /// </summary>
      /// <param name="pFloats">
      /// An std::vector<float> of size 12 or 16 for respectively:
      ///
      /// \f$ \left[\begin{array}{cccc}r1c1 & r1c2 & r1c3 & r1c4 \\ r2c1 & r2c2 & r2c3 & r2c4 \\ r3c1 & r3c2 & r3c3 & r3c4 \\ 0.0 & 0.0 & 0.0 & 1.0 \end{array}\right] = \left[\begin{array}{cccc}pFloats[00] & pFloats[01] & pFloats[02] & pFloats[03] \\ pFloats[04] & pFloats[05] & pFloats[06] & pFloats[07] \\ pFloats[08] & pFloats[09] & pFloats[10] & pFloats[11] \\ 0.0 & 0.0 & 0.0 & 1.0 \end{array}\right]\f$
      /// </param>
      //      ///
      //      /** \f[
      //      * \left[\begin{array}{cccc}r1c1 & r1c2 & r1c3 & r1c4 \\
      //      *   r2c1 & r2c2 & r2c3 & r2c4 \\
      //      *   r3c1 & r3c2 & r3c3 & r3c4 \\
      //      *   0.0 & 0.0 & 0.0 & 1.0
      //      *  \end{array}\right]
      //      * \f]
      //      */
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
      /// Create a Transform initialize with explicit value for translation part. Rotation part is set to identity.
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

      /// <summary>
      /// overloading of operator *= for Transform.
      /// </summary>
      /// <param name="pT2"> the second Transform </param>
      Transform& operator*= (const Transform& pT2);

      /// <summary>
      /// overloading of operator * for Transform.
      /// </summary>
      /// <param name="pT2"> the second Transform </param>
      Transform operator* (const Transform& pT2) const;

      /// <summary>
      /// overloading of operator == for Transform.
      /// </summary>
      /// <param name="pT2"> the second Transform </param>
      bool operator==(const Transform& pT2) const;

      /// <summary>
      /// overloading of operator != for Transform.
      /// </summary>
      /// <param name="pT2"> the second Transform </param>
      bool operator!=(const Transform& pT2) const;

      /// <summary>
      /// Check if the actual Transform is near the one
      /// give in argument.
      ///
      /// </summary>
      /// <param name="pT2"> the second Transform </param>
      /// <param name="pEpsilon"> an optionnal epsilon distance: default: 0.0001 </param>
      /// <returns>
      /// true if the distance between the two Transform is less than pEpsilon
      /// </returns>
      bool isNear(
        const Transform& pT2,
        const float&     pEpsilon=0.0001f) const;

      /// <summary>
      /// Check if the rotation part is correct
      /// The condition checks are:
      /// \f$R^t * R = I\f$
      /// and
      /// det(R) = 1.0
      ///
      /// </summary>
      /// <param name="pEpsilon"> an optionnal epsilon distance. Default: 0.0001 </param>
      /// <returns>
      /// true if the Transform is correct
      /// </returns>
      bool isTransform(
          const float& pEpsilon=0.0001f) const;

      /// <summary>
      /// Compute the norm translation part of the actual Transform
      ///
      /// \f$\sqrt{pT.r1c4^2+pT.r2c4^2+pT.r3c4^2}\f$
      /// </summary>
      /// <returns>
      /// the float norm of the Transform
      /// </returns>
      float norm() const;

      /// <summary>
      /// Compute the determinant of rotation part of the actual Transform
      ///
      /// \f$pT.r1c1*pT.r2c2*pT.r3c3 + pT.r1c2*pT.r2c3*pT.r3c1 + pT.r1c3*pT.r2c1*pT.r3c2 - pT.r1c1*pT.r2c3*pT.r3c2 - pT.r1c2*pT.r2c1*pT.r3c3 - pT.r1c3*pT.r2c2*pT.r3c1\f$
      /// </summary>
      /// <returns>
      /// the float determinant of rotation Transform part
      /// </returns>
      float determinant() const;

      /// <summary>
      /// Compute the transform inverse of the actual Transform
      ///
      /// \f$ pT = \left[\begin{array}{cc}R & r \\ 0_{31} & 1 \end{array}\right]\f$
      ///
      /// \f$ pTOut = \left[\begin{array}{cc}R^t & (-R^t*r) \\ 0_{31} & 1 \end{array}\right]\f$
      ///
      /// </summary>
      /// <returns>
      /// the Transform inverse
      /// </returns>
      Transform inverse() const;

      /// <summary>
      /// Create a Transform initialized with explicit rotation around x axis.
      ///
      /// \f$ pT = \left[\begin{array}{cccc}1.0 & 0.0 & 0.0 & 0.0 \\ 0.0 & cos(pRotX) & -sin(pRotX) & 0.0 \\ 0.0 & sin(pRotX) & cos(pRotX) & 0.0 \\ 0.0 & 0.0 & 0.0 & 1.0 \end{array}\right]\f$
      ///
      /// </summary>
      /// <param name="pRotX"> the float value for angle rotation in radian around x axis </param>
      static Transform fromRotX(const float pRotX);

      /// <summary>
      /// Create a Transform initialized with explicit rotation around y axis.
      ///
      /// \f$ pT = \left[\begin{array}{cccc} cos(pRotY) & 0.0 & sin(pRotY) & 0.0 \\ 0.0 & 1.0 & 0.0 & 0.0 \\ -sin(pRotY) & 0.0 & cos(pRotY) & 0.0 \\ 0.0 & 0.0 & 0.0 & 1.0 \end{array}\right]\f$
      ///
      /// </summary>
      /// <param name="pRotY"> the float value for angle rotation in radian around y axis </param>
      static Transform fromRotY(const float pRotY);

      /// <summary>
      /// Create a Transform initialized with explicit rotation around z axis.
      ///
      /// \f$ pT = \left[\begin{array}{cccc} cos(pRotZ) & -sin(pRotZ) & 0.0 & 0.0 \\ sin(pRotZ) & cos(pRotZ) & 0.0 & 0.0 \\ 0.0 & 0.0 & 1.0 & 0.0 \\ 0.0 & 0.0 & 0.0 & 1.0 \end{array}\right]\f$
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
      ///
      /// \f$ pT = \left[\begin{array}{cccc} 1.0 & 0.0 & 0.0 & pX \\ 0.0 & 1.0 & 0.0 & pY \\ 0.0 & 0.0 & 1.0 & pZ \\ 0.0 & 0.0 & 0.0 & 1.0 \end{array}\right]\f$
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
      ///
      /// H = fromRotZ(pWZ)*fromRotY(pWY)*fromRotX(pWX)
      ///
      /// then
      ///
      /// H.r1_c4 = pX
      ///
      /// H.r2_c4 = pY
      ///
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
      /// \f$(pT1.r1c4-pT2.r1C4)^2+(pT1.r2c4-pT2.r2C4)^2+(pT1.r3c4-pT2.r3C4)^2\f$
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
      /// \f$\sqrt{(pT1.r1c4-pT2.r1c4)^2+(pT1.r2c4-pT2.r2c4)^2+(pT1.r3c4-pT2.r3c4)^2}\f$
      /// </summary>
      /// <param name="pT2"> the second Transform </param>
      /// <returns>
      /// the float distance between the two Transform
      /// </returns>
      float distance(const Transform& pT2) const;

      /// <summary>
      /// return the Transform as a vector of float
      ///
      /// \f$ \begin{array}{cccc} [r1c1, & r1c2, & r1c3, & r1c4, \\ r2c1, & r2c2, & r2c3, & r2c4, \\ r3c1, & r3c2, & r3c3, & r3c4, \\ 0.0, & 0.0, & 0.0, & 1.0] \end{array}\f$
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
    void transformPreMultiply(
      const Transform& pT,
      Transform&       pTOut);

    /// <summary>
    /// compute the norm translation part of the actual Transform
    ///
    /// \f$\sqrt{pT.r1c4^2+pT.r2c4^2+pT.r3c4^2}\f$
    /// </summary>
    /// <param name="pT"> the given Transform </param>
    /// <returns>
    /// the float norm of the given Transform
    /// </returns>
    /// \ingroup Types
    float norm(const Transform& pT);


    /// <summary>
    /// copy the Transform in a vector of float
    ///
    /// \f$ \begin{array}{cccc} [r1c1, & r1c2, & r1c3, & r1c4, \\ r2c1, & r2c2, & r2c3, & r2c4, \\ r3c1, & r3c2, & r3c3, & r3c4, \\ 0.0, & 0.0, & 0.0, & 1.0] \end{array}\f$
    ///
    /// </summary>
    /// <param name="pT"> the given Transform </param>
    /// <param name="pTOut"> the vector of float update to given transform value </param>
    /// \ingroup Types
    void transformToFloatVector(
      const Transform&    pT,
      std::vector<float>& pTOut);

    /// <summary>
    /// return the Transform in a vector of float
    ///
    /// \f$ \begin{array}{cccc} [r1c1, & r1c2, & r1c3, & r1c4, \\ r2c1, & r2c2, & r2c3, & r2c4, \\ r3c1, & r3c2, & r3c3, & r3c4, \\ 0.0, & 0.0, & 0.0, & 1.0] \end{array}\f$
    ///
    /// </summary>
    /// <param name="pT"> the given Transform </param>
    /// <returns>
    /// the vector of float update to given transform value
    /// </returns>
    /// \ingroup Types
    std::vector<float> transformToFloatVector(
      const Transform& pT);


    /// <summary>
    /// compute the determinant of rotation part of the given Transform
    ///
    /// \f$pT.r1c1*pT.r2c2*pT.r3c3 + pT.r1c2*pT.r2c3*pT.r3c1 + pT.r1c3*pT.r2c1 * pT.r3c2 - pT.r1c1*pT.r2c3*pT.r3c2 - pT.r1c2*pT.r2c1*pT.r3c3 - pT.r1c3*pT.r2c2*pT.r3c1\f$
    /// </summary>
    /// <param name="pT"> the given Transform </param>
    /// <returns>
    /// the float determinant of rotation Transform part
    /// </returns>
    /// \ingroup Types
    float determinant(const Transform& pT);

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
    float determinant(const std::vector<float>& pFloats);

    /// <summary>
    /// return the transform inverse of the given Transform
    ///
    /// \f$ pT = \left[\begin{array}{cc} R & r \\ 0_{31} & 1 \end{array}\right]\f$
    ///
    /// \f$ pTOut = \left[\begin{array}{cc} R^t & (-R^t*r) \\ 0_{31} & 1 \end{array}\right]\f$
    ///
    /// </summary>
    /// <param name="pT"> the given Transform </param>
    /// <param name="pTOut"> the inverse of the given Transform </param>
    /// \ingroup Types
    void transformInverse(
      const Transform& pT,
      Transform&       pTOut);

    /// <summary>
    /// return the transform inverse of the given Transform
    ///
    /// \f$ pT = \left[\begin{array}{cc} R & r \\ 0_{31} & 1 \end{array}\right]\f$
    ///
    /// \f$ pTOut = \left[\begin{array}{cc} R^t & (-R^t*r) \\ 0_{31} & 1 \end{array}\right]\f$
    ///
    /// </summary>
    /// <param name="pT"> the given Transform </param>
    /// <returns>
    /// the Transform inverse
    /// </returns>
    /// \ingroup Types
    Transform transformInverse(const Transform& pT);


    /// <summary>
    /// create a Transform initialize with explicit rotation around x axis.
    ///
    /// \f$ pTOut = \left[\begin{array}{cccc}1.0 & 0.0 & 0.0 & 0.0 \\ 0.0 & cos(pRotX) & -sin(pRotX) & 0.0 \\ 0.0 & sin(pRotX) & cos(pRotX) & 0.0 \\ 0.0 & 0.0 & 0.0 & 1.0 \end{array}\right]\f$
    ///
    /// </summary>
    /// <param name="pRotX"> the float value for angle rotation in radian around x axis </param>
    /// <returns>
    /// the Transform
    /// </returns>
    /// \ingroup Types
    Transform transformFromRotX(const float pRotX);

    /// <summary>
    /// create a Transform initialize with explicit rotation around y axis.
    ///
    /// \f$ pTOut = \left[\begin{array}{cccc} cos(pRotY) & 0.0 & sin(pRotY) & 0.0 \\ 0.0 & 1.0 & 0.0 & 0.0 \\ -sin(pRotY) & 0.0 & cos(pRotY) & 0.0 \\ 0.0 & 0.0 & 0.0 & 1.0 \end{array}\right]\f$
    ///
    /// </summary>
    /// <param name="pRotY"> the float value for angle rotation in radian around y axis </param>
    /// <returns>
    /// the Transform
    /// </returns>
    /// \ingroup Types
    Transform transformFromRotY(const float pRotY);

    /// <summary>
    /// create a Transform initialize with explicit rotation around z axis.
    ///
    /// \f$ pTOut = \left[\begin{array}{cccc} cos(pRotZ) & -sin(pRotZ) & 0.0 & 0.0 \\ sin(pRotZ) & cos(pRotZ) & 0.0 & 0.0 \\ 0.0 & 0.0 & 1.0 & 0.0 \\ 0.0 & 0.0 & 0.0 & 1.0 \end{array}\right]\f$
    ///
    /// </summary>
    /// <param name="pRotZ"> the float value for angle rotation in radian around z axis </param>
    /// <returns>
    /// the Transform
    /// </returns>
    /// \ingroup Types
    Transform transformFromRotZ(const float pRotZ);


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
    Transform transformFrom3DRotation(
      const float& pWX,
      const float& pWY,
      const float& pWZ);

    /// <summary>
    /// create a Transform initialize with explicit value for translation part.
    ///
    /// \f$ pTOut = \left[\begin{array}{cccc} 1.0 & 0.0 & 0.0 & pX \\ 0.0 & 1.0 & 0.0 & pY \\ 0.0 & 0.0 & 1.0 & pZ \\ 0.0 & 0.0 & 0.0 & 1.0 \end{array}\right]\f$
    ///
    /// </summary>
    /// <param name="pX"> the float value for translation axis x in meter (r1_c4) </param>
    /// <param name="pY"> the float value for translation axis y in meter (r2_c4) </param>
    /// <param name="pZ"> the float value for translation axis z in meter (r3_c4) </param>
    /// <returns>
    /// the Transform
    /// </returns>
    /// \ingroup Types
    Transform transformFromPosition(
      const float& pX,
      const float& pY,
      const float& pZ);

    /// <summary>
    /// create a Transform initialize with explicit value for translation part and euler angle.
    ///
    /// H = fromRotZ(pWZ)*fromRotY(pWY)*fromRotX(pWX)
    ///
    /// H.r1_c4 = pX
    ///
    /// H.r2_c4 = pY
    ///
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
    Transform transformFromPosition(
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
    void transformInvertInPlace(Transform& pT);


    /// <summary>
    /// alternative name for inverse: return the transform inverse of the given Transform
    ///
    /// \f$ pT = \left[\begin{array}{cc} R & r \\ 0_{31} & 1 \end{array}\right]\f$
    ///
    /// \f$ pTOut = \left[\begin{array}{cc} R^t & (-R^t*r) \\ 0_{31} & 1 \end{array}\right]\f$
    ///
    /// </summary>
    /// <param name="pT"> the given Transform </param>
    /// <param name="pTOut"> the inverse of the given Transform </param>
    /// \ingroup Types
    Transform pinv(const Transform& pT);


    /// <summary>
    /// compute the Transform between the actual Transform and the one give in argument result:
    ///
    /// inverse(pT1)*pT2
    ///
    /// </summary>
    /// <param name="pT1"> the first transform </param>
    /// <param name="pT2"> the second transform </param>
    /// <returns>
    /// the Transform
    /// </returns>
    /// \ingroup Types
    Transform transformDiff(
      const Transform& pT1,
      const Transform& pT2);

    /// <summary>
    /// compute the squared distance between the actual
    /// Transform and the one give in argument (translation part)
    ///
    /// \f$(pT1.r1c4-pT2.r1c4)^2 +(pT1.r2c4-pT2.r2c4)^2+(pT1.r3c4-pT2.r3c4)^2\f$
    /// </summary>
    /// <param name="pT1"> the first Transform </param>
    /// <param name="pT2"> the second Transform </param>
    /// <returns>
    /// the float squared distance between the two Transform: translation part
    /// </returns>
    /// \ingroup Types
    float transformDistanceSquared(
      const Transform& pT1,
      const Transform& pT2);


    /// <summary>
    /// compute the distance between the actual
    /// Transform and the one give in argument
    ///
    /// \f$\sqrt{(pT1.r1c4-pT2.r1c4)^2+(pT1.r2c4-pT2.r2c4)^2+(pT1.r3c4-pT2.r3c4)^2}\f$
    /// </summary>
    /// <param name="pT1"> the first Transform </param>
    /// <param name="pT2"> the second Transform </param>
    /// <returns>
    /// the float distance between the two Transform
    /// </returns>
    /// \ingroup Types
    float transformDistance(
      const Transform& pT1,
      const Transform& pT2);

  } // end namespace Math
} // end namespace AL
#endif  // _LIB_ALMATH_ALMATH_ALTRANSFORM_H_
