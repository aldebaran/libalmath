/*
 * Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */

#include <almath/types/altransform.h>
#include <cmath>
#include <iostream>

namespace AL {
  namespace Math {

    Transform::Transform():
        r1_c1(1.0f), r1_c2(0.0f), r1_c3(0.0f), r1_c4(0.0f),
        r2_c1(0.0f), r2_c2(1.0f), r2_c3(0.0f), r2_c4(0.0f),
        r3_c1(0.0f), r3_c2(0.0f), r3_c3(1.0f), r3_c4(0.0f) {}

    Transform::Transform(const std::vector<float>& pFloats)
    {
      if (pFloats.size() == 12u ||
          pFloats.size() == 16u)
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

        // todo: check it is a real transform
        if (!isTransform())
        {
          normalizeTransform();

          // strange case with nan data
          if (!isTransform())
          {
            std::cerr << "ALMath: WARNING: "
                      << "Transform constructor with wrong vector value. "
                      << "Rotation part is normalized." << std::endl;
          }
        }

      }
      else
      {
        std::cerr << "ALMath: WARNING: "
                  << "Transform constructor call with a wrong size of vector. "
                  << "Size expected: 12 or 16. Size given: " << pFloats.size() << ". "
                  << "Transform is set to identity." << std::endl;

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

    Transform::Transform(
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

    Transform& Transform::operator*= (const Transform& pT2)
    {
      if (this == &pT2)
      {
        // copy to manage case: a *= a
        return *this *= Transform(pT2);
      }

      float c1 = r1_c1;
      float c2 = r1_c2;
      float c3 = r1_c3;

      r1_c1 = (c1 * pT2.r1_c1) + (c2 * pT2.r2_c1) + (c3 * pT2.r3_c1);
      r1_c2 = (c1 * pT2.r1_c2) + (c2 * pT2.r2_c2) + (c3 * pT2.r3_c2);
      r1_c3 = (c1 * pT2.r1_c3) + (c2 * pT2.r2_c3) + (c3 * pT2.r3_c3);
      r1_c4 = (c1 * pT2.r1_c4) + (c2 * pT2.r2_c4) + (c3 * pT2.r3_c4) + r1_c4;
      c1 = r2_c1;
      c2 = r2_c2;
      c3 = r2_c3;
      r2_c1 = (c1 * pT2.r1_c1) + (c2 * pT2.r2_c1) + (c3 * pT2.r3_c1);
      r2_c2 = (c1 * pT2.r1_c2) + (c2 * pT2.r2_c2) + (c3 * pT2.r3_c2);
      r2_c3 = (c1 * pT2.r1_c3) + (c2 * pT2.r2_c3) + (c3 * pT2.r3_c3);
      r2_c4 = (c1 * pT2.r1_c4) + (c2 * pT2.r2_c4) + (c3 * pT2.r3_c4) + r2_c4;
      c1 = r3_c1;
      c2 = r3_c2;
      c3 = r3_c3;
      r3_c1 = (c1 * pT2.r1_c1) + (c2 * pT2.r2_c1) + (c3 * pT2.r3_c1);
      r3_c2 = (c1 * pT2.r1_c2) + (c2 * pT2.r2_c2) + (c3 * pT2.r3_c2);
      r3_c3 = (c1 * pT2.r1_c3) + (c2 * pT2.r2_c3) + (c3 * pT2.r3_c3);
      r3_c4 = (c1 * pT2.r1_c4) + (c2 * pT2.r2_c4) + (c3 * pT2.r3_c4) + r3_c4;
      return *this;
    }

    Transform Transform::operator* (const Transform& pT2) const
    {
      Transform t;
      t.r1_c1 = (r1_c1 * pT2.r1_c1) + (r1_c2 * pT2.r2_c1) + (r1_c3 * pT2.r3_c1);
      t.r1_c2 = (r1_c1 * pT2.r1_c2) + (r1_c2 * pT2.r2_c2) + (r1_c3 * pT2.r3_c2);
      t.r1_c3 = (r1_c1 * pT2.r1_c3) + (r1_c2 * pT2.r2_c3) + (r1_c3 * pT2.r3_c3);
      t.r1_c4 = (r1_c1 * pT2.r1_c4) + (r1_c2 * pT2.r2_c4) + (r1_c3 * pT2.r3_c4) + r1_c4;

      t.r2_c1 = (r2_c1 * pT2.r1_c1) + (r2_c2 * pT2.r2_c1) + (r2_c3 * pT2.r3_c1);
      t.r2_c2 = (r2_c1 * pT2.r1_c2) + (r2_c2 * pT2.r2_c2) + (r2_c3 * pT2.r3_c2);
      t.r2_c3 = (r2_c1 * pT2.r1_c3) + (r2_c2 * pT2.r2_c3) + (r2_c3 * pT2.r3_c3);
      t.r2_c4 = (r2_c1 * pT2.r1_c4) + (r2_c2 * pT2.r2_c4) + (r2_c3 * pT2.r3_c4) + r2_c4;

      t.r3_c1 = (r3_c1 * pT2.r1_c1) + (r3_c2 * pT2.r2_c1) + (r3_c3 * pT2.r3_c1);
      t.r3_c2 = (r3_c1 * pT2.r1_c2) + (r3_c2 * pT2.r2_c2) + (r3_c3 * pT2.r3_c2);
      t.r3_c3 = (r3_c1 * pT2.r1_c3) + (r3_c2 * pT2.r2_c3) + (r3_c3 * pT2.r3_c3);
      t.r3_c4 = (r3_c1 * pT2.r1_c4) + (r3_c2 * pT2.r2_c4) + (r3_c3 * pT2.r3_c4) + r3_c4;
      return t;
    }

    bool Transform::operator==(const Transform& pT2) const
    {
      return (r1_c1 == pT2.r1_c1 &&
              r1_c2 == pT2.r1_c2 &&
              r1_c3 == pT2.r1_c3 &&
              r1_c4 == pT2.r1_c4 &&
              r2_c1 == pT2.r2_c1 &&
              r2_c2 == pT2.r2_c2 &&
              r2_c3 == pT2.r2_c3 &&
              r2_c4 == pT2.r2_c4 &&
              r3_c1 == pT2.r3_c1 &&
              r3_c2 == pT2.r3_c2 &&
              r3_c3 == pT2.r3_c3 &&
              r3_c4 == pT2.r3_c4);
    }

    bool Transform::operator!=(const Transform& pT2) const
    {
      return !(*this==pT2);
    }


    bool Transform::isNear(
        const Transform& pT2,
        const float&     pEpsilon)const
    {
      return (std::abs(r1_c1 - pT2.r1_c1) <= pEpsilon &&
              std::abs(r1_c2 - pT2.r1_c2) <= pEpsilon &&
              std::abs(r1_c3 - pT2.r1_c3) <= pEpsilon &&
              std::abs(r2_c1 - pT2.r2_c1) <= pEpsilon &&
              std::abs(r2_c2 - pT2.r2_c2) <= pEpsilon &&
              std::abs(r2_c3 - pT2.r2_c3) <= pEpsilon &&
              std::abs(r3_c1 - pT2.r3_c1) <= pEpsilon &&
              std::abs(r3_c2 - pT2.r3_c2) <= pEpsilon &&
              std::abs(r3_c3 - pT2.r3_c3) <= pEpsilon &&
              std::abs(r1_c4 - pT2.r1_c4) <= pEpsilon &&
              std::abs(r2_c4 - pT2.r2_c4) <= pEpsilon &&
              std::abs(r3_c4 - pT2.r3_c4) <= pEpsilon);
    }


    bool Transform::isTransform(
      const float& pEpsilon) const
    {
      // This checks that the input is a pure rotation matrix 'm'.
      // The condition for this is:
      // R' * R = I
      // and
      // det(R) = 1

      // det is defined here:
      // http://www.euclideanspace.com/maths/algebra/matrix/functions/determinant/threeD/

      if (std::abs(r1_c2*r1_c1 + r2_c2*r2_c1 + r3_c2*r3_c1) > pEpsilon)
      {
        return false;
      }

      if (std::abs(r1_c3*r1_c1 + r2_c3*r2_c1 + r3_c3*r3_c1) > pEpsilon)
      {
        return false;
      }

      if (std::abs(r1_c3*r1_c2 + r2_c3*r2_c2 + r3_c3*r3_c2) > pEpsilon)
      {
        return false;
      }

      if (std::abs(r1_c1*r1_c1 + r1_c2*r1_c2 + r1_c3*r1_c3 - 1.0f) > pEpsilon)
      {
        return false;
      }

      if (std::abs(r2_c1*r2_c1 + r2_c2*r2_c2 + r2_c3*r2_c3 - 1.0f) > pEpsilon)
      {
        return false;
      }

      if (std::abs(r3_c1*r3_c1 + r3_c2*r3_c2 + r3_c3*r3_c3 - 1.0f) > pEpsilon)
      {
        return false;
      }

      const float det = Math::determinant(*this);

      if (std::abs(det-1.0f) > pEpsilon)
      {
        return false;
      }
      return true;
    }


    void Transform::normalizeTransform(void)
    {
      Math::normalizeTransform(*this);
    }


    float Transform::norm() const
    {
      return Math::norm(*this);
    }

    float Transform::determinant() const
    {
      return Math::determinant(*this);
    }

    Transform Transform::inverse() const
    {
      return Math::transformInverse(*this);
    }


    Transform Transform::fromRotX(const float pRotX)
    {
      return Math::transformFromRotX(pRotX);
    }

    Transform Transform::fromRotY(const float pRotY)
    {
      return Math::transformFromRotY(pRotY);
    }

    Transform Transform::fromRotZ(const float pRotZ)
    {
      return Math::transformFromRotZ(pRotZ);
    }


    Transform Transform::from3DRotation(
      const float& pWX,
      const float& pWY,
      const float& pWZ)
    {
      return Math::transformFrom3DRotation(pWX, pWY, pWZ);
    }

    Transform Transform::fromPosition(
      const float pX,
      const float pY,
      const float pZ)
    {
      return Math::transformFromPosition(pX, pY, pZ);
    }

    Transform Transform::fromPosition(
      const float& pX,
      const float& pY,
      const float& pZ,
      const float& pWX,
      const float& pWY,
      const float& pWZ)
    {
      return Math::transformFromPosition(pX, pY, pZ, pWX, pWY, pWZ);
    }


    Transform Transform::diff(const Transform& pT2) const
    {
      return Math::transformDiff(*this, pT2);
    }

    float Transform::distanceSquared(const Transform& pT2) const
    {
      return Math::transformDistanceSquared(*this, pT2);
    }

    float Transform::distance(const Transform& pT2) const
    {
      return Math::transformDistance(*this, pT2);
    }

    void Transform::toVector(std::vector<float>& pReturnVector) const
    {
      pReturnVector.resize(16);

      pReturnVector[0]  = r1_c1;
      pReturnVector[1]  = r1_c2;
      pReturnVector[2]  = r1_c3;
      pReturnVector[3]  = r1_c4;

      pReturnVector[4]  = r2_c1;
      pReturnVector[5]  = r2_c2;
      pReturnVector[6]  = r2_c3;
      pReturnVector[7]  = r2_c4;

      pReturnVector[8]  = r3_c1;
      pReturnVector[9]  = r3_c2;
      pReturnVector[10] = r3_c3;
      pReturnVector[11] = r3_c4;

      pReturnVector[12] = 0.0f;
      pReturnVector[13] = 0.0f;
      pReturnVector[14] = 0.0f;
      pReturnVector[15] = 1.0f;
    }

    std::vector<float> Transform::toVector(void) const
    {
      std::vector<float> lReturnVector(16, 0.0f);
      this->toVector(lReturnVector);
      return lReturnVector;
    }

    void Transform::writeToVector(std::vector<float>::iterator& pIt) const
    {
      *pIt++ = r1_c1;
      *pIt++ = r1_c2;
      *pIt++ = r1_c3;
      *pIt++ = r1_c4;

      *pIt++ = r2_c1;
      *pIt++ = r2_c2;
      *pIt++ = r2_c3;
      *pIt++ = r2_c4;

      *pIt++ = r3_c1;
      *pIt++ = r3_c2;
      *pIt++ = r3_c3;
      *pIt++ = r3_c4;

      *pIt++ = 0.0f;
      *pIt++ = 0.0f;
      *pIt++ = 0.0f;
      *pIt++ = 1.0f;
    }

    void transformPreMultiply(
      const Transform& pT,
      Transform&       pTOut)
    {
      float r1 = pTOut.r1_c1;
      float r2 = pTOut.r2_c1;
      pTOut.r1_c1 = (pT.r1_c1 * r1) + (pT.r1_c2 * r2) + (pT.r1_c3 * pTOut.r3_c1);
      pTOut.r2_c1 = (pT.r2_c1 * r1) + (pT.r2_c2 * r2) + (pT.r2_c3 * pTOut.r3_c1);
      pTOut.r3_c1 = (pT.r3_c1 * r1) + (pT.r3_c2 * r2) + (pT.r3_c3 * pTOut.r3_c1);

      r1 = pTOut.r1_c2;
      r2 = pTOut.r2_c2;
      pTOut.r1_c2 = (pT.r1_c1 * r1) + (pT.r1_c2 * r2) + (pT.r1_c3 * pTOut.r3_c2);
      pTOut.r2_c2 = (pT.r2_c1 * r1) + (pT.r2_c2 * r2) + (pT.r2_c3 * pTOut.r3_c2);
      pTOut.r3_c2 = (pT.r3_c1 * r1) + (pT.r3_c2 * r2) + (pT.r3_c3 * pTOut.r3_c2);

      r1 = pTOut.r1_c3;
      r2 = pTOut.r2_c3;
      pTOut.r1_c3 = (pT.r1_c1 * r1) + (pT.r1_c2 * r2) + (pT.r1_c3 * pTOut.r3_c3);
      pTOut.r2_c3 = (pT.r2_c1 * r1) + (pT.r2_c2 * r2) + (pT.r2_c3 * pTOut.r3_c3);
      pTOut.r3_c3 = (pT.r3_c1 * r1) + (pT.r3_c2 * r2) + (pT.r3_c3 * pTOut.r3_c3);

      r1 = pTOut.r1_c4;
      r2 = pTOut.r2_c4;
      pTOut.r1_c4 = (pT.r1_c1 * r1) + (pT.r1_c2 * r2) + (pT.r1_c3 * pTOut.r3_c4) + pT.r1_c4;
      pTOut.r2_c4 = (pT.r2_c1 * r1) + (pT.r2_c2 * r2) + (pT.r2_c3 * pTOut.r3_c4) + pT.r2_c4;
      pTOut.r3_c4 = (pT.r3_c1 * r1) + (pT.r3_c2 * r2) + (pT.r3_c3 * pTOut.r3_c4) + pT.r3_c4;
    }


    void normalizeTransform(Transform& pT)
    {
      const float lEpsilon = 0.0001f;

      // z.normalize();
      float lNorm =
          std::sqrt(std::pow(pT.r1_c3, 2) +
                    std::pow(pT.r2_c3, 2) +
                    std::pow(pT.r3_c3, 2));
      if (lNorm < lEpsilon)
      {
        std::cerr << "ALMath: WARNING: "
                  << "normalizeTransform with null column. "
                  << "Rotation part set to identity." << std::endl;

        pT.r1_c1 = 1.0f;
        pT.r1_c2 = 0.0f;
        pT.r1_c3 = 0.0f;

        pT.r2_c1 = 0.0f;
        pT.r2_c2 = 1.0f;
        pT.r2_c3 = 0.0f;

        pT.r3_c1 = 0.0f;
        pT.r3_c2 = 0.0f;
        pT.r3_c3 = 1.0f;
        return;
      }

      pT.r1_c3 /= lNorm;
      pT.r2_c3 /= lNorm;
      pT.r3_c3 /= lNorm;

      // x = cross(y, z);
      const float x1 = pT.r2_c2*pT.r3_c3 - pT.r3_c2*pT.r2_c3;
      const float x2 = pT.r3_c2*pT.r1_c3 - pT.r1_c2*pT.r3_c3;
      const float x3 = pT.r1_c2*pT.r2_c3 - pT.r2_c2*pT.r1_c3;

      // x.normalize();
      lNorm = std::sqrt(std::pow(x1, 2) +
                        std::pow(x2, 2) +
                        std::pow(x3, 2));

      if (lNorm < lEpsilon)
      {
        std::cerr << "ALMath: WARNING: "
                  << "normalizeTransform with null column. "
                  << "Rotation part set to identity." << std::endl;

        pT.r1_c1 = 1.0f;
        pT.r1_c2 = 0.0f;
        pT.r1_c3 = 0.0f;

        pT.r2_c1 = 0.0f;
        pT.r2_c2 = 1.0f;
        pT.r2_c3 = 0.0f;

        pT.r3_c1 = 0.0f;
        pT.r3_c2 = 0.0f;
        pT.r3_c3 = 1.0f;
        return;
      }

      pT.r1_c1 = x1/lNorm;
      pT.r2_c1 = x2/lNorm;
      pT.r3_c1 = x3/lNorm;

      // y = cross(z, x);
      pT.r1_c2 = pT.r2_c3*pT.r3_c1 - pT.r3_c3*pT.r2_c1;
      pT.r2_c2 = pT.r3_c3*pT.r1_c1 - pT.r1_c3*pT.r3_c1;
      pT.r3_c2 = pT.r1_c3*pT.r2_c1 - pT.r2_c3*pT.r1_c1;
    }

    float norm(const Transform& pT)
    {
      return std::sqrt(
            pT.r1_c4*pT.r1_c4+
            pT.r2_c4*pT.r2_c4+
            pT.r3_c4*pT.r3_c4);
    }


    void transformToFloatVector(
      const Transform&    pT,
      std::vector<float>& pTOut)
    {
      std::cerr << "ALMath: WARNING: "
                << "transformToFloatVector is deprecated. "
                << "Use toVector function." << std::endl;

      pTOut.resize(12);
      pTOut[0]  = pT.r1_c1;
      pTOut[1]  = pT.r1_c2;
      pTOut[2]  = pT.r1_c3;
      pTOut[3]  = pT.r1_c4;
      pTOut[4]  = pT.r2_c1;
      pTOut[5]  = pT.r2_c2;
      pTOut[6]  = pT.r2_c3;
      pTOut[7]  = pT.r2_c4;
      pTOut[8]  = pT.r3_c1;
      pTOut[9]  = pT.r3_c2;
      pTOut[10] = pT.r3_c3;
      pTOut[11] = pT.r3_c4;
    }


    std::vector<float> transformToFloatVector(
      const Transform& pT)
    {
      std::vector<float> lTOut;
      transformToFloatVector(pT, lTOut);
      return lTOut;
    }

    float determinant(const Transform& pT)
    {
      return pT.r1_c1 * pT.r2_c2 * pT.r3_c3 +
          pT.r1_c2 * pT.r2_c3 * pT.r3_c1 +
          pT.r1_c3 * pT.r2_c1 * pT.r3_c2 -
          pT.r1_c1 * pT.r2_c3 * pT.r3_c2 -
          pT.r1_c2 * pT.r2_c1 * pT.r3_c3 -
          pT.r1_c3 * pT.r2_c2 * pT.r3_c1;
    }

    float determinant(const std::vector<float>& pFloats)
    {
      if (pFloats.size() != 12u &&
          pFloats.size() != 16u)
      {
        std::cout << "ALMath: WARNING: "
                  << "determinant call with a wrong size of vector. "
                  << "Size expected: 12 or 16. Size given: " << pFloats.size() << ". "
                  << "Determinant is set to zero." << std::endl;
        return 0.0f;
      }

      return pFloats[0] * pFloats[5] * pFloats[10] +
          pFloats[1] * pFloats[6] * pFloats[8] +
          pFloats[2] * pFloats[4] * pFloats[9] -
          pFloats[0] * pFloats[6] * pFloats[9] -
          pFloats[1] * pFloats[4] * pFloats[10] -
          pFloats[2] * pFloats[5] * pFloats[8];
    }


    void transformInverse(
      const Transform& pT,
      Transform&       pTOut)
    {
      // rotation Ri = R'
      pTOut.r1_c1 = pT.r1_c1;
      pTOut.r1_c2 = pT.r2_c1;
      pTOut.r1_c3 = pT.r3_c1;
      pTOut.r2_c1 = pT.r1_c2;
      pTOut.r2_c2 = pT.r2_c2;
      pTOut.r2_c3 = pT.r3_c2;
      pTOut.r3_c1 = pT.r1_c3;
      pTOut.r3_c2 = pT.r2_c3;
      pTOut.r3_c3 = pT.r3_c3;

      // translation ri = -R'*r
      pTOut.r1_c4 = -( pT.r1_c1*pT.r1_c4 + pT.r2_c1*pT.r2_c4 + pT.r3_c1*pT.r3_c4 );
      pTOut.r2_c4 = -( pT.r1_c2*pT.r1_c4 + pT.r2_c2*pT.r2_c4 + pT.r3_c2*pT.r3_c4 );
      pTOut.r3_c4 = -( pT.r1_c3*pT.r1_c4 + pT.r2_c3*pT.r2_c4 + pT.r3_c3*pT.r3_c4 );
    }


    Transform transformInverse(const Transform& pT)
    {
      Transform pTOut;
      transformInverse(pT, pTOut);
      return pTOut;
    }


    Transform transformFromRotX(const float pRotX)
    {
      const float c = std::cos(pRotX);
      const float s = std::sin(pRotX);
      Transform T;
      T.r2_c2 = c;
      T.r2_c3 = -s;
      T.r3_c2 = s;
      T.r3_c3 = c;
      return T;
    }


    Transform transformFromRotY(const float pRotY)
    {
      const float c = std::cos(pRotY);
      const float s = std::sin(pRotY);
      Transform T;
      T.r1_c1 = c;
      T.r1_c3 = s;
      T.r3_c1 = -s;
      T.r3_c3 = c;
      return T;
    }


    Transform transformFromRotZ(const float pRotZ)
    {
      const float c = std::cos(pRotZ);
      const float s = std::sin(pRotZ);
      Transform T;
      T.r1_c1 = c;
      T.r1_c2 = -s;
      T.r2_c1 = s;
      T.r2_c2 = c;
      return T;
    }


    Transform transformFrom3DRotation(
      const float& pWX,
      const float& pWY,
      const float& pWZ)
    {
      Transform T = transformFromRotZ(pWZ);
      T *= transformFromRotY(pWY);
      T *= transformFromRotX(pWX);
      return T;
    }


    Transform transformFromPosition(
      const float& pX,
      const float& pY,
      const float& pZ)
    {
      Transform T;
      T.r1_c4 = pX;
      T.r2_c4 = pY;
      T.r3_c4 = pZ;
      return T;
    }


    Transform transformFromPosition(
      const float& pX,
      const float& pY,
      const float& pZ,
      const float& pWX,
      const float& pWY,
      const float& pWZ)
    {
      Transform T = transformFrom3DRotation(pWX, pWY, pWZ);

      T.r1_c4 = pX;
      T.r2_c4 = pY;
      T.r3_c4 = pZ;
      return T;
    }

    void transformInvertInPlace(Transform& pT)
    {
      float tmp0 = pT.r1_c2;
      pT.r1_c2 = pT.r2_c1;
      pT.r2_c1 = tmp0;

      tmp0 = pT.r1_c3;
      pT.r1_c3 = pT.r3_c1;
      pT.r3_c1 = tmp0;

      tmp0 = pT.r2_c3;
      pT.r2_c3 = pT.r3_c2;
      pT.r3_c2 = tmp0;

      tmp0 =       -(pT.r1_c1 * pT.r1_c4 + pT.r1_c2 * pT.r2_c4 + pT.r1_c3 * pT.r3_c4);
      float tmp1 = -(pT.r2_c1 * pT.r1_c4 + pT.r2_c2 * pT.r2_c4 + pT.r2_c3 * pT.r3_c4);
      pT.r3_c4 =   -(pT.r3_c1 * pT.r1_c4 + pT.r3_c2 * pT.r2_c4 + pT.r3_c3 * pT.r3_c4);
      pT.r2_c4 = tmp1;
      pT.r1_c4 = tmp0;
    }

    Transform pinv(const Transform& pT)
    {
      Transform result = pT;
      transformInvertInPlace(result);
      return result;
    }


    Transform transformDiff(
      const Transform& pT1,
      const Transform& pT2)
    {
      Transform result = pT1;

      transformInvertInPlace(result);
      result *= pT2;
      return result;
    }

    float transformDistanceSquared(
      const Transform& pT1,
      const Transform& pT2)
    {
      float tmp = pT1.r1_c4 - pT2.r1_c4;
      float tot = tmp * tmp;
      tmp = pT1.r2_c4 - pT2.r2_c4;
      tot += tmp * tmp;
      tmp = pT1.r3_c4 - pT2.r3_c4;
      tot += tmp * tmp;
      return tot;
    }


    float transformDistance(
      const Transform& pT1,
      const Transform& pT2)
    {
      return std::sqrt(transformDistanceSquared(pT1, pT2));
    }

  } // end namespace Math
} // end namespace AL

