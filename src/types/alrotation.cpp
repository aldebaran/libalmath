/*
 * Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */

#include <almath/types/alrotation.h>

#include <stdexcept>
# include <cmath>
#include <iostream>

namespace AL {
  namespace Math {

    Rotation::Rotation():
      r1_c1(1.0f), r1_c2(0.0f), r1_c3(0.0f),
      r2_c1(0.0f), r2_c2(1.0f), r2_c3(0.0f),
      r3_c1(0.0f), r3_c2(0.0f), r3_c3(1.0f){}

    Rotation::Rotation (const std::vector<float>& pFloats)
    {
      if (pFloats.size() == 9u)
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
      else if ((pFloats.size() == 12u) || (pFloats.size() == 16u))
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
        std::cerr << "ALMath: WARNING: "
                  << "Rotation constructor call with a wrong size of vector. "
                  << "Size expected: 9, 12 or 16. Size given: " << pFloats.size() << ". "
                  << "Rotation is set to default identity." << std::endl;

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


    Rotation& Rotation::operator*= (const Rotation& pRot2)
    {
      float c1 = r1_c1;
      float c2 = r1_c2;
      float c3 = r1_c3;

      if (this == &pRot2)
      {
        // copy to manage case: a *= a
        return *this *= Rotation(pRot2);
      }

      r1_c1 = (c1 * pRot2.r1_c1) + (c2 * pRot2.r2_c1) + (c3 * pRot2.r3_c1);
      r1_c2 = (c1 * pRot2.r1_c2) + (c2 * pRot2.r2_c2) + (c3 * pRot2.r3_c2);
      r1_c3 = (c1 * pRot2.r1_c3) + (c2 * pRot2.r2_c3) + (c3 * pRot2.r3_c3);

      c1 = r2_c1;
      c2 = r2_c2;
      c3 = r2_c3;

      r2_c1 = (c1 * pRot2.r1_c1) + (c2 * pRot2.r2_c1) + (c3 * pRot2.r3_c1);
      r2_c2 = (c1 * pRot2.r1_c2) + (c2 * pRot2.r2_c2) + (c3 * pRot2.r3_c2);
      r2_c3 = (c1 * pRot2.r1_c3) + (c2 * pRot2.r2_c3) + (c3 * pRot2.r3_c3);

      c1 = r3_c1;
      c2 = r3_c2;
      c3 = r3_c3;

      r3_c1 = (c1 * pRot2.r1_c1) + (c2 * pRot2.r2_c1) + (c3 * pRot2.r3_c1);
      r3_c2 = (c1 * pRot2.r1_c2) + (c2 * pRot2.r2_c2) + (c3 * pRot2.r3_c2);
      r3_c3 = (c1 * pRot2.r1_c3) + (c2 * pRot2.r2_c3) + (c3 * pRot2.r3_c3);

      return *this;
    }


    Rotation Rotation::operator* (const Rotation& pRot2) const
    {
      Rotation pOut = *this;
      pOut *= pRot2;
      return pOut;
    }


    bool Rotation::operator==(const Rotation& pRot2) const
    {
      return (r1_c1 == pRot2.r1_c1 &&
              r1_c2 == pRot2.r1_c2 &&
              r1_c3 == pRot2.r1_c3 &&
              r2_c1 == pRot2.r2_c1 &&
              r2_c2 == pRot2.r2_c2 &&
              r2_c3 == pRot2.r2_c3 &&
              r3_c1 == pRot2.r3_c1 &&
              r3_c2 == pRot2.r3_c2 &&
              r3_c3 == pRot2.r3_c3);
    }

    bool Rotation::operator!=(const Rotation& pRot2) const
    {
      return !(*this==pRot2);
    }


    bool Rotation::isNear(
        const Rotation& pRot2,
        const float&    pEpsilon) const
    {
      return (std::abs(r1_c1 - pRot2.r1_c1) <= pEpsilon &&
              std::abs(r1_c2 - pRot2.r1_c2) <= pEpsilon &&
              std::abs(r1_c3 - pRot2.r1_c3) <= pEpsilon &&
              std::abs(r2_c1 - pRot2.r2_c1) <= pEpsilon &&
              std::abs(r2_c2 - pRot2.r2_c2) <= pEpsilon &&
              std::abs(r2_c3 - pRot2.r2_c3) <= pEpsilon &&
              std::abs(r3_c1 - pRot2.r3_c1) <= pEpsilon &&
              std::abs(r3_c2 - pRot2.r3_c2) <= pEpsilon &&
              std::abs(r3_c3 - pRot2.r3_c3) <= pEpsilon);
    }


    bool Rotation::isRotation(
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


    void Rotation::normalizeRotation(void)
    {
      Math::normalizeRotation(*this);
    }

    Rotation Rotation::transpose() const
    {
      return Math::transpose(*this);
    }

    float Rotation::determinant() const
    {
      return Math::determinant(*this);
    }


    Rotation Rotation::fromQuaternion(
        const float pA,
        const float pB,
        const float pC,
        const float pD)
    {
      return Math::rotationFromQuaternion(pA, pB, pC, pD);
    }

    Rotation Rotation::fromAngleDirection(
        const float pAngle,
        const float pX,
        const float pY,
        const float pZ)
    {
      return Math::rotationFromAngleDirection(pAngle, pX, pY, pZ);
    }

    Rotation Rotation::fromRotX(const float pRotX)
    {
      return Math::rotationFromRotX(pRotX);
    }

    Rotation Rotation::fromRotY(const float pRotY)
    {
      return Math::rotationFromRotY(pRotY);
    }

    Rotation Rotation::fromRotZ(const float pRotZ)
    {
      return Math::rotationFromRotZ(pRotZ);
    }

    Rotation Rotation::from3DRotation(
        const float& pWX,
        const float& pWY,
        const float& pWZ)
    {
      return Math::rotationFrom3DRotation(pWX, pWY, pWZ);
    }

    void Rotation::toVector(std::vector<float>& pReturnVector) const
    {
      pReturnVector.resize(9);
      pReturnVector[0] = r1_c1;
      pReturnVector[1] = r1_c2;
      pReturnVector[2] = r1_c3;

      pReturnVector[3] = r2_c1;
      pReturnVector[4] = r2_c2;
      pReturnVector[5] = r2_c3;

      pReturnVector[6] = r3_c1;
      pReturnVector[7] = r3_c2;
      pReturnVector[8] = r3_c3;
    }

    std::vector<float> Rotation::toVector(void) const
    {
      std::vector<float> returnVector(9, 0.0f);
      this->toVector(returnVector);
      return returnVector;
    }


    Rotation transpose(const Rotation& pIn)
    {
      Rotation pOut;

      pOut.r1_c1 = pIn.r1_c1;
      pOut.r1_c2 = pIn.r2_c1;
      pOut.r1_c3 = pIn.r3_c1;

      pOut.r2_c1 = pIn.r1_c2;
      pOut.r2_c2 = pIn.r2_c2;
      pOut.r2_c3 = pIn.r3_c2;

      pOut.r3_c1 = pIn.r1_c3;
      pOut.r3_c2 = pIn.r2_c3;
      pOut.r3_c3 = pIn.r3_c3;

      return pOut;
    }


    float determinant(const Rotation& pRot)
    {
      return pRot.r1_c1 * pRot.r2_c2 * pRot.r3_c3 +
          pRot.r1_c2 * pRot.r2_c3 * pRot.r3_c1 +
          pRot.r1_c3 * pRot.r2_c1 * pRot.r3_c2 -
          pRot.r1_c1 * pRot.r2_c3 * pRot.r3_c2 -
          pRot.r1_c2 * pRot.r2_c1 * pRot.r3_c3 -
          pRot.r1_c3 * pRot.r2_c2 * pRot.r3_c1;
    }


    Rotation rotationFromQuaternion(
        const float pA,
        const float pB,
        const float pC,
        const float pD)
    {
      Rotation T = Rotation();
      const float t2 =  pA*pB;
      const float t3 =  pA*pC;
      const float t4 =  pA*pD;
      const float t5 = -pB*pB;
      const float t6 =  pB*pC;
      const float t7 =  pB*pD;
      const float t8 = -pC*pC;
      const float t9 =  pC*pD;
      const float t10= -pD*pD;
      T.r1_c1 = 2.0f*(t8 + t10) + 1.0f;
      T.r1_c2 = 2.0f*(t6 - t4);
      T.r1_c3 = 2.0f*(t7 + t3);
      T.r2_c1 = 2.0f*(t6 + t4);
      T.r2_c2 = 2.0f*(t5 + t10) + 1.0f;
      T.r2_c3 = 2.0f*(t9 - t2);
      T.r3_c1 = 2.0f*(t7 - t3);
      T.r3_c2 = 2.0f*(t9 + t2);
      T.r3_c3 = 2.0f*(t5 + t8) + 1.0f;
      return T;
    }

    Rotation rotationFromAngleDirection(
        const float pAngle,
        const float pX,
        const float pY,
        const float pZ)
    {
      // Need to check if |pX^2+pY^2+pZ^2|=1.0

      float norm = pX*pX + pY*pY + pZ*pZ;
      if ((norm > 1.00001f) || (norm < 0.99999f))
      {
        throw std::runtime_error(
              "rotationFromAngleDirection: (pX, pY, pZ) norm must be equal to 1.0.");
      }

      Rotation T = Rotation();
      const float t1 =  std::cos(pAngle);
      const float t2 =  1.0f - t1;
      const float t3 =  pX*pX;
      const float t6 =  t2*pX;
      const float t7 =  t6*pY;
      const float t8 =  std::sin(pAngle);
      const float t9 =  t8*pZ;
      const float t11=  t6*pZ;
      const float t12=  t8*pY;
      const float t15=  pY*pY;
      const float t19=  t2*pY*pZ;
      const float t20=  t8*pX;
      const float t24=  pZ*pZ;
      T.r1_c1 = t1 + t2 * t3;
      T.r1_c2 = t7 - t9;
      T.r1_c3 = t11+t12;
      T.r2_c1 = t7 + t9;
      T.r2_c2 = t1 + t2 * t15;
      T.r2_c3 = t19-t20;
      T.r3_c1 = t11-t12;
      T.r3_c2 = t19+t20;
      T.r3_c3 = t1 + t2 * t24;
      return T;
    }


    void applyRotation(
        const AL::Math::Rotation& pRot,
        float& pX,
        float& pY,
        float& pZ)
    {
      float x = pX;
      float y = pY;
      float z = pZ;
      pX = x*pRot.r1_c1 + y*pRot.r1_c2 + z*pRot.r1_c3;
      pY = x*pRot.r2_c1 + y*pRot.r2_c2 + z*pRot.r2_c3;
      pZ = x*pRot.r3_c1 + y*pRot.r3_c2 + z*pRot.r3_c3;
    }


    Rotation rotationFromRotX(const float pRotX)
    {
      const float c = std::cos(pRotX);
      const float s = std::sin(pRotX);
      Rotation T;
      T.r2_c2 = c;
      T.r2_c3 = -s;
      T.r3_c2 = s;
      T.r3_c3 = c;
      return T;
    }


    Rotation rotationFromRotY(const float pRotY)
    {
      const float c = std::cos(pRotY);
      const float s = std::sin(pRotY);
      Rotation T;
      T.r1_c1 = c;
      T.r1_c3 = s;
      T.r3_c1 = -s;
      T.r3_c3 = c;
      return T;
    }


    Rotation rotationFromRotZ(const float pRotZ)
    {
      const float c = std::cos(pRotZ);
      const float s = std::sin(pRotZ);
      Rotation T;
      T.r1_c1 = c;
      T.r1_c2 = -s;
      T.r2_c1 = s;
      T.r2_c2 = c;
      return T;
    }


    Rotation rotationFrom3DRotation(
        const float& pWX,
        const float& pWY,
        const float& pWZ)
    {
      Rotation T;
      T  = rotationFromRotZ(pWZ);
      T *= rotationFromRotY(pWY);
      T *= rotationFromRotX(pWX);
      return T;
    }

    void normalizeRotation(Rotation& pR)
    {
      const float lEpsilon = 0.0001f;

      // z.normalize();
      float lNorm =
          std::sqrt(std::pow(pR.r1_c3, 2) +
                    std::pow(pR.r2_c3, 2) +
                    std::pow(pR.r3_c3, 2));

      if (lNorm < lEpsilon)
      {
        std::cerr << "ALMath: WARNING: "
                  << "normalizeRotation with null column. "
                  << "Rotation part set to identity." << std::endl;

        pR.r1_c1 = 1.0f;
        pR.r1_c2 = 0.0f;
        pR.r1_c3 = 0.0f;

        pR.r2_c1 = 0.0f;
        pR.r2_c2 = 1.0f;
        pR.r2_c3 = 0.0f;

        pR.r3_c1 = 0.0f;
        pR.r3_c2 = 0.0f;
        pR.r3_c3 = 1.0f;
        return;
      }

      pR.r1_c3 /= lNorm;
      pR.r2_c3 /= lNorm;
      pR.r3_c3 /= lNorm;

      // x = cross(y, z);
      const float x1 = pR.r2_c2*pR.r3_c3 - pR.r3_c2*pR.r2_c3;
      const float x2 = pR.r3_c2*pR.r1_c3 - pR.r1_c2*pR.r3_c3;
      const float x3 = pR.r1_c2*pR.r2_c3 - pR.r2_c2*pR.r1_c3;

      // x.normalize();
      lNorm = std::sqrt(std::pow(x1, 2) +
                        std::pow(x2, 2) +
                        std::pow(x3, 2));

      if (lNorm < lEpsilon)
      {
        std::cerr << "ALMath: WARNING: "
                  << "normalizeRotation with null column. "
                  << "Rotation part set to identity." << std::endl;

        pR.r1_c1 = 1.0f;
        pR.r1_c2 = 0.0f;
        pR.r1_c3 = 0.0f;

        pR.r2_c1 = 0.0f;
        pR.r2_c2 = 1.0f;
        pR.r2_c3 = 0.0f;

        pR.r3_c1 = 0.0f;
        pR.r3_c2 = 0.0f;
        pR.r3_c3 = 1.0f;
        return;
      }

      pR.r1_c1 = x1/lNorm;
      pR.r2_c1 = x2/lNorm;
      pR.r3_c1 = x3/lNorm;

      // y = cross(z, x);
      pR.r1_c2 = pR.r2_c3*pR.r3_c1 - pR.r3_c3*pR.r2_c1;
      pR.r2_c2 = pR.r3_c3*pR.r1_c1 - pR.r1_c3*pR.r3_c1;
      pR.r3_c2 = pR.r1_c3*pR.r2_c1 - pR.r2_c3*pR.r1_c1;
    }

  }
}
