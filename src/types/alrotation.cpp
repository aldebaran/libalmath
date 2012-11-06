/*
 * Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */

#include <almath/types/alrotation.h>

#include <stdexcept>
# include <cmath>

namespace AL {
  namespace Math {

  Rotation::Rotation():
    r1_c1(1.0f), r1_c2(0.0f), r1_c3(0.0f),
    r2_c1(0.0f), r2_c2(1.0f), r2_c3(0.0f),
    r3_c1(0.0f), r3_c2(0.0f), r3_c3(1.0f){}

  Rotation::Rotation (const std::vector<float>& pFloats)
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

    Rotation& Rotation::operator*= (const Rotation& pRot2)
    {
      float c1 = r1_c1;
      float c2 = r1_c2;
      float c3 = r1_c3;

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
      return (fabsf(r1_c1 - pRot2.r1_c1) <= pEpsilon &&
              fabsf(r1_c2 - pRot2.r1_c2) <= pEpsilon &&
              fabsf(r1_c3 - pRot2.r1_c3) <= pEpsilon &&
              fabsf(r2_c1 - pRot2.r2_c1) <= pEpsilon &&
              fabsf(r2_c2 - pRot2.r2_c2) <= pEpsilon &&
              fabsf(r2_c3 - pRot2.r2_c3) <= pEpsilon &&
              fabsf(r3_c1 - pRot2.r3_c1) <= pEpsilon &&
              fabsf(r3_c2 - pRot2.r3_c2) <= pEpsilon &&
              fabsf(r3_c3 - pRot2.r3_c3) <= pEpsilon);
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


    std::vector<float> Rotation::toVector() const
    {
      std::vector<float> returnVector;
      returnVector.resize(9);

      returnVector[0] = r1_c1;
      returnVector[1] = r1_c2;
      returnVector[2] = r1_c3;

      returnVector[3] = r2_c1;
      returnVector[4] = r2_c2;
      returnVector[5] = r2_c3;

      returnVector[6] = r3_c1;
      returnVector[7] = r3_c2;
      returnVector[8] = r3_c3;

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
      float t2 =  pA*pB;
      float t3 =  pA*pC;
      float t4 =  pA*pD;
      float t5 = -pB*pB;
      float t6 =  pB*pC;
      float t7 =  pB*pD;
      float t8 = -pC*pC;
      float t9 =  pC*pD;
      float t10= -pD*pD;
      T.r1_c1 = 2*(t8 + t10) + 1.0f;
      T.r1_c2 = 2*(t6 - t4 );
      T.r1_c3 = 2*(t7 + t3 );
      T.r2_c1 = 2*(t6 + t4 );
      T.r2_c2 = 2*(t5 + t10) + 1.0f;
      T.r2_c3 = 2*(t9 - t2 );
      T.r3_c1 = 2*(t7 - t3 );
      T.r3_c2 = 2*(t9 + t2 );
      T.r3_c3 = 2*(t5 + t8 ) + 1.0f;
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
      float t1 =  cosf(pAngle);
      float t2 =  1.0f - t1;
      float t3 =  pX*pX;
      float t6 =  t2*pX;
      float t7 =  t6*pY;
      float t8 =  sinf(pAngle);
      float t9 =  t8*pZ;
      float t11=  t6*pZ;
      float t12=  t8*pY;
      float t15=  pY*pY;
      float t19=  t2*pY*pZ;
      float t20=  t8*pX;
      float t24=  pZ*pZ;
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
      float c = cosf(pRotX);
      float s = sinf(pRotX);
      Rotation T = Rotation();
      T.r2_c2 = c;
      T.r2_c3 = -s;
      T.r3_c2 = s;
      T.r3_c3 = c;
      return T;
    }


    Rotation rotationFromRotY(const float pRotY)
    {
      float c = cosf(pRotY);
      float s = sinf(pRotY);
      Rotation T = Rotation();
      T.r1_c1 = c;
      T.r1_c3 = s;
      T.r3_c1 = -s;
      T.r3_c3 = c;
      return T;
    }


    Rotation rotationFromRotZ(const float pRotZ)
    {
      float c = cosf(pRotZ);
      float s = sinf(pRotZ);
      Rotation T = Rotation();
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
      Rotation T = Rotation();
      T  = rotationFromRotZ(pWZ);
      T *= rotationFromRotY(pWY);
      T *= rotationFromRotX(pWX);
      return T;
    }

  }
}
