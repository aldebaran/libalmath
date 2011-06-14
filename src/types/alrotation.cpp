/**
* @author Cyrille Collette
* Copyright (c) Aldebaran Robotics 2011 All Rights Reserved
*/

#include <almath/types/alrotation.h>

# include <cmath>

namespace AL {
  namespace Math {

    Rotation& Rotation::operator*= (const Rotation& pT2)
    {
      float c1 = r1_c1;
      float c2 = r1_c2;
      float c3 = r1_c3;

      r1_c1 = (c1 * pT2.r1_c1) + (c2 * pT2.r2_c1) + (c3 * pT2.r3_c1);
      r1_c2 = (c1 * pT2.r1_c2) + (c2 * pT2.r2_c2) + (c3 * pT2.r3_c2);
      r1_c3 = (c1 * pT2.r1_c3) + (c2 * pT2.r2_c3) + (c3 * pT2.r3_c3);

      c1 = r2_c1;
      c2 = r2_c2;
      c3 = r2_c3;

      r2_c1 = (c1 * pT2.r1_c1) + (c2 * pT2.r2_c1) + (c3 * pT2.r3_c1);
      r2_c2 = (c1 * pT2.r1_c2) + (c2 * pT2.r2_c2) + (c3 * pT2.r3_c2);
      r2_c3 = (c1 * pT2.r1_c3) + (c2 * pT2.r2_c3) + (c3 * pT2.r3_c3);

      c1 = r3_c1;
      c2 = r3_c2;
      c3 = r3_c3;

      r3_c1 = (c1 * pT2.r1_c1) + (c2 * pT2.r2_c1) + (c3 * pT2.r3_c1);
      r3_c2 = (c1 * pT2.r1_c2) + (c2 * pT2.r2_c2) + (c3 * pT2.r3_c2);
      r3_c3 = (c1 * pT2.r1_c3) + (c2 * pT2.r2_c3) + (c3 * pT2.r3_c3);
      return *this;
    }


    Rotation Rotation::operator* (const Rotation& pT2) const
    {
      Rotation pOut = *this;
      pOut *= pT2;
      return pOut;
    }


    bool Rotation::isNear(
      const Rotation& pRot,
      const float&    pEpsilon) const
    {
      if (
        (fabsf(r1_c1 - pRot.r1_c1) > pEpsilon) ||
        (fabsf(r1_c2 - pRot.r1_c2) > pEpsilon) ||
        (fabsf(r1_c3 - pRot.r1_c3) > pEpsilon) ||
        (fabsf(r2_c1 - pRot.r2_c1) > pEpsilon) ||
        (fabsf(r2_c2 - pRot.r2_c2) > pEpsilon) ||
        (fabsf(r2_c3 - pRot.r2_c3) > pEpsilon) ||
        (fabsf(r3_c1 - pRot.r3_c1) > pEpsilon) ||
        (fabsf(r3_c2 - pRot.r3_c2) > pEpsilon) ||
        (fabsf(r3_c3 - pRot.r3_c3) > pEpsilon))
      {
        return false;
      }
      else
      {
        return true;
      }
    }


    Rotation Rotation::transpose() const
    {
      return Math::Transpose(*this);
    }

    float Rotation::determinant() const
    {
      return Math::Determinant(*this);
    }


    Rotation Rotation::fromQuaternion(
      const float pA,
      const float pB,
      const float pC,
      const float pD)
    {
      return Math::RotationFromQuaternion(pA, pB, pC, pD);
    }

    Rotation Rotation::fromAngleDirection(
      const float pAngle,
      const float pX,
      const float pY,
      const float pZ)
    {
      return Math::RotationFromAngleDirection(pAngle, pX, pY, pZ);
    }

    Rotation Rotation::fromRotX(const float pRotX)
    {
      return Math::RotationFromRotX(pRotX);
    }

    Rotation Rotation::fromRotY(const float pRotY)
    {
      return Math::RotationFromRotY(pRotY);
    }

    Rotation Rotation::fromRotZ(const float pRotZ)
    {
      return Math::RotationFromRotZ(pRotZ);
    }

    Rotation Rotation::from3DRotation(
      const float& pWX,
      const float& pWY,
      const float& pWZ)
    {
      return Math::RotationFrom3DRotation(pWX, pWY, pWZ);
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


    Rotation Transpose(const Rotation& pIn)
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


    float Determinant(const Rotation& pM)
    {
      float det;

      det = pM.r1_c1 * pM.r2_c2 * pM.r3_c3 +
        pM.r1_c2 * pM.r2_c3 * pM.r3_c1 +
        pM.r1_c3 * pM.r2_c1 * pM.r3_c2 -
        pM.r1_c1 * pM.r2_c3 * pM.r3_c2 -
        pM.r1_c2 * pM.r2_c1 * pM.r3_c3 -
        pM.r1_c3 * pM.r2_c2 * pM.r3_c1;

      return det;
    }


    Rotation RotationFromQuaternion(
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

    Rotation RotationFromAngleDirection(
      const float pAngle,
      const float pX,
      const float pY,
      const float pZ)
    {
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


    void ApplyRotation(
      AL::Math::Rotation &pRotation,
      float & pX,
      float & pY,
      float & pZ)
    {
      float x = pX;
      float y = pY;
      float z = pZ;
      pX = x*pRotation.r1_c1 + y*pRotation.r1_c2 + z*pRotation.r1_c3;
      pY = x*pRotation.r2_c1 + y*pRotation.r2_c2 + z*pRotation.r2_c3;
      pZ = x*pRotation.r3_c1 + y*pRotation.r3_c2 + z*pRotation.r3_c3;
    }


    Rotation RotationFromRotX(const float pTheta)
    {
      float c = cosf(pTheta);
      float s = sinf(pTheta);
      Rotation T = Rotation();
      T.r2_c2 = c;
      T.r2_c3 = -s;
      T.r3_c2 = s;
      T.r3_c3 = c;
      return T;
    }


    Rotation RotationFromRotY(const float pTheta)
    {
      float c = cosf(pTheta);
      float s = sinf(pTheta);
      Rotation T = Rotation();
      T.r1_c1 = c;
      T.r1_c3 = s;
      T.r3_c1 = -s;
      T.r3_c3 = c;
      return T;
    }


    Rotation RotationFromRotZ(const float pTheta)
    {
      float c = cosf(pTheta);
      float s = sinf(pTheta);
      Rotation T = Rotation();
      T.r1_c1 = c;
      T.r1_c2 = -s;
      T.r2_c1 = s;
      T.r2_c2 = c;
      return T;
    }


    Rotation RotationFrom3DRotation(
      const float& pWX,
      const float& pWY,
      const float& pWZ)
    {
      Rotation T = Rotation();
      T = RotationFromRotZ(pWZ);
      T *= RotationFromRotY(pWY);
      T *= RotationFromRotX(pWX);
      return T;
    }

  }
}
