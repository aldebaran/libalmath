/**
* @author Chris Kilner and Cyrille Collette
* Copyright (c) Aldebaran Robotics 2007 All Rights Reserved
*/

#include <almath/types/altransform.h>
#include "math.h"

namespace AL {
  namespace Math {

    Transform& Transform::operator*= (const Transform& pT2)
    {
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
      if (
        (r1_c1 == pT2.r1_c1) &&
        (r1_c2 == pT2.r1_c2) &&
        (r1_c3 == pT2.r1_c3) &&
        (r1_c4 == pT2.r1_c4) &&
        (r2_c1 == pT2.r2_c1) &&
        (r2_c2 == pT2.r2_c2) &&
        (r2_c3 == pT2.r2_c3) &&
        (r2_c4 == pT2.r2_c4) &&
        (r3_c1 == pT2.r3_c1) &&
        (r3_c2 == pT2.r3_c2) &&
        (r3_c3 == pT2.r3_c3) &&
        (r3_c4 == pT2.r3_c4))
      {
        return true;
      }
      else
      {
        return false;
      }
    }

    bool Transform::operator!=(const Transform& pT2) const
    {
      return !(*this==pT2);
    }


    bool Transform::isNear(
      const Transform& pH,
      const float&     pEpsilon)const
    {

      if (
        (fabsf(r1_c1 - pH.r1_c1) > pEpsilon) ||
        (fabsf(r1_c2 - pH.r1_c2) > pEpsilon) ||
        (fabsf(r1_c3 - pH.r1_c3) > pEpsilon) ||
        (fabsf(r2_c1 - pH.r2_c1) > pEpsilon) ||
        (fabsf(r2_c2 - pH.r2_c2) > pEpsilon) ||
        (fabsf(r2_c3 - pH.r2_c3) > pEpsilon) ||
        (fabsf(r3_c1 - pH.r3_c1) > pEpsilon) ||
        (fabsf(r3_c2 - pH.r3_c2) > pEpsilon) ||
        (fabsf(r3_c3 - pH.r3_c3) > pEpsilon) ||
        (fabsf(r1_c4 - pH.r1_c4) > pEpsilon) ||
        (fabsf(r2_c4 - pH.r2_c4) > pEpsilon) ||
        (fabsf(r3_c4 - pH.r3_c4) > pEpsilon))
      {
        return false;
      }
      else
      {
        return true;
      }
    }


    bool Transform::isTransform(
      const float& pEpsilon) const
    {
      // This checks that the input is a pure rotation matrix 'm'.
      // The condition for this is:
      // R' * R = I
      // and
      // det(R) =1

      // det is defined here:
      // http://www.euclideanspace.com/maths/algebra/matrix/functions/determinant/threeD/

      if (fabsf(r1_c2*r1_c1 + r2_c2*r2_c1 + r3_c2*r3_c1) > pEpsilon)
      {
        return false;
      }

      if (fabsf(r1_c3*r1_c1 + r2_c3*r2_c1 + r3_c3*r3_c1) > pEpsilon)
      {
        return false;
      }

      if (fabsf(r1_c3*r1_c2 + r2_c3*r2_c2 + r3_c3*r3_c2) > pEpsilon)
      {
        return false;
      }

      if (fabsf(r1_c1*r1_c1 + r1_c2*r1_c2 + r1_c3*r1_c3 - 1.0f) > pEpsilon)
      {
        return false;
      }

      if (fabsf(r2_c1*r2_c1 + r2_c2*r2_c2 + r2_c3*r2_c3 - 1.0f) > pEpsilon)
      {
        return false;
      }

      if (fabsf(r3_c1*r3_c1 + r3_c2*r3_c2 + r3_c3*r3_c3 - 1.0f) > pEpsilon)
      {
        return false;
      }

      float det = Math::Determinant(*this);

      if (fabsf(det-1.0f) > pEpsilon)
      {
        return false;
      }
      else
      {
        return true;
      }
    } // end isTransform


    float Transform::norm() const
    {
      return Math::norm(*this);
    }

    float Transform::determinant() const
    {
      return Math::Determinant(*this);
    }

    Transform Transform::inverse() const
    {
      return Math::TransformInverse(*this);
    }


    Transform Transform::fromRotX(const float rotx)
    {
      return Math::TransformFromRotX(rotx);
    }

    Transform Transform::fromRotY(const float roty)
    {
      return Math::TransformFromRotY(roty);
    }

    Transform Transform::fromRotZ(const float rotz)
    {
      return Math::TransformFromRotZ(rotz);
    }


    Transform Transform::from3DRotation(
      const float& pWX,
      const float& pWY,
      const float& pWZ)
    {
      return Math::TransformFrom3DRotation(pWX, pWY, pWZ);
    }

    Transform Transform::fromPosition(
      const float pX,
      const float pY,
      const float pZ)
    {
      return Math::TransformFromPosition(pX, pY, pZ);
    }

    Transform Transform::fromPosition(
      const float& pX,
      const float& pY,
      const float& pZ,
      const float& pWX,
      const float& pWY,
      const float& pWZ)
    {
      return Math::TransformFromPosition(pX, pY, pZ, pWX, pWY, pWZ);
    }


    Transform Transform::diff(const Transform &pT2) const
    {
      return Math::TransformDiff(*this, pT2);
    }

    float Transform::squaredDistance(const Transform& pT2) const
    {
      return Math::TransformSqaredDistance(*this, pT2);
    }

    float Transform::distance(const Transform& pT2) const
    {
      return Math::TransformDistance(*this, pT2);
    }

    std::vector<float> Transform::toVector() const
    {
      std::vector<float> returnVector;
      returnVector.resize(16);
      returnVector[0]  = r1_c1;
      returnVector[1]  = r1_c2;
      returnVector[2]  = r1_c3;
      returnVector[3]  = r1_c4;

      returnVector[4]  = r2_c1;
      returnVector[5]  = r2_c2;
      returnVector[6]  = r2_c3;
      returnVector[7]  = r2_c4;

      returnVector[8]  = r3_c1;
      returnVector[9]  = r3_c2;
      returnVector[10] = r3_c3;
      returnVector[11] = r3_c4;

      returnVector[12] = 0.0f;
      returnVector[13] = 0.0f;
      returnVector[14] = 0.0f;
      returnVector[15] = 1.0f;
      return returnVector;
    }

    void TransformPreMultiply(
      const Transform& pT1,
      Transform&       pResult)
    {
      float r1 = pResult.r1_c1;
      float r2 = pResult.r2_c1;
      pResult.r1_c1 = (pT1.r1_c1 * r1) + (pT1.r1_c2 * r2) + (pT1.r1_c3 * pResult.r3_c1);
      pResult.r2_c1 = (pT1.r2_c1 * r1) + (pT1.r2_c2 * r2) + (pT1.r2_c3 * pResult.r3_c1);
      pResult.r3_c1 = (pT1.r3_c1 * r1) + (pT1.r3_c2 * r2) + (pT1.r3_c3 * pResult.r3_c1);

      r1 = pResult.r1_c2;
      r2 = pResult.r2_c2;
      pResult.r1_c2 = (pT1.r1_c1 * r1) + (pT1.r1_c2 * r2) + (pT1.r1_c3 * pResult.r3_c2);
      pResult.r2_c2 = (pT1.r2_c1 * r1) + (pT1.r2_c2 * r2) + (pT1.r2_c3 * pResult.r3_c2);
      pResult.r3_c2 = (pT1.r3_c1 * r1) + (pT1.r3_c2 * r2) + (pT1.r3_c3 * pResult.r3_c2);

      r1 = pResult.r1_c3;
      r2 = pResult.r2_c3;
      pResult.r1_c3 = (pT1.r1_c1 * r1) + (pT1.r1_c2 * r2) + (pT1.r1_c3 * pResult.r3_c3);
      pResult.r2_c3 = (pT1.r2_c1 * r1) + (pT1.r2_c2 * r2) + (pT1.r2_c3 * pResult.r3_c3);
      pResult.r3_c3 = (pT1.r3_c1 * r1) + (pT1.r3_c2 * r2) + (pT1.r3_c3 * pResult.r3_c3);

      r1 = pResult.r1_c4;
      r2 = pResult.r2_c4;
      pResult.r1_c4 = (pT1.r1_c1 * r1) + (pT1.r1_c2 * r2) + (pT1.r1_c3 * pResult.r3_c4) + pT1.r1_c4;
      pResult.r2_c4 = (pT1.r2_c1 * r1) + (pT1.r2_c2 * r2) + (pT1.r2_c3 * pResult.r3_c4) + pT1.r2_c4;
      pResult.r3_c4 = (pT1.r3_c1 * r1) + (pT1.r3_c2 * r2) + (pT1.r3_c3 * pResult.r3_c4) + pT1.r3_c4;
    }


    float norm(const Transform& pH)
    {
      return sqrtf( (pH.r1_c4*pH.r1_c4) + (pH.r2_c4*pH.r2_c4) + (pH.r3_c4*pH.r3_c4) );
    }


    void TransformToFloatVector(
      const Transform&  pT,
      std::vector<float>& pOut)
    {
      pOut.resize(12);
      pOut[0]  = pT.r1_c1;
      pOut[1]  = pT.r1_c2;
      pOut[2]  = pT.r1_c3;
      pOut[3]  = pT.r1_c4;
      pOut[4]  = pT.r2_c1;
      pOut[5]  = pT.r2_c2;
      pOut[6]  = pT.r2_c3;
      pOut[7]  = pT.r2_c4;
      pOut[8]  = pT.r3_c1;
      pOut[9]  = pT.r3_c2;
      pOut[10] = pT.r3_c3;
      pOut[11] = pT.r3_c4;
    }

    std::vector<float> TransformToFloatVector(
      const Transform&  pT)
    {
      std::vector<float> pOut;
      pOut.resize(12);
      pOut[0]  = pT.r1_c1;
      pOut[1]  = pT.r1_c2;
      pOut[2]  = pT.r1_c3;
      pOut[3]  = pT.r1_c4;
      pOut[4]  = pT.r2_c1;
      pOut[5]  = pT.r2_c2;
      pOut[6]  = pT.r2_c3;
      pOut[7]  = pT.r2_c4;
      pOut[8]  = pT.r3_c1;
      pOut[9]  = pT.r3_c2;
      pOut[10] = pT.r3_c3;
      pOut[11] = pT.r3_c4;
      return pOut;
    }

    float Determinant(const Transform& pH)
    {
      float det;

      det = pH.r1_c1 * pH.r2_c2 * pH.r3_c3 +
        pH.r1_c2 * pH.r2_c3 * pH.r3_c1 +
        pH.r1_c3 * pH.r2_c1 * pH.r3_c2 -
        pH.r1_c1 * pH.r2_c3 * pH.r3_c2 -
        pH.r1_c2 * pH.r2_c1 * pH.r3_c3 -
        pH.r1_c3 * pH.r2_c2 * pH.r3_c1;

      return det;
    }

    float Determinant(const std::vector<float>& pH)
    {
      float det;

      det = pH[0] * pH[5] * pH[10] +
        pH[1] * pH[6] * pH[8] +
        pH[2] * pH[4] * pH[9] -
        pH[0] * pH[6] * pH[9] -
        pH[1] * pH[4] * pH[10] -
        pH[2] * pH[5] * pH[8];

      return det;
    }

    void TransformInverse(
      const Transform& pIn,
      Transform&       pOut)
    {
      // rotation Ri = R'
      pOut.r1_c1 = pIn.r1_c1;
      pOut.r1_c2 = pIn.r2_c1;
      pOut.r1_c3 = pIn.r3_c1;
      pOut.r2_c1 = pIn.r1_c2;
      pOut.r2_c2 = pIn.r2_c2;
      pOut.r2_c3 = pIn.r3_c2;
      pOut.r3_c1 = pIn.r1_c3;
      pOut.r3_c2 = pIn.r2_c3;
      pOut.r3_c3 = pIn.r3_c3;

      // translation ri = -R'*r
      pOut.r1_c4 = -( pIn.r1_c1*pIn.r1_c4 + pIn.r2_c1*pIn.r2_c4 + pIn.r3_c1*pIn.r3_c4 );
      pOut.r2_c4 = -( pIn.r1_c2*pIn.r1_c4 + pIn.r2_c2*pIn.r2_c4 + pIn.r3_c2*pIn.r3_c4 );
      pOut.r3_c4 = -( pIn.r1_c3*pIn.r1_c4 + pIn.r2_c3*pIn.r2_c4 + pIn.r3_c3*pIn.r3_c4 );
    }

    Transform TransformInverse(const Transform& pIn)
    {
      Transform pOut;
      TransformInverse(pIn, pOut);
      return pOut;
    }

    Transform TransformFromRotX(const float pTheta)
    {
      float c = cosf(pTheta);
      float s = sinf(pTheta);
      Transform T = Transform();
      T.r2_c2 = c;
      T.r2_c3 = -s;
      T.r3_c2 = s;
      T.r3_c3 = c;
      return T;
    }

    Transform TransformFromRotY(const float pTheta)
    {
      float c = cosf(pTheta);
      float s = sinf(pTheta);
      Transform T = Transform();
      T.r1_c1 = c;
      T.r1_c3 = s;
      T.r3_c1 = -s;
      T.r3_c3 = c;
      return T;
    }


    Transform TransformFromRotZ(const float pTheta)
    {
      float c = cosf(pTheta);
      float s = sinf(pTheta);
      Transform T = Transform();
      T.r1_c1 = c;
      T.r1_c2 = -s;
      T.r2_c1 = s;
      T.r2_c2 = c;
      return T;
    }


    Transform TransformFrom3DRotation(
      const float& pWX,
      const float& pWY,
      const float& pWZ)
    {
      Transform T = Transform();
      T = TransformFromRotZ(pWZ);
      T *= TransformFromRotY(pWY);
      T *= TransformFromRotX(pWX);
      return T;
    }

    Transform TransformFromPosition(
      const float x,
      const float y,
      const float z)
    {
      Transform T = Transform();
      T.r1_c4 = x;
      T.r2_c4 = y;
      T.r3_c4 = z;
      return T;
    }


    Transform TransformFromPosition(
      const float& pX,
      const float& pY,
      const float& pZ,
      const float& pWX,
      const float& pWY,
      const float& pWZ)
    {
      Transform T = TransformFrom3DRotation(pWX, pWY, pWZ);

      T.r1_c4 = pX;
      T.r2_c4 = pY;
      T.r3_c4 = pZ;
      return T;
    }

    void TransformInvertInPlace(Transform& pT)
    {
      float tmp0;
      tmp0 = pT.r1_c2;
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
      TransformInvertInPlace(result);
      return result;
    }

    Transform TransformDiff(
      const Transform& pT1,
      const Transform& pT2)
    {
      Transform result = pT1;

      TransformInvertInPlace(result);
      result *= pT2;

      // bad code
      //result.r1_c1 = 1.0f;
      //result.r2_c2 = 1.0f;
      //result.r3_c3 = 1.0f;

      //result.r1_c4 = pT2.r1_c4 - pT1.r1_c4;
      //result.r2_c4 = pT2.r2_c4 - pT1.r2_c4;
      //result.r3_c4 = pT2.r3_c4 - pT1.r3_c4;

      //result.r2_c3 = 0.5f * ( ((pT1.r2_c1 * pT2.r3_c1) - (pT1.r3_c1 * pT2.r2_c1)) +
      //  ((pT1.r2_c2 * pT2.r3_c2) - (pT1.r3_c2 * pT2.r2_c2)) +
      //  ((pT1.r2_c3 * pT2.r3_c3) - (pT1.r3_c3 * pT2.r2_c3)) );
      //result.r3_c2 = -result.r2_c3;

      //result.r1_c3 = 0.5f * ( ((pT1.r3_c1 * pT2.r1_c1) - (pT1.r1_c1 * pT2.r3_c1)) +
      //  ((pT1.r3_c2 * pT2.r1_c2) - (pT1.r1_c2 * pT2.r3_c2)) +
      //  ((pT1.r3_c3 * pT2.r1_c3) - (pT1.r1_c3 * pT2.r3_c3)) );
      //result.r3_c1 = -result.r1_c3;

      //result.r2_c1 = 0.5f * ( ((pT1.r1_c1 * pT2.r2_c1) - (pT1.r2_c1 * pT2.r1_c1)) +
      //  ((pT1.r1_c2 * pT2.r2_c2) - (pT1.r2_c2 * pT2.r1_c2)) +
      //  ((pT1.r1_c3 * pT2.r2_c3) - (pT1.r2_c3 * pT2.r1_c3)) );
      //result.r1_c2 = - result.r2_c1;
      return result;
    }

    float TransformSqaredDistance(
      const Transform& pT1,
      const Transform& pT2)
    {
      float tmp = 0.0f;
      float tot = 0.0f;
      tmp = pT1.r1_c4 - pT2.r1_c4;
      tot += tmp * tmp;
      tmp = pT1.r2_c4 - pT2.r2_c4;
      tot += tmp * tmp;
      tmp = pT1.r3_c4 - pT2.r3_c4;
      tot += tmp * tmp;
      return tot;
    }

    float TransformDistance(
      const Transform& pT1,
      const Transform& pT2)
    {
      return sqrtf(TransformSqaredDistance(pT1, pT2));
    } // end TransformDistance

  } // end namespace Math
} // end namespace AL

