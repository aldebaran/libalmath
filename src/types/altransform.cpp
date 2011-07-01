/**
* @author Chris Kilner and Cyrille Collette
* Copyright (c) Aldebaran Robotics 2007 All Rights Reserved
*/

#include <almath/types/altransform.h>
#include <cmath>

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
      const Transform& pT2,
      const float&     pEpsilon)const
    {

      if (
        (fabsf(r1_c1 - pT2.r1_c1) > pEpsilon) ||
        (fabsf(r1_c2 - pT2.r1_c2) > pEpsilon) ||
        (fabsf(r1_c3 - pT2.r1_c3) > pEpsilon) ||
        (fabsf(r2_c1 - pT2.r2_c1) > pEpsilon) ||
        (fabsf(r2_c2 - pT2.r2_c2) > pEpsilon) ||
        (fabsf(r2_c3 - pT2.r2_c3) > pEpsilon) ||
        (fabsf(r3_c1 - pT2.r3_c1) > pEpsilon) ||
        (fabsf(r3_c2 - pT2.r3_c2) > pEpsilon) ||
        (fabsf(r3_c3 - pT2.r3_c3) > pEpsilon) ||
        (fabsf(r1_c4 - pT2.r1_c4) > pEpsilon) ||
        (fabsf(r2_c4 - pT2.r2_c4) > pEpsilon) ||
        (fabsf(r3_c4 - pT2.r3_c4) > pEpsilon))
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
      // det(R) = 1

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


    Transform Transform::fromRotX(const float pRotX)
    {
      return Math::TransformFromRotX(pRotX);
    }

    Transform Transform::fromRotY(const float pRotY)
    {
      return Math::TransformFromRotY(pRotY);
    }

    Transform Transform::fromRotZ(const float pRotZ)
    {
      return Math::TransformFromRotZ(pRotZ);
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

    float Transform::distanceSquared(const Transform& pT2) const
    {
      return Math::TransformDistanceSquared(*this, pT2);
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


    float norm(const Transform& pT)
    {
      return sqrtf( (pT.r1_c4*pT.r1_c4) + (pT.r2_c4*pT.r2_c4) + (pT.r3_c4*pT.r3_c4) );
    }


    void TransformToFloatVector(
      const Transform&    pT,
      std::vector<float>& pTOut)
    {
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

    std::vector<float> TransformToFloatVector(
      const Transform&  pT)
    {
      std::vector<float> pTOut;
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
      return pTOut;
    }

    float Determinant(const Transform& pT)
    {
      float det;

      det = pT.r1_c1 * pT.r2_c2 * pT.r3_c3 +
        pT.r1_c2 * pT.r2_c3 * pT.r3_c1 +
        pT.r1_c3 * pT.r2_c1 * pT.r3_c2 -
        pT.r1_c1 * pT.r2_c3 * pT.r3_c2 -
        pT.r1_c2 * pT.r2_c1 * pT.r3_c3 -
        pT.r1_c3 * pT.r2_c2 * pT.r3_c1;

      return det;
    }

    float Determinant(const std::vector<float>& pT)
    {
      float det;

      det = pT[0] * pT[5] * pT[10] +
        pT[1] * pT[6] * pT[8] +
        pT[2] * pT[4] * pT[9] -
        pT[0] * pT[6] * pT[9] -
        pT[1] * pT[4] * pT[10] -
        pT[2] * pT[5] * pT[8];

      return det;
    }

    void TransformInverse(
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

    Transform TransformInverse(const Transform& pT)
    {
      Transform pTOut;
      TransformInverse(pT, pTOut);
      return pTOut;
    }

    Transform TransformFromRotX(const float pRotX)
    {
      float c = cosf(pRotX);
      float s = sinf(pRotX);
      Transform T = Transform();
      T.r2_c2 = c;
      T.r2_c3 = -s;
      T.r3_c2 = s;
      T.r3_c3 = c;
      return T;
    }

    Transform TransformFromRotY(const float pRotY)
    {
      float c = cosf(pRotY);
      float s = sinf(pRotY);
      Transform T = Transform();
      T.r1_c1 = c;
      T.r1_c3 = s;
      T.r3_c1 = -s;
      T.r3_c3 = c;
      return T;
    }


    Transform TransformFromRotZ(const float pRotZ)
    {
      float c = cosf(pRotZ);
      float s = sinf(pRotZ);
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
      return result;
    }

    float TransformDistanceSquared(
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
      return sqrtf(TransformDistanceSquared(pT1, pT2));
    } // end TransformDistance

  } // end namespace Math
} // end namespace AL

