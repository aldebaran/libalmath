/**
* @author Chris Kilner
* Copyright (c) Aldebaran Robotics 2009 All Rights Reserved
*/

#include <almath/types/alquaternion.h>
#include <cmath>
#include <stdexcept>
#include <almath/tools/altrigonometry.h>

namespace AL {
  namespace Math {

    Quaternion::Quaternion() : w(1.0f), x(0.0f), y(0.0f), z(0.0f) {}

    Quaternion::Quaternion(
      float pW,
      float pX,
      float pY,
      float pZ):
      w(pW), x(pX), y(pY), z(pZ) {}

    Quaternion::Quaternion(const std::vector<float>& pFloats)
    {
      if (pFloats.size() == 4)
      {
        w = pFloats[0];
        x = pFloats[1];
        y = pFloats[2];
        z = pFloats[3];
      }
      else
      {
        w = 0.0f;
        x = 0.0f;
        y = 0.0f;
        z = 0.0f;
      }
    }

    Quaternion& Quaternion::operator*= (const Quaternion& pQua2)
    {
      float w1 = w;
      float x1 = x;
      float y1 = y;
      float z1 = z;
      w = w1*pQua2.w - x1*pQua2.x - y1*pQua2.y - z1*pQua2.z;
      x = w1*pQua2.x + pQua2.w*x1 + y1*pQua2.z - z1*pQua2.y;
      y = w1*pQua2.y + pQua2.w*y1 + z1*pQua2.x - x1*pQua2.z;
      z = w1*pQua2.z + pQua2.w*z1 + x1*pQua2.y - y1*pQua2.x;
      return *this;
    }

    Quaternion Quaternion::operator* (const Quaternion& pQua2) const
    {
      Quaternion qua;
      qua.w = w*pQua2.w - x*pQua2.x - y*pQua2.y - z*pQua2.z;
      qua.x = w*pQua2.x + pQua2.w*x + y*pQua2.z - z*pQua2.y;
      qua.y = w*pQua2.y + pQua2.w*y + z*pQua2.x - x*pQua2.z;
      qua.z = w*pQua2.z + pQua2.w*z + x*pQua2.y - y*pQua2.x;
      return qua;
    }


    bool Quaternion::isNear(
      const Quaternion& pQua2,
      const float&      pEpsilon) const
    {
      if (
        (fabsf(w - pQua2.w) > pEpsilon) ||
        (fabsf(x - pQua2.x) > pEpsilon) ||
        (fabsf(y - pQua2.y) > pEpsilon) ||
        (fabsf(z - pQua2.z) > pEpsilon))
      {
        return false;
      }
      else
      {
        return true;
      }
    }


    Quaternion& Quaternion::operator*= (float pVal)
    {
      w *= pVal;
      x *= pVal;
      y *= pVal;
      z *= pVal;
      return *this;
    }


    Quaternion& Quaternion::operator/= (float pVal)
    {
      if (pVal == 0.0f)
      {
        throw std::runtime_error(
          "ALQuaternion: operator/= Division by zeros.");
      }
      *this *= (1.0f/pVal);
      return *this;
    }


    bool Quaternion::operator== (const Quaternion& pQua2) const
    {
      if(
        (w == pQua2.w) &&
        (x == pQua2.x) &&
        (y == pQua2.y) &&
        (z == pQua2.z))
      {
        return true;
      }
      else
      {
        return false;
      }
    }

    bool Quaternion::operator!= (const Quaternion& pQua2) const
    {
      return !(*this==pQua2);
    }


    Quaternion quaternionFromAngleAndAxisRotation(
      const float pAngle,
      const float pAxisX,
      const float pAxisY,
      const float pAxisZ)
    {
      Quaternion qua = Quaternion();

      float sin_a = sinf(0.5f*pAngle);
      float cos_a = cosf(0.5f*pAngle);

      qua.w = cos_a;
      qua.x = pAxisX*sin_a;
      qua.y = pAxisY*sin_a;
      qua.z = pAxisZ*sin_a;

      qua.normalize();
      return qua;
    }


    Quaternion Quaternion::fromAngleAndAxisRotation(
      const float pAngle,
      const float pAxisX,
      const float pAxisY,
      const float pAxisZ)
    {
      return Math::quaternionFromAngleAndAxisRotation(
            pAngle,
            pAxisX,
            pAxisY,
            pAxisZ);
    }


    std::vector<float> Quaternion::toVector() const
    {
      std::vector<float> returnVector;
      returnVector.resize(4);
      returnVector[0] = w;
      returnVector[1] = x;
      returnVector[2] = y;
      returnVector[3] = z;

      return returnVector;
    }

    float Quaternion::norm() const
    {
      return Math::norm(*this);
    }

    Quaternion Quaternion::normalize() const
    {
      return Math::normalize(*this);
    }

    float norm(const Quaternion& p)
    {
      return sqrtf( (p.w*p.w) + (p.x*p.x) + (p.y*p.y) + (p.z*p.z) );
    }

    Quaternion normalize(const Quaternion& pQua)
    {
      Quaternion ret;
      ret = pQua;
      float tmpNorm = norm(pQua);

      if (tmpNorm == 0.0f)
      {
        throw std::runtime_error(
          "ALQuaternion: normalize Division by zeros.");
      }

      ret /= tmpNorm;
      return ret;
    }


    Quaternion Quaternion::inverse() const
    {
      return Math::quaternionInverse(*this);
    }


    void quaternionInverse(
      const Quaternion& pQua,
      Quaternion&       pQuaOut)
    {
      pQuaOut.w = pQua.w;
      pQuaOut.x = -pQua.x;
      pQuaOut.y = -pQua.y;
      pQuaOut.z = -pQua.z;
    }


    Quaternion quaternionInverse(const Quaternion& pQua)
    {
      Quaternion pQuaOut;
      quaternionInverse(pQua, pQuaOut);
      return pQuaOut;
    }


    void angleAndAxisRotationFromQuaternion(
      const Quaternion& pQuaternion,
      float& pAngle,
      float& pAxisX,
      float& pAxisY,
      float& pAxisZ)
    {
      Quaternion copy = pQuaternion;
      copy.normalize();
      pAngle      = acos(copy.w) * AL::Math::_2_PI_;
      float sin_angle  = sqrtf( 1.0 - powf(copy.w,2));

      if (fabsf(sin_angle) < 0.0005f)
      {
        sin_angle = 1.0f;
      }
      pAxisX = copy.x / sin_angle;
      pAxisY = copy.y / sin_angle;
      pAxisZ = copy.z / sin_angle;
    }

  } // end namespace math
} // end namespace al

