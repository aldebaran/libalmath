/**
* @author Chris Kilner
* Copyright (c) Aldebaran Robotics 2009 All Rights Reserved
*/

#include <almath/types/alquaternion.h>
#include <cmath>
#include <stdexcept>

namespace AL {
  namespace Math {

    Quaternion::Quaternion() : w(0.0f), x(0.0f), y(0.0f), z(0.0f) {}

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

  } // end namespace math
} // end namespace al

