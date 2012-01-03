/*
 * Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */

#include <almath/types/alposition6d.h>
#include <cmath>
#include <stdexcept>

namespace AL {
  namespace Math {

    Position6D::Position6D() : x(0.0f),
      y(0.0f),
      z(0.0f),
      wx(0.0f),
      wy(0.0f),
      wz(0.0f) {}

    Position6D::Position6D(float pInit) : x(pInit),
      y(pInit),
      z(pInit),
      wx(pInit),
      wy(pInit),
      wz(pInit) {}

    Position6D::Position6D(
      float pX,
      float pY,
      float pZ,
      float pWx,
      float pWy,
      float pWz) : x(pX),
      y(pY),
      z(pZ),
      wx(pWx),
      wy(pWy),
      wz(pWz) {}

    Position6D::Position6D(const std::vector<float>& pFloats)
    {
      if (pFloats.size() == 6)
      {
        x = pFloats[0];
        y = pFloats[1];
        z = pFloats[2];

        wx = pFloats[3];
        wy = pFloats[4];
        wz = pFloats[5];
      }
      else
      {
        x = 0.0f;
        y = 0.0f;
        z = 0.0f;

        wx = 0.0f;
        wy = 0.0f;
        wz = 0.0f;
      }
    }

    Position6D Position6D::operator+ (const Position6D& pPos2) const
    {
      Position6D res;
      res.x  = x + pPos2.x;
      res.y  = y + pPos2.y;
      res.z  = z + pPos2.z;
      res.wx = wx + pPos2.wx;
      res.wy = wy + pPos2.wy;
      res.wz = wz + pPos2.wz;
      return res;
    }

    Position6D Position6D::operator- (const Position6D& pPos2) const
    {
      Position6D res;
      res.x  = x - pPos2.x;
      res.y  = y - pPos2.y;
      res.z  = z - pPos2.z;
      res.wx = wx - pPos2.wx;
      res.wy = wy - pPos2.wy;
      res.wz = wz - pPos2.wz;
      return res;
    }

    Position6D Position6D::operator+ () const
    {
      Position6D res;
      res.x  = x;
      res.y  = y;
      res.z  = z;
      res.wx = wx;
      res.wy = wy;
      res.wz = wz;
      return res;
    }

    Position6D Position6D::operator- () const
    {
      Position6D res;
      res.x  = -x;
      res.y  = -y;
      res.z  = -z;
      res.wx = -wx;
      res.wy = -wy;
      res.wz = -wz;
      return res;
    }


    Position6D& Position6D::operator+= (const Position6D& pPos2)
    {
      x  += pPos2.x;
      y  += pPos2.y;
      z  += pPos2.z;
      wx += pPos2.wx;
      wy += pPos2.wy;
      wz += pPos2.wz;
      return *this;
    }


    Position6D& Position6D::operator-= (const Position6D& pPos2)
    {
      x  -= pPos2.x;
      y  -= pPos2.y;
      z  -= pPos2.z;
      wx -= pPos2.wx;
      wy -= pPos2.wy;
      wz -= pPos2.wz;
      return *this;
    }


    bool Position6D::isNear(
      const Position6D& pPos2,
      const float&      pEpsilon) const
    {
      if (
        (fabsf(x - pPos2.x) > pEpsilon) ||
        (fabsf(y - pPos2.y) > pEpsilon) ||
        (fabsf(z - pPos2.z) > pEpsilon) ||
        (fabsf(wx - pPos2.wx) > pEpsilon) ||
        (fabsf(wy - pPos2.wy) > pEpsilon) ||
        (fabsf(wz - pPos2.wz) > pEpsilon))
      {
        return false;
      }
      else
      {
        return true;
      }
    }

    Position6D Position6D::operator* (float pVal) const
    {
      Position6D res;
      res.x  = x * pVal;
      res.y  = y * pVal;
      res.z  = z * pVal;
      res.wx = wx * pVal;
      res.wy = wy * pVal;
      res.wz = wz * pVal;
      return res;
    }

    Position6D Position6D::operator/ (float pVal) const
    {
      if (pVal == 0.0f)
      {
        throw std::runtime_error(
          "ALPosition6D: operator/ Division by zeros.");
      }
      return *this * (1.0f/pVal);
    }

    bool Position6D::operator== (const Position6D& pPos2) const
    {
      if(
          (x == pPos2.x) &&
          (y == pPos2.y) &&
          (z == pPos2.z) &&
          (wx == pPos2.wx) &&
          (wy == pPos2.wy) &&
          (wz == pPos2.wz))
      {
        return true;
      }
      else
      {
        return false;
      }
    }

    bool Position6D::operator!= (const Position6D& pPos2) const
    {
      return !(*this==pPos2);
    }

    Position6D& Position6D::operator*= (const float pVal)
    {
      x  *= pVal;
      y  *= pVal;
      z  *= pVal;
      wx *= pVal;
      wy *= pVal;
      wz *= pVal;
      return *this;
    }

    Position6D& Position6D::operator/= (float pVal)
    {
      if (pVal == 0.0f)
      {
        throw std::runtime_error(
          "ALPosition6D: operator/= Division by zeros.");
      }
      *this *= (1.0f/pVal);
      return *this;
    }

    float Position6D::distance(const Position6D& pPos2) const
    {
      return Math::distance(*this, pPos2);
    }

    float Position6D::distanceSquared(const Position6D& pPos2) const
    {
      return Math::distanceSquared(*this, pPos2);
    }

    float Position6D::norm() const
    {
      return Math::norm(*this);
    }

    std::vector<float> Position6D::toVector() const
    {
      std::vector<float> returnVector;
      returnVector.resize(6);
      returnVector[0] = x;
      returnVector[1] = y;
      returnVector[2] = z;
      returnVector[3] = wx;
      returnVector[4] = wy;
      returnVector[5] = wz;

      return returnVector;
    }

    float distanceSquared(
      const Position6D& pPos1,
      const Position6D& pPos2)
    {
      return (pPos1.x-pPos2.x)*(pPos1.x-pPos2.x)+
          (pPos1.y-pPos2.y)*(pPos1.y-pPos2.y)+
          (pPos1.z-pPos2.z)*(pPos1.z-pPos2.z);
    }

    float distance(
      const Position6D& pPos1,
      const Position6D& pPos2)
    {
      return sqrtf(distanceSquared(pPos1, pPos2));
    }

    float norm(const Position6D& pPos)
    {
      return sqrtf( (pPos.x*pPos.x) + (pPos.y*pPos.y) + (pPos.z*pPos.z) +
                   (pPos.wx*pPos.wx) + (pPos.wy*pPos.wy) + (pPos.wz*pPos.wz) );
    }

    Position6D normalize(const Position6D& pPos)
    {
      Position6D ret;
      ret = pPos;
      float tmpNorm = norm(pPos);

      if (tmpNorm == 0.0f)
      {
        throw std::runtime_error(
          "ALPosition6D: normalize Division by zeros.");
      }

      ret /= tmpNorm;
      return ret;
    }

  } // end namespace math
} // end namespace al

