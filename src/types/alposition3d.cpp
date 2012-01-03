/*
 * Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */

#include <almath/types/alposition3d.h>
#include <cmath>
#include <stdexcept>

namespace AL {
  namespace Math {

    Position3D::Position3D() : x(0.0f), y(0.0f), z(0.0f) {}

    Position3D::Position3D(float pInit) : x(pInit), y(pInit), z(pInit) {}

    Position3D::Position3D(
      float pX,
      float pY,
      float pZ):
      x(pX), y(pY), z(pZ) {}

    Position3D::Position3D(const std::vector<float>& pFloats)
    {
      if (pFloats.size() == 3)
      {
        x = pFloats[0];
        y = pFloats[1];
        z = pFloats[2];
      }
      else
      {
        x = 0.0f;
        y = 0.0f;
        z = 0.0f;
      }
    }

    Position3D Position3D::operator+ (const Position3D& pPos2) const
    {
      Position3D res;
      res.x = x + pPos2.x;
      res.y = y + pPos2.y;
      res.z = z + pPos2.z;
      return res;
    }

    Position3D Position3D::operator- (const Position3D& pPos2) const
    {
      Position3D res;
      res.x = x - pPos2.x;
      res.y = y - pPos2.y;
      res.z = z - pPos2.z;
      return res;
    }

    Position3D Position3D::operator+ () const
    {
      Position3D res;
      res.x = x;
      res.y = y;
      res.z = z;
      return res;
    }

    Position3D Position3D::operator- () const
    {
      Position3D res;
      res.x = -x;
      res.y = -y;
      res.z = -z;
      return res;
    }


    Position3D& Position3D::operator+= (const Position3D& pPos2)
    {
      x += pPos2.x;
      y += pPos2.y;
      z += pPos2.z;
      return *this;
    }

    Position3D& Position3D::operator-= (const Position3D& pPos2)
    {
      x -= pPos2.x;
      y -= pPos2.y;
      z -= pPos2.z;
      return *this;
    }


    bool Position3D::isNear(
      const Position3D& pPos2,
      const float&      pEpsilon) const
    {
      if (
        (fabsf(x - pPos2.x) > pEpsilon) ||
        (fabsf(y - pPos2.y) > pEpsilon) ||
        (fabsf(z - pPos2.z) > pEpsilon))
      {
        return false;
      }
      else
      {
        return true;
      }
    }

    Position3D Position3D::operator* (float pVal) const
    {
      Position3D res;
      res.x = x * pVal;
      res.y = y * pVal;
      res.z = z * pVal;
      return res;
    }

    Position3D operator* (
      const float       pVal,
      const Position3D& pPos1)
    {
      return pPos1 * pVal;
    }

    Position3D Position3D::operator/ (float pVal) const
    {
      if (pVal == 0.0f)
      {
        throw std::runtime_error(
          "ALPosition3D: operator/ Division by zeros.");
      }
      return *this * (1.0f/pVal);
    }

    Position3D operator/ (
      const float       pVal,
      const Position3D& pPos1)
    {
      if (pVal == 0.0f)
      {
        throw std::runtime_error(
          "ALPosition3D: operator/ Division by zeros.");
      }
      return (1.0f/pVal) * pPos1;
    }

    Position3D& Position3D::operator*= (float pVal)
    {
      x *= pVal;
      y *= pVal;
      z *= pVal;
      return *this;
    }

    Position3D& Position3D::operator/= (float pVal)
    {
      if (pVal == 0.0f)
      {
        throw std::runtime_error(
          "ALPosition3D: operator/= Division by zeros.");
      }
      *this *= (1.0f/pVal);
      return *this;
    }

    bool Position3D::operator== (const Position3D& pPos2) const
    {
      if(
        (x == pPos2.x) &&
        (y == pPos2.y) &&
        (z == pPos2.z))
      {
        return true;
      }
      else
      {
        return false;
      }
    }

    bool Position3D::operator!= (const Position3D& pPos2) const
    {
      return !(*this==pPos2);
    }

    float Position3D::distanceSquared(const Position3D& pPos2) const
    {
      return Math::distanceSquared(*this, pPos2);
    }

    float Position3D::distance(const Position3D& pPos2) const
    {
      return Math::distance(*this, pPos2);
    }

    float Position3D::norm() const
    {
      return Math::norm(*this);
    }

    Position3D Position3D::normalize() const
    {
      return Math::normalize(*this);
    }

    float Position3D::dotProduct(const Position3D& pPos2) const
    {
      return Math::dotProduct(*this, pPos2);
    }

    Position3D Position3D::crossProduct(const Position3D& pPos2) const
    {
      return Math::crossProduct(*this, pPos2);
    }

    std::vector<float> Position3D::toVector() const
    {
      std::vector<float> returnVector;
      returnVector.resize(3);
      returnVector[0] = x;
      returnVector[1] = y;
      returnVector[2] = z;

      return returnVector;
    }


    float distanceSquared(
      const Position3D& pPos1,
      const Position3D& pPos2)
    {
      return (pPos1.x-pPos2.x)*(pPos1.x-pPos2.x)+
          (pPos1.y-pPos2.y)*(pPos1.y-pPos2.y)+
          (pPos1.z-pPos2.z)*(pPos1.z-pPos2.z);
    }


    float distance(
      const Position3D& pPos1,
      const Position3D& pPos2)
    {
      return sqrtf(distanceSquared(pPos1, pPos2));
    }

    float norm(const Position3D& p)
    {
      return sqrtf( (p.x*p.x) + (p.y*p.y) + (p.z*p.z) );
    }

    Position3D normalize(const Position3D& pPos)
    {
      Position3D ret;
      ret = pPos;
      float tmpNorm = norm(pPos);

      if (tmpNorm == 0.0f)
      {
        throw std::runtime_error(
          "ALPosition3D: normalize Division by zeros.");
      }

      ret /= tmpNorm;
      return ret;
    }

    float dotProduct(
      const Position3D& pPos1,
      const Position3D& pPos2)
    {
      return (pPos1.x * pPos2.x + pPos1.y * pPos2.y + pPos1.z * pPos2.z);
    }

    void crossProduct(
      const Position3D& pPos1,
      const Position3D& pPos2,
      Position3D&       pRes)
    {
      pRes.x = pPos1.y*pPos2.z - pPos1.z*pPos2.y;
      pRes.y = pPos1.z*pPos2.x - pPos1.x*pPos2.z;
      pRes.z = pPos1.x*pPos2.y - pPos1.y*pPos2.x;
    }

    Position3D crossProduct(
      const Position3D& pPos1,
      const Position3D& pPos2)
    {
      Position3D res;
      crossProduct(pPos1, pPos2, res);
      return res;
    }

  } // end namespace math
} // end namespace al

