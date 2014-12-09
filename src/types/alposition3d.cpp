/*
 * Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */

#include <almath/types/alposition3d.h>
#include <cmath>
#include <stdexcept>
#include <iostream>

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
      if (pFloats.size() == 3u)
      {
        x = pFloats[0];
        y = pFloats[1];
        z = pFloats[2];
      }
      else
      {
        std::cerr << "ALMath: WARNING: "
                  << "Position3D constructor call with a wrong size of vector. "
                  << "Size expected: 3. Size given: " << pFloats.size() << ". "
                  << "Position3D is set to default value." << std::endl;

        x = 0.0f;
        y = 0.0f;
        z = 0.0f;
      }
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
      return (std::abs(x - pPos2.x) <= pEpsilon &&
              std::abs(y - pPos2.y) <= pEpsilon &&
              std::abs(z - pPos2.z) <= pEpsilon);
    }

    Position3D Position3D::operator* (float pVal) const
    {
      return Position3D(x*pVal, y*pVal, z*pVal);
    }

    Position3D Position3D::operator/ (float pVal) const
    {
      if (pVal == 0.0f)
      {
        throw std::runtime_error(
          "ALPosition3D: operator/ Division by zero.");
      }
      return *this * (1.0f/pVal);
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
          "ALPosition3D: operator/= Division by zero.");
      }
      *this *= (1.0f/pVal);
      return *this;
    }

    bool Position3D::operator== (const Position3D& pPos2) const
    {
      return (x == pPos2.x &&
              y == pPos2.y &&
              z == pPos2.z);
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

    void Position3D::toVector(std::vector<float>& pReturnVector) const
    {
      pReturnVector.resize(3);
      pReturnVector[0] = x;
      pReturnVector[1] = y;
      pReturnVector[2] = z;
    }

    std::vector<float> Position3D::toVector(void) const
    {
      std::vector<float> returnVector(3, 0.0f);
      this->toVector(returnVector);
      return returnVector;
    }

    void Position3D::writeToVector(std::vector<float>::iterator& pIt) const
    {
      *pIt++ = x;
      *pIt++ = y;
      *pIt++ = z;
    }

    bool Position3D::isUnitVector(const float& pEpsilon) const
    {
      return Math::isUnitVector(*this, pEpsilon);
    }

    bool Position3D::isOrthogonal(const Position3D& pPos, const float& pEpsilon) const
    {
      return Math::isOrthogonal(*this, pPos, pEpsilon);
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
      return std::sqrt(distanceSquared(pPos1, pPos2));
    }

    float norm(const Position3D& p)
    {
      return std::sqrt( (p.x*p.x) + (p.y*p.y) + (p.z*p.z) );
    }

    Position3D normalize(const Position3D& pPos)
    {
      const float tmpNorm = norm(pPos);

      if (tmpNorm == 0.0f)
      {
        throw std::runtime_error(
          "ALPosition3D: normalize Division by zero.");
      }
      return pPos/tmpNorm;
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

    bool isUnitVector(const Position3D& pPos,
                      const float& pEpsilon)
    {
      return std::abs(1.0f - pPos.norm()) <= pEpsilon;
    }

    bool isOrthogonal(const Position3D& pPos1,
                      const Position3D& pPos2,
                      const float& pEpsilon)
    {
      return std::abs(pPos1.dotProduct(pPos2)) <= pEpsilon;
    }

  } // end namespace math
} // end namespace al

