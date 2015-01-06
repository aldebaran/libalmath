/*
 * Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */

#include <almath/types/alposition2d.h>
#include <cmath>
#include <stdexcept>
#include <iostream>

namespace AL {
  namespace Math {

    Position2D::Position2D(): x(0.0f), y(0.0f) {}

    Position2D::Position2D(float pInit): x(pInit), y(pInit) {}

    Position2D::Position2D(float pX, float pY): x(pX), y(pY) {}

    Position2D::Position2D (const std::vector<float>& pFloats)
    {
      if (pFloats.size() == 2u)
      {
        x = pFloats[0];
        y = pFloats[1];
      }
      else
      {
        std::cerr << "ALMath: WARNING: "
                  << "Position2D constructor call with a wrong size of vector. "
                  << "Size expected: 2. Size given: " << pFloats.size() << ". "
                  << "Position2D is set to default value." << std::endl;

        x = 0.0f;
        y = 0.0f;
      }
    }

    Position2D& Position2D::operator+= (const Position2D& pPos2)
    {
      x += pPos2.x;
      y += pPos2.y;
      return *this;
    }

    Position2D& Position2D::operator-= (const Position2D& pPos2)
    {
      x -= pPos2.x;
      y -= pPos2.y;
      return *this;
    }


    bool Position2D::operator!=(const Position2D& pPos2) const
    {
      return !(*this==pPos2);
    }


    bool Position2D::isNear(
        const Position2D& pPos2,
        const float&      pEpsilon) const
    {
      return (std::abs(x - pPos2.x) <= pEpsilon &&
              std::abs(y - pPos2.y) <= pEpsilon);
    }

    Position2D operator* (
      const float       pVal,
      const Position2D& pPos1)
    {
      return pPos1*pVal;
    }

    Position2D Position2D::operator/ (float pVal) const
    {
      if (pVal == 0.0f)
      {
        throw std::runtime_error(
          "ALPosition2D: operator/ Division by zero.");
      }
      return *this * (1.0f/pVal);
    }

    Position2D& Position2D::operator*= (float pVal)
    {
      x *= pVal;
      y *= pVal;
      return *this;
    }

    Position2D& Position2D::operator/= (float pVal)
    {
      if (pVal == 0.0f)
      {
        throw std::runtime_error(
          "ALPosition2D: operator/= Division by zero.");
      }
      *this *= (1.0f/pVal);
      return *this;
    }


    float Position2D::distanceSquared(const Position2D& pPos2) const
    {
      return Math::distanceSquared(*this, pPos2);
    }

    float Position2D::distance(const Position2D& pPos2) const
    {
      return Math::distance(*this, pPos2);
    }

    float Position2D::norm() const
    {
      return Math::norm(*this);
    }

    Position2D Position2D::normalize() const
    {
      return Math::normalize(*this);
    }

    float Position2D::dotProduct(const Position2D& pPos2) const
    {
      return Math::dotProduct(*this, pPos2);
    }

    float Position2D::crossProduct(const Position2D& pPos2) const
    {
      return Math::crossProduct(*this, pPos2);
    }

    bool Position2D::operator==(const Position2D& pPos2) const
    {
      return (x == pPos2.x &&
              y == pPos2.y);
    }

    void Position2D::toVector(std::vector<float>& pReturnVector) const
    {
      pReturnVector.resize(2);
      pReturnVector[0] = x;
      pReturnVector[1] = y;
    }

    std::vector<float> Position2D::toVector(void) const
    {
      std::vector<float> returnVector(2, 0.0f);
      this->toVector(returnVector);
      return returnVector;
    }

    void Position2D::writeToVector(std::vector<float>::iterator& pIt) const
    {
      *pIt++ = x;
      *pIt++ = y;
    }

    float Position2D::getAngle() const
    {
      return std::atan2(y, x);
    }

    float distanceSquared(
      const Position2D& pPos1,
      const Position2D& pPos2)
    {
      return (pPos1.x-pPos2.x)*(pPos1.x-pPos2.x)+(pPos1.y-pPos2.y)*(pPos1.y-pPos2.y);
    }

    float distance(
      const Position2D& pPos1,
      const Position2D& pPos2)
    {
      return std::sqrt(distanceSquared(pPos1, pPos2));
    }

    float norm(const Position2D& p)
    {
      return std::sqrt( (p.x*p.x) + (p.y*p.y) );
    }

    Position2D normalize(const Position2D& p)
    {
      const float tmpNorm = norm(p);
      if (tmpNorm == 0.0f)
      {
        throw std::runtime_error(
          "ALPosition2D: normalize Division by zero.");
      }
      return p/tmpNorm;
    }

    float dotProduct(
      const Position2D& pPos1,
      const Position2D& pPos2)
    {
      return (pPos1.x * pPos2.x + pPos1.y * pPos2.y);
    }


    float crossProduct(
      const Position2D& pPos1,
      const Position2D& pPos2)
    {
      return (pPos1.x*pPos2.y - pPos1.y*pPos2.x);
    }


    void crossProduct(
      const Position2D& pPos1,
      const Position2D& pPos2,
      float&            result)
    {
      result = (pPos1.x*pPos2.y - pPos1.y*pPos2.x);
    }

    Position2D Position2D::fromPolarCoordinates(
        const float pRadius,
        const float pAngle)
    {
      return Position2D(pRadius * std::cos(pAngle),
                        pRadius * std::sin(pAngle));
    }

  } // end namespace math
} // end namespace al

