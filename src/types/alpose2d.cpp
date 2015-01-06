/*
 * Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */

#include <almath/types/alpose2d.h>
#include <stdexcept>
#include <iostream>

namespace AL {
  namespace Math {

    Pose2D::Pose2D():x(0.0f), y(0.0f), theta(0.0f) {}

    Pose2D::Pose2D(float pInit):x(pInit), y(pInit), theta(pInit) {}

    Pose2D::Pose2D(
      float pX,
      float pY,
      float pTheta)
      :x(pX)
      ,y(pY)
      ,theta(pTheta) {}

    Pose2D::Pose2D (const std::vector<float>& pFloats)
    {
      if (pFloats.size() == 3u)
      {
        x = pFloats[0];
        y = pFloats[1];
        theta = pFloats[2];
      }
      else
      {
        std::cerr << "ALMath: WARNING: "
                  << "Pose2D constructor call with a wrong size of vector. "
                  << "Size expected: 3. Size given: " << pFloats.size() << ". "
                  << "Pose2D is set to default value." << std::endl;

        x = 0.0f;
        y = 0.0f;
        theta = 0.0f;
      }
    }

    Pose2D& Pose2D::operator*= (const Pose2D& pPos2)
    {
      const float x0 = pPos2.x;
      const float y0 = pPos2.y;
      const float cs = std::cos(theta);
      const float sn = std::sin(theta);

      x += cs*x0 - sn*y0;
      y += sn*x0 + cs*y0;
      theta += pPos2.theta;

      return *this;
    }

    Pose2D& Pose2D::operator+= (const Pose2D& pPos2)
    {
      x     += pPos2.x;
      y     += pPos2.y;
      theta += pPos2.theta;
      return *this;
    }

    Pose2D& Pose2D::operator-= (const Pose2D& pPos2)
    {
      x     -= pPos2.x;
      y     -= pPos2.y;
      theta -= pPos2.theta;
      return *this;
    }

    bool Pose2D::operator==(const Pose2D& pPos2) const
    {
      return (x == pPos2.x &&
              y == pPos2.y &&
              theta == pPos2.theta);
    }

    bool Pose2D::operator!=(const Pose2D& pPos2) const
    {
      return ! (*this==pPos2);
    }

    float Pose2D::distanceSquared(const Pose2D& pPos) const
    {
      return Math::distanceSquared(*this, pPos);
    }

    float Pose2D::distance(const Pose2D& pPos2) const
    {
      return Math::distance(*this, pPos2);
    }

    Pose2D Pose2D::operator/ (float pVal) const
    {
      if (pVal == 0.0f)
      {
        throw std::runtime_error(
          "ALPose2D: operator/ Division by zero.");
      }
      return *this * (1.0f/pVal);
    }

    Pose2D& Pose2D::operator*= (const float pVal)
    {
      x     *= pVal;
      y     *= pVal;
      theta *= pVal;
      return *this;
    }

    Pose2D& Pose2D::operator/= (float pVal)
    {
      if (pVal == 0.0f)
      {
        throw std::runtime_error(
          "ALPose2D: operator/= Division by zero.");
      }
      *this *= (1.0f/pVal);
      return *this;
    }

    Pose2D Pose2D::diff(const Pose2D& pPos2) const
    {
      return Math::pose2dDiff(*this, pPos2);
    }

    void Pose2D::toVector(std::vector<float>& pReturnVector) const
    {
      pReturnVector.resize(3);
      pReturnVector[0] = x;
      pReturnVector[1] = y;
      pReturnVector[2] = theta;
    }

    std::vector<float> Pose2D::toVector(void) const
    {
      std::vector<float> returnVector(3, 0.0f);
      this->toVector(returnVector);
      return returnVector;
    }

    void Pose2D::writeToVector(std::vector<float>::iterator& pIt) const
    {
      *pIt++ = x;
      *pIt++ = y;
      *pIt++ = theta;
    }

    Pose2D Pose2D::normalize() const
    {
      const float tmpNorm = this->norm();
      if (std::abs(tmpNorm) < 1e-4f)
      {
        throw std::runtime_error(
          "ALPose2D: normalize Division by zero.");
      }
      return Pose2D(x / tmpNorm, y / tmpNorm, theta);
    }

    float distanceSquared(
      const Pose2D& pPos1,
      const Pose2D& pPos2)
    {
      return (pPos1.x-pPos2.x)*(pPos1.x-pPos2.x)+(pPos1.y-pPos2.y)*(pPos1.y-pPos2.y);
    }

    float distance(
      const Pose2D& pPos1,
      const Pose2D& pPos2)
    {
      return std::sqrt(distanceSquared(pPos1, pPos2));
    }


    bool Pose2D::isNear(
        const Pose2D& pPos2,
        const float&  pEpsilon) const
    {
      return (std::abs(x - pPos2.x) <= pEpsilon &&
              std::abs(y - pPos2.y) <= pEpsilon &&
              std::abs(theta - pPos2.theta) <= pEpsilon);
    }

    Pose2D Pose2D::inverse() const
    {
      return Math::pose2DInverse(*this);
    }

    void pose2DInverse(
      const Pose2D& pIn,
      Pose2D&       pOut)
    {
      pOut = pIn;
      pose2dInvertInPlace(pOut);
    }

    void pose2dInvertInPlace(Pose2D& pPos)
    {
      pPos.theta = -pPos.theta;

      const float cos = std::cos(pPos.theta);
      const float sin = std::sin(pPos.theta);
      const float x   = pPos.x;

      pPos.x = -(x*cos - pPos.y*sin);
      pPos.y = -(pPos.y*cos + x*sin);
    }

    Pose2D pose2dDiff(
      const Pose2D& pPos1,
      const Pose2D& pPos2)
    {
      Pose2D result = pPos1;
      pose2dInvertInPlace(result);
      result *= pPos2;
      return result;
    }

    Pose2D pose2DInverse(const Pose2D& pIn)
    {
      Pose2D pOut;
      pose2DInverse(pIn, pOut);
      return pOut;
    }

    Pose2D pinv(const Pose2D& pPos)
    {
      Pose2D result = pPos;
      pose2dInvertInPlace(result);
      return result;
    }

    Pose2D Pose2D::fromPolarCoordinates(
        const float pRadius,
        const float pAngle)
    {
      return AL::Math::Pose2D(pRadius * std::cos(pAngle),
                              pRadius * std::sin(pAngle),
                              pAngle);
    }
  } // end namespace math
} // end namespace AL
