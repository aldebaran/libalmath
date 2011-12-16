/*
 * Copyright (c) 2012, Aldebaran Robotics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Aldebaran Robotics nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Aldebaran Robotics BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <almath/types/alpose2d.h>
#include <cmath>

namespace AL {
  namespace Math {

    Pose2D::Pose2D():x(0.0f), y(0.0f), theta(0.0f) {}

    Pose2D::Pose2D(float pInit):x(pInit), y(pInit), theta(pInit) {}

    Pose2D::Pose2D(
      float pX,
      float pY,
      float pTheta):
      x(pX),
      y(pY),
      theta(pTheta) {}

    Pose2D::Pose2D (const std::vector<float>& pFloats)
    {
      if (pFloats.size() == 3)
      {
        x = pFloats[0];
        y = pFloats[1];
        theta = pFloats[2];
      }
      else
      {
        x = 0.0f;
        y = 0.0f;
        theta = 0.0f;
      }
    }

    Pose2D Pose2D::operator+ (const Pose2D& pPos2) const
    {
      Pose2D res;
      res.x = x + pPos2.x;
      res.y = y + pPos2.y;
      res.theta = theta + pPos2.theta;
      return res;
    }

    Pose2D Pose2D::operator- (const Pose2D& pPos2) const
    {
      Pose2D res;
      res.x = x - pPos2.x;
      res.y = y - pPos2.y;
      res.theta = theta - pPos2.theta;
      return res;
    }

    Pose2D Pose2D::operator+ () const
    {
      Pose2D res;
      res.x = x;
      res.y = y;
      res.theta = theta;
      return res;
    }

    Pose2D Pose2D::operator- () const
    {
      Pose2D res;
      res.x = -x;
      res.y = -y;
      res.theta = -theta;
      return res;
    }

    Pose2D Pose2D::operator* (const Pose2D& pPos2) const
    {
      Pose2D pOut;
      pOut.x = x + cosf(theta) * pPos2.x - sinf(theta) * pPos2.y;
      pOut.y = y + sinf(theta) * pPos2.x + cosf(theta) * pPos2.y;
      pOut.theta = theta + pPos2.theta;

      return pOut;
    }

    Pose2D& Pose2D::operator*= (const Pose2D& pPos2)
    {
      x += cosf(theta) * pPos2.x - sinf(theta) * pPos2.y;
      y += sinf(theta) * pPos2.x + cosf(theta) * pPos2.y;
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
       if (
         (x == pPos2.x) &&
         (y == pPos2.y) &&
         (theta == pPos2.theta) )
       {
        return true;
      }
      else
      {
        return false;
      }
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

    std::vector<float> Pose2D::toVector() const
    {
      std::vector<float> returnVector;
      returnVector.resize(3);
      returnVector[0] = x;
      returnVector[1] = y;
      returnVector[2] = theta;

      return returnVector;
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
      return sqrtf(distanceSquared(pPos1, pPos2));
    }


    bool Pose2D::isNear(
      const Pose2D& pPos2,
      const float&  pEpsilon) const
    {

      if (
        (fabsf(x - pPos2.x) > pEpsilon) ||
        (fabsf(y - pPos2.y) > pEpsilon) ||
        (fabsf(theta - pPos2.theta) > pEpsilon))
      {
        return false;
      }
      else
      {
        return true;
      }
    }

    Pose2D Pose2D::inverse() const
    {
      return Math::pose2DInverse(*this);
    }

    void pose2DInverse(
      const Pose2D& pIn,
      Pose2D&       pOut)
    {
      pOut.theta = -pIn.theta;

      float cos = cosf(pOut.theta);
      float sin = sinf(pOut.theta);

      pOut.x = -( pIn.x*cos - pIn.y*sin);
      pOut.y = -( pIn.y*cos + pIn.x*sin);
    }

    Pose2D pose2DInverse(const Pose2D& pIn)
    {
      Pose2D pOut;
      pose2DInverse(pIn, pOut);
      return pOut;
    }
  } // end namespace math
} // end namespace AL
