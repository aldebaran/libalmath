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

#include <almath/types/alvelocity3d.h>
#include <cmath>
#include <stdexcept>

namespace AL {
  namespace Math {

  Velocity3D::Velocity3D(): xd(0.0f),
    yd(0.0f),
    zd(0.0f) {}


  Velocity3D::Velocity3D(float pInit): xd(pInit),
    yd(pInit),
    zd(pInit) {}

  Velocity3D::Velocity3D(
    float pXd,
    float pYd,
    float pZd): xd(pXd),
    yd(pYd),
    zd(pZd) {}

  Velocity3D::Velocity3D(const std::vector<float>& pFloats)
  {
    if (pFloats.size() == 3)
    {
      xd = pFloats[0];
      yd = pFloats[1];
      zd = pFloats[2];
    }
    else
    {
      xd = 0.0f;
      yd = 0.0f;
      zd = 0.0f;
    }
  }

    Velocity3D Velocity3D::operator+ (const Velocity3D& pVel2) const
    {
      Velocity3D res;
      res.xd = xd + pVel2.xd;
      res.yd = yd + pVel2.yd;
      res.zd = zd + pVel2.zd;
      return res;
    }

    Velocity3D Velocity3D::operator- (const Velocity3D& pVel2) const
    {
      Velocity3D res;
      res.xd = xd - pVel2.xd;
      res.yd = yd - pVel2.yd;
      res.zd = zd - pVel2.zd;
      return res;
    }

    Velocity3D Velocity3D::operator+ () const
    {
      Velocity3D res;
      res.xd = xd;
      res.yd = yd;
      res.zd = zd;
      return res;
    }


    Velocity3D Velocity3D::operator- () const
    {
      Velocity3D res;
      res.xd = -xd;
      res.yd = -yd;
      res.zd = -zd;
      return res;
    }


    Velocity3D& Velocity3D::operator+= (const Velocity3D& pVel2)
    {
      xd += pVel2.xd;
      yd += pVel2.yd;
      zd += pVel2.zd;
      return *this;
    }

    Velocity3D& Velocity3D::operator-= (const Velocity3D& pVel2)
    {
      xd -= pVel2.xd;
      yd -= pVel2.yd;
      zd -= pVel2.zd;
      return *this;
    }


    bool Velocity3D::isNear(
      const Velocity3D& pVel2,
      const float&      pEpsilon) const
    {
      if (
        (fabsf(xd - pVel2.xd) > pEpsilon) ||
        (fabsf(yd - pVel2.yd) > pEpsilon) ||
        (fabsf(zd - pVel2.zd) > pEpsilon))
      {
        return false;
      }
      else
      {
        return true;
      }
    }


    Velocity3D Velocity3D::operator* (const float pVal) const
    {
      Velocity3D res;
      res.xd = xd * pVal;
      res.yd = yd * pVal;
      res.zd = zd * pVal;
      return res;
    }

    Velocity3D operator* (
      const float       pVal,
      const Velocity3D& pVel)
    {
      return pVel*pVal;
    }


    Velocity3D Velocity3D::operator/ (const float pVal) const
    {
      if (pVal == 0.0f)
      {
        throw std::runtime_error(
          "ALVelocity3D: operator/ Division by zeros.");
      }
      return (*this) * (1.0f/pVal);
    }


    bool Velocity3D::operator== (const Velocity3D& pVel2) const
    {
      if(
        (xd == pVel2.xd) &&
        (yd == pVel2.yd) &&
        (zd == pVel2.zd))
      {
        return true;
      }
      else
      {
        return false;
      }
    }


    bool Velocity3D::operator!= (const Velocity3D& pVel2) const
    {
      return !(*this==pVel2);
    }


    Velocity3D& Velocity3D::operator*= (const float pVal)
    {
      xd *= pVal;
      yd *= pVal;
      zd *= pVal;
      return *this;
    }

    Velocity3D& Velocity3D::operator/= (const float pVal)
    {
      if (pVal == 0.0f)
      {
        throw std::runtime_error(
          "ALVelocity3D: operator/= Division by zeros.");
      }
      (*this) *= (1.0f/pVal);
      return (*this);
    }


    float Velocity3D::norm () const
    {
      return Math::norm(*this);
    }

    Velocity3D Velocity3D::normalize() const
    {
      return Math::normalize(*this);
    }

    std::vector<float> Velocity3D::toVector() const
    {
      std::vector<float> returnVector;
      returnVector.resize(3);

      returnVector[0] = xd;
      returnVector[1] = yd;
      returnVector[2] = zd;

      return returnVector;
    }


    float norm (const Velocity3D& pVel)
    {
      return sqrtf( (pVel.xd*pVel.xd) + (pVel.yd*pVel.yd) + (pVel.zd*pVel.zd) );
    }


    Velocity3D normalize(const Velocity3D& pVel)
    {
      Velocity3D ret;
      ret = pVel;
      float tmpNorm = norm(pVel);

      if (tmpNorm == 0.0f)
      {
        throw std::runtime_error(
          "ALVelocity3D: normalize Division by zeros.");
      }

      ret /= tmpNorm;
      return ret;
    }

  } // end namespace Math
} // end namespace AL

