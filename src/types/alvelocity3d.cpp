/*
 * Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */

#include <almath/types/alvelocity3d.h>
#include <cmath>
#include <stdexcept>
#include <iostream>

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
      std::cerr << "ALMath: WARNING: "
                << "Velocity3D constructor call with a wrong size of vector. "
                << "Size expected: 3. Size given: " << pFloats.size() << ". "
                << "Velocity3D is set to default value." << std::endl;

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
      return (fabsf(xd - pVel2.xd) <= pEpsilon &&
              fabsf(yd - pVel2.yd) <= pEpsilon &&
              fabsf(zd - pVel2.zd) <= pEpsilon);
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
      return (xd == pVel2.xd &&
              yd == pVel2.yd &&
              zd == pVel2.zd);
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
      std::vector<float> returnVector(3, 0.0f);

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
      const float tmpNorm = norm(pVel);
      if (tmpNorm == 0.0f)
      {
        throw std::runtime_error(
          "ALVelocity3D: normalize Division by zeros.");
      }

      Velocity3D ret = pVel;
      ret /= tmpNorm;
      return ret;
    }

  } // end namespace Math
} // end namespace AL

