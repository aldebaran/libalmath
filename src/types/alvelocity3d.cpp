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
      if (pFloats.size() == 3u)
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
      return (std::abs(xd - pVel2.xd) <= pEpsilon &&
              std::abs(yd - pVel2.yd) <= pEpsilon &&
              std::abs(zd - pVel2.zd) <= pEpsilon);
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
              "ALVelocity3D: operator/ Division by zero.");
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
              "ALVelocity3D: operator/= Division by zero.");
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

    void Velocity3D::toVector(std::vector<float>& pReturnVector) const
    {
      pReturnVector.resize(3);
      pReturnVector[0] = xd;
      pReturnVector[1] = yd;
      pReturnVector[2] = zd;
    }

    std::vector<float> Velocity3D::toVector(void) const
    {
      std::vector<float> returnVector(3, 0.0f);
      this->toVector(returnVector);
      return returnVector;
    }

    void Velocity3D::writeToVector(std::vector<float>::iterator& pIt) const
    {
      *pIt++ = xd;
      *pIt++ = yd;
      *pIt++ = zd;
    }

    float norm (const Velocity3D& pVel)
    {
      return std::sqrt(
            pVel.xd*pVel.xd +
            pVel.yd*pVel.yd +
            pVel.zd*pVel.zd);
    }


    Velocity3D normalize(const Velocity3D& pVel)
    {
      const float tmpNorm = norm(pVel);
      if (tmpNorm == 0.0f)
      {
        throw std::runtime_error(
              "ALVelocity3D: normalize Division by zero.");
      }
      return pVel/tmpNorm;
    }

  } // end namespace Math
} // end namespace AL

