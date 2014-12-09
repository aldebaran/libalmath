/*
 * Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */

#include <almath/types/alvelocity6d.h>
#include <cmath>
#include <stdexcept>
#include <iostream>

namespace AL {
  namespace Math {

    Velocity6D::Velocity6D():
      xd(0.0f),
      yd(0.0f),
      zd(0.0f),
      wxd(0.0f),
      wyd(0.0f),
      wzd(0.0f) {}

    Velocity6D::Velocity6D(float pInit):
      xd(pInit),
      yd(pInit),
      zd(pInit),
      wxd(pInit),
      wyd(pInit),
      wzd(pInit) {}

    Velocity6D::Velocity6D(
        float pXd,
        float pYd,
        float pZd,
        float pWxd,
        float pWyd,
        float pWzd):
      xd(pXd),
      yd(pYd),
      zd(pZd),
      wxd(pWxd),
      wyd(pWyd),
      wzd(pWzd) {}

    Velocity6D::Velocity6D(const std::vector<float>& pFloats)
    {
      if (pFloats.size() == 6u)
      {
        xd  = pFloats[0];
        yd  = pFloats[1];
        zd  = pFloats[2];
        wxd = pFloats[3];
        wyd = pFloats[4];
        wzd = pFloats[5];
      }
      else
      {
        std::cerr << "ALMath: WARNING: "
                  << "Velocity6D constructor call with a wrong size of vector. "
                  << "Size expected: 6. Size given: " << pFloats.size() << ". "
                  << "Velocity6D is set to default value." << std::endl;

        xd  = 0.0f;
        yd  = 0.0f;
        zd  = 0.0f;
        wxd = 0.0f;
        wyd = 0.0f;
        wzd = 0.0f;
      }
    }

    Velocity6D Velocity6D::operator/ (const float pVal) const
    {
      if (pVal == 0.0f)
      {
        throw std::runtime_error(
              "ALVelocity6D: operator/ Division by zero.");
      }
      return  (*this) * (1.0f/pVal);
    }


    bool Velocity6D::operator== (const Velocity6D& pVel2) const
    {
      return (xd == pVel2.xd &&
              yd == pVel2.yd &&
              zd == pVel2.zd &&
              wxd == pVel2.wxd &&
              wyd == pVel2.wyd &&
              wzd == pVel2.wzd);
    }


    bool Velocity6D::operator!= (const Velocity6D& pVel2) const
    {
      return !(*this==pVel2);
    }


    Velocity6D& Velocity6D::operator*= (const float pVal)
    {
      xd  *= pVal;
      yd  *= pVal;
      zd  *= pVal;
      wxd *= pVal;
      wyd *= pVal;
      wzd *= pVal;

      return *this;
    }


    Velocity6D& Velocity6D::operator/= (const float pVal)
    {
      if (pVal == 0.0f)
      {
        throw std::runtime_error(
              "ALVelocity6D: operator/= Division by zero.");
      }
      (*this) *= (1.0f/pVal);
      return *this;
    }

    bool Velocity6D::isNear(
        const Velocity6D& pVel2,
        const float&      pEpsilon) const
    {
      return (std::abs(xd  - pVel2.xd)  <= pEpsilon &&
              std::abs(yd  - pVel2.yd)  <= pEpsilon &&
              std::abs(zd  - pVel2.zd)  <= pEpsilon &&
              std::abs(wxd - pVel2.wxd) <= pEpsilon &&
              std::abs(wyd - pVel2.wyd) <= pEpsilon &&
              std::abs(wzd - pVel2.wzd) <= pEpsilon);
    }

    float Velocity6D::norm() const
    {
      return Math::norm(*this);
    }

    Velocity6D Velocity6D::normalize() const
    {
      return Math::normalize(*this);
    }

    void Velocity6D::toVector(std::vector<float>& pReturnVector) const
    {
      pReturnVector.resize(6);

      pReturnVector[0] = xd;
      pReturnVector[1] = yd;
      pReturnVector[2] = zd;

      pReturnVector[3] = wxd;
      pReturnVector[4] = wyd;
      pReturnVector[5] = wzd;
    }

    std::vector<float> Velocity6D::toVector(void) const
    {
      std::vector<float> returnVector(6, 0.0f);
      this->toVector(returnVector);
      return returnVector;
    }

    void Velocity6D::writeToVector(std::vector<float>::iterator& pIt) const
    {
      *pIt++ = xd;
      *pIt++ = yd;
      *pIt++ = zd;
      *pIt++ = wxd;
      *pIt++ = wyd;
      *pIt++ = wzd;
    }

    Velocity6D operator* (
        const float       pVal,
        const Velocity6D& pVel)
    {
      return pVel * pVal;
    }


    float norm(const Velocity6D& pVel)
    {
      // norm of a 6 component vector
      return std::sqrt(pVel.xd*pVel.xd +
                       pVel.yd*pVel.yd +
                       pVel.zd*pVel.zd +
                       pVel.wxd*pVel.wxd +
                       pVel.wyd*pVel.wyd +
                       pVel.wzd*pVel.wzd);
    }


    Velocity6D normalize(const Velocity6D& pVel)
    {
      const float tmpNorm = norm(pVel);
      if (tmpNorm == 0.0f)
      {
        throw std::runtime_error(
              "ALVelocity6D: normalize Division by zero.");
      }
      return pVel/tmpNorm;
    }

  } // end namespace Math
} // end namespace AL

