/*
 * Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */

#include <almath/types/alposition6d.h>
#include <cmath>
#include <stdexcept>
#include <iostream>

namespace AL {
  namespace Math {

    Position6D::Position6D()
      :x(0.0f)
      ,y(0.0f)
      ,z(0.0f)
      ,wx(0.0f)
      ,wy(0.0f)
      ,wz(0.0f) {}

    Position6D::Position6D(float pInit)
      :x(pInit)
      ,y(pInit)
      ,z(pInit)
      ,wx(pInit)
      ,wy(pInit)
      ,wz(pInit) {}

    Position6D::Position6D(
      float pX,
      float pY,
      float pZ,
      float pWx,
      float pWy,
      float pWz)
      :x(pX)
      ,y(pY)
      ,z(pZ)
      ,wx(pWx)
      ,wy(pWy)
      ,wz(pWz) {}

    Position6D::Position6D(const std::vector<float>& pFloats)
    {
      if (pFloats.size() == 6u)
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
        std::cerr << "ALMath: WARNING: "
                  << "Position6D constructor call with a wrong size of vector. "
                  << "Size expected: 6. Size given: " << pFloats.size() << ". "
                  << "Position6D is set to default value." << std::endl;

        x = 0.0f;
        y = 0.0f;
        z = 0.0f;

        wx = 0.0f;
        wy = 0.0f;
        wz = 0.0f;
      }
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
      return (std::abs(x - pPos2.x) <= pEpsilon &&
              std::abs(y - pPos2.y) <= pEpsilon &&
              std::abs(z - pPos2.z) <= pEpsilon &&
              std::abs(wx - pPos2.wx) <= pEpsilon &&
              std::abs(wy - pPos2.wy) <= pEpsilon &&
              std::abs(wz - pPos2.wz) <= pEpsilon);
    }

    Position6D Position6D::operator/ (float pVal) const
    {
      if (pVal == 0.0f)
      {
        throw std::runtime_error(
          "ALPosition6D: operator/ Division by zero.");
      }
      return *this * (1.0f/pVal);
    }

    bool Position6D::operator== (const Position6D& pPos2) const
    {
      return (x == pPos2.x &&
              y == pPos2.y &&
              z == pPos2.z &&
              wx == pPos2.wx &&
              wy == pPos2.wy &&
              wz == pPos2.wz);
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
          "ALPosition6D: operator/= Division by zero.");
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

    void Position6D::toVector(std::vector<float>& pReturnVector) const
    {
      pReturnVector.resize(6);
      pReturnVector[0] = x;
      pReturnVector[1] = y;
      pReturnVector[2] = z;
      pReturnVector[3] = wx;
      pReturnVector[4] = wy;
      pReturnVector[5] = wz;
    }

    std::vector<float> Position6D::toVector(void) const
    {
      std::vector<float> returnVector(6, 0.0f);
      this->toVector(returnVector);
      return returnVector;
    }

    void Position6D::writeToVector(std::vector<float>::iterator& pIt) const
    {
      *pIt++ = x;
      *pIt++ = y;
      *pIt++ = z;
      *pIt++ = wx;
      *pIt++ = wy;
      *pIt++ = wz;
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
      return std::sqrt(distanceSquared(pPos1, pPos2));
    }

    float norm(const Position6D& pPos)
    {
      return std::sqrt( (pPos.x*pPos.x) + (pPos.y*pPos.y) + (pPos.z*pPos.z) +
                   (pPos.wx*pPos.wx) + (pPos.wy*pPos.wy) + (pPos.wz*pPos.wz) );
    }

    Position6D normalize(const Position6D& pPos)
    {
      const float tmpNorm = norm(pPos);
      if (tmpNorm == 0.0f)
      {
        throw std::runtime_error(
          "ALPosition6D: normalize Division by zero.");
      }
      return pPos/tmpNorm;
    }

  } // end namespace math
} // end namespace al

