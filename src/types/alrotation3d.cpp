/*
 * Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */

#include <almath/types/alrotation3d.h>
#include <cmath>
#include <stdexcept>
#include <iostream>

namespace AL {
  namespace Math {

    Rotation3D::Rotation3D()
      :wx(0.0f)
      ,wy(0.0f)
      ,wz(0.0f) {}

    Rotation3D::Rotation3D(float pInit)
      :wx(pInit)
      ,wy(pInit)
      ,wz(pInit) {}

    Rotation3D::Rotation3D(
        float pWx,
        float pWy,
        float pWz)
      :wx(pWx)
      ,wy(pWy)
      ,wz(pWz) {}

    Rotation3D::Rotation3D(const std::vector<float>& pFloats)
    {
      if (pFloats.size() == 3u)
      {
        wx = pFloats[0];
        wy = pFloats[1];
        wz = pFloats[2];
      }
      else
      {
        std::cerr << "ALMath: WARNING: "
                  << "Rotation3D constructor call with a wrong size of vector. "
                  << "Size expected: 3. Size given: " << pFloats.size() << ". "
                  << "Rotation3D is set to default value." << std::endl;

        wx = 0.0f;
        wy = 0.0f;
        wz = 0.0f;
      }
    }

    Rotation3D& Rotation3D::operator+= (const Rotation3D& pRot2)
    {
      wx += pRot2.wx;
      wy += pRot2.wy;
      wz += pRot2.wz;
      return *this;
    }


    Rotation3D& Rotation3D::operator-= (const Rotation3D& pRot2)
    {
      wx -= pRot2.wx;
      wy -= pRot2.wy;
      wz -= pRot2.wz;
      return *this;
    }


    bool Rotation3D::operator== (const Rotation3D& pRot2) const
    {
      return (wx == pRot2.wx &&
              wy == pRot2.wy &&
              wz == pRot2.wz);
    }

    bool Rotation3D::operator!= (const Rotation3D& pRot2) const
    {
      return !(*this==pRot2);
    }

    Rotation3D Rotation3D::operator/ (const float pVal) const
    {
      if (pVal == 0.0f)
      {
        throw std::runtime_error(
              "ALRotation3D: operator/ Division by zero.");
      }
      return (*this) * (1.0f/pVal);
    }

    Rotation3D& Rotation3D::operator*= (const float pVal)
    {
      wx *= pVal;
      wy *= pVal;
      wz *= pVal;
      return *this;
    }

    Rotation3D& Rotation3D::operator/= (const float pVal)
    {
      if (pVal == 0.0f)
      {
        throw std::runtime_error(
              "ALRotation3D: operator/= Division by zero.");
      }
      (*this) *= (1.0f/pVal);
      return *this;
    }


    bool Rotation3D::isNear(
        const Rotation3D& pRot2,
        const float&      pEpsilon) const
    {
      return (std::abs(wx - pRot2.wx) <= pEpsilon &&
              std::abs(wy - pRot2.wy) <= pEpsilon &&
              std::abs(wz - pRot2.wz) <= pEpsilon);
    }


    float Rotation3D::norm() const
    {
      return Math::norm(*this);
    }


    void Rotation3D::toVector(std::vector<float>& pReturnVector) const
    {
      pReturnVector.resize(3);
      pReturnVector[0] = wx;
      pReturnVector[1] = wy;
      pReturnVector[2] = wz;
    }

    std::vector<float> Rotation3D::toVector(void) const
    {
      std::vector<float> returnVector(3, 0.0f);
      this->toVector(returnVector);
      return returnVector;
    }


    float norm(const Rotation3D& pRot)
    {
      return std::sqrt(
            (pRot.wx*pRot.wx) + (pRot.wy*pRot.wy) + (pRot.wz*pRot.wz));
    }

  } // end namespace Math
} // end namespace AL
