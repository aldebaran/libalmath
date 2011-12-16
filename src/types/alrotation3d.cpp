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

#include <almath/types/alrotation3d.h>
#include <cmath>
#include <stdexcept>

namespace AL {
  namespace Math {

  Rotation3D::Rotation3D(): wx(0.0f),
    wy(0.0f),
    wz(0.0f) {}

  Rotation3D::Rotation3D(float pInit): wx(pInit),
    wy(pInit),
    wz(pInit) {}

  Rotation3D::Rotation3D(
    float pWx,
    float pWy,
    float pWz): wx(pWx),
    wy(pWy),
    wz(pWz) {}

  Rotation3D::Rotation3D(const std::vector<float>& pFloats)
  {
    if (pFloats.size() == 3)
    {
      wx = pFloats[0];
      wy = pFloats[1];
      wz = pFloats[2];
    }
    else
    {
      wx = 0.0f;
      wy = 0.0f;
      wz = 0.0f;
    }
  }

    Rotation3D Rotation3D::operator+ (const Rotation3D& pRot2) const
    {
      Rotation3D res;
      res.wx = wx + pRot2.wx;
      res.wy = wy + pRot2.wy;
      res.wz = wz + pRot2.wz;
      return res;
    }

    Rotation3D Rotation3D::operator- (const Rotation3D& pRot2) const
    {
      Rotation3D res;
      res.wx = wx - pRot2.wx;
      res.wy = wy - pRot2.wy;
      res.wz = wz - pRot2.wz;
      return res;
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
      if(
          (wx == pRot2.wx) &&
          (wy == pRot2.wy) &&
          (wz == pRot2.wz))
      {
        return true;
      }
      else
      {
        return false;
      }
    }

    bool Rotation3D::operator!= (const Rotation3D& pRot2) const
    {
      return !(*this==pRot2);
    }

    Rotation3D Rotation3D::operator* (const float pVal) const
    {
      Rotation3D res;
      res.wx = wx * pVal;
      res.wy = wy * pVal;
      res.wz = wz * pVal;
      return res;
    }

    Rotation3D Rotation3D::operator/ (const float pVal) const
    {
      if (pVal == 0.0f)
      {
        throw std::runtime_error(
          "ALRotation3D: operator/ Division by zeros.");
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
          "ALRotation3D: operator/= Division by zeros.");
      }
      (*this) *= (1.0f/pVal);
      return *this;
    }


    bool Rotation3D::isNear(
      const Rotation3D& pRot2,
      const float&      pEpsilon) const
    {
      if (
        (fabsf(wx - pRot2.wx) > pEpsilon) ||
        (fabsf(wy - pRot2.wy) > pEpsilon) ||
        (fabsf(wz - pRot2.wz) > pEpsilon))
      {
        return false;
      }
      else
      {
        return true;
      }
    }

    float Rotation3D::norm() const
    {
      return Math::norm(*this);
    }


    std::vector<float> Rotation3D::toVector() const
    {
      std::vector<float> returnVector;
      returnVector.resize(3);

      returnVector[0] = wx;
      returnVector[1] = wy;
      returnVector[2] = wz;

      return returnVector;
    }


    float norm(const Rotation3D& pRot)
    {
      return sqrtf( (pRot.wx*pRot.wx) + (pRot.wy*pRot.wy) + (pRot.wz*pRot.wz) );
    }

  } // end namespace Math
} // end namespace AL
