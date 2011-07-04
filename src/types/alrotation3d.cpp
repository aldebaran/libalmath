/**
* @author Chris Kilner
* Copyright (c) Aldebaran Robotics 2009 All Rights Reserved
*
*/

#include <almath/types/alrotation3d.h>
#include <cmath>
#include <stdexcept>

namespace AL {
  namespace Math {


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
