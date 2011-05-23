/**
* @author Chris Kilner
* Copyright (c) Aldebaran Robotics 2009 All Rights Reserved
*
*/

#include <almath/types/alrotation3d.h>
#include "math.h"
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

    bool Rotation3D::isNear(
      const Rotation3D& pRot,
      const float&      pEpsilon) const
    {
      if (
        (fabsf(wx - pRot.wx) > pEpsilon) ||
        (fabsf(wy - pRot.wy) > pEpsilon) ||
        (fabsf(wz - pRot.wz) > pEpsilon))
      {
        return false;
      }
      else
      {
        return true;
      }
    }


    Rotation3D Rotation3D::operator* (const float pM) const
    {
      Rotation3D res;
      res.wx = wx * pM;
      res.wy = wy * pM;
      res.wz = wz * pM;
      return res;
    }

    Rotation3D Rotation3D::operator/ (const float pM) const
    {
      if (pM == 0.0f)
      {
        throw std::runtime_error(
          "ALRotation3D: operator/ Division by zeros.");
      }
      return (*this) * (1.0f/pM);
    }

    Rotation3D& Rotation3D::operator*= (const float pM)
    {
      wx *= pM;
      wy *= pM;
      wz *= pM;
      return *this;
    }

    Rotation3D& Rotation3D::operator/= (const float pM)
    {
      if (pM == 0.0f)
      {
        throw std::runtime_error(
          "ALRotation3D: operator/= Division by zeros.");
      }
      (*this) *= (1.0f/pM);
      return *this;
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


    float norm(const Rotation3D& p)
    {
      return sqrtf( (p.wx*p.wx) + (p.wy*p.wy) + (p.wz*p.wz) );
    }

  } // end namespace Math
} // end namespace AL
