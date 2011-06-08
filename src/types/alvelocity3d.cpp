/**
* @author Chris Kilner
* Copyright (c) Aldebaran Robotics 2009 All Rights Reserved
*/

#include <almath/types/alvelocity3d.h>
#include "math.h"
#include <stdexcept>

namespace AL {
  namespace Math {

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
      const Velocity3D& pVel,
      const float&      pEpsilon) const
    {
      if (
        (fabsf(xd - pVel.xd) > pEpsilon) ||
        (fabsf(yd - pVel.yd) > pEpsilon) ||
        (fabsf(zd - pVel.zd) > pEpsilon))
      {
        return false;
      }
      else
      {
        return true;
      }
    }


    Velocity3D Velocity3D::operator* (const float pM) const
    {
      Velocity3D res;
      res.xd = xd * pM;
      res.yd = yd * pM;
      res.zd = zd * pM;
      return res;
    }

    Velocity3D operator* (
      const float       pM,
      const Velocity3D& pVel1)
    {
      return pVel1*pM;
    }


    Velocity3D Velocity3D::operator/ (const float pM) const
    {
      if (pM == 0.0f)
      {
        throw std::runtime_error(
          "ALVelocity3D: operator/ Division by zeros.");
      }
      return (*this) * (1.0f/pM);
    }


    Velocity3D& Velocity3D::operator*= (const float pM)
    {
      xd *=pM;
      yd *=pM;
      zd *=pM;
      return *this;
    }

    Velocity3D& Velocity3D::operator/= (const float pM)
    {
      if (pM == 0.0f)
      {
        throw std::runtime_error(
          "ALVelocity3D: operator/= Division by zeros.");
      }
      (*this) *= (1.0f/pM);
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


    float norm (const Velocity3D& p)
    {
      return sqrtf( (p.xd*p.xd) + (p.yd*p.yd) + (p.zd*p.zd) );
    }


    Velocity3D normalize(const Velocity3D& p)
    {
      Velocity3D ret;
      ret = p;
      float tmpNorm = norm(p);

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

