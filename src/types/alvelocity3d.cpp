/**
* @author Chris Kilner
* Copyright (c) Aldebaran Robotics 2009 All Rights Reserved
*/

#include <almath/types/alvelocity3d.h>
#include <cmath>
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

